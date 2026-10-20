#pragma once

#include "integration/ChartPresentationPolicy.h"
#include "XNavChartResources.h"
#include <wx/bmpbndl.h>
#include <wx/image.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace opennav::integration {
// Generated from the same reviewed artwork as the atlas. The generator flattens
// CSS classes: NanoSVG receives explicit paint attributes, not browser-only CSS.
inline const chart_style::generated::SymbolSvg* ChartSymbolSvg(
    bool verified, const char* symbol, const char* scheme) {
  if (!verified || !symbol || !scheme) return nullptr;
  if (!std::strcmp(scheme, "DAY")) scheme = "DAY_BRIGHT";
  for (const auto& item : chart_style::generated::symbol_svgs)
    if (!std::strncmp(symbol, item.name, 8) && !std::strcmp(scheme, item.scheme))
      return &item;
  return nullptr;
}

inline double ChartAidScale(const chart_style::generated::SymbolSvg* svg) {
  if (!svg) return 1.;
  // Only reviewed aid artwork. A sounding, danger mark, service or generic
  // structure keeps its own dimensions; no feature or selector is reclassified.
  for (const char* name : {
           "BOYCAR01", "BOYCAR02", "BOYCAR03", "BOYCAR04",
           "XNLAT013", "XNLAT014", "XNLAT023", "XNLAT024",
           "XNCAN072", "XNCAN073", "XNCON066", "XNCON067",
           "XNLIT011", "XNLIT012", "XNLIT013", "XNBCNG01",
           "XNSPPW01", "XNSPPY01", "XNSPPT01",
           "BCNCAR01", "BCNCAR02", "BCNCAR03", "BCNCAR04", "BCNISD21",
           "BCNLAT15", "BCNLAT16", "BCNLAT21", "BCNLAT22", "BCNSAW13",
           "BCNSAW21", "BCNSPP13", "BCNSPP21"})
    if (!std::strcmp(svg->name, name)) return 1.12;
  return 1.;
}

inline int ChartSymbolPixels(int logical, double scale) {
  if (logical <= 0 || logical > 512 || !std::isfinite(scale) ||
      scale <= 0 || scale > 16) return 0;
  const double pixels = logical * scale;
  return pixels <= 512 ? (std::max)(1, int(pixels)) : 0;
}

inline std::string ChartSymbolSvgInk(const char* svg, const SymbolInkRgb& black,
                                     const SymbolInkRgb& gray) {
  std::string result(svg);
  const auto hex = [](char c) {
    return c >= '0' && c <= '9' ? c - '0' :
           c >= 'a' && c <= 'f' ? c - 'a' + 10 :
           c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
  };
  for (std::size_t i = 0; i + 7 <= result.size(); ++i) {
    if (result[i] != '#' || (i + 7 < result.size() && hex(result[i+7]) >= 0))
      continue;
    SymbolInkRgb color{};
    bool valid = true;
    for (unsigned c = 0; c < 3; ++c) {
      const int a = hex(result[i+1+c*2]), b = hex(result[i+2+c*2]);
      if (a < 0 || b < 0) { valid = false; break; }
      color[c] = static_cast<unsigned char>(a * 16 + b);
    }
    if (valid && RecolorVeryLightObjectPixel(color.data(), black, gray))
      result.replace(i, 7, "#595959");
    i += 6;
  }
  return result;
}

// CPU-only, library-owned cache. No GL object survives a draw/context change.
// At most 256 entries / one million pixels (~4 MiB RGBA plus native bitmaps).
// Native FlushSymbolCaches clears it as well.
class ChartSymbolRasterCache {
 public:
  struct Raster { wxImage image; wxBitmap bitmap; };
  void Clear() { entries_.clear(); pixels_ = 0; }
  unsigned Builds() const { return builds_; }

  template<class Source>
  const Raster* Get(bool verified, const char* symbol, const char* scheme,
                    int logicalWidth, int logicalHeight, double scale,
                    bool veryLight, const SymbolInkRgb& black,
                    const SymbolInkRgb& gray, Source source) {
    if (!verified || !symbol || !scheme) return nullptr;
    const auto* svg = ChartSymbolSvg(true, symbol, scheme);
    const bool neutral = VeryLightObjectSymbol(veryLight, scheme, symbol);
    if (!svg && !neutral) return nullptr;
    if (svg && (svg->width != logicalWidth || svg->height != logicalHeight))
      return nullptr;  // Mismatched artwork must never move the native pivot.
    const int width = ChartSymbolPixels(logicalWidth, scale);
    const int height = ChartSymbolPixels(logicalHeight, scale);
    if (!width || !height) return nullptr;
    const std::string name(symbol, 8);
    auto found = std::find_if(entries_.begin(), entries_.end(),
        [&](const Entry& e) { return e.name == name && e.scheme == scheme; });
    if (found != entries_.end() && found->width == width &&
        found->height == height && found->neutral == neutral &&
        found->black == black && found->gray == gray)
      return &found->raster;

    wxImage image;
    if (svg) {
      // Rasterize geometry once at the final pixel dimensions. Never enlarge
      // the 24x28 browser-rasterized atlas or rescale the rendered bitmap.
      const auto inked = neutral ? ChartSymbolSvgInk(svg->svg, black, gray)
                                : std::string();
      const auto bundle = wxBitmapBundle::FromSVG(
          neutral ? inked.c_str() : svg->svg, wxSize(width, height));
      if (!bundle.IsOk()) return nullptr;
      const auto bitmap = bundle.GetBitmap(wxSize(width, height));
      if (!bitmap.IsOk()) return nullptr;
      image = bitmap.ConvertToImage();
    } else {
      image = source();
      if (!image.IsOk() || image.GetWidth() != logicalWidth ||
          image.GetHeight() != logicalHeight) return nullptr;
      // Recolour native flat pixels before interpolation, preserving their
      // original coverage and all chromatic/semantic fills.
      image = image.Copy();
      Recolor(image, black, gray);
      image.Rescale(width, height, wxIMAGE_QUALITY_HIGH);
    }
    if (!image.IsOk() || image.GetWidth() != width || image.GetHeight() != height ||
        !image.HasAlpha()) return nullptr;
    wxBitmap bitmap(image, 32);
    if (!bitmap.IsOk()) return nullptr;
    Entry next{name, scheme, width, height, neutral, black, gray, {image, bitmap}};
    if (found != entries_.end()) {
      pixels_ -= found->width * found->height;
      entries_.erase(found);
    }
    const std::size_t pixels = std::size_t(width) * height;
    while (!entries_.empty() &&
           (entries_.size() >= 256 || pixels_ + pixels > 1024 * 1024)) {
      pixels_ -= entries_.front().width * entries_.front().height;
      entries_.erase(entries_.begin());
    }
    entries_.push_back(std::move(next));
    pixels_ += pixels;
    ++builds_;
    return &entries_.back().raster;
  }

 private:
  static void Recolor(wxImage& image, const SymbolInkRgb& black,
                      const SymbolInkRgb& gray) {
    auto* rgb = image.GetData();
    for (int i = 0; i < image.GetWidth() * image.GetHeight(); ++i)
      RecolorVeryLightObjectPixel(rgb + 3 * i, black, gray);
  }
  struct Entry {
    std::string name, scheme;
    int width, height;
    bool neutral;
    SymbolInkRgb black, gray;
    Raster raster;
  };
  std::vector<Entry> entries_;
  std::size_t pixels_ = 0;
  unsigned builds_ = 0;
};
}  // namespace opennav::integration
