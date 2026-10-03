#pragma once

#include "ui/Theme.h"
#include <wx/bitmap.h>
#include <wx/image.h>
#include <cstdint>
#include <vector>
#include <utility>

namespace opennav::ui {
// Coverage is derived only from the unchanged, SHA-256-verified approved crop.
// Its flattened teal matte is textured: a single RGB transparency key cannot
// remove it. Green carries both the off-white SKAGER and mint APP strokes.
// The measured background-only margins/band stay <=52; map the remaining
// 52..255 range continuously to coverage, without tracing or moving a pixel.
// This recovers a reviewed raster approximation, not an original vector alpha.
class SkagerWordmark {
 public:
  // Display scale only: the approved raster and 180 x 68 DIP identity slot stay fixed.
  static constexpr int HeaderWidthDip = 124;
  static constexpr int HeaderLeftDip = 28;
  explicit SkagerWordmark(const wxImage& original) : original_(original) {
    if (!original.IsOk() || original.GetWidth()!=680 || original.GetHeight()!=214 ||
        original.HasAlpha() || original.HasMask()) return;
    const auto* rgb=original.GetData();
    // Decoded-pixel identity guard, independent of PNG container encoding.
    // Build-time verify-skager-brand.py retains the cryptographic source lock.
    std::uint64_t identity=14695981039346656037ull;
    for (int i=0;i<680*214*3;++i) identity=(identity^rgb[i])*1099511628211ull;
    if (identity!=0x7c8a65f6859fcafdull) return;
    std::vector<unsigned char> coverage(680*214);
    for (int y=0;y<214;++y) for (int x=0;x<680;++x) {
      const auto green=rgb[(y*680+x)*3+1];
      const bool background=x<8 || x>=672 || y<8 || y>=205 || (y>=120 && y<160);
      if (background && green>52) return;
      coverage[y*680+x]=green<=52 ? 0 : ((green-52)*255+101)/203;
    }
    coverage_=std::move(coverage);
  }

  bool UsesCoverage() const { return !coverage_.empty(); }
  unsigned Builds() const { return builds_; }
  const wxImage& Raster() const { return raster_; }
  const wxBitmap& Bitmap(int width, LightMode mode) {
    const auto colors=Theme(mode);
    const auto primary=mode==LightMode::Night ? colors.secondary : colors.primary;
    if (width<=0 || width>2048 || !original_.IsOk()) return empty_;
    if (bitmap_.IsOk() && width==width_ && primary==primary_ && colors.accent==accent_)
      return bitmap_;
    const int height=width*original_.GetHeight()/original_.GetWidth();
    if (height<=0) return empty_;
    wxImage next;
    if (UsesCoverage()) {
      next=wxImage(680,214);
      if (next.IsOk()) next.InitAlpha();
      if (next.IsOk() && next.HasAlpha()) {
        auto* rgb=next.GetData(); auto* alpha=next.GetAlpha();
        for (int y=0;y<214;++y) for (int x=0;x<680;++x) {
          const int at=y*680+x;
          const auto ink=y<140 ? primary : colors.accent;
          // Also fill transparent pixels with the foreground color: resizing
          // cannot mix the old teal matte into antialiased letter edges.
          rgb[3*at]=ink>>16; rgb[3*at+1]=ink>>8; rgb[3*at+2]=ink;
          alpha[at]=coverage_[at];
        }
      } else next=wxImage();
    }
    if (!next.IsOk()) next=original_;
    next=next.Scale(width,height,wxIMAGE_QUALITY_HIGH);
    wxBitmap bitmap(next);
    if (!bitmap.IsOk()) return empty_;
    raster_=next;bitmap_=bitmap;width_=width;primary_=primary;accent_=colors.accent;++builds_;
    return bitmap_;
  }
 private:
  wxImage original_,raster_;
  std::vector<unsigned char> coverage_;
  wxBitmap bitmap_,empty_;
  int width_=0;
  std::uint32_t primary_=0,accent_=0;
  unsigned builds_=0;
};
} // namespace opennav::ui
