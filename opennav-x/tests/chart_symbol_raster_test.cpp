#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/init.h>
#include "integration/ChartSymbolRaster.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "actual-chart-symbol-display.inc"

namespace {
int checks = 0;
void Check(bool ok, const char* message) {
  ++checks;
  if (!ok) throw std::runtime_error(message);
}
bool Equal(const wxImage& a, const wxImage& b) {
  if (!a.IsOk() || !b.IsOk() || a.GetSize() != b.GetSize() ||
      a.HasAlpha() != b.HasAlpha()) return false;
  const auto pixels = std::size_t(a.GetWidth()) * a.GetHeight();
  return !std::memcmp(a.GetData(), b.GetData(), pixels * 3) &&
         (!a.HasAlpha() || !std::memcmp(a.GetAlpha(), b.GetAlpha(), pixels));
}
int Run() {
  using namespace opennav::integration;
  const SymbolInkRgb black{83, 100, 95}, gray{83, 100, 95};
  const auto* aid = ChartSymbolSvg(true, "XNLAT013", "DAY");
  Check(aid && aid->width == 24 && aid->height == 28, "verified aid geometry");
  Check(!ChartSymbolSvg(false, "XNLAT013", "DAY"), "stock presentation stays stock");
  Check(!ChartSymbolSvg(true, "SOUND001", "DAY"), "soundings are not aid artwork");
  Check(ChartAidScale(aid) == 1.12, "bounded aid gain");
  for (const char* name : {"WRECKS01", "OBSTRN01", "UWTROC03", "SOUND001"})
    Check(ChartAidScale(ChartSymbolSvg(true, name, "DAY")) == 1.,
          "hazards and soundings keep native dimensions");
  Check(ChartSymbolPixels(24, NAN) == 0 && ChartSymbolPixels(24, 100) == 0 &&
        ChartSymbolPixels(0, 1) == 0, "invalid raster sizes refuse");
  Check(ChartSymbolSvgInk(
      "<path fill=\"#687b7a\" stroke=\"#B66E6C\" opacity=\".35\"/>", black, gray) ==
      "<path fill=\"#595959\" stroke=\"#B66E6C\" opacity=\".35\"/>",
      "neutral paint changes before alpha rasterization; colour/opacity retained");

  ChartSymbolRasterCache cache;
  unsigned loads = 0;
  auto noAtlas = [&] { ++loads; return wxImage(); };
  wxImage one;
  for (int percent : {100, 125, 150, 100}) {
    const double scale = percent / 100. * ChartAidScale(aid);
    const auto* item = cache.Get(true, aid->name, "DAY", 24, 28, scale,
                                 false, black, gray, noAtlas);
    Check(item && item->image.GetWidth() == int(24*scale) &&
          item->image.GetHeight() == int(28*scale), "exact final pixel dimensions");
    const auto direct = wxBitmapBundle::FromSVG(aid->svg, item->image.GetSize())
                            .GetBitmap(item->image.GetSize()).ConvertToImage();
    Check(Equal(item->image, direct), "final raster equals direct vector geometry");
    const auto builds = cache.Builds();
    Check(cache.Get(true, aid->name, "DAY", 24, 28, scale,
                    false, black, gray, noAtlas) == item &&
          cache.Builds() == builds, "unchanged fractional scale hits cache");
    if (percent == 100) one = item->image.Copy();
    if (percent == 150) {
      auto enlarged = one.Copy();
      enlarged.Rescale(item->image.GetWidth(), item->image.GetHeight(),
                       wxIMAGE_QUALITY_HIGH);
      Check(!Equal(enlarged, item->image), "150 percent does not enlarge low-res art");
    }
  }
  Check(loads == 0, "owned SVG never loads low-resolution atlas");
  Check(!cache.Get(true, aid->name, "DAY", 23, 28, 1., false, black, gray, noAtlas),
        "mismatched logical geometry refuses");
  for (const char* scheme : {"DAY_BRIGHT", "DUSK", "NIGHT"}) {
    const auto* item = cache.Get(true, aid->name, scheme, 24, 28, 1.5,
                                 true, black, gray, noAtlas);
    Check(item && item->image.IsOk(), "all three palettes render at final size");
    const auto expected = wxBitmapBundle::FromSVG(
        ChartSymbolSvg(true, aid->name, scheme)->svg, wxSize(36,42))
        .GetBitmap(wxSize(36,42)).ConvertToImage();
    Check(Equal(item->image, expected), "Very Light never neutralizes chromatic aid");
  }
  const auto* hazard = ChartSymbolSvg(true, "WRECKS01", "DAY");
  Check(hazard, "reviewed hazard vector exists");
  const auto* ordinary = cache.Get(true, hazard->name, "DAY", hazard->width,
      hazard->height, 1.25, false, black, gray, noAtlas);
  Check(ordinary, "ordinary hazard paint");
  const wxImage ordinaryImage = ordinary->image.Copy();
  const auto* light = cache.Get(true, hazard->name, "DAY", hazard->width,
      hazard->height, 1.25, true, black, gray, noAtlas);
  Check(light && !Equal(ordinaryImage, light->image), "Very Light changes hazard ink");
  const auto count = std::size_t(light->image.GetWidth()) * light->image.GetHeight();
  Check(!std::memcmp(ordinaryImage.GetAlpha(), light->image.GetAlpha(), count),
        "neutral SVG paint preserves every alpha coverage value");
  const auto* restored = cache.Get(true, hazard->name, "DAY", hazard->width,
      hazard->height, 1.25, false, black, gray, noAtlas);
  Check(restored && Equal(restored->image, ordinaryImage), "style change rebuilds ink");

  // Run the actual patched core/private configuration, preparation, dispatch
  // and owned rendering methods, with only chart projection/data providers
  // stubbed. Paint into a native wxDC, not a synthetic scale-only model.
  s52plib library;
  wxBitmap target(200, 200, 32);
  wxMemoryDC dc(target);
  library.m_pdc = &dc;
  Rule rule;
  std::memcpy(rule.name.SYNM, "XNLAT013", 8);
  rule.pos.line.pivot_x.SYCL = 12; rule.pos.line.pivot_y.SYRW = 14;
  Object object; std::memcpy(object.FeatureName, "BOYLAT", 6);
  ObjRazRules node{&object};
  wxPoint anchor(80, 90);
  unsigned lastBuilds = 0;
  for (int percent : {100, 125, 150, 100}) {
    library.ConfigurePresentationDisplay(false, percent);
    library.SetScaleFactorExp(1.); // Real incoming stock/options base factor.
    library.PrepareForRender(nullptr);
    Check(library.m_ChartScaleFactorExp == percent/100.,
          "actual Prepare retains display scale exactly once");
    dc.SetBackground(*wxWHITE_BRUSH); dc.Clear();
    Check(library.RenderRasterSymbol(&node, &rule, anchor, 0),
          "actual raster dispatch paints owned SVG");
    Check(library.m_presentationRasterSubmitted, "actual paint publishes submission");
    const double scale = percent / 100. * 1.12;
    Check(object.BBObj.width == int(24*scale) &&
          object.BBObj.height == int(28*scale) &&
          object.BBObj.x == anchor.x-int(12*scale) &&
          object.BBObj.y == anchor.y-int(14*scale),
          "actual dimensions and pivot follow percentage");
    Check(library.m_presentationSymbolRasters.Builds() == lastBuilds+1,
          "percentage transition rebuilds actual image");
    lastBuilds = library.m_presentationSymbolRasters.Builds();
    library.RenderRasterSymbol(&node, &rule, anchor, 0);
    Check(library.m_presentationSymbolRasters.Builds() == lastBuilds,
          "repeated paint reuses actual raster cache");
  }
  const auto hashChanges = library.hashChanges;
  library.ConfigurePresentationDisplay(false, 100);
  library.ConfigurePresentationDisplay(false, 126);
  Check(library.hashChanges == hashChanges, "unchanged/invalid percent is inert");
  library.m_presentationLightSymbols = false;
  Check(!library.RenderRasterSymbol(&node, &rule, anchor, 0),
        "Standard/Legacy dispatch falls through untouched");
  dc.SelectObject(wxNullBitmap);
  std::cout << checks << " native wx raster and actual renderer checks passed\n";
  return 0;
}
class App : public wxApp {
 public:
  bool OnInit() override {
    try { result = Run(); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; result = 1; }
    return false;
  }
  int result = 1;
};
wxIMPLEMENT_APP_NO_MAIN(App);
}  // namespace
int main(int argc, char** argv) {
  if (!wxEntryStart(argc, argv)) return 2;
  wxTheApp->CallOnInit();
  const int result = static_cast<App*>(wxTheApp)->result;
  wxEntryCleanup();
  return result;
}
