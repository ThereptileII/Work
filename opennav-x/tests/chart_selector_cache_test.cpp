// Actual Piano atlas construction and draw preamble, with captured GL uploads.
// No chart model, navigation input or GPU acceptance is supplied by this test.
#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <wx/dcmemory.h>
#include <wx/wx.h>

// Pinned S52 color declarations require the wx types above.
#include "color_types.h"
#include "ui/Theme.h"
namespace opennav::integration {
bool xnav_mode = true, active = true;
#include "selector-resolver.inc"
} // namespace opennav::integration
wxColour outline(7, 7, 7);
wxColour GetGlobalColor(const wxString &name) {
  if (name == "CHBLK")
    return outline;
  const std::array<const char *, 10> keys = {
      "UIBDR", "BLUE2", "BLUE1", "GREEN2", "GREEN1",
      "VIO01", "VIO02", "YELO2", "YELO1",  "UINFD"};
  for (std::size_t i = 0; i < keys.size(); ++i)
    if (name == keys[i])
      return wxColour(20 + i * 19, 30 + i * 17, 40 + i * 13);
  throw std::runtime_error("Unknown role");
}
using GLuint = unsigned;
constexpr int GL_TEXTURE_2D = 1, GL_TEXTURE_MIN_FILTER = 2,
              GL_TEXTURE_MAG_FILTER = 3, GL_NEAREST = 4, GL_RGBA = 5,
              GL_UNSIGNED_BYTE = 6;
unsigned uploads = 0;
std::vector<unsigned char> texture;
void glGenTextures(int, GLuint *p) { *p = 1; }
void glBindTexture(int, unsigned) {}
void glTexParameteri(int, int, int) {}
void glTexImage2D(int, int, int, int w, int h, int, int, int, const void *p) {
  ++uploads;
  const auto *b = static_cast<const unsigned char *>(p);
  texture.assign(b, b + 4 * w * h);
}
void glTexSubImage2D(int, int, int, int, int, int, int, int, const void *) {}
int NextPow2(int n) {
  int v = 1;
  while (v < n)
    v *= 2;
  return v;
}
double OCPN_GetWinDIPScaleFactor() { return 1.; }
struct Platform {
  double GetDisplayDPmm() { return 2.; }
} platform, *g_Platform = &platform;
namespace ocpnStyle {
struct Style {
  bool chartStatusWindowTransparent = false;
};
} // namespace ocpnStyle
struct Styles {
  ocpnStyle::Style style;
  ocpnStyle::Style *GetCurrentStyle() { return &style; }
} styles, *g_StyleManager = &styles;
struct Canvas {
  ColorScheme scheme = GLOBAL_COLOR_SCHEME_DAY;
  ColorScheme GetColorScheme() { return scheme; }
  wxSize GetClientSize() { return {1014, 566}; }
  double GetContentScaleFactor() { return 1.; }
};
struct Piano {
  Canvas canvas;
  Canvas *m_parentCanvas = &canvas;
  wxBrush m_backBrush, m_rBrush, m_srBrush, m_vBrush, m_svBrush, m_utileBrush,
      m_tileBrush, m_cBrush, m_scBrush, m_unavailableBrush;
  wxColour m_tex_outline_color;
  unsigned m_tex = 0, m_texw = 0, m_texh = 0, m_tex_piano_height = 0;
  int m_ref = 0, m_pad = 0, m_radius = 0, m_texPitch = 0, height = 22;
  wxBitmap *m_pInVizIconBmp = nullptr, *m_pTmercIconBmp = nullptr,
           *m_pSkewIconBmp = nullptr, *m_pPolyIconBmp = nullptr;
  int GetHeight() { return height; }
  void SetColorScheme(ColorScheme);
  void SyncChartPresentation();
  void BuildGLTexture();
  void DrawGLSL(int);
};
#include "selector-methods.inc"
struct TestApp : wxApp {
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(TestApp);
int main(int argc, char **argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit())
    return 2;
  int result = 0, checks = 0;
  try {
    auto require = [&](bool ok, const char *msg) {
      ++checks;
      if (!ok)
        throw std::runtime_error(msg);
    };
    auto contains = [](wxColour c) {
      for (std::size_t i = 0; i < texture.size(); i += 4)
        if (texture[i] == c.Red() && texture[i + 1] == c.Green() &&
            texture[i + 2] == c.Blue() && texture[i + 3] == 255)
          return true;
      return false;
    };
    wxImage image(16, 16);
    image.SetRGB(wxRect(0, 0, 16, 16), 1, 2, 3);
    image.InitAlpha();
    wxBitmap icon(image);
    Piano p;
    p.SetColorScheme(GLOBAL_COLOR_SCHEME_DAY);
    p.DrawGLSL(0);
    require(uploads == 0, "missing icons must defer construction");
    p.m_pInVizIconBmp = p.m_pTmercIconBmp = p.m_pSkewIconBmp =
        p.m_pPolyIconBmp = &icon;
    p.DrawGLSL(0);
    require(uploads == 1 && contains(outline),
            "initial fallback outline must be baked");
    p.DrawGLSL(0);
    require(uploads == 1, "unchanged first frame must reuse atlas");
    const auto old = outline;
    outline = wxColour(83, 100, 95);
    p.DrawGLSL(0);
    require(uploads == 2 && contains(outline) && !contains(old),
            "lazy S52 color change must replace stale outline");
    const auto initial_day = texture;
    p.DrawGLSL(0);
    require(uploads == 2, "unchanged styled frame must reuse atlas");
    for (auto scheme : {GLOBAL_COLOR_SCHEME_DUSK, GLOBAL_COLOR_SCHEME_NIGHT,
                        GLOBAL_COLOR_SCHEME_DAY}) {
      p.canvas.scheme = scheme;
      outline = scheme == GLOBAL_COLOR_SCHEME_DUSK    ? wxColour(225, 229, 216)
                : scheme == GLOBAL_COLOR_SCHEME_NIGHT ? wxColour(106, 127, 120)
                                                      : wxColour(83, 100, 95);
      const auto count = uploads;
      p.SetColorScheme(scheme);
      p.DrawGLSL(0);
      require(uploads == count + 1 && contains(outline),
              "each theme must bake its actual outline");
      p.DrawGLSL(0);
      require(uploads == count + 1, "stable theme must not rebuild");
    }
    require(texture == initial_day,
            "Day-return atlas must exactly match initialized Day");
    // The outline changes while construction is deferred: do not falsely
    // commit.
    p.m_pInVizIconBmp = nullptr;
    outline = wxColour(31, 32, 33);
    const auto count = uploads;
    p.DrawGLSL(0);
    p.DrawGLSL(0);
    require(uploads == count, "deferred construction must not upload");
    p.m_pInVizIconBmp = &icon;
    p.DrawGLSL(0);
    require(uploads == count + 1 && contains(outline),
            "retry must bake deferred current ink");
    p.height = 28;
    p.DrawGLSL(0);
    require(uploads == count + 2, "existing height invalidation retained");
    for (bool enabled : {false, true}) {
      opennav::integration::active = enabled;
      outline = enabled ? wxColour(83, 100, 95) : wxColour(7, 7, 7);
      const auto n = uploads;
      p.DrawGLSL(0);
      require(uploads == n + 1 && contains(outline),
              "style fallback/reactivation must resolve current ink");
      p.DrawGLSL(0);
      require(uploads == n + 1,
              "stable fallback/active frame must reuse atlas");
    }
    opennav::integration::xnav_mode = false;
    outline = wxColour(7, 7, 7);
    p.DrawGLSL(0);
    require(contains(outline), "Legacy must retain stock resolved ink");
    const auto n = uploads;
    p.DrawGLSL(0);
    require(uploads == n, "stable Legacy must reuse atlas");
    std::cout << checks
              << " actual atlas/cache checks passed; GL uploads captured, no "
                 "GPU acceptance\n";
  } catch (const std::exception &e) {
    std::cerr << "FAILED: " << e.what() << '\n';
    result = 1;
  }
  wxTheApp->OnExit();
  wxEntryCleanup();
  return result;
}
