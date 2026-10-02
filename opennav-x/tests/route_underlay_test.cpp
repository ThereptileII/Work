// Runs production painter code with real wx software rendering and a recording
// GL interface. The latter checks submission/state only, never driver output.
#include "integration/ChartRouteGeometry.h"
#include "integration/ChartRouteUnderlay.h"
#include "reference-underlay.h"
#include "tesselator.h"
#include "ui/Theme.h"
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <wx/fileconf.h>
#include <wx/graphics.h>
#include <wx/sstream.h>
#include <wx/wx.h>
class App : public wxApp {
public:
  bool OnInit() override { return true; }
};
wxIMPLEMENT_APP_NO_MAIN(App);
constexpr int GLOBAL_COLOR_SCHEME_DAY = 0, GLOBAL_COLOR_SCHEME_DUSK = 1,
              GLOBAL_COLOR_SCHEME_NIGHT = 2;
namespace opennav::ui {
wxColour Colour(std::uint32_t c) {
  return wxColour(c >> 16, (c >> 8) & 255, c & 255);
}
} // namespace opennav::ui
struct VP {
  float vp_matrix_transform[16]{};
};
struct ChartCanvas {
  int dpi = 100, theme = 0;
  VP vp;
  int GetColorScheme() const { return theme; }
  int FromDIP(int v) { return v * dpi / 100; }
  VP *GetpVP() { return &vp; }
};
struct ocpnDC {
  wxDC *native = nullptr;
  int m_canvasIndex = 0;
  void GetSize(int *w, int *h) {
    if (native)
      native->GetSize(w, h);
    else {
      *w = 900;
      *h = 570;
    }
  }
  wxDC *GetDC() { return native; }
  void CalcBoundingBox(int x, int y) {
    if (native)
      native->CalcBoundingBox(x, y);
  }
};
#define ocpnUSE_GL
using GLint = int;
using GLfloat = float;
constexpr int GL_CURRENT_PROGRAM = 1, GL_BLEND = 2, GL_TRIANGLES = 3,
              GL_BLEND_SRC_RGB = 4, GL_BLEND_DST_RGB = 5,
              GL_BLEND_SRC_ALPHA = 6, GL_BLEND_DST_ALPHA = 7,
              GL_BLEND_EQUATION_RGB = 8, GL_BLEND_EQUATION_ALPHA = 9,
              GL_SRC_ALPHA = 10, GL_ONE_MINUS_SRC_ALPHA = 11, GL_FUNC_ADD = 12;
int src_rgb = 21, dst_rgb = 22, src_alpha = 23, dst_alpha = 24, eq_rgb = 25,
    eq_alpha = 26;
void glBlendFunc(int a, int b) {
  src_rgb = src_alpha = a;
  dst_rgb = dst_alpha = b;
}
void glBlendFuncSeparate(int a, int b, int c, int d) {
  src_rgb = a;
  dst_rgb = b;
  src_alpha = c;
  dst_alpha = d;
}
void glBlendEquationSeparate(int a, int b) {
  eq_rgb = a;
  eq_alpha = b;
}
int program = 17, draws = 0, vertices = 0;
bool blend = true;
void glGetIntegerv(int name, int *v) {
  switch (name) {
  case GL_CURRENT_PROGRAM:
    *v = program;
    break;
  case GL_BLEND_SRC_RGB:
    *v = src_rgb;
    break;
  case GL_BLEND_DST_RGB:
    *v = dst_rgb;
    break;
  case GL_BLEND_SRC_ALPHA:
    *v = src_alpha;
    break;
  case GL_BLEND_DST_ALPHA:
    *v = dst_alpha;
    break;
  case GL_BLEND_EQUATION_RGB:
    *v = eq_rgb;
    break;
  case GL_BLEND_EQUATION_ALPHA:
    *v = eq_alpha;
    break;
  }
}
bool glIsEnabled(int) { return blend; }
void glEnable(int) { blend = true; }
void glDisable(int) { blend = false; }
void glUseProgram(int v) { program = v; }
void glDrawArrays(int mode, int, int count) {
  if (mode != GL_TRIANGLES || count % 3)
    throw std::runtime_error("GL triangle topology");
  ++draws;
  vertices = count;
}
struct Shader {
  float color[4]{};
  const float *positions = nullptr;
  void Bind() { program = 29; }
  void UnBind() { program = 0; }
  void SetUniformMatrix4fv(const char *, float *) {}
  void SetUniform4fv(const char *, float *v) { std::copy(v, v + 4, color); }
  void SetAttributePointerf(const char *, float *v) { positions = v; }
};
Shader shader;
Shader *pcolor_tri_shader_program[2]{&shader, &shader};
namespace opennav::integration {
bool enabled = true, xnav_mode = true, active = true;

bool ChartActiveRouteInk(ChartCanvas &c, wxColour &ink) {
  if (!enabled)
    return false;
  const unsigned colors[]{0x267c76, 0xb0dfc8, 0x91bca2};
  auto v = colors[c.theme];
  ink = wxColour(v >> 16, (v >> 8) & 255, v & 255);
  return true;
}
#define max(a, b) WINDOWS_MAX_MACRO_MUST_NOT_EXPAND
#include "production-foreground.h"
#include "production-underlay.h"
#undef max
} // namespace opennav::integration
int checks = 0;
void Check(bool value, const char *reason) {
  ++checks;
  if (!value)
    throw std::runtime_error(reason);
}
bool Contains(const std::vector<float> &m, double x, double y) {
  for (size_t i = 0; i < m.size(); i += 6) {
    const auto *t = m.data() + i;
    if (std::abs((t[2] - t[0]) * (t[5] - t[1]) -
                 (t[3] - t[1]) * (t[4] - t[0])) < 1e-10)
      continue;
    double c[3];
    for (int j = 0; j < 3; ++j) {
      int a = i + j * 2, b = i + (j + 1) % 3 * 2;
      c[j] =
          (m[b] - m[a]) * (y - m[a + 1]) - (m[b + 1] - m[a + 1]) * (x - m[a]);
    }
    if (!((c[0] < 0 || c[1] < 0 || c[2] < 0) &&
          (c[0] > 0 || c[1] > 0 || c[2] > 0)))
      return true;
  }
  return false;
}
double Area(const opennav::integration::RouteUnderlayMesh &m) {
  double area = 0;
  for (size_t i = 0; i < m.triangles.size(); i += 6) {
    const auto *p = m.triangles.data() + i;
    area += std::abs((p[2] - p[0]) * (p[5] - p[1]) -
                     (p[3] - p[1]) * (p[4] - p[0])) /
            2;
  }
  return area;
}
int Coverage(const std::vector<float> &m, double x, double y) {
  int inside = 0;
  for (size_t i = 0; i < m.size(); i += 6) {
    double c[3];
    for (int j = 0; j < 3; ++j) {
      int a = i + j * 2, b = i + (j + 1) % 3 * 2;
      c[j] =
          (m[b] - m[a]) * (y - m[a + 1]) - (m[b + 1] - m[a + 1]) * (x - m[a]);
    }
    if ((c[0] > 1e-7 && c[1] > 1e-7 && c[2] > 1e-7) ||
        (c[0] < -1e-7 && c[1] < -1e-7 && c[2] < -1e-7))
      ++inside;
  }
  return inside;
}
// Raw pinned-tess2 counterexample retained separately from production guards.
// --raw-tess2-defect intentionally fails its correct-union oracle.
void ReproducePinnedDefect() {
  std::unique_ptr<TESStesselator, decltype(&tessDeleteTess)> tess(
      tessNewTess(nullptr), tessDeleteTess);
  const std::vector<std::vector<float>> contours{
      {39.99f, 43, 39.99f, 37, 40, 37, 40, 43},
      {40, 37, 43, 37, 43, 40, 40, 40.01f, 39.99f, 40},
      {37, 40, 43, 40, 43, 40.01f, 37, 40.01f}};
  for (const auto &c : contours)
    tessAddContour(tess.get(), 2, c.data(), 2 * sizeof(float), c.size() / 2);
  const float normal[]{0, 0, 1};
  Check(tessTesselate(tess.get(), TESS_WINDING_NONZERO, TESS_POLYGONS, 3, 2,
                      normal),
        "raw tessellation failed");
  opennav::integration::RouteUnderlayMesh mesh;
  mesh.valid = true;
  auto *indices = tessGetElements(tess.get());
  auto *vertices = tessGetVertices(tess.get());
  for (int i = 0; i < tessGetElementCount(tess.get()) * 3; ++i) {
    mesh.triangles.push_back(vertices[indices[i] * 2]);
    mesh.triangles.push_back(vertices[indices[i] * 2 + 1]);
  }
  std::cout << "Pinned raw union area " << Area(mesh)
            << "; correct finite stroke area 9.1199; stray point "
            << Contains(mesh.triangles, 40.297, 40.42) << "\n";
  Check(std::abs(Area(mesh) - 9.1199) < .001 &&
            !Contains(mesh.triangles, 40.297, 40.42),
        "KNOWN PINNED TESS2 DEFECT: incorrect union; production guard must "
        "reject this entire layer");
}
int main(int argc, char **argv) {
  if (!wxEntryStart(argc, argv) || !wxTheApp->CallOnInit())
    return 2;
  try {
    using namespace opennav::integration;
    if (argc > 2 && std::string(argv[2]) == "--raw-tess2-defect")
      ReproducePinnedDefect();
    ChartRouteUnderlay line;
    line.Add(0, 20, 80, 120, 80);
    for (double dpi : {1., 1.25, 1.5, 2.}) {
      auto m = line.Mesh(dpi, 900, 570);
      Check(m.valid, "line tessellation failed");
      Check(std::abs(Area(m) - 100 * ref_width * dpi) < .001,
            "nominal width differs from HTML");
      Check(!Contains(m.triangles, 19.9, 80) &&
                !Contains(m.triangles, 120.1, 80),
            "butt endpoint expanded");
    }
    ChartRouteUnderlay duplicate;
    duplicate.Add(0, 20, 80, 120, 80);
    duplicate.Add(7, 20, 80, 120, 80);
    auto twice = duplicate.Mesh(1, 900, 570);
    Check(twice.valid && std::abs(Area(twice) - 600) < .001,
          "doubled leg accumulates geometry");
    ChartRouteUnderlay crossing;
    crossing.Add(0, 20, 80, 120, 80);
    crossing.Add(7, 70, 30, 70, 130);
    auto cross = crossing.Mesh(1, 900, 570);
    Check(cross.valid && std::abs(Area(cross) - 1164) < .001,
          "crossing was not unioned");
    for (double y = 25.123; y < 135; y += .731)
      for (double x = 15.217; x < 125; x += .913)
        Check(Coverage(cross.triangles, x, y) <= 1,
              "triangles overlap and would darken alpha");
    ChartRouteUnderlay turn;
    turn.Add(0, 20, 80, 120, 80);
    turn.Add(1, 120, 80, 120, 180);
    auto right = turn.Mesh(1, 900, 570);
    Check(right.valid && std::abs(Area(right) - 1200) < .001,
          "right miter area incorrect");
    Check(Contains(right.triangles, 122.5, 77.5),
          "miter corner replaced by round/bevel");
    ChartRouteUnderlay sharp;
    sharp.Add(0, 40, 160, 200, 160);
    sharp.Add(1, 200, 160, 120, 206.1880215);
    auto acute = sharp.Mesh(1, 900, 570);
    Check(acute.valid && Contains(acute.triangles, 210, 157.5),
          "sharp miter missing");
    ChartRouteUnderlay limit;
    limit.Add(0, 40, 160, 200, 160);
    limit.Add(1, 200, 160, 40, 188.2123169);
    auto bevel = limit.Mesh(1, 900, 570);
    Check(bevel.valid && !Contains(bevel.triangles, 210, 157.5),
          "SVG miter limit exceeded");
    // An independent area oracle sweeps both turn directions and rotations.
    // For a long mitered pair, outer join area exactly balances inner overlap.
    // At the SVG miter limit, the bevel removes r^2(tan(a/2)-sin(a)/2).
    for (int degrees = -178; degrees <= 178; ++degrees)
      if (degrees) {
        const double a = degrees * std::acos(-1.) / 180.;
        for (int rotation : {0, 17, 90}) {
          const double r = rotation * std::acos(-1.) / 180.;
          ChartRouteUnderlay sweep;
          sweep.Add(0, 2000 - 1000 * std::cos(r), 2000 - 1000 * std::sin(r),
                    2000, 2000);
          sweep.Add(1, 2000, 2000, 2000 + 1000 * std::cos(r + a),
                    2000 + 1000 * std::sin(r + a));
          auto mesh = sweep.Mesh(1, 8192, 8192);
          const double expected =
              12000 -
              (1 / std::cos(a / 2) > 4
                   ? 9 * (std::tan(std::abs(a) / 2) - std::sin(std::abs(a)) / 2)
                   : 0);
          Check(mesh.valid && std::abs(Area(mesh) - expected) < .8,
                "rotated turn union area differs from independent oracle");
          for (double y = 1988.217; y < 2012; y += 1.733)
            for (double x = 1988.123; x < 2012; x += 1.817)
              Check(Coverage(mesh.triangles, x, y) <= 1,
                    "turn union has overlapping triangles");
        }
      }
    // Compare short joined legs against an independent point-in-stroke oracle:
    // two finite butt rectangles plus only the outer SVG join polygon.
    for (double length : {.01, .5, 1., 3., 5., 6., 7., 10., 12.})
      for (int degrees : {-170, -150, -90, -30, 30, 90, 150, 170}) {
        const double a = degrees * std::acos(-1.) / 180., vx = std::cos(a),
                     vy = std::sin(a);
        const double side = vy > 0 ? -1 : 1;
        ChartRouteUnderlay short_route;
        short_route.Add(0, 40 - length, 40, 40, 40);
        short_route.Add(1, 40, 40, 40 + length * vx, 40 + length * vy);
        auto mesh = short_route.Mesh(1, 100, 100);
        if (length <= 6) {
          Check(!mesh.valid && mesh.triangles.empty(),
                "delicate join emitted partial or stray underlay geometry");
          continue;
        }
        Check(mesh.valid, "supported short join tessellation failed");
        std::vector<std::array<double, 2>> polygon{{0, 0}, {0, side * 3}};
        if (1 / std::cos(a / 2) <= 4)
          polygon.push_back({side * -vy * 3 / (1 + vx), side * 3});
        polygon.push_back({side * -vy * 3, side * vx * 3});
        for (double y = -12.137; y < 12; y += .433)
          for (double x = -12.213; x < 12; x += .417) {
            bool positive = false, negative = false;
            for (size_t k = 0; k < polygon.size(); ++k) {
              const auto &p = polygon[k],
                         &q = polygon[(k + 1) % polygon.size()];
              const double cross =
                  (q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0]);
              positive |= cross > 0;
              negative |= cross < 0;
            }
            const double along = x * vx + y * vy, normal = -x * vy + y * vx;
            const bool expected =
                (x >= -length && x <= 0 && std::abs(y) <= 3) ||
                (along >= 0 && along <= length && std::abs(normal) <= 3) ||
                !(positive && negative);
            Check(Contains(mesh.triangles, x + 40, y + 40) == expected,
                  "short join extends beyond intended stroke");
          }
      }
    ChartRouteUnderlay repeated;
    repeated.Add(0, 20, 80, 120, 80);
    repeated.Add(1, 120, 80, 120, 80);
    repeated.Add(2, 120, 80, 120, 180);
    auto repeat = repeated.Mesh(1, 900, 570);
    Check(repeat.valid && std::abs(Area(repeat) - 1200) < .001,
          "repeated waypoint breaks miter");
    ChartRouteUnderlay split;
    split.Add(0, 20, 80, 120, 80);
    split.Add(2, 120, 80, 120, 180);
    auto missing = split.Mesh(1, 900, 570);
    Check(missing.valid && !Contains(missing.triangles, 122.5, 77.5),
          "missing leg synthesized a join");
    ChartRouteUnderlay clipped;
    clipped.Add(0, 20, 80, 120, 80, true, false);
    clipped.Add(1, 120, 80, 120, 180, false, true);
    Check(!Contains(clipped.Mesh(1, 900, 570).triangles, 122.5, 77.5),
          "clipped edge became a waypoint miter");
    ChartRouteUnderlay wrapped;
    wrapped.Add(0, 20, 80, 120, 80);
    wrapped.Add(0, 620, 80, 720, 80);
    wrapped.Add(1, 120, 80, 120, 180);
    wrapped.Add(1, 720, 80, 720, 180);
    auto wrap = wrapped.Mesh(1, 900, 570);
    Check(wrap.valid && std::abs(Area(wrap) - 2400) < .001,
          "wrapped copies were joined or lost");
    Check(!Contains(wrap.triangles, 420, 80) &&
              Contains(wrap.triangles, 122.5, 77.5) &&
              Contains(wrap.triangles, 722.5, 77.5),
          "wrap gap bridged or copy join missing");
    ChartRouteUnderlay longline;
    longline.Add(0, -1e8, 80, 1e8, 80);
    auto bounded = longline.Mesh(1, 900, 570);
    Check(bounded.valid && bounded.triangles.size() < 100,
          "clipped input not bounded");
    for (float v : bounded.triangles)
      Check(std::isfinite(v) && v >= -12 && v <= 912,
            "offscreen geometry not bounded");
    ChartRouteUnderlay empty;
    empty.Add(0, 20, 20, 20, 20);
    Check(empty.Mesh(1, 900, 570).valid &&
              empty.Mesh(1, 900, 570).triangles.empty(),
          "zero segment generated paint");
    ChartRouteUnderlay invalid;
    invalid.Add(0, NAN, 20, 40, 20);
    Check(!invalid.Mesh(1, 900, 570).valid, "invalid coordinates accepted");
    ChartRouteUnderlay excess;
    for (int i = 0; i < 1025; ++i)
      excess.Add(i, 20, 30, 100, 30);
    Check(!excess.Mesh(1, 900, 570).valid,
          "overbudget route partially rendered");
    ChartCanvas canvas;
    ocpnDC gl;
    for (bool old : {false, true}) {
      blend = old;
      program = 17;
      int before = draws;
      Check(crossing.Draw(gl, canvas) && draws == before + 1,
            "union not submitted in one GL batch");
      Check(std::abs(shader.color[3] - ref_alpha) < 1e-6,
            "underlay opacity differs from HTML");
      Check(blend == old && program == 17 && src_rgb == 21 && dst_rgb == 22 &&
                src_alpha == 23 && dst_alpha == 24 && eq_rgb == 25 &&
                eq_alpha == 26,
            "GL state leaked");
    }
    for (int dpi : {100, 125, 150, 200}) {
      canvas.dpi = dpi;
      ChartRouteUnderlay delicate;
      delicate.Add(0, 20, 80, 120,
                   80); // Earlier valid leg must not leak partial paint.
      delicate.Add(1, 120, 80, 120, 80 + .01 * dpi / 100.);
      int previous = draws;
      Check(!delicate.Draw(gl, canvas) && draws == previous,
            "delicate join reached GL painter or emitted a partial union");
    }
    canvas.dpi = 100;
    enabled = false;
    int before = draws;
    Check(!crossing.Draw(gl, canvas) && draws == before,
          "Standard/Legacy/Safe fallback painted");
    enabled = true;
    wxInitAllImageHandlers();
    wxBitmap bmp(900, 570, 24);
    wxMemoryDC target(bmp);
    ocpnDC dc;
    dc.native = &target;
    target.SetBackground(*wxBLACK_BRUSH);
    target.Clear();
    ChartRouteUnderlay delicate;
    delicate.Add(0, 20, 80, 120, 80);
    delicate.Add(1, 120, 80, 120, 80.01);
    Check(!delicate.Draw(dc, canvas), "delicate join reached software painter");
    wxColour unchanged;
    target.GetPixel(70, 80, &unchanged);
    Check(unchanged == *wxBLACK,
          "delicate join left a partial software underlay");
    const unsigned waters[]{0xd5e5e5, 0x344f59, 0x121e24};
    const unsigned fills[]{0xf7f8f0, 0x243a40, 0x152129};
    for (int theme = 0; theme < 3; ++theme) {
      canvas.theme = theme;
      int offset = theme * 190;
      target.SetPen(*wxTRANSPARENT_PEN);
      target.SetBrush(wxBrush(opennav::ui::Colour(waters[theme])));
      target.DrawRectangle(0, offset, 900, 190);
      target.SetTextForeground(opennav::ui::Colour(theme==0?0x233e3e:0xaabdbd));
      target.DrawText(theme == 0   ? "Day: sharp miter / overlapping legs and "
                                     "crossing / foreground above underlay"
                      : theme == 1 ? "Dusk"
                                   : "Night",
                      15, offset + 8);
      ChartRouteUnderlay acute_fixture;
      acute_fixture.Add(0, 30, offset + 80, 180, offset + 80);
      acute_fixture.Add(1, 180, offset + 80, 80, offset + 137.7350269);
      Check(acute_fixture.Draw(dc, canvas),
            "sharp-turn software painter refused");
      ChartRouteUnderlay overlap;
      overlap.Add(0, 300, offset + 90, 480, offset + 90);
      overlap.Add(7, 480, offset + 90, 300, offset + 90);
      overlap.Add(9, 390, offset + 45, 390, offset + 145);
      Check(overlap.Draw(dc, canvas), "overlap software painter refused");
      ChartRouteUnderlay composed;
      composed.Add(0, 580, offset + 140, 720, offset + 60);
      composed.Add(1, 720, offset + 60, 850, offset + 140);
      Check(composed.Draw(dc, canvas), "composed software painter refused");
      Check(DrawChartRouteSegment(dc, canvas, 580, offset + 140, 720,
                                  offset + 60, false, true) &&
                DrawChartRouteSegment(dc, canvas, 720, offset + 60, 850,
                                      offset + 140, true, false),
            "foreground ordering fixture refused");
      // A stand-in point drawn last makes ordering visible, without claiming
      // production waypoint conformance (the separate waypoint task owns it).
      target.SetPen(wxPen(opennav::ui::Colour(opennav::ui::ActiveRouteInk(
                              static_cast<opennav::ui::LightMode>(theme))),
                          2));
      target.SetBrush(wxBrush(opennav::ui::Colour(fills[theme])));
      target.DrawCircle(720, offset + 60, 5);
    }
    target.SelectObject(wxNullBitmap);
    auto image = bmp.ConvertToImage();
    for (int theme = 0; theme < 3; ++theme) {
      int y = theme * 190 + 90;
      for (int channel = 0; channel < 3; ++channel) {
        auto pixel = [&](int x) {
          return channel == 0   ? image.GetRed(x, y)
                 : channel == 1 ? image.GetGreen(x, y)
                                : image.GetBlue(x, y);
        };
        Check(pixel(340) == pixel(390),
              "crossing or doubled leg darkens real wx alpha");
        const int shift = 16 - channel * 8;
        const double expected =
            ((fills[theme] >> shift) & 255) * ref_alpha +
            ((waters[theme] >> shift) & 255) * (1 - ref_alpha);
        Check(std::abs(pixel(340) - std::lround(expected)) <= 1,
              "software underlay alpha wrong");
      }
    }
    Check(image.SaveFile(argv[1], wxBITMAP_TYPE_PNG),
          "cannot save visual proof");
    auto started = std::chrono::steady_clock::now();
    for (int i = 0; i < 1000; ++i) {
      auto m = wrapped.Mesh(1, 1920, 1080);
      if (!m.valid)
        throw std::runtime_error("benchmark failed");
    }
    std::cout << checks << " checks passed; 1000 four-leg wrapped unions "
              << std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - started)
                     .count()
              << " ms\n";
    for (int count : {128, 1024}) {
      ChartRouteUnderlay dense;
      const auto point = [](int i) {
        int row = i / 100, col = i % 100;
        return std::array<double, 2>{10. + (row % 2 ? 100 - col : col) * 18.,
                                     50. + row * 18.};
      };
      for (int i = 0; i < count; ++i) {
        const auto a = point(i), b = point(i + 1);
        dense.Add(i, a[0], a[1], b[0], b[1]);
      }
      started = std::chrono::steady_clock::now();
      for (int i = 0; i < 10; ++i)
        Check(dense.Mesh(1, 1920, 1080).valid,
              "bounded long route unexpectedly failed");
      std::cout << count << "-leg union mean "
                << std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - started)
                           .count() /
                       10
                << " ms\n";
    }

    std::cout << checks << " total geometry/painter/workload checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  wxTheApp->OnExit();
  wxEntryCleanup();
  return 0;
}
