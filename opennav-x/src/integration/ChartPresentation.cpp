#include "integration/ChartPresentation.h"
#include "integration/ChartNameTypography.h"
#include "ui/Controls.h" // Before GL/X11 headers which define None.
#include "XNavChartResources.h"
#include "model/base_platform.h"
#include "picosha2.h"
#include "s52plib.h"
#include "chcanv.h"
#include "chartbase.h"
#include "ocpndc.h"
#include "FontMgr.h"

#include "integration/ChartRouteGeometry.h"
#include "integration/ChartCogPredictor.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/config_vars.h"
#include <wx/graphics.h>
#include <memory>
#ifdef ocpnUSE_GL
#include "shaders.h"
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <wx/ffile.h>
#include <wx/fileconf.h>
#include <wx/filename.h>
#include <wx/fontenum.h>
#include <wx/log.h>
#include <wx/thread.h>

extern BasePlatform *g_BasePlatform;
namespace opennav::integration {
namespace {
wxFileConfig *preferences = nullptr;
bool xnav_mode = false, requested = true, active = false;
std::string status = "Standard OpenCPN presentation";
CogPredictorStyleOwnership cog_style;
constexpr const char *key = "/OpenNav/ChartPresentationV1";
wxString ResourceDirectory() {
  if (!g_BasePlatform) return {};
  wxFileName folder(g_BasePlatform->GetSharedDataDir(), "");
  folder.AppendDir("opennav");
  folder.AppendDir("chart-style");
  folder.AppendDir("v1");
  return folder.GetPath();
}
wxColour Color(std::uint32_t c) {
  return {static_cast<unsigned char>(c >> 16),
          static_cast<unsigned char>(c >> 8), static_cast<unsigned char>(c)};
}
wxFont *GeographicNameFont(const char *feature, const char *instruction, bool tx,
                           double *tracking, unsigned char *opacity) {
  const auto role = GeographicChartName(feature, instruction, tx);
  if (role == ChartNameRole::Unchanged) return nullptr;
  *tracking = role == ChartNameRole::Land ? 1.0 : 5.0;
  *opacity = role == ChartNameRole::Land ? 255 : 92; // round(.36 * 255)
  // The selected chart style owns geographic-name typography. Never rewrite
  // FontMgr's persisted ChartTexts preference; Standard retains it verbatim.
  static const wxString face = [] {
    for (const auto *candidate : {"Segoe UI Variable Display", "Segoe UI", "Arial"})
      if (wxFontEnumerator::IsValidFacename(candidate)) return wxString(candidate);
    return wxString("Arial");
  }();
  // 12/16 CSS px are 9/12 points at 96 DPI. The upstream renderer retains its
  // DIP/content scale and the user's chart-text scale; do not scale twice.
  return FontMgr::Get().FindOrCreateFont(
      role == ChartNameRole::Land ? 9 : 12, wxFONTFAMILY_SWISS,
      role == ChartNameRole::Land ? wxFONTSTYLE_NORMAL : wxFONTSTYLE_ITALIC,
      wxFONTWEIGHT_NORMAL, false, face);
}
bool Verify(const wxString &folder) {
  for (const auto &resource : chart_style::generated::resources) {
    wxFFile file(
        wxFileName(folder, wxString::FromUTF8(resource.name)).GetFullPath(),
        "rb");
    if (!file.IsOpened() || file.Length() < 0 ||
        static_cast<std::uint64_t>(file.Length()) != resource.bytes)
      return false;
    picosha2::hash256_one_by_one hash;
    std::array<unsigned char, 8192> buffer{};
    std::uint64_t remaining = resource.bytes;
    while (remaining) {
      const auto count = static_cast<std::size_t>(
          (std::min)(remaining, std::uint64_t(buffer.size())));
      if (file.Read(buffer.data(), count) != count)
        return false;
      hash.process(buffer.begin(), buffer.begin() + count);
      remaining -= count;
    }
    hash.finish();
    if (picosha2::get_hash_hex_string(hash) != resource.sha256)
      return false;
  }
  return true;
}
void CaptureChartCogPredictorStyle(wxFileConfig &config) {
  // Snapshot configuration before ShipIndicatorsDraw can increase its width.
  // Saved factory defaults count as equivalent; do not rewrite any preference.
  long cog_width = 3, cog_pen_style = 105;
  wxString cog_color = "rgb(255,0,0)";
  const bool valid_width = !config.HasEntry("/Settings/OwnshipCOGPredictorWidth") ||
      config.Read("/Settings/OwnshipCOGPredictorWidth", &cog_width);
  const bool valid_style = !config.HasEntry("/Settings/OwnshipCOGPredictorStyle") ||
      config.Read("/Settings/OwnshipCOGPredictorStyle", &cog_pen_style);
  const bool valid_color = !config.HasEntry("/Settings/OwnshipCOGPredictorColor") ||
      config.Read("/Settings/OwnshipCOGPredictorColor", &cog_color);
  cog_style.Capture(valid_width && cog_width == 3 ? 3 : -1,
      valid_style && cog_pen_style == 105 ? 105 : -1,
      valid_color && wxColour(cog_color) == wxColour(255, 0, 0));
}
} // namespace
void ConfigureChartPresentation(wxFileConfig &config, bool xnav) {
  if (!wxIsMainThread())
    return;
  preferences = &config;
  CaptureChartCogPredictorStyle(config);
  xnav_mode = xnav;
  active = false;
  wxString saved;
  requested = !config.Read(key, &saved) || saved == "XNav";
  if (!saved.empty() && saved != "XNav" && saved != "Standard") {
    requested = false;
    status = "Invalid chart-style setting; using Standard";
  } else if (xnav_mode && requested) {
    // S-52 is lazy-loaded by upstream only when an ENC is encountered. The
    // coastline-only view still needs the same independently verified palette.
    const auto directory = ResourceDirectory();
    active = !directory.empty() && Verify(directory);
    status = active ? "SKAGER presentation v1 / verified palette; ENC not loaded"
                    : "SKAGER presentation resources missing or changed; Standard fallback";
  } else {
    status = "Standard OpenCPN presentation";
  }
}
s52plib *CreateChartPresentation(const wxString &stock_path,
                                 bool force_legacy) {
  if (!wxIsMainThread() || !xnav_mode)
    return new s52plib(stock_path, force_legacy);
  active = false;
  if (requested && g_BasePlatform) {
    const auto directory = ResourceDirectory();
    if (Verify(directory)) {
      auto *library = new s52plib(
          wxFileName(directory, "S52RAZDS.RLE").GetFullPath(), false, false, true);
      if (library->m_bOK) {
        library->SetTextFontResolver(GeographicNameFont);
        active = true;
        status = "SKAGER presentation v1 / pinned symbols";
        wxLogMessage("SKAGER chart presentation: verified SKAGER resources");
        return library;
      }
      delete library;
      status = "SKAGER presentation could not load; Standard fallback";
    } else
      status =
          "SKAGER presentation resources missing or changed; Standard fallback";
    wxLogWarning("SKAGER %s", wxString::FromUTF8(status));
  } else
    status = "Standard OpenCPN presentation";
  // XNav's explicit Standard fallback is not shadowed by a working-directory
  // chartsymbols.xml. Normal Legacy/Safe retain the pinned loader behavior.
  return new s52plib(stock_path, force_legacy, false);
}
bool ChartVectorSelectorInk(ColorScheme scheme, wxColour &selected,
                            wxColour &unselected) {
  if (!xnav_mode || !active)
    return false;
  const auto mode = scheme == GLOBAL_COLOR_SCHEME_NIGHT ? ui::LightMode::Night
                  : scheme == GLOBAL_COLOR_SCHEME_DUSK ? ui::LightMode::Dusk
                                                      : ui::LightMode::Day;
  selected = Color(ui::ActiveRouteInk(mode));
  unselected = Color(ui::FloatingTheme(mode).secondary);
  return true;
}
bool ChartBackground(ColorScheme scheme, wxColour &land, wxColour &water) {
  if (!xnav_mode || !active)
    return false;
  const auto index = scheme == GLOBAL_COLOR_SCHEME_NIGHT  ? 2
                     : scheme == GLOBAL_COLOR_SCHEME_DUSK ? 1
                                                          : 0;
  const auto p = chart_style::generated::backgrounds[index];
  land = Color(p.land);
  water = Color(p.water);
  return true;
}
bool XNavChartRequested() { return requested; }
bool ChartActiveRouteInk(ChartCanvas &canvas, wxColour &ink) {
  if (!wxIsMainThread() || !xnav_mode || !active) return false;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  ink = ui::Colour(ui::ActiveRouteInk(mode));
  return true;
}
bool DefaultChartRouteStyle(Route &route) {
  if (!route.IsVisible() || !route.m_bRtIsActive || route.m_bRtIsSelected ||
      route.m_bIsBeingEdited || route.m_hiliteWidth ||
      route.m_width != WIDTH_UNDEFINED || route.m_style != wxPENSTYLE_INVALID ||
      !route.m_Colour.empty() || g_route_line_width != 2)
    return false;
  // Both manual and AIS MOB routes carry the upstream "mob" waypoint icon.
  // Do not identify emergency routes by translated/user-editable route names.
  for (auto *node = route.pRoutePointList->GetFirst(); node; node = node->GetNext())
    if (node->GetData()->GetIconName() == "mob") return false;
  return true;
}
bool DrawChartRouteSegment(ocpnDC &dc, ChartCanvas &canvas, double ax, double ay,
                            double bx, double by, bool join_start, bool join_end) {
  wxColour ink;
  if (!ChartActiveRouteInk(canvas, ink)) return false;
  int width = 0, height = 0; dc.GetSize(&width, &height);
  auto triangles = ChartRouteSegmentMesh(ax, ay, bx, by,
      canvas.FromDIP(100) / 100.0, width, height, join_start, join_end);
  if (triangles.empty()) return false;
  if (auto *native = dc.GetDC()) {
    std::unique_ptr<wxGraphicsContext> gc(
        wxGraphicsContext::CreateFromUnknownDC(*native));
    if (!gc) return false;
    auto path = gc->CreatePath();
    for (std::size_t i = 0; i < triangles.size(); i += 6) {
      path.MoveToPoint(triangles[i], triangles[i+1]);
      path.AddLineToPoint(triangles[i+2], triangles[i+3]);
      path.AddLineToPoint(triangles[i+4], triangles[i+5]);
      path.CloseSubpath();
    }
    gc->SetBrush(wxBrush(ink));
    gc->FillPath(path, wxWINDING_RULE);
  } else {
#ifdef ocpnUSE_GL
    if (dc.m_canvasIndex < 0 || dc.m_canvasIndex >= 2) return false;
    auto *shader = pcolor_tri_shader_program[dc.m_canvasIndex];
    if (!shader) return false;
    // Reuse the chart's existing solid-color shader. No bitmap allocation,
    // texture upload, global line-width override or renderer-wide change.
    GLint old_program = 0; glGetIntegerv(GL_CURRENT_PROGRAM, &old_program);
    const auto blended = glIsEnabled(GL_BLEND);
    glDisable(GL_BLEND); // Opaque foreground: overlapping joins stay one ink.
    shader->Bind();
    shader->SetUniformMatrix4fv("MVMatrix",
        reinterpret_cast<GLfloat *>(canvas.GetpVP()->vp_matrix_transform));
    float color[] = {ink.Red()/256.f, ink.Green()/256.f, ink.Blue()/256.f, 1.f};
    shader->SetUniform4fv("color", color);
    shader->SetAttributePointerf("position", triangles.data());
    glDrawArrays(GL_TRIANGLES, 0, triangles.size()/2);
    shader->UnBind();
    glUseProgram(old_program);
    if (blended) glEnable(GL_BLEND);
#else
    return false;
#endif
  }
  // Dirty-region bookkeeping includes fractional stroke/join extents.
  for (std::size_t i = 0; i < triangles.size(); i += 2) {
    dc.CalcBoundingBox(std::floor(triangles[i]), std::floor(triangles[i+1]));
    dc.CalcBoundingBox(std::ceil(triangles[i]), std::ceil(triangles[i+1]));
  }
  return true;
}
bool UseChartCogPredictorStyle(int width, int style, const wxString &color,
                               int density_width) {
  if (!wxIsMainThread()) return false;
  const bool owned = cog_style.BeforeDensity(width, style,
      wxColour(color) == wxColour(255, 0, 0), density_width);
  return xnav_mode && active && owned;
}
bool DrawChartCogPredictor(ocpnDC &dc, ChartCanvas &canvas,
                           double ax, double ay, double bx, double by) {
  wxColour ink;
  if (!ChartActiveRouteInk(canvas, ink)) return false;
  int width=0, height=0; dc.GetSize(&width,&height);
  bool valid = false;
  auto triangles=ChartCogPredictorMesh(ax,ay,bx,by,
      canvas.FromDIP(100)/100.0,width,height,&valid);
  if (triangles.empty()) return valid;
  if (auto *native=dc.GetDC()) {
    std::unique_ptr<wxGraphicsContext> gc(
        wxGraphicsContext::CreateFromUnknownDC(*native));
    if (!gc) return false;
    auto path=gc->CreatePath();
    for (std::size_t i=0;i<triangles.size();i+=6) {
      path.MoveToPoint(triangles[i],triangles[i+1]);
      path.AddLineToPoint(triangles[i+2],triangles[i+3]);
      path.AddLineToPoint(triangles[i+4],triangles[i+5]); path.CloseSubpath();
    }
    // wxColour's 8-bit alpha rounds the prototype's .65 to 166/255.
    gc->SetBrush(wxBrush(wxColour(ink.Red(),ink.Green(),ink.Blue(),166)));
    gc->FillPath(path,wxWINDING_RULE);
  } else {
#ifdef ocpnUSE_GL
    if (dc.m_canvasIndex<0 || dc.m_canvasIndex>=2) return false;
    auto *shader=pcolor_tri_shader_program[dc.m_canvasIndex];
    if (!shader) return false;
    GLint old_program=0, src_rgb=0, dst_rgb=0, src_alpha=0, dst_alpha=0;
    GLint equation_rgb=0, equation_alpha=0;
    glGetIntegerv(GL_CURRENT_PROGRAM,&old_program);
    glGetIntegerv(GL_BLEND_SRC_RGB,&src_rgb); glGetIntegerv(GL_BLEND_DST_RGB,&dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA,&src_alpha); glGetIntegerv(GL_BLEND_DST_ALPHA,&dst_alpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB,&equation_rgb); glGetIntegerv(GL_BLEND_EQUATION_ALPHA,&equation_alpha);
    const auto blended=glIsEnabled(GL_BLEND);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquationSeparate(GL_FUNC_ADD,GL_FUNC_ADD);
    shader->Bind();
    shader->SetUniformMatrix4fv("MVMatrix",
        reinterpret_cast<GLfloat *>(canvas.GetpVP()->vp_matrix_transform));
    float color[]={ink.Red()/256.f,ink.Green()/256.f,ink.Blue()/256.f,.65f};
    shader->SetUniform4fv("color",color);
    shader->SetAttributePointerf("position",triangles.data());
    glDrawArrays(GL_TRIANGLES,0,triangles.size()/2);
    shader->UnBind(); glUseProgram(old_program);
    glBlendFuncSeparate(src_rgb,dst_rgb,src_alpha,dst_alpha);
    glBlendEquationSeparate(equation_rgb,equation_alpha);
    if (!blended) glDisable(GL_BLEND);
#else
    return false;
#endif
  }
  for (std::size_t i=0;i<triangles.size();i+=2) {
    dc.CalcBoundingBox(std::floor(triangles[i]),std::floor(triangles[i+1]));
    dc.CalcBoundingBox(std::ceil(triangles[i]),std::ceil(triangles[i+1]));
  }
  return true;
}
bool DrawChartOwnship(ocpnDC &dc, ChartCanvas &canvas, double x, double y,
                      double angle, double scale) {
  if (!wxIsMainThread() || !xnav_mode || !active ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(angle) ||
      !std::isfinite(scale) || scale <= 0)
    return false;
  // Immutable prototype index.html ownShipHeading: M0-19 11 16 0 10-11 16Z.
  // These are logical chart SVG pixels, not vessel dimensions or meters.
  // Measure a larger DIP span so fractional Windows DPI is not rounded to 1.
  scale *= canvas.FromDIP(100) / 100.0;
  // Reserve room for rotation, the miter and dirty bounds before narrowing to
  // wx integer coordinates. Refuse unusable geometry and let upstream draw.
  constexpr double safe_limit = (std::numeric_limits<int>::max)() / 2.0;
  if (!std::isfinite(scale) || scale <= 0 ||
      (std::max)(std::abs(x), std::abs(y)) + 64 * scale > safe_limit)
    return false;
  // Start at the right stern: ocpnDC's four-point GL strip then uses the
  // notch-to-bow diagonal, inside this concave polygon. Starting at the bow
  // would select the outside stern-to-stern diagonal and fill the notch.
  const std::array<wxPoint, 4> outline{{{11, 16}, {0, 10}, {-11, 16}, {0, -19}}};
  std::array<wxPoint, 4> points;
  const double c = std::cos(angle), s = std::sin(angle);
  int left = 0, right = 0, top = 0, bottom = 0;
  for (std::size_t i = 0; i < outline.size(); ++i) {
    const double px = outline[i].x * scale, py = outline[i].y * scale;
    points[i] = wxPoint(std::lround(x + px * c - py * s),
                        std::lround(y + px * s + py * c));
    if (!i) {
      left = right = points[i].x;
      top = bottom = points[i].y;
    } else {
      left = (std::min)(left, points[i].x); right = (std::max)(right, points[i].x);
      top = (std::min)(top, points[i].y); bottom = (std::max)(bottom, points[i].y);
    }
  }
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  // The immutable Night chart ancestor applies brightness(.78). Apply it to
  // this new artwork only; do not filter real chart content or route semantics.
  const auto ink = [mode](std::uint32_t value) {
    if (mode == ui::LightMode::Night) {
      const auto dim = [](unsigned channel) {
        return static_cast<unsigned>(std::lround(channel * .78));
      };
      value = (dim((value >> 16) & 255) << 16) |
              (dim((value >> 8) & 255) << 8) | dim(value & 255);
    }
    return Color(value);
  };
  const auto old_pen = dc.GetPen(); const auto old_brush = dc.GetBrush();
  wxPen pen(ink(ui::FloatingTheme(mode).surface),
            (std::max)(1, static_cast<int>(std::lround(3 * scale))));
  pen.SetJoin(wxJOIN_MITER);
  dc.SetPen(pen); dc.SetBrush(wxBrush(ink(ui::ActiveRouteInk(mode))));
  // The shared DC path is native in software and GL; no stock texture tint or
  // ownship texture cache can retain a previous light-mode color.
  dc.StrokePolygon(points.size(), points.data(), 0, 0);
  const int margin = static_cast<int>(std::ceil(6 * scale)); // Miter + rounding.
  dc.CalcBoundingBox(left - margin, top - margin);
  dc.CalcBoundingBox(right + margin, bottom + margin);
  dc.SetBrush(old_brush); dc.SetPen(old_pen);
  return true;
}
bool DrawChartDepthUnit(ocpnDC &dc, ChartCanvas &canvas) {
  if (!wxIsMainThread() || !xnav_mode || !active || !canvas.GetShowDepthUnits())
    return false;
  wxString unit;
  switch (canvas.GetChartDepthUnit()) {
    case DEPTH_UNIT_FEET: unit = _("Feet"); break;
    case DEPTH_UNIT_METERS: unit = _("Meters"); break;
    case DEPTH_UNIT_FATHOMS: unit = _("Fathoms"); break;
    default: return false; // Mixed/unknown units are never guessed.
  }
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto font = dc.GetFont();
  const auto ink = dc.GetTextForeground();
  dc.SetFont(ui::UiFont(canvas, 8)); // Final .map-disclaimer rule in the HTML.
  dc.SetTextForeground(ui::Colour(ui::FloatingTheme(mode).secondary));
  const wxString label = _("Chart depths") + ": " + unit;
  wxCoord width = 0, height = 0;
  dc.GetTextExtent(label, &width, &height);
  const auto size = canvas.GetClientSize();
  const int x = size.x - canvas.FromDIP(22) - width;
  // Keep the actual chart selector accessible until its workflow is redesigned.
  const int inset = (std::max)(canvas.FromDIP(15),
                               canvas.GetPianoHeight() + canvas.FromDIP(4));
  const int y = size.y - inset - height;
  const bool fits = x >= 0 && y >= 0 && width > 0 && height > 0;
  if (fits) dc.DrawText(label, x, y);
  dc.SetFont(font);
  dc.SetTextForeground(ink);
  return fits;
}
std::string ChartPresentationStatus() { return status; }
bool ChartScaleGeometry(ChartCanvas &canvas, int &x, int &y,
                        int &reference_width) {
  if (!wxIsMainThread() || !xnav_mode || !active ||
      canvas.GetClientSize().x < canvas.FromDIP(480)) return false;
  // .map-bottom-left: 28px inset, native Follow boat width 142px, 25px gap.
  x = canvas.FromDIP(28 + 142 + 25);
  y = canvas.GetClientSize().y - canvas.FromDIP(37);
  // Upstream halves this span, selects a nice distance in the user's units,
  // then projects that actual distance back to pixels. Never draw a fixed
  // 65px bar with an independently guessed distance label.
  reference_width = canvas.FromDIP(65 * 2);
  return true;
}
bool DrawChartScale(ocpnDC &dc, ChartCanvas &canvas, const wxString &label,
                    int x, int y, int length, wxRect &bounds) {
  if (!wxIsMainThread() || !xnav_mode || !active || length <= 0 ||
      x < 0 || y < 0 || length > canvas.GetClientSize().x - x) return false;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto ink = ui::Colour(ui::FloatingTheme(mode).secondary);
  const auto old_font = dc.GetFont(); const auto old_ink = dc.GetTextForeground();
  const auto old_pen = dc.GetPen(); const auto old_brush = dc.GetBrush();
  dc.SetFont(ui::UiFont(canvas, 8)); dc.SetTextForeground(ink);
  int width = 0, height = 0; dc.GetTextExtent(label, &width, &height);
  const int arm = canvas.FromDIP(5), gap = canvas.FromDIP(5);
  const int top = y - arm - gap - height;
  if (top >= 0) {
    // A real ENC can have a sounding directly behind this legend. Give the
    // scale a small neutral backing so charted depth cannot read as scale text.
    // The illustrative HTML never exercises this overlap; distance is still
    // the exact upstream result, and the legend's content geometry is unchanged.
    const int pad = canvas.FromDIP(4);
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(ui::Colour(ui::FloatingTheme(mode).surface)));
    dc.DrawRoundedRectangle(x - pad, top - pad,
        (std::max)(length, width) + 2 * pad, y - top + 1 + 2 * pad,
        canvas.FromDIP(3));
    dc.SetPen(wxPen(ink, canvas.FromDIP(1)));
    dc.DrawText(label, x, top);
    dc.DrawLine(x, y - arm, x, y);
    dc.DrawLine(x, y, x + length, y);
    dc.DrawLine(x + length, y, x + length, y - arm);
    bounds = wxRect(x - pad, top - pad, (std::max)(length, width) + 2 * pad,
                    y - top + 1 + 2 * pad);
  }
  dc.SetBrush(old_brush); dc.SetPen(old_pen); dc.SetFont(old_font); dc.SetTextForeground(old_ink);
  return top >= 0;
}
application::CommandResult SetXNavChartRequested(bool enabled) {
  if (!wxIsMainThread() || !preferences || !xnav_mode)
    return {false, "Chart style is unavailable in this interface"};
  wxString old;
  const bool existed = preferences->Read(key, &old);
  if (!preferences->Write(key, wxString(enabled ? "XNav" : "Standard")) ||
      !preferences->Flush()) {
    if (existed)
      preferences->Write(key, old);
    else
      preferences->DeleteEntry(key);
    const bool restored = preferences->Flush();
    return {false, restored ? "Chart style could not be saved"
                            : "Chart style save failed; check profile storage"};
  }
  requested = enabled;
  return {true, "Chart style saved. Restart SKAGER to apply."};
}
} // namespace opennav::integration
