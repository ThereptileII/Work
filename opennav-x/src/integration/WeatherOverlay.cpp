#include "integration/WeatherOverlay.h"
#include "integration/ChartPresentation.h"
#include "integration/ChartCanvasInk.h"
#include "application/ChartDeclutter.h"
#include "weather/ForecastView.h"
#include "ui/Controls.h"
#include "ui/Theme.h"
#include "chcanv.h"
#include "ocpndc.h"
#include "viewport.h"
#include "model/georef.h"
#include <wx/datetime.h>
#include <wx/graphics.h>
#include <wx/thread.h>
#include <wx/weakref.h>
#include <cmath>
#include <memory>
#include <vector>
#ifdef ocpnUSE_GL
#include "shaders.h"
#endif

namespace opennav::integration {
namespace {
// Application thread only (asserted by every entry point).
std::function<weather::ForecastSnapshot()> source;
std::optional<weather::WallTime> display_time;
bool visible = false;  // SCRUM-328: off by default, never persisted here.
// SCRUM-357: the badge can be collapsed to a small FORECAST chip for the
// session. It is never removed outright: chart wind must not appear without
// its provenance marker, so the chip stays as long as arrows are drawn.
bool badge_collapsed = false;
// Weak: the filter outlives any one canvas (mode restart, shutdown).
wxWeakRef<ChartCanvas> badge_canvas;
wxRect badge_hit;  // Canvas client coordinates of the toggle target.

// Painted on the canvas, so the badge has no window of its own to click. A
// process-wide filter sees the press first, toggles only when it lands on the
// badge's target, and consumes it so the same tap does not also pan or select.
class BadgeToggleFilter final : public wxEventFilter {
 public:
  int FilterEvent(wxEvent &event) override {
    ChartCanvas *canvas = badge_canvas.get();
    if (event.GetEventType() != wxEVT_LEFT_DOWN || !canvas || badge_hit.IsEmpty())
      return Event_Skip;
    auto *window = dynamic_cast<wxWindow *>(event.GetEventObject());
    auto *mouse = dynamic_cast<wxMouseEvent *>(&event);
    if (!window || !mouse || (window != canvas && !canvas->IsDescendant(window)))
      return Event_Skip;
    // A GL child receives the press in its own coordinates.
    const auto at = canvas->ScreenToClient(window->ClientToScreen(mouse->GetPosition()));
    if (!badge_hit.Contains(at)) return Event_Skip;
    badge_collapsed = !badge_collapsed;
    canvas->Refresh(false);
    return Event_Processed;
  }
};
void EnsureBadgeFilter() {
  static BadgeToggleFilter *filter = nullptr;
  if (!filter) { filter = new BadgeToggleFilter; wxEvtHandler::AddFilter(filter); }
}

ui::LightMode CanvasMode(ChartCanvas &canvas) {
  return canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT ? ui::LightMode::Night
       : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK  ? ui::LightMode::Dusk
                                                              : ui::LightMode::Day;
}
bool Project(ChartCanvas &canvas, ViewPort &vp, weather::Coordinate c, wxPoint &point) {
  if (!vp.IsValid() || !canvas.GetCanvasPointPixVP(vp, c.latitude_deg, c.longitude_deg, &point))
    return false;
  // Upstream may return true with INVALID_COORD; clip before arithmetic.
  return point.x >= -64 && point.y >= -64 && point.x <= vp.pix_width + 64 &&
         point.y <= vp.pix_height + 64;
}
// Clockwise screen angle (from up) toward which the air moves, using the
// canvas projection (rotation, skew and raster georeferencing included).
std::optional<double> DownwindAngle(ChartCanvas &canvas, ViewPort &vp,
                                    weather::Coordinate c, wxPoint point,
                                    double direction_from_deg) {
  if (!std::isfinite(vp.view_scale_ppm) || vp.view_scale_ppm <= 0) return {};
  const double distance = (100.0 / vp.view_scale_ppm) / 1852.0;
  if (!std::isfinite(distance) || distance <= 0 || distance > 100) return {};
  double lat = 0, lon = 0;
  ll_gc_ll(c.latitude_deg, c.longitude_deg,
           weather::NormalizeDegrees(direction_from_deg + 180), distance, &lat, &lon);
  if (!std::isfinite(lat) || !std::isfinite(lon) || std::abs(lat) > 90) return {};
  wxPoint ahead;
  if (!canvas.GetCanvasPointPixVP(vp, lat, lon, &ahead) ||
      std::abs(static_cast<double>(ahead.x)) > 1e6 ||
      std::abs(static_cast<double>(ahead.y)) > 1e6 || ahead == point) return {};
  return std::atan2(static_cast<double>(ahead.x) - point.x,
                    -(static_cast<double>(ahead.y) - point.y));
}
std::vector<float> Disc(double cx, double cy, double radius) {
  std::vector<float> v;
  constexpr int segments = 32;
  for (int i = 0; i < segments; ++i) {
    const double a = i * 2 * 3.141592653589793 / segments;
    const double b = (i + 1) * 2 * 3.141592653589793 / segments;
    v.insert(v.end(), {float(cx), float(cy), float(cx + radius * std::cos(a)),
                       float(cy + radius * std::sin(a)), float(cx + radius * std::cos(b)),
                       float(cy + radius * std::sin(b))});
  }
  return v;
}
struct Layer { wxColour colour; std::vector<float> triangles; };
struct Label { wxString text; int x, y; };

// Fills opaque triangle lists in order on either render path. Same technique
// as the XNav route waypoint marker (ChartRouteWaypoint.cpp).
bool FillLayers(ocpnDC &dc, ChartCanvas &canvas, const std::vector<Layer> &layers) {
  if (auto *native = dc.GetDC()) {
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::CreateFromUnknownDC(*native));
    if (!gc) return false;
    gc->SetPen(*wxTRANSPARENT_PEN);
    for (const auto &layer : layers) {
      if (layer.triangles.size() < 6) continue;
      auto path = gc->CreatePath();
      for (std::size_t i = 0; i + 5 < layer.triangles.size(); i += 6) {
        path.MoveToPoint(layer.triangles[i], layer.triangles[i + 1]);
        path.AddLineToPoint(layer.triangles[i + 2], layer.triangles[i + 3]);
        path.AddLineToPoint(layer.triangles[i + 4], layer.triangles[i + 5]);
        path.CloseSubpath();
      }
      gc->SetBrush(wxBrush(layer.colour));
      gc->FillPath(path, wxWINDING_RULE);
    }
    return true;
  }
#ifdef ocpnUSE_GL
  if (dc.m_canvasIndex < 0 || dc.m_canvasIndex >= 2) return false;
  auto *shader = pcolor_tri_shader_program[dc.m_canvasIndex];
  if (!shader || !canvas.GetpVP()) return false;
  GLint program = 0, texture = 0, src_rgb = 0, dst_rgb = 0, src_alpha = 0, dst_alpha = 0;
  glGetIntegerv(GL_CURRENT_PROGRAM, &program);
  glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture);
  glGetIntegerv(GL_BLEND_SRC_RGB, &src_rgb); glGetIntegerv(GL_BLEND_DST_RGB, &dst_rgb);
  glGetIntegerv(GL_BLEND_SRC_ALPHA, &src_alpha); glGetIntegerv(GL_BLEND_DST_ALPHA, &dst_alpha);
  const auto blended = glIsEnabled(GL_BLEND);
  const auto textured = glIsEnabled(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
  shader->Bind();
  shader->SetUniformMatrix4fv("MVMatrix",
      reinterpret_cast<GLfloat *>(canvas.GetpVP()->vp_matrix_transform));
  for (const auto &layer : layers) {
    if (layer.triangles.size() < 6) continue;
    float rgba[] = {layer.colour.Red() / 256.f, layer.colour.Green() / 256.f,
                    layer.colour.Blue() / 256.f, 1.f};
    shader->SetUniform4fv("color", rgba);
    auto vertices = layer.triangles;
    shader->SetAttributePointerf("position", vertices.data());
    glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 2);
  }
  shader->UnBind();
  glUseProgram(program);
  glBindTexture(GL_TEXTURE_2D, texture);
  glBlendFuncSeparate(src_rgb, dst_rgb, src_alpha, dst_alpha);
  if (textured) glEnable(GL_TEXTURE_2D); else glDisable(GL_TEXTURE_2D);
  if (blended) glEnable(GL_BLEND); else glDisable(GL_BLEND);
  return true;
#else
  (void)canvas;
  return false;
#endif
}
wxString W(const std::string &text) { return wxString::FromUTF8(text); }
wxString LocalTime(weather::WallTime t) {
  return wxDateTime(static_cast<time_t>(
      std::chrono::duration_cast<std::chrono::seconds>(t.time_since_epoch()).count()))
      .Format("%H:%M");
}
// Floating provenance badge: chart wind is never shown without FORECAST,
// source, run age, step and (when stale) fetch age.
void DrawBadge(ocpnDC &dc, ChartCanvas &canvas, ViewPort &vp, ui::LightMode mode,
               const std::vector<wxString> &lines, bool attention, const wxString &chip) {
  EnsureBadgeFilter();
  badge_canvas = &canvas;
  const auto floating = ui::FloatingTheme(mode);
  const auto font = ui::UiFontWeight(canvas, 10, 600);
  dc.SetFont(font);
  const int pad = canvas.FromDIP(8), gap = canvas.FromDIP(2);
  // Touch-sized target even though the glyph itself is small.
  const int target = canvas.FromDIP(28), glyph = canvas.FromDIP(5);
  const auto border = wxPen(ui::Colour(attention ? ui::Theme(mode).attention
                                                 : floating.compass_light),
                            canvas.FromDIP(1));
  if (badge_collapsed) {
    wxCoord w = 0, h = 0; dc.GetTextExtent(chip, &w, &h);
    const int box_w = w + 2 * pad, box_h = h + 2 * pad;
    const int x = std::max(pad, (vp.pix_width - box_w) / 2), y = canvas.FromDIP(12);
    dc.SetPen(border);
    dc.SetBrush(wxBrush(ui::Colour(floating.surface)));
    dc.DrawRoundedRectangle(x, y, box_w, box_h, canvas.FromDIP(8));
    dc.SetTextForeground(ui::Colour(floating.primary));
    dc.DrawText(chip, x + pad, y + pad);
    badge_hit = wxRect(x, y, box_w, std::max(box_h, target));  // Tap to expand.
    return;
  }
  wxCoord width = 0, line_height = 0;
  for (const auto &line : lines) {
    wxCoord w = 0, h = 0; dc.GetTextExtent(line, &w, &h);
    width = std::max(width, w); line_height = std::max(line_height, h);
  }
  const int box_w = width + 2 * pad + target,
            box_h = static_cast<int>(lines.size()) * (line_height + gap) - gap + 2 * pad;
  const int x = std::max(pad, (vp.pix_width - box_w) / 2), y = canvas.FromDIP(12);
  dc.SetPen(border);
  dc.SetBrush(wxBrush(ui::Colour(floating.surface)));
  dc.DrawRoundedRectangle(x, y, box_w, box_h, canvas.FromDIP(8));
  for (std::size_t i = 0; i < lines.size(); ++i) {
    dc.SetTextForeground(ui::Colour(i == 0 ? floating.primary : floating.secondary));
    dc.DrawText(lines[i], x + pad, y + pad + static_cast<int>(i) * (line_height + gap));
  }
  // Close mark, vertically centred in the badge's right-hand target column.
  const int cx = x + box_w - target / 2, cy = y + box_h / 2;
  dc.SetPen(wxPen(ui::Colour(floating.secondary), canvas.FromDIP(1)));
  dc.DrawLine(cx - glyph, cy - glyph, cx + glyph + 1, cy + glyph + 1);
  dc.DrawLine(cx - glyph, cy + glyph, cx + glyph + 1, cy - glyph - 1);
  badge_hit = wxRect(x + box_w - target, y, target, std::max(box_h, target));
}
} // namespace

void SetWeatherSource(std::function<weather::ForecastSnapshot()> next) {
  if (wxIsMainThread()) source = std::move(next);
}
void SetWeatherDisplayTime(std::optional<weather::WallTime> valid_time) {
  if (wxIsMainThread()) display_time = valid_time;
}
std::optional<weather::WallTime> WeatherDisplayTime() {
  return wxIsMainThread() ? display_time : std::nullopt;
}
void SetWeatherOverlayVisible(bool show) {
  if (!wxIsMainThread()) return;
  if (show && !visible) badge_collapsed = false;  // Turning the layer on shows it fully.
  visible = show;
}
bool WeatherOverlayVisible() { return wxIsMainThread() && visible; }

std::string WeatherLayerReason() {
  if (!wxIsMainThread() || !source) return "Forecast wind provider unavailable";
  const auto snapshot = source();
  const auto now = weather::WallTime::clock::now();
  if (weather::DisplayState(snapshot, now) == weather::ForecastDisplay::Unavailable)
    return weather::UnavailableReason(snapshot, now);
  return weather::ProvenanceLabel(snapshot, now) + ". Advisory, not measured wind.";
}

void DrawWeatherOverlay(ocpnDC &dc, ViewPort &vp, ChartCanvas &canvas) {
  if (wxIsMainThread() && badge_canvas.get() == &canvas) badge_hit = wxRect();
  if (!wxIsMainThread() || !visible || !source) return;
  // Verified XNav presentation only; Standard/Legacy/Safe stay untouched.
  wxColour land, water;
  if (!ChartBackground(canvas.GetColorScheme(), land, water)) return;
  const auto snapshot = source();
  const auto now = weather::WallTime::clock::now();
  const auto mode = CanvasMode(canvas);
  const auto pen = dc.GetPen(); const auto brush = dc.GetBrush();
  const auto old_font = dc.GetFont(); const auto old_ink = dc.GetTextForeground();
  const auto restore = [&] {
    dc.SetFont(old_font); dc.SetTextForeground(old_ink); dc.SetBrush(brush); dc.SetPen(pen);
  };
  const auto display = weather::DisplayState(snapshot, now);
  if (display == weather::ForecastDisplay::Unavailable) {
    DrawBadge(dc, canvas, vp, mode,
              {"FORECAST WIND UNAVAILABLE", W(weather::UnavailableReason(snapshot, now))}, true,
              "FORECAST UNAVAILABLE");
    restore();
    return;
  }
  const bool stale = display == weather::ForecastDisplay::Stale;
  const auto times = weather::ValidTimes(snapshot);
  const auto step = weather::ResolveDisplayTime(times, display_time, now);
  if (!step) { restore(); return; }
  const auto winds = weather::WindsAt(snapshot, *step);

  // SCRUM-355: lay arrows on a screen grid, each taking the nearest model
  // sample (NearestWindSample), so a forecast that covers the view always
  // shows. Pitch follows the declutter spacing (SCRUM-317), widened so the
  // whole view fits under the arrow cap.
  const double scale = canvas.FromDIP(100) / 100.0;
  const auto detail = application::ChartDetailForScale(vp.chart_scale);
  const double pitch = weather::ArrowGridPitch(vp.pix_width, vp.pix_height,
                                              weather::ArrowSpacingPx(detail) * scale);
  struct Cell { wxPoint p; weather::Coordinate at; std::size_t wind; };
  std::vector<Cell> cells;
  if (pitch > 0) {
    for (double y = pitch / 2; y < vp.pix_height; y += pitch)
      for (double x = pitch / 2; x < vp.pix_width; x += pitch) {
        const wxPoint p(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)));
        double lat = 0, lon = 0;
        vp.GetLLFromPix(p, &lat, &lon);
        const weather::Coordinate at{lat, lon};
        if (const auto nearest = weather::NearestWindSample(winds, at))
          cells.push_back({p, at, *nearest});
      }
  }
  // --cyan (prototype .wind-arrow) with a float-text halo for contrast on
  // light Day water. Stale: hollow muted arrows, never the live ink.
  const auto ink = ui::Colour(ChartCanvasInk(mode, ui::NavigationContextInk(mode)));
  const auto halo = ui::Colour(ChartCanvasInk(mode, ui::FloatingTheme(mode).primary));
  const auto muted = ui::Colour(ChartCanvasInk(mode, ui::FloatingTheme(mode).secondary));
  const auto hollow = ui::Colour(ChartCanvasInk(mode, ui::FloatingTheme(mode).surface));
  Layer outer{stale ? muted : halo, {}}, inner{stale ? hollow : ink, {}};
  std::vector<Label> labels;
  const bool numbers = application::ShowSecondaryLabels(detail);
  for (const auto &cell : cells) {
    const auto &w = winds[cell.wind];
    const auto p = cell.p;
    const double kn = weather::KnotsFromMps(w.speed_mps);
    // Angle at the cell itself, so a rotated or course-up chart stays true.
    const auto angle = kn >= weather::kCalmKn
        ? DownwindAngle(canvas, vp, cell.at, p, w.direction_from_true_deg)
        : std::nullopt;
    std::vector<float> o, i;
    if (angle) {
      o = weather::ArrowTriangles(p.x, p.y, *angle, kn, scale, (stale ? 1.2 : 1.5) * scale);
      i = weather::ArrowTriangles(p.x, p.y, *angle, kn, scale, stale ? -.6 * scale : 0);
    } else {
      // Calm or unprojectable direction: a ring, never a guessed direction.
      o = Disc(p.x, p.y, 5.5 * scale);
      i = Disc(p.x, p.y, 3.5 * scale);
    }
    outer.triangles.insert(outer.triangles.end(), o.begin(), o.end());
    inner.triangles.insert(inner.triangles.end(), i.begin(), i.end());
    if (numbers)
      labels.push_back({wxString::Format("%.0f", kn), p.x + static_cast<int>(10 * scale),
                        p.y + static_cast<int>(6 * scale)});
  }
  FillLayers(dc, canvas, {outer, inner});
  if (!labels.empty()) {
    dc.SetFont(ui::UiFontWeight(canvas, 9, 600));
    dc.SetTextForeground(stale ? muted : halo);
    for (const auto &l : labels) dc.DrawText(l.text, l.x, l.y);
  }
  const auto step_text = W(weather::StepLabel(*step, now));
  DrawBadge(dc, canvas, vp, mode,
            {W(weather::ProvenanceLabel(snapshot, now)),
             "Wind valid " + LocalTime(*step) + " (" + step_text + ")" +
                 wxString::FromUTF8(" \xC2\xB7 kn \xC2\xB7 advisory, not measured")},
            stale, stale ? "STALE FORECAST" : "FORECAST");
  restore();
}
} // namespace opennav::integration
