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

  // SCRUM-360: arrows sit on a geographic lattice -- whole multiples of a
  // nice degree step chosen from the zoom -- so they stay at chart positions
  // while panning, as the prototype's wind layer does. Each lattice point
  // takes its nearest model sample (SCRUM-355 coverage). The step follows the
  // declutter spacing, widened so the view stays under the arrow cap.
  const double scale = canvas.FromDIP(100) / 100.0;
  const auto detail = application::ChartDetailForScale(vp.chart_scale);
  const double pitch = weather::ArrowGridPitch(vp.pix_width, vp.pix_height,
                                              weather::ArrowSpacingPx(detail) * scale);
  struct Cell { wxPoint p; weather::Coordinate at; std::size_t wind; };
  std::vector<Cell> cells;
  const auto box = vp.GetBBox();
  const double cos_lat = std::cos(vp.clat * 3.141592653589793 / 180.0);
  if (pitch > 0 && box.GetValid() && std::isfinite(vp.view_scale_ppm) &&
      vp.view_scale_ppm > 0 && cos_lat > 0.05) {
    const double metres = pitch / vp.view_scale_ppm;
    const double lat_step = weather::NiceDegreeStep(metres / 111120.0);
    const double lon_step = weather::NiceDegreeStep(metres / (111120.0 * cos_lat));
    if (lat_step > 0 && lon_step > 0 &&
        (box.GetMaxLat() - box.GetMinLat()) / lat_step < 400 &&
        (box.GetMaxLon() - box.GetMinLon()) / lon_step < 400) {
      for (double lat = std::floor(box.GetMinLat() / lat_step) * lat_step;
           lat <= box.GetMaxLat() + lat_step && cells.size() < 4 * weather::kMaxWindArrows;
           lat += lat_step)
        for (double lon = std::floor(box.GetMinLon() / lon_step) * lon_step;
             lon <= box.GetMaxLon() + lon_step; lon += lon_step) {
          const weather::Coordinate at{lat, lon};
          wxPoint p;
          if (!Project(canvas, vp, at, p) || p.x < 0 || p.y < 0 ||
              p.x > vp.pix_width || p.y > vp.pix_height) continue;
          if (const auto nearest = weather::NearestWindSample(winds, at))
            cells.push_back({p, at, *nearest});
        }
    }
  }
  // Prototype wind layer: open chevrons in route ink, translucent, no number
  // per arrow (speed lives in the forecast box). Stale: muted ink.
  const auto route = ui::Colour(ChartCanvasInk(mode, ui::ActiveRouteInk(mode)));
  const auto muted = ui::Colour(ChartCanvasInk(mode, ui::FloatingTheme(mode).secondary));
  const auto base = stale ? muted : route;
  const wxColour ink(base.Red(), base.Green(), base.Blue(), 150);
  const int width = std::max(1, static_cast<int>(std::lround(1.6 * scale)));
  std::unique_ptr<wxGraphicsContext> gc;
  if (auto *native = dc.GetDC()) {
    gc.reset(wxGraphicsContext::CreateFromUnknownDC(*native));
    if (gc) gc->SetPen(wxPen(ink, width));
  } else {
    dc.SetPen(wxPen(ink, width));
  }
  const auto segment = [&](double x1, double y1, double x2, double y2) {
    if (gc) gc->StrokeLine(x1, y1, x2, y2);
    else dc.DrawLine(static_cast<int>(std::lround(x1)), static_cast<int>(std::lround(y1)),
                     static_cast<int>(std::lround(x2)), static_cast<int>(std::lround(y2)), true);
  };
  for (const auto &cell : cells) {
    const auto &w = winds[cell.wind];
    const auto p = cell.p;
    const double kn = weather::KnotsFromMps(w.speed_mps);
    // Angle at the cell itself, so a rotated or course-up chart stays true.
    const auto angle = kn >= weather::kCalmKn
        ? DownwindAngle(canvas, vp, cell.at, p, w.direction_from_true_deg)
        : std::nullopt;
    if (angle) {
      const auto s = weather::ChevronSegments(p.x, p.y, *angle, scale);
      for (std::size_t i = 0; i + 3 < s.size(); i += 4) segment(s[i], s[i + 1], s[i + 2], s[i + 3]);
    } else {
      // Calm or unprojectable direction: a small ring, never a guessed direction.
      const double r = 3.5 * scale;
      for (int k = 0; k < 12; ++k) {
        const double a = k * 3.141592653589793 / 6, b = (k + 1) * 3.141592653589793 / 6;
        segment(p.x + r * std::cos(a), p.y + r * std::sin(a), p.x + r * std::cos(b), p.y + r * std::sin(b));
      }
    }
  }
  gc.reset();
  const auto step_text = W(weather::StepLabel(*step, now));
  DrawBadge(dc, canvas, vp, mode,
            {W(weather::ProvenanceLabel(snapshot, now)),
             "Wind valid " + LocalTime(*step) + " (" + step_text + ")" +
                 wxString::FromUTF8(" \xC2\xB7 kn \xC2\xB7 advisory, not measured")},
            stale, stale ? "STALE FORECAST" : "FORECAST");
  restore();
}
} // namespace opennav::integration
