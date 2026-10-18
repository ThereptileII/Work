#include "integration/ActiveRoutePassageOverlay.h"
#include "integration/RoutePassageSettings.h"
#include "integration/OpenCPNIntegration.h"
#include "integration/ChartPresentation.h"
#include "integration/ChartCanvasInk.h"
#include "ui/Controls.h"
#include "ui/Theme.h"
#include "chcanv.h"
#include "ocpndc.h"
#include "viewport.h"
#include "model/georef.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <vector>
#include <wx/thread.h>

namespace opennav::integration {
namespace {
constexpr std::size_t kMaxRoutePoints = 2048, kMaxLegs = 128;
constexpr unsigned kMaxLegSamples = 64, kRingSamples = 256;
constexpr unsigned kMaxCorridorStrokes = 8192;
struct Geo { double lat = 0, lon = 0; };
struct Pixel { double x = 0, y = 0; };
struct Passage {
  std::string id;
  std::vector<Geo> path; // current leg origin, active point, remaining points
  bool limited = false;
};
bool Valid(Geo p) {
  // Mercator leg sampling has no useful polar representation. Withhold that
  // geometry instead of connecting through the singularity.
  return std::isfinite(p.lat) && std::isfinite(p.lon) &&
      std::abs(p.lat) < 89.5 && std::abs(p.lon) <= 180;
}
std::optional<Passage> CopyPassage() {
  if (!g_pRouteMan || !pRouteList) return {};
  auto* route = g_pRouteMan->GetpActiveRoute();
  if (!route || !g_pRouteMan->IsRouteValid(route)) return {};
  if (!route->IsVisible() || !route->IsActive() || route->m_bIsBeingEdited ||
      route->m_bIsBeingCreated || route->m_bIsInLayer ||
      !route->pRoutePointList || route->GetnPoints() < 1 ||
      route->pRoutePointList->GetCount() > kMaxRoutePoints) return {};
  auto* active = g_pRouteMan->GetpActivePoint();
  auto* origin = g_pRouteMan->GetpActiveRouteSegmentBeginPoint();
  if (!active || active != route->m_pRouteActivePoint || !origin ||
      origin->m_bRPIsBeingEdited || origin->m_bIsInLayer ||
      origin->GetIconName() == "mob" || !Valid({origin->m_lat, origin->m_lon})) return {};
  Passage copy;
  copy.id = route->GetGUID().ToStdString(wxConvUTF8);
  if (copy.id.empty()) return {};
  copy.path.push_back({origin->m_lat, origin->m_lon});
  std::set<std::string> identities;
  unsigned active_matches = 0;
  bool remaining = false;
  for (auto* node = route->pRoutePointList->GetFirst(); node; node = node->GetNext()) {
    auto* point = node->GetData();
    if (!point || point->m_bRPIsBeingEdited || point->m_bIsInLayer ||
        point->GetIconName() == "mob" || !Valid({point->m_lat, point->m_lon})) return {};
    const auto id = point->m_GUID.ToStdString(wxConvUTF8);
    if (id.empty() || !identities.insert(id).second) return {};
    if (point == active) { remaining = true; ++active_matches; }
    if (!remaining) continue;
    if (copy.path.size() <= kMaxLegs) copy.path.push_back({point->m_lat, point->m_lon});
    else copy.limited = true;
  }
  if (active_matches != 1 || copy.path.size() < 2) return {};
  return copy; // no upstream pointer escapes, including the virtual origin
}
bool Project(ChartCanvas& canvas, ViewPort& vp, Geo p, Pixel& out) {
  if (!Valid(p)) return false;
  wxPoint2DDouble projected;
  canvas.GetDoubleCanvasPointPixVP(vp, p.lat, p.lon, &projected);
  out = {projected.m_x, projected.m_y};
  // Clip in doubles before rounding; pinned integer projection can return true
  // with INVALID_COORD when an otherwise valid point is far outside the view.
  return std::isfinite(out.x) && std::isfinite(out.y) &&
      std::abs(out.x) < 1e12 && std::abs(out.y) < 1e12;
}
bool Clip(Pixel& a, Pixel& b, const ViewPort& vp) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  double first = 0, last = 1;
  const auto edge = [&](double p, double q) {
    if (p == 0) return q >= 0;
    const double t = q / p;
    if (p < 0) { if (t > last) return false; first = (std::max)(first, t); }
    else { if (t < first) return false; last = (std::min)(last, t); }
    return true;
  };
  if (!edge(-dx, a.x) || !edge(dx, vp.pix_width - a.x) ||
      !edge(-dy, a.y) || !edge(dy, vp.pix_height - a.y)) return false;
  b = {a.x + last * dx, a.y + last * dy};
  a = {a.x + first * dx, a.y + first * dy};
  return true;
}
bool Segment(ocpnDC& dc, ChartCanvas& canvas, ViewPort& vp, Geo a, Geo b,
             double* dash_phase = nullptr, unsigned* strokes_left = nullptr) {
  // Break at the projection's opposite meridian. A wrapped endpoint must not
  // create a line across the whole chart; the adjacent pieces remain separate.
  const double phase_a = std::remainder(a.lon - vp.clon, 360.0);
  const double phase_b = std::remainder(b.lon - vp.clon, 360.0);
  if (!Valid(a) || !Valid(b) || std::abs(phase_a - phase_b) >= 180) return false;
  Pixel pa, pb;
  if (!Project(canvas, vp, a, pa) || !Project(canvas, vp, b, pb)) return false;
  const auto original = pa;
  const double full_length = std::hypot(pb.x - pa.x, pb.y - pa.y);
  const double dash = canvas.FromDIP(6) * ChartDisplayScale(), period = dash + canvas.FromDIP(4) * ChartDisplayScale();
  const double start_phase = dash_phase ? *dash_phase : 0;
  if (dash_phase) *dash_phase = std::fmod(start_phase + full_length, period);
  if (!Clip(pa, pb, vp)) return false;
  const auto stroke = [&](Pixel p, Pixel q) {
    dc.DrawLine(wxRound(p.x), wxRound(p.y), wxRound(q.x), wxRound(q.y));
  };
  if (!dash_phase) stroke(pa, pb);
  else {
    // Explicit common dash geometry avoids per-segment/native GL stipple
    // differences. Retain phase across samples and bound physical draw calls.
    const double length = std::hypot(pb.x - pa.x, pb.y - pa.y);
    double phase = std::fmod(start_phase + std::hypot(pa.x - original.x, pa.y - original.y), period);
    for (double at = 0; at + 1e-6 < length && strokes_left && *strokes_left;) {
      const double run = (std::min)(length - at, (phase < dash ? dash : period) - phase);
      if (phase < dash) {
        const double first = at / length, last = (at + run) / length;
        stroke({pa.x + (pb.x - pa.x) * first, pa.y + (pb.y - pa.y) * first},
               {pa.x + (pb.x - pa.x) * last, pa.y + (pb.y - pa.y) * last});
        --*strokes_left;
      }
      at += run;
      phase = phase + run >= period - 1e-6 ? 0 : phase + run;
    }
  }
  dc.CalcBoundingBox(wxRound(pa.x), wxRound(pa.y));
  dc.CalcBoundingBox(wxRound(pb.x), wxRound(pb.y));
  return true;
}
std::optional<Geo> Offset(Geo p, double bearing, double metres) {
  Geo result;
  ll_gc_ll(p.lat, p.lon, bearing, metres / 1852.0, &result.lat, &result.lon);
  if (std::isfinite(result.lon)) result.lon = std::remainder(result.lon, 360.0);
  return Valid(result) ? std::optional<Geo>{result} : std::nullopt;
}
void Leg(ocpnDC& dc, ChartCanvas& canvas, ViewPort& vp, Geo from, Geo to,
         double half_width, unsigned& strokes_left) {
  const double longitude_span = std::remainder(to.lon - from.lon, 360.0);
  if (std::abs(longitude_span) >= 180) return;
  double course = 0, distance = 0, east = 0, north = 0;
  // These are display-only samples of the same rhumb-line leg. They never
  // change progress, XTE, arrival or an autopilot command.
  DistanceBearingMercator(to.lat, to.lon, from.lat, from.lon, &course, &distance);
  toSM(to.lat, to.lon, from.lat, from.lon, &east, &north);
  if (!std::isfinite(course) || !std::isfinite(distance) || distance <= 0 ||
      !std::isfinite(east) || !std::isfinite(north)) return;
  const auto parts = static_cast<unsigned>((std::min)(double(kMaxLegSamples),
      (std::max)(1.0, std::ceil((std::max)(std::abs(to.lat - from.lat), std::abs(longitude_span)) * 4))));
  std::optional<Geo> previous_left, previous_right;
  double left_phase = 0, right_phase = 0;
  for (unsigned i = 0; i <= parts && strokes_left; ++i) {
    Geo center;
    const double fraction = double(i) / parts;
    fromSM(east * fraction, north * fraction, from.lat, from.lon, &center.lat, &center.lon);
    if (std::isfinite(center.lon)) center.lon = std::remainder(center.lon, 360.0);
    if (!Valid(center)) { previous_left.reset(); previous_right.reset(); continue; }
    const auto left = Offset(center, course - 90, half_width);
    const auto right = Offset(center, course + 90, half_width);
    if (left && previous_left) Segment(dc, canvas, vp, *previous_left, *left, &left_phase, &strokes_left);
    if (right && previous_right) Segment(dc, canvas, vp, *previous_right, *right, &right_phase, &strokes_left);
    previous_left = left; previous_right = right;
  }
  // Separate leg boundaries intentionally have no invented miter/corner area.
}
void ArrivalRing(ocpnDC& dc, ChartCanvas& canvas, ViewPort& vp, Geo at, double radius) {
  std::optional<Geo> previous;
  for (unsigned i = 0; i <= kRingSamples; ++i) {
    const auto next = Offset(at, 360.0 * i / kRingSamples, radius);
    if (previous && next) Segment(dc, canvas, vp, *previous, *next);
    previous = next;
  }
}
void WidthLabel(ocpnDC& dc, ChartCanvas& canvas, ViewPort& vp,
                const Passage& passage, double width, bool drawing_limited) {
  Pixel a, b;
  if (std::abs(std::remainder(passage.path[0].lon - vp.clon, 360.0) -
               std::remainder(passage.path[1].lon - vp.clon, 360.0)) >= 180 ||
      !Project(canvas, vp, passage.path[0], a) || !Project(canvas, vp, passage.path[1], b) ||
      !Clip(a, b, vp)) return;
  const auto text = wxString::Format("Passage full width %.6g m", width) +
      (passage.limited ? wxString::Format(" / next %u legs", unsigned(kMaxLegs)) : wxString{}) +
      (drawing_limited ? " / display limited" : "");
  dc.SetFont(ScaleChartFont(ui::UiFont(canvas, 10)));
  wxCoord w = 0, h = 0; dc.GetTextExtent(text, &w, &h);
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  if (length < ChartDisplayPixels(canvas, 32)) return;
  const int gap = ChartDisplayPixels(canvas, 12);
  const int x = wxRound((a.x + b.x) / 2 - (b.y - a.y) / length * gap);
  const int y = wxRound((a.y + b.y) / 2 + (b.x - a.x) / length * gap);
  if (x < 0 || y < 0 || x + w >= vp.pix_width || y + h >= vp.pix_height) return;
  dc.DrawText(text, x, y);
}
} // namespace

void DrawActiveRoutePassage(ocpnDC& dc, ChartCanvas& canvas, ViewPort& vp) {
  if (!wxIsMainThread() || !opennav::IsXNav() || !PassageVisible() || !vp.IsValid() ||
      !canvas.m_bShowNavobjects ||
      !std::isfinite(vp.clon) || vp.pix_width <= 0 || vp.pix_height <= 0 ||
      vp.pix_width > 65536 || vp.pix_height > 65536) return;
  const auto passage = CopyPassage();
  if (!passage) return;
  const auto settings = ReadRoutePassage(passage->id);
  if (!application::ValidateRoutePassage(settings).empty()) return;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT ? ui::LightMode::Night
      : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK ? ui::LightMode::Dusk : ui::LightMode::Day;
  // A route-planning width, not surveyed water, measured clearance or a
  // steering tolerance. No fill conceals chart depths, obstructions or labels.
  const auto raw = ui::ActiveRouteInk(mode);
  const auto ink = ui::Colour(XNavChartPresentationActive() ? ChartCanvasInk(mode, raw) : raw);
  const auto pen = dc.GetPen(); const auto brush = dc.GetBrush();
  const auto font = dc.GetFont(); const auto text_color = dc.GetTextForeground();
  dc.SetPen(wxPen(ink, (std::max)(1, int(std::lround(canvas.FromDIP(1) * ChartDisplayScale()))), wxPENSTYLE_SOLID));
  dc.SetBrush(*wxTRANSPARENT_BRUSH);
  unsigned strokes_left = kMaxCorridorStrokes;
  for (std::size_t i = 1; i < passage->path.size() && strokes_left; ++i)
    Leg(dc, canvas, vp, passage->path[i - 1], passage->path[i],
        settings.corridor_width_m / 2, strokes_left);
  // Only this explicit route policy uses radial arrival. Stock OpenCPN's
  // crossing/moving-away rule gets no misleading circle. Highlight the actual
  // next waypoint only; its original marker is painted afterwards by OpenCPN.
  if (settings.arrival_radius_m) {
    dc.SetPen(wxPen(ink, (std::max)(1, int(std::lround(canvas.FromDIP(1) * ChartDisplayScale()))), wxPENSTYLE_SOLID));
    ArrivalRing(dc, canvas, vp, passage->path[1], *settings.arrival_radius_m);
  }
  dc.SetTextForeground(ink);
  WidthLabel(dc, canvas, vp, *passage, settings.corridor_width_m, !strokes_left);
  dc.SetTextForeground(text_color); dc.SetFont(font); dc.SetBrush(brush); dc.SetPen(pen);
}
} // namespace opennav::integration
