#include "integration/ChartRouteWaypoint.h"
#include "integration/ChartPresentation.h"
#include "integration/ChartCanvasInk.h"
#include "application/ChartDeclutter.h"
#include "ui/Controls.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "chcanv.h"
#include "ocpndc.h"
#include <wx/dcmemory.h>
#include <wx/graphics.h>
#include <wx/thread.h>
#include <cmath>
#include <memory>
#include <vector>
#ifdef ocpnUSE_GL
#include "shaders.h"
#endif
extern Routeman *g_pRouteMan;
extern float g_MarkScaleFactorExp;
extern wxString g_default_wp_icon, g_default_routepoint_icon;
namespace opennav::integration {
static int RoutePointOrdinal(ChartCanvas &canvas, RoutePoint &point,
                             bool pinned_icon, bool label) {
  wxColour ink;
  if (!ChartActiveRouteInk(canvas, ink)) return 0;
  if (!wxIsMainThread() || !g_pRouteMan) return 0;
  // Active icon blinking does not hide the upstream name. Only the actual
  // current pointer may retain its label; unrelated blinking points stay stock.
  const bool active_label = label && point.m_bIsActive &&
      g_pRouteMan->GetpActivePoint() == &point;
  if (!pinned_icon || !pRouteList ||
      point.GetIconName() != "diamond" || !point.m_bIsInRoute ||
      point.IsShared() || point.m_bIsInLayer ||
      (point.m_bIsActive && !active_label) || point.m_bPtIsSelected ||
      (point.m_bBlink && !active_label) || point.m_bRPIsBeingEdited ||
      point.IsDragHandleEnabled() || &point == pAnchorWatchPoint1 ||
      &point == pAnchorWatchPoint2 ||
      (point.m_bShowWaypointRangeRings && point.m_iWaypointRangeRingsNumber))
    return 0;
  int ordinal = 0, occurrences = 0, visited = 0;
  // GetIndexOf returns the first match: insufficient for repeated/shared points.
  // Count pointer identity across all actual routes, including hidden routes.
  for (auto *r = pRouteList->GetFirst(); r; r = r->GetNext()) {
    if (++visited > 4096) return 0;
    auto *route = r->GetData();
    int index = 0;
    for (auto *p = route->pRoutePointList->GetFirst(); p; p = p->GetNext()) {
      if (++visited > 4096) return 0; // Bound paint work for large route libraries.
      ++index;
      if (p->GetData() != &point) continue;
      if (++occurrences > 1) return 0;
      if (DefaultChartRouteStyle(*route) && !route->m_bIsBeingCreated)
        ordinal = index;
    }
  }
  // Two digits fit the prototype circle. Larger routes retain upstream icons;
  // never truncate, wrap, or fabricate a route ordinal.
  return occurrences == 1 && ordinal <= 99 ? ordinal : 0;
}
int ChartRouteWaypointOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon) {
  return RoutePointOrdinal(canvas, point, pinned_icon, false);
}
int ChartRouteLabelOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon) {
  return RoutePointOrdinal(canvas, point, pinned_icon, true);
}
int ChartRouteWaypointExtent(ChartCanvas &canvas) {
  const double scale = canvas.FromDIP(100) / 100.0 * g_MarkScaleFactorExp;
  return std::isfinite(scale) && scale >= .25 && scale <= 8
      ? static_cast<int>(std::ceil(11 * scale)) : 0;
}

namespace {
// Generic shape icons carry no user meaning beyond "a point here". Meaningful
// symbols (fuel, anchorage, hazard, custom user icons, MOB) keep their artwork.
bool GenericWaypointIcon(const wxString &icon) {
  static const char *const generic[] = {"diamond", "circle", "triangle", "square",
                                        "xmblue", "xmgreen", "xmred", "Symbol-Diamond-Red"};
  for (const auto *name : generic)
    if (icon == name) return true;
  return (!g_default_wp_icon.empty() && icon == g_default_wp_icon) ||
         (!g_default_routepoint_icon.empty() && icon == g_default_routepoint_icon);
}
double MarkerScale(ChartCanvas &canvas) {
  const double scale = canvas.FromDIP(100) / 100.0 * g_MarkScaleFactorExp;
  return std::isfinite(scale) && scale >= .25 && scale <= 8 ? scale : 0;
}
ui::LightMode CanvasMode(ChartCanvas &canvas) {
  return canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT ? ui::LightMode::Night
       : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK  ? ui::LightMode::Dusk
                                                              : ui::LightMode::Day;
}
wxColour MarkerFill(ChartCanvas &canvas) {
  const auto mode = CanvasMode(canvas);
  auto fill = ui::Colour(ui::FloatingTheme(mode).surface);
  if (mode == ui::LightMode::Night)  // Prototype Night chart brightness(.78).
    fill = wxColour(std::lround(fill.Red()*.78), std::lround(fill.Green()*.78),
                    std::lround(fill.Blue()*.78));
  return fill;
}
wxColour Mix(const wxColour &a, const wxColour &b, double t) {
  return wxColour(std::lround(a.Red() + (b.Red() - a.Red()) * t),
                  std::lround(a.Green() + (b.Green() - a.Green()) * t),
                  std::lround(a.Blue() + (b.Blue() - a.Blue()) * t));
}
struct MarkerPaint {
  WaypointMarkerKind kind;
  int ordinal = 0;
  bool selected = false;
  // SCRUM-317 level of detail: compact markers without ordinals when zoomed
  // out; names only at full detail. Active and selected points stay full.
  bool compact = false, names = true;
  wxColour ink, fill, ring, text, halo;
  double scale = 0;
};
bool ResolvePaint(ChartCanvas &canvas, int marker, MarkerPaint &paint) {
  const auto decoded = DecodeWaypointMarker(marker);
  if (!decoded.valid) return false;
  wxColour route_ink;
  if (!ChartActiveRouteInk(canvas, route_ink)) return false;
  paint.kind = decoded.kind;
  paint.ordinal = decoded.ordinal;
  paint.selected = decoded.selected;
  paint.scale = MarkerScale(canvas);
  if (!paint.scale) return false;
  const auto mode = CanvasMode(canvas);
  // Same route-state inks as the route line (ChartRouteInk).
  paint.ink = decoded.role == WaypointMarkerRole::Inactive
      ? ui::Colour(ChartCanvasInk(mode, ui::InactiveRouteInk(mode)))
      : decoded.role == WaypointMarkerRole::SelectedRoute
      ? ui::Colour(ChartCanvasInk(mode, ui::Theme(mode).ais))
      : route_ink;
  paint.fill = MarkerFill(canvas);
  paint.ring = paint.text = paint.ink;
  const auto *vp = canvas.GetpVP();
  const auto detail = application::ChartDetailForScale(vp ? vp->chart_scale : 0);
  paint.names = application::ShowSecondaryLabels(detail);
  paint.compact = !application::ShowMarkerDetail(detail) && !paint.selected &&
                  paint.kind != WaypointMarkerKind::Active;
  if (paint.compact) paint.ordinal = 0;
  if (paint.kind == WaypointMarkerKind::Active) {
    // The next point is solid: route ink disc, floating-colour ordinal.
    paint.text = paint.fill;
  } else if (paint.kind == WaypointMarkerKind::Visited) {
    // Already passed points recede without disappearing.
    paint.ring = paint.text = Mix(paint.ink, paint.fill, .55);
  }
  paint.halo = Mix(paint.ink, paint.fill, .5);
  return true;
}
// Outer radii (logical px at 100%) of each concentric layer, outside first.
struct Layer { double radius; wxColour colour; };
std::vector<Layer> Layers(const MarkerPaint &p) {
  const bool standalone = p.kind == WaypointMarkerKind::Standalone;
  // Immutable .map-waypoint: r=10, stroke 2 (r=9 for saved waypoints).
  // Overview: same ring weight around a compact r=5 disc.
  const double outer = p.compact ? 6 : standalone ? 10 : 11,
               inner = p.compact ? 4 : standalone ? 8 : 9;
  std::vector<Layer> layers;
  if (p.selected) {
    layers.push_back({16, p.halo});
    layers.push_back({14, p.fill});
  } else if (p.kind == WaypointMarkerKind::Active) {
    layers.push_back({14.5, p.halo});
    layers.push_back({13, p.fill});
  }
  layers.push_back({outer, p.ring});
  layers.push_back({inner, p.kind == WaypointMarkerKind::Active ? p.ink : p.fill});
  return layers;
}
// Flag glyph for saved waypoints (prototype uses U+2691; drawn as vectors so
// no font fallback can substitute a different symbol).
std::vector<wxPoint2DDouble> FlagTriangles(double x, double y, double s) {
  const double pole_l = x - 3 * s, pole_r = x - 1.8 * s, top = y - 4.5 * s,
               bottom = y + 4.5 * s;
  return {{pole_l, top}, {pole_r, top}, {pole_l, bottom},
          {pole_r, top}, {pole_r, bottom}, {pole_l, bottom},
          {pole_r, top}, {x + 4 * s, y - 2 * s}, {pole_r, y + .5 * s}};
}
}  // namespace

int ChartWaypointMarker(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon) {
  wxColour ink;
  if (!ChartActiveRouteInk(canvas, ink)) return 0;
  if (!wxIsMainThread() || !g_pRouteMan || !MarkerScale(canvas)) return 0;
  // Anchor watch, MOB, layer content and meaningful custom icons keep their
  // own artwork; a live drag handle keeps OpenCPN's editing presentation.
  if (point.m_bIsInLayer || point.GetIconName() == "mob" ||
      &point == pAnchorWatchPoint1 || &point == pAnchorWatchPoint2 ||
      point.IsDragHandleEnabled() ||
      (!pinned_icon && !GenericWaypointIcon(point.GetIconName())))
    return 0;
  const bool selected = point.m_bPtIsSelected || point.m_bRPIsBeingEdited;
  if (!point.m_bIsInRoute)
    return EncodeWaypointMarker(WaypointMarkerKind::Standalone,
                                WaypointMarkerRole::Route, 0, selected);
  if (!pRouteList) return 0;
  Route *owner = nullptr;
  int ordinal = 0, occurrences = 0, visited = 0;
  for (auto *r = pRouteList->GetFirst(); r; r = r->GetNext()) {
    if (++visited > 4096) return 0;  // Bound paint work for large libraries.
    auto *route = r->GetData();
    int index = 0;
    for (auto *p = route->pRoutePointList->GetFirst(); p; p = p->GetNext()) {
      if (++visited > 4096) return 0;
      ++index;
      if (p->GetData() != &point) continue;
      // A custom/emergency-styled or being-created route owns its own paint.
      if (!DefaultChartRouteStyle(*route) || route->m_bIsBeingCreated) return 0;
      if (++occurrences == 1) { owner = route; ordinal = index; }
    }
  }
  if (!owner) return 0;
  // Never truncate, wrap or guess an ordinal: shared/repeated points and
  // routes longer than 99 points use the unnumbered marker.
  if (occurrences > 1 || point.IsShared() || ordinal > 99) ordinal = 0;
  auto role = WaypointMarkerRole::Route;
  if (occurrences == 1) {
    if (owner->m_bRtIsSelected) role = WaypointMarkerRole::SelectedRoute;
    else if (!owner->m_bRtIsActive) role = WaypointMarkerRole::Inactive;
  }
  auto kind = WaypointMarkerKind::RoutePoint;
  auto *active_route = g_pRouteMan->GetpActiveRoute();
  auto *active_point = g_pRouteMan->GetpActivePoint();
  if (point.m_bIsActive && active_point == &point) {
    kind = WaypointMarkerKind::Active;
  } else if (active_route == owner && active_point && occurrences == 1) {
    // Points before the active point on the active route have been passed.
    int index = 0, active_index = 0, point_index = 0;
    for (auto *p = owner->pRoutePointList->GetFirst(); p; p = p->GetNext()) {
      ++index;
      if (p->GetData() == active_point && !active_index) active_index = index;
      if (p->GetData() == &point && !point_index) point_index = index;
    }
    if (active_index && point_index && point_index < active_index)
      kind = WaypointMarkerKind::Visited;
  }
  return EncodeWaypointMarker(kind, role, ordinal, selected);
}

bool ChartWaypointMarkerOwnsName(int marker) {
  const auto decoded = DecodeWaypointMarker(marker);
  return decoded.valid && decoded.kind == WaypointMarkerKind::Standalone;
}

static bool StandaloneLabel(ChartCanvas &canvas, RoutePoint &point, double scale,
                            wxFont &font, wxString &text) {
  if (!point.m_bShowName || point.GetName().empty()) return false;
  font = ui::UiFontWeight(canvas, 10, 650);
  font.SetFractionalPointSize(font.GetFractionalPointSize() * g_MarkScaleFactorExp);
  text = point.GetName();
  (void)scale;
  return true;
}

wxRect ChartWaypointMarkerBounds(ChartCanvas &canvas, RoutePoint &point, int marker) {
  MarkerPaint paint;
  if (!ResolvePaint(canvas, marker, paint)) return {};
  const double radius = Layers(paint).front().radius * paint.scale;
  const int extent = static_cast<int>(std::ceil(radius));
  wxRect bounds(-extent, -extent, 2 * extent, 2 * extent);
  wxFont font;
  wxString text;
  if (paint.kind == WaypointMarkerKind::Standalone && paint.names &&
      StandaloneLabel(canvas, point, paint.scale, font, text)) {
    wxCoord w = 0, h = 0;
    // One reusable measuring DC: this runs per mark on every chart paint.
    static wxBitmap measure_bitmap(1, 1);
    static wxMemoryDC measure(measure_bitmap);
    measure.SetFont(font);
    measure.GetTextExtent(text, &w, &h);
    const int centre = static_cast<int>(std::lround(24 * paint.scale));
    bounds.Union(wxRect(-w / 2 - 1, centre - h / 2 - 1, w + 2, h + 2));
  }
  return bounds;
}

bool DrawChartWaypointMarker(ocpnDC &dc, ChartCanvas &canvas, RoutePoint *point,
                             int x, int y, int marker) {
  MarkerPaint paint;
  if (!ResolvePaint(canvas, marker, paint)) return false;
  const double s = paint.scale;
  const auto layers = Layers(paint);
  const bool standalone = paint.kind == WaypointMarkerKind::Standalone;
  const auto label = paint.ordinal ? wxString::Format("%02d", paint.ordinal) : wxString();
  // Prototype ordinal: 8px/650, centered at y+.5. Honors the mark-size
  // preference independently of display DPI.
  auto ordinal_font = ui::UiFontWeight(canvas, 8, 650);
  ordinal_font.SetFractionalPointSize(ordinal_font.GetFractionalPointSize()*g_MarkScaleFactorExp);
  wxFont name_font;
  wxString name;
  const bool named = standalone && paint.names && point &&
                     StandaloneLabel(canvas, *point, s, name_font, name);
  // Compact overview markers: ring and centre dot only.
  const bool glyph = standalone && !paint.compact;
  if (auto *native = dc.GetDC()) {
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::CreateFromUnknownDC(*native));
    if (!gc) return false;
    gc->SetPen(*wxTRANSPARENT_PEN);
    for (const auto &layer : layers) {
      gc->SetBrush(wxBrush(layer.colour));
      gc->DrawEllipse(x - layer.radius * s, y - layer.radius * s,
                      2 * layer.radius * s, 2 * layer.radius * s);
    }
    if (glyph) {
      auto path = gc->CreatePath();
      const auto t = FlagTriangles(x, y, s);
      for (std::size_t i = 0; i + 2 < t.size(); i += 3) {
        path.MoveToPoint(t[i]); path.AddLineToPoint(t[i+1]);
        path.AddLineToPoint(t[i+2]); path.CloseSubpath();
      }
      gc->SetBrush(wxBrush(paint.ink));
      gc->FillPath(path);
    } else if (!label.empty() && !standalone) {
      gc->SetFont(ordinal_font, paint.text);
      double w = 0, h = 0; gc->GetTextExtent(label, &w, &h);
      gc->DrawText(label, x - w / 2, y + .5 * s - h / 2);
    } else {
      // Unnumbered: centre dot, no guessed ordinal.
      const double dot = paint.compact ? 1.5 : 2.5;
      gc->SetBrush(wxBrush(paint.kind == WaypointMarkerKind::Standalone ? paint.ink : paint.text));
      gc->DrawEllipse(x - dot * s, y - dot * s, 2 * dot * s, 2 * dot * s);
    }
    if (named) {
      gc->SetFont(name_font, paint.ink);
      double w = 0, h = 0; gc->GetTextExtent(name, &w, &h);
      gc->DrawText(name, x - w / 2, y + 24 * s - h / 2);
    }
  } else {
#ifdef ocpnUSE_GL
    if (dc.m_canvasIndex < 0 || dc.m_canvasIndex >= 2) return false;
    auto *shader = pcolor_tri_shader_program[dc.m_canvasIndex];
    if (!shader) return false;
    GLint program=0, texture=0, src_rgb=0, dst_rgb=0, src_alpha=0, dst_alpha=0;
    glGetIntegerv(GL_CURRENT_PROGRAM,&program);
    glGetIntegerv(GL_TEXTURE_BINDING_2D,&texture);
    glGetIntegerv(GL_BLEND_SRC_RGB,&src_rgb); glGetIntegerv(GL_BLEND_DST_RGB,&dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA,&src_alpha); glGetIntegerv(GL_BLEND_DST_ALPHA,&dst_alpha);
    const auto blended=glIsEnabled(GL_BLEND);
    const auto textured=glIsEnabled(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    shader->Bind();
    shader->SetUniformMatrix4fv("MVMatrix",
        reinterpret_cast<GLfloat *>(canvas.GetpVP()->vp_matrix_transform));
    const auto submit=[&](const wxColour &color, std::vector<float> &vertices) {
      float rgba[]={color.Red()/256.f,color.Green()/256.f,color.Blue()/256.f,1.f};
      shader->SetUniform4fv("color",rgba);
      shader->SetAttributePointerf("position",vertices.data());
      glDrawArrays(GL_TRIANGLES,0,vertices.size()/2);
    };
    // Opaque triangle fans share the SVG's outer/inner radii. No line-width
    // limits, bitmap icon cache or four-point strip ambiguity.
    const auto disc=[&](double cx, double cy, double radius, const wxColour &color) {
      std::vector<float> vertices;
      constexpr int segments=96;
      for (int i=0;i<segments;++i) {
        const double a=i*2*3.141592653589793/segments;
        const double b=(i+1)*2*3.141592653589793/segments;
        vertices.insert(vertices.end(),{float(cx),float(cy),
            float(cx+radius*std::cos(a)),float(cy+radius*std::sin(a)),
            float(cx+radius*std::cos(b)),float(cy+radius*std::sin(b))});
      }
      submit(color, vertices);
    };
    for (const auto &layer : layers) disc(x, y, layer.radius * s, layer.colour);
    if (glyph) {
      std::vector<float> vertices;
      for (const auto &v : FlagTriangles(x, y, s))
        vertices.insert(vertices.end(), {float(v.m_x), float(v.m_y)});
      submit(paint.ink, vertices);
    } else if (label.empty() || standalone) {
      disc(x, y, (paint.compact ? 1.5 : 2.5) * s,
           paint.kind == WaypointMarkerKind::Standalone ? paint.ink : paint.text);
    }
    shader->UnBind();
    const auto old_font=dc.GetFont(); const auto old_ink=dc.GetTextForeground();
    if (!label.empty() && !standalone) {
      dc.SetFont(ordinal_font); dc.SetTextForeground(paint.text);
      wxCoord w=0,h=0; dc.GetTextExtent(label,&w,&h);
      dc.DrawText(label,std::lround(x-w/2.),std::lround(y+.5*s-h/2.));
    }
    if (named) {
      dc.SetFont(name_font); dc.SetTextForeground(paint.ink);
      wxCoord w=0,h=0; dc.GetTextExtent(name,&w,&h);
      dc.DrawText(name,std::lround(x-w/2.),std::lround(y+24*s-h/2.));
    }
    dc.SetFont(old_font); dc.SetTextForeground(old_ink);
    glUseProgram(program);
    glBindTexture(GL_TEXTURE_2D,texture);
    glBlendFuncSeparate(src_rgb,dst_rgb,src_alpha,dst_alpha);
    if (textured) glEnable(GL_TEXTURE_2D); else glDisable(GL_TEXTURE_2D);
    if (blended) glEnable(GL_BLEND); else glDisable(GL_BLEND);
#else
    return false;
#endif
  }
  const int extent=static_cast<int>(std::ceil(layers.front().radius * s));
  dc.CalcBoundingBox(x-extent,y-extent); dc.CalcBoundingBox(x+extent,y+extent);
  return true;
}

bool DrawChartRouteWaypoint(ocpnDC &dc, ChartCanvas &canvas,
                            int x, int y, int ordinal) {
  if (ordinal < 1 || ordinal > 99) return false;
  return DrawChartWaypointMarker(dc, canvas, nullptr, x, y,
      EncodeWaypointMarker(WaypointMarkerKind::RoutePoint,
                           WaypointMarkerRole::Route, ordinal, false));
}
} // namespace opennav::integration
