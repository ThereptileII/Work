#include "integration/ChartRouteWaypoint.h"
#include "integration/ChartPresentation.h"
#include "ui/Controls.h"
#include "model/route.h"
#include "model/route_point.h"
#include "model/routeman.h"
#include "chcanv.h"
#include "ocpndc.h"
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
namespace opennav::integration {
int ChartRouteWaypointOrdinal(ChartCanvas &canvas, RoutePoint &point, bool pinned_icon) {
  wxColour ink;
  if (!ChartActiveRouteInk(canvas, ink)) return 0;
  if (!wxIsMainThread() || !pinned_icon || !g_pRouteMan || !pRouteList ||
      point.GetIconName() != "diamond" || !point.m_bIsInRoute ||
      point.IsShared() || point.m_bIsInLayer || point.m_bIsActive ||
      point.m_bPtIsSelected || point.m_bBlink || point.m_bRPIsBeingEdited ||
      point.IsDragHandleEnabled() || &point == pAnchorWatchPoint1 ||
      &point == pAnchorWatchPoint2 ||
      (point.m_bShowWaypointRangeRings && point.m_iWaypointRangeRingsNumber))
    return 0;
  auto *active = g_pRouteMan->GetpActiveRoute();
  if (!active || !DefaultChartRouteStyle(*active) || active->m_bIsBeingCreated)
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
      if (route == active) ordinal = index;
    }
  }
  // Two digits fit the prototype circle. Larger routes retain upstream icons;
  // never truncate, wrap, or fabricate a route ordinal.
  return occurrences == 1 && ordinal <= 99 ? ordinal : 0;
}
int ChartRouteWaypointExtent(ChartCanvas &canvas) {
  const double scale = canvas.FromDIP(100) / 100.0 * g_MarkScaleFactorExp;
  return std::isfinite(scale) && scale >= .25 && scale <= 8
      ? static_cast<int>(std::ceil(11 * scale)) : 0;
}
bool DrawChartRouteWaypoint(ocpnDC &dc, ChartCanvas &canvas,
                            int x, int y, int ordinal) {
  wxColour ink;
  if (ordinal < 1 || ordinal > 99 || !ChartActiveRouteInk(canvas, ink)) return false;
  const double scale = canvas.FromDIP(100) / 100.0 * g_MarkScaleFactorExp;
  if (!std::isfinite(scale) || scale < .25 || scale > 8) return false;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  auto fill = ui::Colour(ui::FloatingTheme(mode).surface);
  if (mode == ui::LightMode::Night)
    fill = wxColour(std::lround(fill.Red()*.78), std::lround(fill.Green()*.78),
                    std::lround(fill.Blue()*.78));
  // Immutable .map-waypoint: radius 10, centered stroke 2, route ink,
  // floating fill, 8px/650 ordinal centered at y+.5. Positions remain upstream.
  const auto font = ui::UiFontWeight(canvas, 8, 650);
  auto marker_font = font;
  // Honor the existing mark-size preference independently of display DPI.
  marker_font.SetFractionalPointSize(font.GetFractionalPointSize()*g_MarkScaleFactorExp);
  const auto label = wxString::Format("%02d", ordinal);
  if (auto *native = dc.GetDC()) {
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::CreateFromUnknownDC(*native));
    if (!gc) return false;
    gc->SetBrush(wxBrush(fill));
    gc->SetPen(gc->CreatePen(wxGraphicsPenInfo(ink).Width(2*scale)));
    gc->DrawEllipse(x-10*scale,y-10*scale,20*scale,20*scale);
    gc->SetFont(marker_font,ink);
    double w=0,h=0; gc->GetTextExtent(label,&w,&h);
    gc->DrawText(label,x-w/2,y+.5*scale-h/2);
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
    // Two opaque triangle fans exactly share the SVG's outer/inner radii.
    // No line-width limits, bitmap icon cache, or four-point strip ambiguity.
    for (int layer=0;layer<2;++layer) {
      const auto color=layer ? fill : ink;
      float rgba[]={color.Red()/256.f,color.Green()/256.f,color.Blue()/256.f,1.f};
      shader->SetUniform4fv("color",rgba);
      std::vector<float> vertices;
      const double radius=(layer ? 9 : 11)*scale;
      constexpr int segments=96;
      for (int i=0;i<segments;++i) {
        const double a=i*2*3.141592653589793/segments;
        const double b=(i+1)*2*3.141592653589793/segments;
        vertices.insert(vertices.end(),{float(x),float(y),float(x+radius*std::cos(a)),
            float(y+radius*std::sin(a)),float(x+radius*std::cos(b)),float(y+radius*std::sin(b))});
      }
      shader->SetAttributePointerf("position",vertices.data());
      glDrawArrays(GL_TRIANGLES,0,vertices.size()/2);
    }
    shader->UnBind();
    const auto old_font=dc.GetFont(); const auto old_ink=dc.GetTextForeground();
    dc.SetFont(marker_font); dc.SetTextForeground(ink);
    wxCoord w=0,h=0; dc.GetTextExtent(label,&w,&h);
    dc.DrawText(label,std::lround(x-w/2.),std::lround(y+.5*scale-h/2.));
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
  const int extent=ChartRouteWaypointExtent(canvas);
  dc.CalcBoundingBox(x-extent,y-extent); dc.CalcBoundingBox(x+extent,y+extent);
  return true;
}
} // namespace opennav::integration
