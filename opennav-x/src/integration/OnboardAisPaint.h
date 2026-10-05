#pragma once
#include "integration/OnboardAisBody.h"
#include "ui/Theme.h"
#include <wx/graphics.h>
#include <wx/brush.h>
#include <memory>

namespace opennav::integration {
inline wxColour OnboardAisInk(std::uint32_t value, ui::LightMode mode) {
  const auto channel=[&](unsigned v) { return mode==ui::LightMode::Night
      ? static_cast<unsigned>(std::lround(v*.78)) : v; };
  return wxColour(channel((value>>16)&255),channel((value>>8)&255),channel(value&255));
}
// Shared actual wx painter, also used by the offline pixel fixture.
inline bool PaintOnboardAisBody(wxDC &dc, const OnboardAisMesh &mesh,
                               const wxColour &fill, const wxColour &outline) {
  if (mesh.fill.empty()) return false;
  std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::CreateFromUnknownDC(dc));
  if (!gc) return false;
  const auto paint=[&](const std::vector<float> &triangles,const wxColour &color) {
    auto path=gc->CreatePath();
    for (std::size_t i=0;i<triangles.size();i+=6) {
      path.MoveToPoint(triangles[i],triangles[i+1]);
      path.AddLineToPoint(triangles[i+2],triangles[i+3]);
      path.AddLineToPoint(triangles[i+4],triangles[i+5]);path.CloseSubpath();
    }
    gc->SetBrush(wxBrush(color));gc->FillPath(path,wxWINDING_RULE);
  };
  paint(mesh.fill,fill);paint(mesh.outline,outline);return true;
}
} // namespace opennav::integration
