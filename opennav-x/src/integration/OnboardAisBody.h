#pragma once
#include <array>
#include <cmath>
#include <vector>

namespace opennav::integration {
// Paint-time values copied by the pinned AIS renderer. No target is retained.
// Numeric class/status/alert identities are checked against upstream in .cpp.
struct OnboardAisAppearance {
  int target_class = -1, navigation_status = -1, ship_type = -1, alert = -1;
  bool active = false, lost = true, position_valid = false, doubtful = true;
  bool name_valid = false, cached_name = true, inland = true, euro_inland = true;
  bool aircraft = true, follower = true, blue_paddle = true;
  bool direction_valid = false, realtime_prediction = true;
};
inline bool UseOnboardAisBody(const OnboardAisAppearance &a) {
  const bool vessel = a.target_class == 0 || a.target_class == 1;
  // Class B ordinarily has no navigation-status report (15). All statuses
  // carrying a native glyph keep stock rendering, including HSC ShipType.
  const bool ordinary = a.navigation_status == 0 || a.navigation_status == 8 ||
      (a.target_class == 1 && a.navigation_status == 15);
  return vessel && ordinary && !(a.ship_type >= 40 && a.ship_type < 50) &&
      a.alert == 0 && a.active && !a.lost && a.position_valid && !a.doubtful &&
      a.name_valid && !a.cached_name && !a.inland && !a.euro_inland &&
      !a.aircraft && !a.follower && !a.blue_paddle && a.direction_valid &&
      !a.realtime_prediction;
}
struct OnboardAisMesh { std::vector<float> fill, outline; };
inline OnboardAisMesh OnboardAisBodyMesh(bool class_b, double x, double y,
                                       double angle, double scale) {
  OnboardAisMesh mesh;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(angle) ||
      !std::isfinite(scale) || std::abs(x) > 1e7 || std::abs(y) > 1e7 ||
      scale < .25 || scale > 16) return mesh;
  struct Point { double x, y; };
  // Prototype Class B: M0-12 6 9 0 5-6 9Z. Class A deliberately keeps
  // a straight stern, preserving the upstream A/B transponder distinction.
  std::vector<Point> points = class_b
      ? std::vector<Point>{{6,9},{0,5},{-6,9},{0,-12}}
      : std::vector<Point>{{6,9},{-6,9},{0,-12}};
  mesh.fill.reserve(class_b?12:6);mesh.outline.reserve(points.size()*12);
  const auto transform = [&](Point p) {
    return Point{x+scale*(p.x*std::cos(angle)-p.y*std::sin(angle)),
                 y+scale*(p.x*std::sin(angle)+p.y*std::cos(angle))};
  };
  const auto triangle = [&](std::vector<float> &out, Point a, Point b, Point c) {
    for (auto p : {a,b,c}) { p=transform(p); out.push_back(p.x); out.push_back(p.y); }
  };
  if (class_b) {
    triangle(mesh.fill,points[0],points[1],points[3]);
    triangle(mesh.fill,points[1],points[2],points[3]);
  } else triangle(mesh.fill,points[0],points[1],points[2]);
  std::vector<Point> outer, inner;
  outer.reserve(points.size());inner.reserve(points.size());
  for (std::size_t i=0;i<points.size();++i) {
    const auto before=points[(i+points.size()-1)%points.size()], p=points[i],
               after=points[(i+1)%points.size()];
    const double ax=p.x-before.x, ay=p.y-before.y, bx=after.x-p.x, by=after.y-p.y;
    const double al=std::hypot(ax,ay), bl=std::hypot(bx,by);
    const Point n1{-ay/al,ax/al}, n2{-by/bl,bx/bl};
    // Exact centered 1.6 CSS-pixel SVG miter stroke, transformed with the body.
    const double offset=.8/(1+n1.x*n2.x+n1.y*n2.y);
    const Point delta{(n1.x+n2.x)*offset,(n1.y+n2.y)*offset};
    outer.push_back({p.x+delta.x,p.y+delta.y});
    inner.push_back({p.x-delta.x,p.y-delta.y});
  }
  for (std::size_t i=0;i<points.size();++i) {
    const auto j=(i+1)%points.size();
    triangle(mesh.outline,outer[i],outer[j],inner[i]);
    triangle(mesh.outline,inner[i],outer[j],inner[j]);
  }
  return mesh;
}
} // namespace opennav::integration
