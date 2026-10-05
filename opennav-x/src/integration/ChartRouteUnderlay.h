#pragma once
#include <cstddef>
#include <vector>
class ocpnDC;
class ChartCanvas;
namespace opennav::integration {
struct RouteUnderlayLeg {
  double ax, ay, bx, by;
  std::size_t leg;
  bool join_start = true, join_end = true;
};
struct RouteUnderlayMesh {
  bool valid = false;
  std::vector<float> triangles;
};
// Already projected/clipped/wrapped caller decisions, never geographic data.
class ChartRouteUnderlay {
public:
  void Add(std::size_t leg, double ax, double ay, double bx, double by,
           bool join_start = true, bool join_end = true);
  RouteUnderlayMesh Mesh(double scale, double width, double height) const;
  bool Draw(ocpnDC &dc, ChartCanvas &canvas) const;

private:
  bool valid_ = true;
  std::vector<RouteUnderlayLeg> legs_;
};
} // namespace opennav::integration
