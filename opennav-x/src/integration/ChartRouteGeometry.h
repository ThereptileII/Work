#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace opennav::integration {
// Screen-space artwork only. The caller supplies upstream projected/wrapped
// endpoints. Width is logical chart pixels, never nautical corridor distance.
inline std::vector<float> ChartRouteSegmentMesh(
    double ax, double ay, double bx, double by, double scale,
    double viewport_width, double viewport_height, bool join_start,
    bool join_end) {
  std::vector<float> triangles;
  for (double value : {ax, ay, bx, by, scale, viewport_width, viewport_height})
    if (!std::isfinite(value)) return triangles;
  if (scale <= 0 || scale > 16 || viewport_width <= 0 || viewport_height <= 0 ||
      viewport_width > 65536 || viewport_height > 65536) return triangles;
  const double radius = 1.3 * scale; // Final immutable .chart-route: 2.6px.
  double dx = bx - ax, dy = by - ay;
  if (!std::isfinite(dx) || !std::isfinite(dy)) return triangles;
  const double length = std::hypot(dx, dy);
  if (!std::isfinite(length) || length == 0) return triangles;
  const double nx = -dy / length * radius, ny = dx / length * radius;
  // Liang-Barsky against an expanded viewport: keep the join just outside the
  // edge, bound GPU coordinates, and never join across an upstream wrap break.
  double begin = 0, end = 1;
  const auto clip = [&](double p, double q) {
    if (p == 0) return q >= 0;
    const double t = q / p;
    if (p < 0) { if (t > end) return false; begin = (std::max)(begin, t); }
    else { if (t < begin) return false; end = (std::min)(end, t); }
    return true;
  };
  if (!clip(-dx, ax + radius) || !clip(dx, viewport_width + radius - ax) ||
      !clip(-dy, ay + radius) || !clip(dy, viewport_height + radius - ay))
    return triangles;
  join_start = join_start && begin == 0;
  join_end = join_end && end == 1;
  bx = ax + end * dx; by = ay + end * dy;
  ax += begin * dx; ay += begin * dy;
  triangles.reserve(12 + 192 * (int(join_start) + int(join_end)));
  const auto triangle = [&](double x1, double y1, double x2, double y2,
                            double x3, double y3) {
    triangles.insert(triangles.end(), {float(x1), float(y1), float(x2),
                                      float(y2), float(x3), float(y3)});
  };
  triangle(ax+nx, ay+ny, ax-nx, ay-ny, bx+nx, by+ny);
  triangle(bx+nx, by+ny, ax-nx, ay-ny, bx-nx, by-ny);
  const auto join = [&](double x, double y) {
    // Inscribed 32-sided disk: maximum radial error < 0.0063px at 100%.
    constexpr double tau = 6.2831853071795864769;
    for (int i = 0; i < 32; ++i)
      triangle(x, y, x + radius * std::cos(i*tau/32),
               y + radius * std::sin(i*tau/32),
               x + radius * std::cos((i+1)*tau/32),
               y + radius * std::sin((i+1)*tau/32));
  };
  if (join_start) join(ax, ay);
  if (join_end) join(bx, by);
  return triangles;
}
} // namespace opennav::integration
