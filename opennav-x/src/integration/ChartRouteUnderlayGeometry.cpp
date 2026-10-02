#include "integration/ChartRouteUnderlay.h"
#include "tesselator.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <unordered_map>
namespace opennav::integration {
namespace {
constexpr std::size_t max_legs = 1024, tess_budget = 8 * 1024 * 1024;
struct Budget {
  std::size_t used = 0;
};
struct alignas(std::max_align_t) Allocation {
  std::size_t bytes;
};
void *Allocate(void *opaque, unsigned bytes) {
  auto &b = *static_cast<Budget *>(opaque);
  if (bytes > tess_budget - b.used)
    return nullptr;
  auto *p = static_cast<Allocation *>(std::malloc(sizeof(Allocation) + bytes));
  if (!p)
    return nullptr;
  p->bytes = bytes;
  b.used += bytes;
  return p + 1;
}
void Free(void *opaque, void *ptr) {
  if (!ptr)
    return;
  auto *p = static_cast<Allocation *>(ptr) - 1;
  static_cast<Budget *>(opaque)->used -= p->bytes;
  std::free(p);
}
void *Reallocate(void *opaque, void *ptr, unsigned bytes) {
  if (!ptr)
    return Allocate(opaque, bytes);
  auto &b = *static_cast<Budget *>(opaque);
  auto *old = static_cast<Allocation *>(ptr) - 1;
  if (bytes > tess_budget - (b.used - old->bytes))
    return nullptr;
  const auto previous = old->bytes;
  auto *p =
      static_cast<Allocation *>(std::realloc(old, sizeof(Allocation) + bytes));
  if (!p)
    return nullptr;
  p->bytes = bytes;
  b.used = b.used - previous + bytes;
  return p + 1;
}
using Point = std::array<double, 2>;
void Contour(TESStesselator *tess, std::vector<Point> points) {
  double area = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto &a = points[i], &b = points[(i + 1) % points.size()];
    area += a[0] * b[1] - b[0] * a[1];
  }
  if (area == 0)
    return;
  if (area < 0)
    std::reverse(points.begin(), points.end());
  std::vector<float> xy;
  for (const auto &p : points) {
    xy.push_back(p[0]);
    xy.push_back(p[1]);
  }
  tessAddContour(tess, 2, xy.data(), 2 * sizeof(float), points.size());
}
bool Clip(RouteUnderlayLeg &p, double width, double height, double margin) {
  const double dx = p.bx - p.ax, dy = p.by - p.ay;
  double begin = 0, end = 1;
  const auto edge = [&](double n, double q) {
    if (n == 0)
      return q >= 0;
    const double t = q / n;
    if (n < 0) {
      if (t > end)
        return false;
      begin = (std::max)(begin, t);
    } else {
      if (t < begin)
        return false;
      end = (std::min)(end, t);
    }
    return true;
  };
  if (!edge(-dx, p.ax + margin) || !edge(dx, width + margin - p.ax) ||
      !edge(-dy, p.ay + margin) || !edge(dy, height + margin - p.ay))
    return false;
  p.join_start = p.join_start && begin == 0;
  p.join_end = p.join_end && end == 1;
  p.bx = p.ax + end * dx;
  p.by = p.ay + end * dy;
  p.ax += begin * dx;
  p.ay += begin * dy;
  return p.ax != p.bx || p.ay != p.by;
}
} // namespace
void ChartRouteUnderlay::Add(std::size_t leg, double ax, double ay, double bx,
                             double by, bool join_start, bool join_end) {
  if (!valid_)
    return;
  for (double v : {ax, ay, bx, by})
    if (!std::isfinite(v) || std::abs(v) > 1e9) {
      valid_ = false;
      return;
    }
  if (legs_.size() >= max_legs) {
    valid_ = false;
    return;
  }
  try {
    legs_.push_back({ax, ay, bx, by, leg, join_start, join_end});
  } catch (const std::bad_alloc &) {
    valid_ = false;
  }
}
RouteUnderlayMesh ChartRouteUnderlay::Mesh(double scale, double width,
                                           double height) const {
  RouteUnderlayMesh result;
  if (!valid_ || !std::isfinite(scale) || scale < .25 || scale > 16 ||
      !std::isfinite(width) || !std::isfinite(height) || width <= 0 ||
      height <= 0 || width > 65536 || height > 65536)
    return result;
  if (legs_.empty()) {
    result.valid = true;
    return result;
  }
  try {
    Budget budget;
    TESSalloc allocator{};
    allocator.memalloc = Allocate;
    allocator.memrealloc = Reallocate;
    allocator.memfree = Free;
    allocator.userData = &budget;
    std::unique_ptr<TESStesselator, decltype(&tessDeleteTess)> tess(
        tessNewTess(&allocator), tessDeleteTess);
    if (!tess)
      return result;
    const double radius = 3 * scale,
                 miter_limit = 4; // SVG defaults: miter/butt.
    std::vector<RouteUnderlayLeg> clipped;
    std::unordered_multimap<std::size_t, std::size_t> starts;
    std::unordered_multimap<std::size_t, Point> repeated;
    for (const auto &leg : legs_)
      if (leg.ax == leg.bx && leg.ay == leg.by && leg.join_start &&
          leg.join_end)
        repeated.emplace(leg.leg, Point{leg.ax, leg.ay});
    for (auto leg : legs_)
      if (Clip(leg, width, height, radius * miter_limit)) {
        starts.emplace(leg.leg, clipped.size());
        clipped.push_back(leg);
      }
    if (clipped.empty()) {
      result.valid = true;
      return result;
    }
    for (const auto &p : clipped) {
      const double length = std::hypot(p.bx - p.ax, p.by - p.ay);
      const double ux = (p.bx - p.ax) / length, uy = (p.by - p.ay) / length;
      const double nx = -uy * radius, ny = ux * radius;
      Contour(tess.get(), {{p.ax + nx, p.ay + ny},
                           {p.ax - nx, p.ay - ny},
                           {p.bx - nx, p.by - ny},
                           {p.bx + nx, p.by + ny}});
      if (!p.join_end || p.leg == (std::numeric_limits<std::size_t>::max)())
        continue;
      auto next_leg = p.leg + 1;
      // SVG ignores zero-length intermediate legs when constructing a join.
      // Skip only explicit identical-point legs, never missing/wrapped
      // geometry.
      while (next_leg != (std::numeric_limits<std::size_t>::max)()) {
        bool found = false;
        const auto zeros = repeated.equal_range(next_leg);
        for (auto z = zeros.first; z != zeros.second; ++z)
          if (z->second[0] == p.bx && z->second[1] == p.by) {
            found = true;
            break;
          }
        if (!found)
          break;
        ++next_leg;
      }
      const auto range = starts.equal_range(next_leg);
      for (auto it = range.first; it != range.second; ++it) {
        const auto &q = clipped[it->second];
        if (!q.join_start || p.bx != q.ax || p.by != q.ay)
          continue;
        const double next = std::hypot(q.bx - q.ax, q.by - q.ay);
        // The pinned float tessellator can add/drop triangles where a short
        // leg's cap intersects its neighboring join. Keep both incident legs
        // longer than a full stroke width; otherwise omit this whole decorative
        // layer, never a partial union. Foreground/waypoints remain upstream.
        if (length <= 2 * radius + 1e-6 || next <= 2 * radius + 1e-6)
          return {};
        const double vx = (q.bx - q.ax) / next, vy = (q.by - q.ay) / next;
        const double cross = ux * vy - uy * vx, dot = ux * vx + uy * vy;
        if (std::abs(cross) < 1e-12)
          continue;
        const double side = cross > 0 ? -1 : 1;
        const Point a{p.bx + side * nx, p.by + side * ny};
        const Point b{p.bx - side * vy * radius, p.by + side * vx * radius};
        std::vector<Point> join{a};
        if (1 + dot > 0) {
          const double mx = side * (-uy - vy) * radius / (1 + dot);
          const double my = side * (ux + vx) * radius / (1 + dot);
          if (std::hypot(mx, my) <= radius * miter_limit)
            join.push_back({p.bx + mx, p.by + my});
        }
        join.push_back(b);
        // Overlap the inner side already covered by the segment rectangles;
        // quadrant-only joins trigger coincident T-junction defects in tess2.
        // Bound that overlap by both leg lengths so very short legs do not
        // acquire paint beyond their butt caps.
        const double inner =
            (std::min)({1.0, length / (2 * radius), next / (2 * radius)});
        join.push_back({p.bx - side * nx * inner, p.by - side * ny * inner});
        join.push_back({p.bx + side * vy * radius * inner,
                        p.by - side * vx * radius * inner});
        Contour(tess.get(), join);
      }
    }
    // Union all positive stroke contours before emitting alpha triangles. In
    // particular, doubled legs and self-crossings never accumulate opacity.
    const TESSreal normal[]{0, 0, 1};
    if (!tessTesselate(tess.get(), TESS_WINDING_NONZERO, TESS_POLYGONS, 3, 2,
                       normal))
      return result;
    const int count = tessGetElementCount(tess.get()),
              vertices = tessGetVertexCount(tess.get());
    if (count < 0 || count > 262144 || vertices < 0)
      return result;
    const auto *indices = tessGetElements(tess.get());
    const auto *xy = tessGetVertices(tess.get());
    result.triangles.reserve(std::size_t(count) * 6);
    for (int i = 0; i < count * 3; ++i) {
      const int index = indices[i];
      if (index < 0 || index >= vertices)
        return {};
      for (int c = 0; c < 2; ++c) {
        const float value = xy[index * 2 + c];
        if (!std::isfinite(value))
          return {};
        result.triangles.push_back(value);
      }
    }
    result.valid = true;
  } catch (const std::bad_alloc &) {
    return {};
  }
  return result;
}
} // namespace opennav::integration
