#include "weather/QueryBuilder.h"
#include <algorithm>
#include <cmath>

namespace opennav::weather {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxRouteInput = 1000;
constexpr double kMinRouteSpacingDeg = 0.1;  // ≈ 6 nm; GFS grid is 0.25°.
bool Valid(const Coordinate &c) {
  return std::isfinite(c.latitude_deg) && std::isfinite(c.longitude_deg) &&
         std::abs(c.latitude_deg) <= 90 && std::abs(c.longitude_deg) <= 540;
}
double LegDeg(const Coordinate &a, const Coordinate &b) {
  const double dlat = b.latitude_deg - a.latitude_deg;
  const double dlon = std::remainder(b.longitude_deg - a.longitude_deg, 360.0) *
                      std::cos((a.latitude_deg + b.latitude_deg) * 0.5 * kPi / 180.0);
  return std::hypot(dlat, dlon);
}
} // namespace

std::vector<Coordinate> SampleRoute(const std::vector<Coordinate> &route, std::size_t count) {
  std::vector<Coordinate> points;
  for (const auto &c : route) {
    if (points.size() >= kMaxRouteInput) break;
    if (Valid(c)) points.push_back(c);
  }
  if (points.size() < 2 || count == 0) {
    if (points.size() > count) points.resize(count);
    return points;
  }
  std::vector<double> legs;
  double total = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    legs.push_back(LegDeg(points[i - 1], points[i]));
    total += legs.back();
  }
  if (!(total > 1e-9)) return {points.front()};
  const auto wanted = static_cast<std::size_t>(std::floor(total / kMinRouteSpacingDeg)) + 1;
  const std::size_t n = std::clamp<std::size_t>(wanted, 2, std::max<std::size_t>(count, 2));
  std::vector<Coordinate> out;
  std::size_t leg = 0;
  double before = 0;
  for (std::size_t i = 0; i < n && out.size() < count; ++i) {
    const double at = total * static_cast<double>(i) / static_cast<double>(n - 1);
    while (leg + 1 < legs.size() && before + legs[leg] < at) before += legs[leg++];
    const double f = legs[leg] > 0 ? std::clamp((at - before) / legs[leg], 0.0, 1.0) : 0.0;
    const auto &a = points[leg], &b = points[leg + 1];
    out.push_back({a.latitude_deg + (b.latitude_deg - a.latitude_deg) * f,
                   a.longitude_deg +
                       std::remainder(b.longitude_deg - a.longitude_deg, 360.0) * f});
  }
  return out;
}

std::vector<Coordinate> ChartGrid(const ChartBox &box, std::size_t budget) {
  const std::size_t per_axis = std::min<std::size_t>(
      kMaxGridPerAxis, static_cast<std::size_t>(std::floor(std::sqrt(static_cast<double>(budget)))));
  if (per_axis < 2 || !std::isfinite(box.min_lat) || !std::isfinite(box.max_lat) ||
      !std::isfinite(box.min_lon) || !std::isfinite(box.max_lon) || box.max_lat < box.min_lat)
    return {};
  const double min_lat = std::max(-90.0, box.min_lat), max_lat = std::min(90.0, box.max_lat);
  double min_lon = box.min_lon, max_lon = box.max_lon;
  if (max_lon < min_lon) max_lon += 360.0;  // Antimeridian wrap.
  if (max_lon - min_lon > 360.0) max_lon = min_lon + 360.0;
  if (min_lat > max_lat) return {};
  static const double steps[] = {0.25, 0.5, 1, 2, 5, 10, 20, 45};
  for (double step : steps) {
    const auto lat0 = std::floor(min_lat / step), lat1 = std::ceil(max_lat / step);
    const auto lon0 = std::floor(min_lon / step), lon1 = std::ceil(max_lon / step);
    if (lat1 - lat0 + 1 > per_axis || lon1 - lon0 + 1 > per_axis) continue;
    std::vector<Coordinate> out;
    for (double i = lat0; i <= lat1; ++i) {
      const double lat = i * step;
      if (std::abs(lat) > 90) continue;
      for (double j = lon0; j <= lon1; ++j)
        out.push_back({lat, std::remainder(j * step, 360.0)});
    }
    return out;
  }
  return {};
}

ForecastQuery BuildForecastQuery(const QueryInputs &in) {
  ForecastQuery q;
  if (in.vessel && Valid(*in.vessel)) q.points.push_back(*in.vessel);
  for (const auto &c : SampleRoute(in.route, kMaxRouteSamples)) q.points.push_back(c);
  if (in.chart && q.points.size() < kMaxForecastPoints)
    for (const auto &c : ChartGrid(*in.chart, kMaxForecastPoints - q.points.size()))
      q.points.push_back(c);
  if (q.points.size() > kMaxForecastPoints) q.points.resize(kMaxForecastPoints);
  q.from = std::chrono::floor<std::chrono::hours>(in.now);
  q.until = q.from + kForecastHorizon;
  return q;
}
} // namespace opennav::weather
