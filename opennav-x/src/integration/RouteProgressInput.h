#pragma once

#include "vessel/RouteProgress.h"
#include <vector>

namespace opennav::integration {

// Integration-only copies; no upstream objects enter the public vessel contract.
struct RoutePointCopy {
  std::string id;
  double latitude_deg = 0, longitude_deg = 0, incoming_leg_nm = 0;
};
struct RouteCopy {
  bool active = false, registered = true, active_point_consistent = true;
  std::string id, active_point_id;
  std::optional<std::size_t> active_index;
  std::vector<RoutePointCopy> points;
};
struct RouteRead {
  bool interrupted = false;
  RouteCopy route;
  vessel::Navigation position;
  bool upstream_position_valid = false;
  double upstream_latitude_deg = 0, upstream_longitude_deg = 0;
  std::optional<double> range_to_active_nm;
};

bool SameRoute(const RouteCopy& a, const RouteCopy& b);  // geometry, order, IDs
bool SameActivePoint(const RouteCopy& a, const RouteCopy& b);

// GUI-thread writer. A pass supplies copies bracketing normal upstream progress;
// neither this reducer nor its consumer invokes navigation/control processing.
class RouteProgressInput {
 public:
  explicit RouteProgressInput(std::string revision_scope);
  void Complete(const RouteRead& before, const RouteRead& after, vessel::Time now);
  // A consumer read can invalidate changed route state, never recompute/freshen.
  void CheckCurrent(const RouteCopy& current, vessel::Time now);
  vessel::RouteProgress Current() const { return current_; }
 private:
  void Publish(vessel::RouteProgressSnapshot result);
  vessel::RouteProgressSnapshot Describe(const RouteRead& read, vessel::Time now) const;
  std::string scope_;
  std::uint64_t revision_ = 0;
  std::optional<RouteCopy> last_route_;
  std::optional<vessel::Time> observation_watermark_, position_watermark_;
  vessel::RouteProgress current_;
};
}  // namespace opennav::integration
