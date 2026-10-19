#include "integration/NativeRouteOutput.h"
#include "model/autopilot_output.h"
#include "model/routeman.h"
#include <cmath>

namespace opennav::integration {
namespace {
bool SameOrigin(const RouteRead& a, const RouteRead& b) {
  return a.segment_begin && b.segment_begin &&
      !a.segment_begin->id.empty() && a.segment_begin->id == b.segment_begin->id &&
      a.segment_begin->latitude_deg == b.segment_begin->latitude_deg &&
      a.segment_begin->longitude_deg == b.segment_begin->longitude_deg;
}
bool PositionCurrent(const RouteRead &read, vessel::Time now) {
  const auto &lat = read.position.latitude_deg, &lon = read.position.longitude_deg;
  return read.upstream_position_valid && lat.value && lon.value &&
      lat.validity == vessel::Validity::Measured && lon.validity == vessel::Validity::Measured &&
      !lat.source.empty() && lat.source == lon.source && lat.observed_at == lon.observed_at &&
      lat.observed_at <= now && now - lat.observed_at < std::chrono::seconds(3) &&
      std::isfinite(*lat.value) && std::isfinite(*lon.value) &&
      std::abs(*lat.value) <= 90 && std::abs(*lon.value) <= 180 &&
      *lat.value == read.upstream_latitude_deg && *lon.value == read.upstream_longitude_deg;
}
// Existing upstream serializers write into this stack-only collector. No
// registry entry or physical transport is created, and no pointers escape.
class NavigationCollector final : public AbstractCommDriver {
 public:
  explicit NavigationCollector(const std::string &iface)
      : AbstractCommDriver(NavAddr::Bus::N2000, iface) {}
  bool SendMessage(std::shared_ptr<const NavMsg> message,
                   std::shared_ptr<const NavAddr> address) override {
    const auto n2k = std::dynamic_pointer_cast<const Nmea2000Msg>(message);
    const auto dest = std::dynamic_pointer_cast<const NavAddr2000>(address);
    const std::uint32_t order[] = {129285, 129284, 129283};
    if (!n2k || !dest || frames.size() >= 3 || n2k->PGN.pgn != order[frames.size()] ||
        n2k->priority != 6 || dest->iface != iface || dest->address != 255 ||
        dest->name.value.Name || n2k->payload.empty() || n2k->payload.size() > 223) return false;
    // The wider upstream PGN has already matched this exact canonical value.
    frames.push_back({order[frames.size()], n2k->payload});
    return true;
  }
  std::vector<NavigationOutputFrame> frames;
};
} // namespace

bool NativeRouteOutput::Current(const RouteRead &read, vessel::Time now) const {
  if (!accepted_ || !PositionCurrent(read, now) ||
      !SameRoute(accepted_->route, read.route) ||
      !SameActivePoint(accepted_->route, read.route) ||
      !SameOrigin(*accepted_, read) || read.route.editing) return false;
  const auto &old = accepted_->position, &current = read.position;
  return old.latitude_deg.source == current.latitude_deg.source &&
      old.latitude_deg.observed_at == current.latitude_deg.observed_at &&
      old.latitude_deg.value == current.latitude_deg.value &&
      old.longitude_deg.value == current.longitude_deg.value;
}

void NativeRouteOutput::Update(Routeman &manager, const RouteRead &read,
                              const vessel::RouteProgress &progress,
                              OpenCPNPilot &pilot, vessel::Time now) {
  const auto session = pilot.NavigationSession();
  if (!session) {
    session_.reset();
    pilot.InvalidateNavigationOutput("Enable control with the pilot in AUTO before TRACK");
    return;
  }
  if (!progress || progress->state != vessel::RouteState::Valid ||
      progress->route_id != read.route.id ||
      progress->active_waypoint_id != read.route.active_point_id ||
      progress->active_waypoint_index != read.route.active_index ||
      progress->position_observed_at != read.position.latitude_deg.observed_at ||
      progress->position_source != read.position.latitude_deg.source ||
      progress->observed_at > now || now - progress->observed_at >= std::chrono::seconds(3) ||
      !PositionCurrent(read, now) || read.interrupted ||
      !read.range_to_active_nm || !std::isfinite(*read.range_to_active_nm) ||
      *read.range_to_active_nm < 0 || *read.range_to_active_nm > 409.5 ||
      !read.cross_track_error_nm || !std::isfinite(*read.cross_track_error_nm) ||
      *read.cross_track_error_nm < 0 || *read.cross_track_error_nm > 40.95 ||
      !read.cross_track_direction ||
      (*read.cross_track_direction != -1 && *read.cross_track_direction != 1) ||
      !read.bearing_to_active_true_deg || !std::isfinite(*read.bearing_to_active_true_deg) ||
      *read.bearing_to_active_true_deg < 0 || *read.bearing_to_active_true_deg > 360 ||
      !read.segment_begin || read.segment_begin->id.empty() ||
      !std::isfinite(manager.GetCurrentSegmentCourse()) ||
      manager.GetCurrentSegmentCourse() < 0 || manager.GetCurrentSegmentCourse() > 360) {
    pilot.InvalidateNavigationOutput("TRACK needs a fresh, stable active route and measured position");
    return;
  }
  const bool identity_changed = !session_ || session_->epoch != session->epoch ||
      session_->generation != session->generation || session_->interface_id != session->interface_id ||
      !accepted_ || revision_ != progress->route_revision ||
      !SameRoute(accepted_->route, read.route) || !SameActivePoint(accepted_->route, read.route) ||
      !SameOrigin(*accepted_, read);
  if (identity_changed) {
    pilot.InvalidateNavigationOutput("Route changed; waiting for fresh navigation output");
    const auto token = allocate_ ? allocate_() : std::nullopt;
    if (!token || !*token || *token > 65531) {
      session_.reset();
      pilot.InvalidateNavigationOutput("Cannot reserve persistent waypoint identifiers for TRACK");
      return;
    }
    origin_token_ = *token;
  }
  accepted_ = read;
  session_ = session;
  revision_ = progress->route_revision;
  NavigationCollector collector(session->interface_id);
  // True bearings remain true. The bridge must supply fresh qualified magnetic
  // variation; no unaged gVar value or synthetic correction is substituted.
  if (!SendPGN129285(manager, &collector, origin_token_, origin_token_ + 1) ||
      !SendPGN129284(manager, &collector, origin_token_, origin_token_ + 1, false) ||
      !SendPGN129283(manager, &collector) || collector.frames.size() != 3 ||
      !pilot.SendNavigationBatch(*session, collector.frames, origin_token_ + 1,
                                 read.position.latitude_deg.observed_at))
    pilot.InvalidateNavigationOutput("Navigation output was withheld; review the route and pilot connection");
}
} // namespace opennav::integration
