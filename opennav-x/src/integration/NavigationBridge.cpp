#include "integration/NavigationBridge.h"

#include "model/comm_appmsg.h"
#include "model/cutil.h"

#include <chrono>
#include <utility>

namespace opennav {
namespace {
std::optional<vessel::Time> ObservationTime(const timespec& sent) {
  // Sample our clock first. If preempted between clock reads, this ordering
  // overestimates age rather than making a delayed observation look newer.
  const auto received = vessel::Clock::now();
  timespec current{};
  if (sent.tv_sec < 0 || sent.tv_nsec < 0 || sent.tv_nsec >= 1000000000 ||
      clock_gettime(CLOCK_MONOTONIC, &current) != 0) return std::nullopt;
  const auto age = std::chrono::seconds(current.tv_sec - sent.tv_sec)
                 + std::chrono::nanoseconds(current.tv_nsec - sent.tv_nsec);
  if (age < std::chrono::nanoseconds::zero()) return std::nullopt;
  // Convert between clock epochs by elapsed time, retaining queued-message age.
  return received - std::chrono::duration_cast<vessel::Clock::duration>(age);
}
}  // namespace

NavigationBridge::NavigationBridge(std::function<void(const vessel::VesselState&)> receive)
    : receive_(std::move(receive)) {
  listener_.Init(AppMsg(AppMsg::Type::BasicNavData), [this](ObservedEvt& event) {
    const auto message = UnpackEvtPointer<BasicNavDataMsg>(event);
    if (!message) return;
    vessel::NavigationUpdate update;
    // BasicNavData does not carry the physical sensor's identity. Do not invent it.
    update.source = "OpenCPN selected navigation (sensor identity unavailable)";
    update.observed_at = ObservationTime(message->set_time);
    update.position_updated = (message->vflag & POS_UPDATE) != 0;
    update.position_valid = (message->vflag & POS_VALID) != 0;
    update.sog_updated = (message->vflag & SOG_UPDATE) != 0;
    update.cog_updated = (message->vflag & COG_UPDATE) != 0;
    update.latitude_deg = message->pos.lat;
    update.longitude_deg = message->pos.lon;
    update.sog_kn = message->sog;
    update.cog_deg = message->cog;
    // Heading can be derived from magnetic heading + variation without an
    // estimate flag. Leave it unavailable until that provenance is preserved.
    input_.Apply(update);
    receive_(input_.State());
  });
}

}  // namespace opennav
