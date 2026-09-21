#include "vessel/NavigationInput.h"

#include <cmath>

namespace opennav::vessel {
namespace {
bool InRange(std::optional<double> value, double low, double high) {
  return value && std::isfinite(*value) && *value >= low && *value <= high;
}

void Set(Sample& target, std::optional<double> value, const NavigationUpdate& update) {
  // Queued old messages must not replace a newer observation or renew its age.
  if (update.observed_at && *update.observed_at < target.observed_at) return;
  const bool valid = value && update.observed_at && !update.source.empty();
  target = {valid ? value : std::nullopt, update.source,
            update.observed_at.value_or(target.observed_at), valid ? Validity::Measured : Validity::Invalid};
}
}  // namespace

void NavigationInput::Apply(const NavigationUpdate& update) {
  if (update.position_updated) {
    const bool valid = update.position_valid && InRange(update.latitude_deg, -90, 90)
                       && InRange(update.longitude_deg, -180, 180);
    Set(state_.navigation.latitude_deg, valid ? update.latitude_deg : std::nullopt, update);
    Set(state_.navigation.longitude_deg, valid ? update.longitude_deg : std::nullopt, update);
  }
  if (update.sog_updated) {
    const bool valid = update.sog_kn && std::isfinite(*update.sog_kn) && *update.sog_kn >= 0;
    Set(state_.navigation.sog_kn, valid ? update.sog_kn : std::nullopt, update);
  }
  if (update.cog_updated) {
    auto course = InRange(update.cog_deg, 0, 360) ? update.cog_deg : std::nullopt;
    if (course == 360) course = 0;  // Equivalent true-north bearings.
    Set(state_.navigation.cog_deg, course, update);
  }
}

}  // namespace opennav::vessel
