#include "vessel/VesselState.h"

#include <cmath>
#include <stdexcept>

namespace opennav::vessel {

const char *QualityName(Quality q) {
  switch (q) {
  case Quality::Live:
    return "LIVE";
  case Quality::Aging:
    return "AGING";
  case Quality::Stale:
    return "STALE";
  case Quality::Unavailable:
    return "UNAVAILABLE";
  case Quality::Estimated:
    return "ESTIMATED";
  case Quality::Uncertain:
    return "UNCERTAIN";
  }
  return "UNAVAILABLE";
}
const char *ValidityName(Validity v) {
  switch (v) {
  case Validity::Measured:
    return "Measured";
  case Validity::Estimated:
    return "Estimated";
  case Validity::Uncertain:
    return "Uncertain";
  case Validity::Invalid:
    return "Invalid";
  }
  return "Invalid";
}
TextAssessment AssessText(const TextSample &s, Time now, Freshness f) {
  const auto assessment =
      Assess(Sample{s.value && !s.value->empty() ? std::optional<double>{1}
                                                 : std::nullopt,
                    s.source, s.observed_at, s.validity},
             now, f);
  return {assessment.quality, assessment.value ? s.value : std::nullopt,
          assessment.age};
}

Assessment Assess(const Sample &sample, Time now, Freshness freshness) {
  if (freshness.aging_after < Duration::zero() ||
      freshness.stale_after <= freshness.aging_after) {
    throw std::invalid_argument("Freshness requires 0 <= aging < stale");
  }
  if (!sample.value || !std::isfinite(*sample.value) || sample.source.empty() ||
      (sample.validity != Validity::Measured &&
       sample.validity != Validity::Estimated &&
       sample.validity != Validity::Uncertain)) {
    return {};
  }
  if (sample.observed_at > now) {
    // Clock mismatch/future observations are not current vessel measurements.
    return {Quality::Uncertain, std::nullopt, std::nullopt};
  }
  const auto age =
      std::chrono::duration_cast<Duration>(now - sample.observed_at);
  if (age >= freshness.stale_after)
    return {Quality::Stale, sample.value, age};
  if (sample.validity == Validity::Uncertain)
    return {Quality::Uncertain, sample.value, age};
  if (sample.validity == Validity::Estimated)
    return {Quality::Estimated, sample.value, age};
  if (age >= freshness.aging_after)
    return {Quality::Aging, sample.value, age};
  return {Quality::Live, sample.value, age};
}

VesselState SimulatorFixture(Time observed_at) {
  const auto measured = [observed_at](double value) {
    return Sample{value, "OpenNav simulator", observed_at, Validity::Measured};
  };
  VesselState state;
  state.simulated = true;
  state.navigation.sog_kn = measured(6.3);
  state.navigation.cog_deg = measured(147.0);
  state.environment.depth_below_transducer_m = measured(8.4);
  state.wind.apparent_speed_kn = measured(16.2);
  state.wind.apparent_angle_deg = measured(72.0);
  // Deliberately no position, heading or control-device state: this fixture
  // must not move the real chart, imply an autopilot mode or emit commands.
  return state;
}

} // namespace opennav::vessel
