#pragma once
#include "vessel/RouteProgress.h"
#include "vessel/AisState.h"

namespace opennav::vessel {
// Definitions live in opennav_test_fixtures, never a production runtime library.
enum class DemoScenario {
  Cruise,
  Stale,
  Unavailable,
  RouteInactive,
  RouteEnding,
  LowSoc,
  HighPower,
  Insufficient
};
const char *ScenarioName(DemoScenario scenario);
// Deterministic synthetic trip. Elapsed seconds advance the trip at 60x; wall
// clock never enters the fixture. It does not calculate real route geometry.
VesselState DemoFixture(DemoScenario scenario, unsigned seconds,
                        Time observed_at);
AisState DemoAis(const VesselState& synthetic);
class DemoSource {
public:
  explicit DemoSource(Time start) : start_(start) {}
  void Select(DemoScenario scenario, Time now) {
    scenario_ = scenario;
    start_ = now;
    paused_ = false;
  }
  void Pause(bool value, Time now);
  VesselState Read(Time now) const;
  DemoScenario Scenario() const { return scenario_; }
  bool Paused() const { return paused_; }

private:
  DemoScenario scenario_ = DemoScenario::Cruise;
  Time start_, paused_at_{};
  bool paused_ = false;
};
} // namespace opennav::vessel
