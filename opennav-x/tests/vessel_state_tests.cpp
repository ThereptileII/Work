#include "vessel/VesselState.h"

#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav::vessel;
using namespace std::chrono_literals;

void Require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main() {
  try {
    const Time time{100s};
    Require(Assess({}, time).quality == Quality::Unavailable, "Missing is unavailable");
    Sample sample{0.0, "test sensor", time, Validity::Measured};
    Require(Assess(sample, time).value == 0.0, "A measured zero is valid");
    Require(Assess(sample, time + 1999ms).quality == Quality::Live, "Before aging boundary");
    Require(Assess(sample, time + 2s).quality == Quality::Aging, "At aging boundary");
    Require(Assess(sample, time + 5s).quality == Quality::Stale, "At stale boundary");
    Require(Assess(sample, time + 5s).age == 5000ms, "Stale age remains inspectable");
    Require(Assess(sample, time - 1ms).quality == Quality::Uncertain, "Future timestamp rejected");
    Require(!Assess(sample, time - 1ms).value, "Clock mismatch suppresses value");
    for (const auto invalid : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()}) {
      sample.value = invalid;
      Require(Assess(sample, time).quality == Quality::Unavailable, "Non-finite rejected");
    }
    sample.value = 42;
    sample.source.clear();
    Require(!Assess(sample, time).value, "Unsourced measurement rejected");
    sample.source = "model";
    sample.validity = Validity::Estimated;
    Require(Assess(sample, time + 3s).quality == Quality::Estimated, "Estimate remains explicit while aging");
    Require(Assess(sample, time + 6s).quality == Quality::Stale, "Estimate still expires");
    sample.validity = Validity::Uncertain;
    Require(Assess(sample, time).quality == Quality::Uncertain, "Uncertainty retained");
    sample.validity = Validity::Invalid;
    Require(!Assess(sample, time).value, "Invalid data suppressed");
    sample.validity = static_cast<Validity>(99);
    Require(!Assess(sample, time).value, "Unknown validity fails closed");
    bool invalid_policy_rejected = false;
    try { (void)Assess(sample, time, Freshness{5s, 2s}); }
    catch (const std::invalid_argument&) { invalid_policy_rejected = true; }
    Require(invalid_policy_rejected, "Inverted freshness thresholds rejected");
    const auto fixture = SimulatorFixture(time);
    Require(fixture.simulated, "Simulation explicitly labelled");
    Require(Assess(fixture.navigation.sog_kn, time).value == 6.3, "Fixture expected SOG");
    Require(Assess(fixture.navigation.sog_kn, time + 6s).quality == Quality::Stale, "Stopped simulator ages naturally");
    Require(!fixture.navigation.latitude_deg.value, "No simulated chart position injected");
    Require(!VesselState{}.navigation.sog_kn.value, "Normal startup has no fabricated SOG");
    std::cout << "Vessel quality and simulator contracts passed.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
