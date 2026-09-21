#include "vessel/NavigationInput.h"

#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav::vessel;
using namespace std::chrono_literals;
static void Require(bool pass, const char* why) { if (!pass) throw std::runtime_error(why); }

int main() {
  try {
    NavigationInput input;
    const Time t{100s};
    NavigationUpdate update;
    update.source = "selected source";
    update.observed_at = t;
    update.sog_updated = update.cog_updated = true;
    update.sog_kn = 6.3;
    update.cog_deg = 147;
    input.Apply(update);
    Require(input.State().navigation.sog_kn.value == 6.3, "Selected SOG accepted in knots");
    Require(input.State().navigation.cog_deg.value == 147, "Selected COG accepted in true degrees");
    update.sog_updated = update.cog_updated = false;
    update.position_updated = update.position_valid = true;
    update.latitude_deg = 56.7; update.longitude_deg = 12.6;
    update.observed_at = t + 6s;
    // Full upstream message contains old velocity values: flags control freshness.
    input.Apply(update);
    Require(Assess(input.State().navigation.sog_kn, t + 6s).quality == Quality::Stale,
            "Position traffic cannot refresh old velocity");
    Require(Assess(input.State().navigation.latitude_deg, t + 6s).quality == Quality::Live,
            "Position has its own freshness");
    update.position_valid = false;
    input.Apply(update);
    Require(!input.State().navigation.latitude_deg.value && !input.State().navigation.longitude_deg.value,
            "Invalid fix suppresses both coordinates even when numbers look valid");
    update.position_valid = true; update.latitude_deg = 91;
    input.Apply(update);
    Require(!input.State().navigation.longitude_deg.value, "Out-of-range coordinate invalidates the pair");
    update.position_updated = false;
    update.sog_updated = update.cog_updated = true;
    update.observed_at = t + 7s;
    update.sog_kn = 0; update.cog_deg = 360;
    input.Apply(update);
    Require(input.State().navigation.sog_kn.value == 0, "Measured zero is not missing");
    Require(input.State().navigation.cog_deg.value == 0, "360 degrees is true north");
    update.observed_at = t; update.sog_kn = 20;
    input.Apply(update);
    Require(input.State().navigation.sog_kn.value == 0, "Queued older update cannot replace new data");
    update.observed_at = t + 8s;
    update.sog_kn = -1; update.cog_deg = std::numeric_limits<double>::quiet_NaN();
    input.Apply(update);
    Require(!input.State().navigation.sog_kn.value && !input.State().navigation.cog_deg.value,
            "Invalid numbers do not leave plausible current values");
    update.sog_kn = 4; update.source.clear();
    input.Apply(update);
    Require(!input.State().navigation.sog_kn.value, "Unsourced data is not current");
    update.source = "source"; update.observed_at.reset();
    input.Apply(update);
    Require(!input.State().navigation.sog_kn.value, "Unknown timestamp cannot refresh data");
    update.observed_at = t;
    input.Apply(update);
    Require(!input.State().navigation.sog_kn.value, "Clock failure cannot let old packets resurrect data");
    Require(!input.State().navigation.heading_true_deg.value && !input.State().battery.soc_percent.value,
            "Navigation does not invent heading or battery state");
    Require(!input.State().simulated, "Read-only input does not silently become a simulator");
    std::cout << "Navigation delta, validity, units and freshness contracts passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
