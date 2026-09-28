#pragma once
#include "ais/Provider.h"
#include <map>

namespace opennav::ais {
constexpr const char *OnlineSource = "AISSTREAM_ONLINE";
enum class TargetAge { Live, Aging, Stale, Lost, Expired, Invalid };
TargetAge Age(vessel::Time observed, vessel::Time now);

// Decoder output only. A position message and static message have independent
// clocks; receiving static information never rejuvenates the last position.
struct PositionReport {
  int mmsi = 0;
  double latitude = 91, longitude = 181;
  std::optional<double> sog, cog, heading, navigation_status;
  vessel::Time observed_at{}; // receipt projected to monotonic observation clock
};
struct StaticReport {
  int mmsi = 0;
  std::optional<std::string> name, callsign, destination;
  std::optional<double> ship_type, length_m, beam_m;
  vessel::Time observed_at{};
};
class TargetCache {
 public:
  static constexpr std::size_t Capacity = 2000;
  bool Observe(const PositionReport &report, vessel::Time received_at);
  bool Observe(const StaticReport &report, vessel::Time received_at);
  vessel::AisState Read(vessel::Time now) const;
  std::size_t Size() const { return targets_.size(); }
 private:
  struct Entry {
    vessel::AisTarget target;
    vessel::Time position_time{}, static_time{};
    bool has_position = false, has_static = false;
  };
  Entry *FindOrInsert(int mmsi, vessel::Time received_at);
  std::map<int, Entry> targets_;
};
} // namespace opennav::ais
