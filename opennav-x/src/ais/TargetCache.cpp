#include "ais/TargetCache.h"
#include <cmath>

namespace opennav::ais {
using namespace std::chrono_literals;
namespace {
bool Mmsi(int value) { return value >= 100000000 && value <= 999999999; }
bool Number(double n, double low, double high) { return std::isfinite(n) && n >= low && n <= high; }
bool OptionalNumber(const std::optional<double> &n, double low, double high) { return !n || Number(*n, low, high); }
bool Text(const std::optional<std::string> &s, std::size_t bound) {
  if (!s) return true;
  if (s->size() > bound) return false;
  for (unsigned char c : *s) if (c < 32 || c == 127) return false;
  return true; // UTF-8 validation is also mandatory at the JSON decoder boundary.
}
vessel::Sample Sample(std::optional<double> value, vessel::Time at) {
  return {value, OnlineSource, at, value ? vessel::Validity::Measured : vessel::Validity::Invalid, {15s, 60s}};
}
vessel::TextSample StaticText(const std::optional<std::string> &value, vessel::Time at) {
  return {value, OnlineSource, at, value ? vessel::Validity::Measured : vessel::Validity::Invalid, {1h, 6h}};
}
bool Stamp(vessel::Time observation, vessel::Time received) {
  const auto age = Age(observation, received);
  return age != TargetAge::Invalid && age != TargetAge::Expired;
}
} // namespace
TargetAge Age(vessel::Time observed, vessel::Time now) {
  if (observed == vessel::Time{} || now < observed) return TargetAge::Invalid;
  // Compare before subtracting; adversarial extreme time points must not
  // overflow a signed clock duration.
  const auto at_least = [&](vessel::Duration span) {
    return now >= vessel::Time::min() + span && observed <= now - span;
  };
  if (at_least(10min)) return TargetAge::Expired;
  if (at_least(2min)) return TargetAge::Lost;
  if (at_least(60s)) return TargetAge::Stale;
  if (at_least(15s)) return TargetAge::Aging;
  return TargetAge::Live;
}
TargetCache::Entry *TargetCache::FindOrInsert(int mmsi, vessel::Time at) {
  auto existing = targets_.find(mmsi);
  if (existing != targets_.end()) return &existing->second;
  for (auto it = targets_.begin(); it != targets_.end();) {
    const auto last = std::max(it->second.position_time, it->second.static_time);
    if (Age(last, at) == TargetAge::Expired) it = targets_.erase(it); else ++it;
  }
  if (targets_.size() >= Capacity) return nullptr;
  auto &entry = targets_[mmsi];
  entry.target.mmsi = mmsi; entry.target.source = OnlineSource;
  entry.target.origin = vessel::AisOrigin::AisStreamOnline;
  return &entry;
}
bool TargetCache::Observe(const PositionReport &r, vessel::Time received) {
  if (!Mmsi(r.mmsi) || !Stamp(r.observed_at, received) ||
      !Number(r.latitude, -90, 90) || !Number(r.longitude, -180, 180) ||
      !OptionalNumber(r.sog, 0, 102.2) || !OptionalNumber(r.cog, 0, 359.999999) ||
      !OptionalNumber(r.heading, 0, 359) || !OptionalNumber(r.navigation_status, 0, 14)) return false;
  auto *entry = FindOrInsert(r.mmsi, received);
  if (!entry || (entry->has_position && r.observed_at <= entry->position_time)) return false;
  auto &target = entry->target;
  target.active = true; target.lost = false;
  target.time_basis = r.receipt_time_only ? vessel::AisTimeBasis::OnlineReceipt : vessel::AisTimeBasis::OnlineService;
  target.latitude_deg = Sample(r.latitude, r.observed_at);
  target.longitude_deg = Sample(r.longitude, r.observed_at);
  target.sog_kn = Sample(r.sog, r.observed_at); target.cog_deg = Sample(r.cog, r.observed_at);
  target.heading_true_deg = Sample(r.heading, r.observed_at);
  target.navigation_status = Sample(r.navigation_status, r.observed_at);
  target.observed_at = entry->position_time = r.observed_at;
  entry->has_position = true;
  return true;
}
bool TargetCache::Observe(const StaticReport &r, vessel::Time received) {
  if (!Mmsi(r.mmsi) || !Stamp(r.observed_at, received) || !Text(r.name, 128) ||
      !Text(r.callsign, 32) || !Text(r.destination, 128) ||
      !OptionalNumber(r.ship_type, 0, 99) || !OptionalNumber(r.length_m, 0, 1022) ||
      !OptionalNumber(r.beam_m, 0, 126)) return false;
  auto *entry = FindOrInsert(r.mmsi, received);
  if (!entry || (entry->has_static && r.observed_at < entry->static_time)) return false;
  auto &target = entry->target;
  if (r.name) target.name = *r.name;
  if (r.callsign) target.callsign = StaticText(r.callsign, r.observed_at);
  if (r.destination) target.destination = StaticText(r.destination, r.observed_at);
  const auto field = [&](vessel::Sample &out, std::optional<double> value) {
    if (value) { out = Sample(value, r.observed_at); out.freshness = {1h, 6h}; }
  };
  field(target.ship_type, r.ship_type); field(target.length_m, r.length_m); field(target.beam_m, r.beam_m);
  entry->static_time = r.observed_at; entry->has_static = true;
  return true;
}
vessel::AisState TargetCache::Read(vessel::Time now) const {
  vessel::AisState state;
  state.source = OnlineSource; state.available = true;
  // Never set observation time to the UI read time.
  for (const auto &[id, entry] : targets_) {
    if (!entry.has_position) continue; // static metadata does not establish a vessel position
    const auto age = Age(entry.position_time, now);
    if (age == TargetAge::Expired || age == TargetAge::Invalid) continue;
    auto target = entry.target;
    target.lost = age == TargetAge::Lost;
    target.active = !target.lost;
    state.observed_at = std::max(state.observed_at, entry.position_time);
    state.targets.push_back(std::move(target));
  }
  return state;
}
} // namespace opennav::ais
