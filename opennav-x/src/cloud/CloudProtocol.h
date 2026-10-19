#pragma once
// SKAGER Cloud wire format (SCRUM-44, SCRUM-369). Pure functions: no network,
// credential or UI access. The server contract is web/edge/cloud.ts; the key
// and range table below must stay identical to TELEMETRY_KEYS there.
#include "vessel/VesselState.h"
#include <chrono>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace opennav::cloud {

inline constexpr const char *kOrigin = "https://beta.skager.app";
// Upload cadence for the latest snapshot; the server keeps one sample a minute.
inline constexpr std::chrono::seconds kTelemetryInterval{15};
// A device token as issued by pairing: "skg1_" + 64 lowercase hex.
bool ValidToken(std::string_view token);
// What a person reads aloud or types: "BCDF-2345".
std::string FormatUserCode(const std::string &code);

struct TelemetryKey {
  const char *key;
  double min, max;
  const vessel::Sample &(*sample)(const vessel::VesselState &);
};
const std::vector<TelemetryKey> &TelemetryKeys();

// {"items":{"navigation.sog":{"v":5.2,"age":0.4,"q":"live","src":"GPS"},...}}
// Only measured values with a source are sent, never simulated or replayed
// state, nothing older than a day and nothing outside the documented range.
// Returns nothing when no value qualifies: missing data is never sent as zero.
std::optional<std::string> EncodeTelemetry(const vessel::VesselState &state,
                                           vessel::Time now);

struct LinkOffer {
  std::string device_code, user_code, verification_url;
  std::chrono::seconds expires_in{600}, interval{5};
};
std::optional<LinkOffer> ParseLinkOffer(const std::string &body);
std::string EncodeLinkRequest(const std::string &device_name,
                              const std::string &platform,
                              const std::string &version);
std::string EncodePoll(const std::string &device_code);

enum class PollState { Pending, Approved, Denied, Expired, SlowDown, Invalid };
struct PollResult {
  PollState state = PollState::Invalid;
  std::string token;  // Approved only.
};
PollResult ParsePoll(const std::string &body);

struct AccountStatus {
  std::string email, device_name;
  bool cloud_active = false;
  std::string cloud_status;  // Stripe subscription status, empty without one.
  std::string renews_at;     // ISO time of the current period end, if any.
  bool cancel_at_period_end = false;
};
std::optional<AccountStatus> ParseStatus(const std::string &body);
}  // namespace opennav::cloud
