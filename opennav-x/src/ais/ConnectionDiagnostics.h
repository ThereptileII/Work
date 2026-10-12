#pragma once
#include "ais/Provider.h"
#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <ratio>
#include <string>
#include <string_view>
#include <type_traits>

namespace opennav::ais {
// Explicit process-environment opt-in. The default diagnostics contain no tuple.
inline bool ConnectionDiagnosticsEnabled(std::string_view value) {
  return value == "1";
}
enum class ConnectionDiagnosticReason { Observed, NotConnected, InvalidObservation };
inline const char *ConnectionDiagnosticReasonName(ConnectionDiagnosticReason reason) {
  switch (reason) {
  case ConnectionDiagnosticReason::Observed: return "observed";
  case ConnectionDiagnosticReason::NotConnected: return "not-connected";
  case ConnectionDiagnosticReason::InvalidObservation: return "invalid-observation";
  }
  return "invalid-observation";
}
struct DiagnosticEndpoint {
  std::string address; // exactly 32 lowercase network-byte hex digits
  std::uint16_t port;
  std::uint32_t scope;
};
struct DiagnosticConnectionObservation {
  std::string family, generation, captured_monotonic_ns, observed_monotonic_ns, age_ms;
  DiagnosticEndpoint local, remote;
};
struct ConnectionDiagnostic {
  ConnectionDiagnosticReason reason = ConnectionDiagnosticReason::InvalidObservation;
  // Unavailable observations never carry even a zero-valued endpoint tuple.
  std::optional<DiagnosticConnectionObservation> observation;
};
namespace connection_diagnostics_detail {
inline bool UnicastV4(const std::uint8_t *address) {
  return address[0] != 0 && address[0] < 224;
}
inline bool ValidEndpoint(const ConnectionEndpoint &endpoint, AddressFamily family) {
  if (!endpoint.port) return false;
  const auto &a = endpoint.address;
  if (family == AddressFamily::IPv4)
    return endpoint.scope == 0 && UnicastV4(a.data()) &&
        std::all_of(a.begin() + 4, a.end(), [](std::uint8_t n) { return n == 0; });
  if (family != AddressFamily::IPv6 || a[0] == 0xff ||
      std::all_of(a.begin(), a.end(), [](std::uint8_t n) { return n == 0; }))
    return false;
  const bool mapped = std::all_of(a.begin(), a.begin() + 10,
      [](std::uint8_t n) { return n == 0; }) && a[10] == 0xff && a[11] == 0xff;
  return !mapped; // explicit family only; mapped IPv4 is ambiguous here
}
inline DiagnosticEndpoint CopyEndpoint(const ConnectionEndpoint &endpoint) {
  static constexpr char hex[] = "0123456789abcdef";
  std::string address(32, '0');
  for (std::size_t i = 0; i < endpoint.address.size(); ++i) {
    address[i * 2] = hex[endpoint.address[i] >> 4];
    address[i * 2 + 1] = hex[endpoint.address[i] & 15];
  }
  return {address, endpoint.port, endpoint.scope};
}
template <class Tick>
inline std::optional<std::uint64_t> DurationNs(Tick duration) {
  // Check before scaling: MSVC steady_clock can use a coarser tick than ns.
  using Scale = std::ratio_divide<typename Tick::period, std::nano>;
  static_assert(std::is_integral<typename Tick::rep>::value, "integral steady clock required");
  static_assert(sizeof(typename Tick::rep) <= sizeof(std::uint64_t), "bounded steady clock required");
  const auto ticks = duration.count();
  if constexpr (std::is_signed<typename Tick::rep>::value)
    if (ticks < 0) return {};
  const auto count = static_cast<std::uint64_t>(ticks);
  constexpr auto limit = std::numeric_limits<std::uint64_t>::max();
  constexpr auto numerator = static_cast<std::uint64_t>(Scale::num);
  constexpr auto denominator = static_cast<std::uint64_t>(Scale::den);
  const auto quotient = count / denominator, remainder = count % denominator;
  if (quotient > limit / numerator || remainder > limit / numerator) return {};
  const auto whole = quotient * numerator, fraction = remainder * numerator / denominator;
  if (whole > limit - fraction) return {};
  return whole + fraction;
}
inline std::optional<std::uint64_t> MonotonicNs(vessel::Time time) {
  return DurationNs(time.time_since_epoch());
}
} // namespace connection_diagnostics_detail

// Pure read-only projection of owned scalar values. Neither clock is refreshed
// in the provider; age describes connection capture age, never traffic liveness.
inline ConnectionDiagnostic ReadConnectionDiagnostic(
    const ConnectionObservation &connection, const ProviderHealth &health,
    vessel::Time now) {
  using namespace connection_diagnostics_detail;
  if (health.connection != Connection::Connected && health.connection != Connection::Subscribing)
    return {ConnectionDiagnosticReason::NotConnected, {}};
  if ((health.connection == Connection::Connected &&
       (!health.subscription_confirmed || health.subscription_awaiting_confirmation)) ||
      (health.connection == Connection::Subscribing && health.subscription_confirmed) ||
      health.state_since <= vessel::Time{} || health.state_since > now ||
      connection.captured_at > health.state_since || !connection.generation ||
      connection.captured_at <= vessel::Time{} || connection.captured_at > now ||
      !ValidEndpoint(connection.local, connection.family) ||
      !ValidEndpoint(connection.remote, connection.family))
    return {};
  const auto captured = MonotonicNs(connection.captured_at), observed = MonotonicNs(now);
  if (!captured || !observed || *captured > *observed) return {};
  return {ConnectionDiagnosticReason::Observed,
      DiagnosticConnectionObservation{
          connection.family == AddressFamily::IPv4 ? "ipv4" : "ipv6",
          std::to_string(connection.generation), std::to_string(*captured),
          std::to_string(*observed), std::to_string((*observed - *captured) / 1000000),
          CopyEndpoint(connection.local), CopyEndpoint(connection.remote)}};
}
} // namespace opennav::ais
