#include "ais/ConnectionDiagnostics.h"
#include <iostream>
#include <stdexcept>
using namespace opennav;
using namespace std::chrono_literals;
namespace {
int checks = 0;
#define CHECK(value) do { ++checks; if (!(value)) throw std::runtime_error(\
    "connection diagnostic check at line " + std::to_string(__LINE__)); } while (false)
const auto captured = vessel::Time{} + 100h;
ais::ConnectionObservation V4() {
  return {ais::AddressFamily::IPv4, {{{192, 168, 1, 4}}, 45000, 0},
          {{{8, 8, 8, 8}}, 443, 0}, 7, captured};
}
ais::ProviderHealth Connected() {
  ais::ProviderHealth health;
  health.connection = ais::Connection::Connected;
  health.subscription_confirmed = true;
  health.state_since = captured + 1s;
  return health;
}
void CheckInvalid(const ais::ConnectionObservation &connection,
                  const ais::ProviderHealth &health = Connected(),
                  vessel::Time now = captured + 2s) {
  const auto result = ais::ReadConnectionDiagnostic(connection, health, now);
  CHECK(!result.observation);
  CHECK(result.reason == ais::ConnectionDiagnosticReason::InvalidObservation);
}
void ValidCopies() {
  auto input = V4();
  const auto health = Connected();
  const auto first = ais::ReadConnectionDiagnostic(input, health, captured + 2s + 999us);
  CHECK(first.reason == ais::ConnectionDiagnosticReason::Observed);
  CHECK(first.observation);
  const auto &value = *first.observation;
  CHECK(value.family == "ipv4" && value.generation == "7");
  CHECK(value.local.address == "c0a80104000000000000000000000000");
  CHECK(value.remote.address == "08080808000000000000000000000000");
  CHECK(value.local.port == 45000 && value.remote.port == 443);
  CHECK(value.local.scope == 0 && value.remote.scope == 0);
  CHECK(value.captured_monotonic_ns == "360000000000000");
  CHECK(value.observed_monotonic_ns == "360002000999000" && value.age_ms == "2000");
  input.local.address = {};
  CHECK(value.local.address == "c0a80104000000000000000000000000"); // owned DTO
  input = V4();
  const auto later = ais::ReadConnectionDiagnostic(input, health, captured + 24h);
  CHECK(later.observation->captured_monotonic_ns == value.captured_monotonic_ns);
  CHECK(later.observation->age_ms == "86400000"); // capture age is not liveness
  CHECK(input.captured_at == captured); // projection never refreshes capture
  auto subscribing = health;
  subscribing.connection = ais::Connection::Subscribing;
  subscribing.subscription_confirmed = false;
  subscribing.subscription_awaiting_confirmation = true;
  CHECK(ais::ReadConnectionDiagnostic(input, subscribing, captured + 2s).observation);
  subscribing.subscription_awaiting_confirmation = false;
  subscribing.subscription_pending = true; // Open before first send
  CHECK(ais::ReadConnectionDiagnostic(input, subscribing, captured + 2s).observation);
  input.generation = std::numeric_limits<std::uint64_t>::max();
  CHECK(ais::ReadConnectionDiagnostic(input, health, captured + 2s).observation->generation ==
      "18446744073709551615");
}
void InvalidAndTeardown() {
  for (auto state : {ais::Connection::Disabled, ais::Connection::CredentialMissing,
       ais::Connection::Connecting, ais::Connection::Backoff, ais::Connection::Offline}) {
    auto health = Connected(); health.connection = state;
    const auto result = ais::ReadConnectionDiagnostic(V4(), health, captured + 2s);
    CHECK(!result.observation && result.reason == ais::ConnectionDiagnosticReason::NotConnected);
  }
  auto health = Connected(); health.subscription_confirmed = false;
  CheckInvalid(V4(), health);
  health = Connected(); health.subscription_awaiting_confirmation = true;
  CheckInvalid(V4(), health);
  health = Connected(); health.connection = ais::Connection::Subscribing;
  CheckInvalid(V4(), health);
  health = Connected(); health.state_since = captured + 3s; CheckInvalid(V4(), health);
  health = Connected(); health.state_since = {}; CheckInvalid(V4(), health);
  auto input = V4(); input.generation = 0; CheckInvalid(input);
  input = V4(); input.family = ais::AddressFamily::Unavailable; CheckInvalid(input);
  input = V4(); input.captured_at = {}; CheckInvalid(input);
  input = V4(); input.captured_at = vessel::Time{} - 1s; CheckInvalid(input);
  input = V4(); input.captured_at = captured + 3s; CheckInvalid(input);
  CheckInvalid(V4(), Connected(), vessel::Time{} - 1s);
  for (bool remote : {false, true}) {
    for (int first : {0, 224, 239, 255}) {
      input = V4(); (remote ? input.remote : input.local).address[0] = first; CheckInvalid(input);
    }
    input = V4(); (remote ? input.remote : input.local).port = 0; CheckInvalid(input);
    input = V4(); (remote ? input.remote : input.local).scope = 1; CheckInvalid(input);
    for (std::size_t i = 4; i < 16; ++i) {
      input = V4(); (remote ? input.remote : input.local).address[i] = 1; CheckInvalid(input);
    }
  }
  CheckInvalid({}); // no zero endpoint fabrication
}
void V6AndLimits() {
  auto input = V4(); input.family = ais::AddressFamily::IPv6;
  input.local = {{{0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}}, 65535, 12};
  input.remote = {{{0x20, 1, 0x48, 0x60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x88, 0x88}}, 443, 0};
  const auto valid = ais::ReadConnectionDiagnostic(input, Connected(), captured + 2s);
  CHECK(valid.observation && valid.observation->family == "ipv6");
  CHECK(valid.observation->local.address == "fe800000000000000000000000000001");
  CHECK(valid.observation->local.scope == 12 && valid.observation->local.port == 65535);
  for (bool remote : {false, true}) {
    auto invalid = input; (remote ? invalid.remote : invalid.local).address = {}; CheckInvalid(invalid);
    invalid = input; (remote ? invalid.remote : invalid.local).address[0] = 0xff; CheckInvalid(invalid);
    invalid = input;
    (remote ? invalid.remote : invalid.local).address = {{0,0,0,0,0,0,0,0,0,0,0xff,0xff,8,8,8,8}};
    CheckInvalid(invalid);
  }
  const auto limit = ais::connection_diagnostics_detail::MonotonicNs(vessel::Time::max());
  const auto at_limit = ais::ReadConnectionDiagnostic(V4(), Connected(), vessel::Time::max());
  CHECK(static_cast<bool>(limit) == static_cast<bool>(at_limit.observation));
  if (at_limit.observation) {
    CHECK(at_limit.observation->observed_monotonic_ns == std::to_string(*limit));
    CHECK(at_limit.observation->age_ms == std::to_string((*limit - 360000000000000ULL) / 1000000));
  }
  CHECK(!ais::connection_diagnostics_detail::MonotonicNs(vessel::Time::min()));
  // Exercise the MSVC-style 100 ns conversion even on Linux's ns clock.
  using WindowsTicks = std::chrono::duration<std::int64_t, std::ratio<1, 10000000>>;
  const auto safe_ticks = std::numeric_limits<std::uint64_t>::max() / 100;
  CHECK(ais::connection_diagnostics_detail::DurationNs(WindowsTicks(safe_ticks)) == safe_ticks * 100);
  CHECK(!ais::connection_diagnostics_detail::DurationNs(WindowsTicks(safe_ticks + 1)));
  CHECK(!ais::connection_diagnostics_detail::DurationNs(WindowsTicks(-1)));
  using FineTicks = std::chrono::duration<std::int64_t, std::pico>;
  CHECK(ais::connection_diagnostics_detail::DurationNs(FineTicks(1999)) == 1);
  auto loopback = V4(); loopback.local.address = {{127, 0, 0, 1}};
  loopback.remote.address = {{127, 0, 0, 1}};
  CHECK(ais::ReadConnectionDiagnostic(loopback, Connected(), captured + 2s).observation);
}
} // namespace
int main() {
  try {
    for (const char *value : {"", "0", "true", "yes", "01", "1 ", " 1", "2"})
      CHECK(!ais::ConnectionDiagnosticsEnabled(value));
    CHECK(ais::ConnectionDiagnosticsEnabled("1"));
    ValidCopies(); InvalidAndTeardown(); V6AndLimits();
    CHECK(std::string(ais::ConnectionDiagnosticReasonName(ais::ConnectionDiagnosticReason::Observed)) == "observed");
    CHECK(std::string(ais::ConnectionDiagnosticReasonName(ais::ConnectionDiagnosticReason::NotConnected)) == "not-connected");
    CHECK(std::string(ais::ConnectionDiagnosticReasonName(ais::ConnectionDiagnosticReason::InvalidObservation)) == "invalid-observation");
    std::cout << checks << " AIS connection diagnostic checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
