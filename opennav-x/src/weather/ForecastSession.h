#pragma once
// Network-free forecast lifecycle (SCRUM-325/326/327): fetch scheduling,
// backoff (incl. HTTP 429 Retry-After), credential failures, bounded cache and
// Live/Stale/discard freshness. Time is injected: steady for scheduling, wall
// for data age. Not thread-safe; the service guards it with one mutex.
#include "weather/Weather.h"
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace opennav::weather {
using Steady = std::chrono::steady_clock::time_point;

// Refresh an unchanged area at most this often (quota protection).
constexpr std::chrono::minutes kRefreshEvery{30};
// A changed area (pan/zoom/route) may refetch no sooner than this.
constexpr std::chrono::minutes kAreaChangeMinimum{2};
constexpr std::chrono::minutes kMaxBackoff{30};
constexpr std::chrono::minutes kDefaultRateLimit{15};

// Transport outcome. The body is untrusted and never copied into a status.
struct HttpResult {
  bool transport_ok = false;  // false: DNS/connect/TLS/timeout/cancel.
  bool too_large = false;     // Body exceeded the bound and was abandoned.
  int status = 0;
  std::optional<std::chrono::seconds> retry_after;
  std::string body;
};
// RFC 9110 delta-seconds only (HTTP-date is ignored -> default backoff).
std::optional<std::chrono::seconds> ParseRetryAfter(std::string_view value);

class ForecastSession {
public:
  struct Request {
    std::uint64_t id = 0;
    std::string url, body;
    bool test = false;
  };
  void SetEnabled(bool enabled);
  void SetCredentialPresent(bool present);  // User stored/removed a token.
  void SetModel(std::string model);
  void SetQuery(const ForecastQuery &query, WallTime now);
  void RequestTest();  // User pressed Test connection.
  // Returns the request to perform now, if any, and marks it in flight.
  std::optional<Request> Next(Steady now, WallTime wall);
  // The worker could not read a token for the in-flight request.
  void CredentialMissing(std::uint64_t id, bool unreadable);
  void Complete(std::uint64_t id, const HttpResult &result, Steady now, WallTime wall);
  // Frees cache older than kCacheFor.
  void Prune(WallTime wall);
  ForecastSnapshot Snapshot(WallTime now) const;
  ConnectionTest LastTest() const { return last_test_; }
  bool TestPending() const { return test_pending_ || (in_flight_ && in_flight_test_); }
  bool Enabled() const { return enabled_; }

private:
  struct Cache {
    std::vector<ForecastWind> winds;
    WallTime fetched_at{};
    std::string model;
  };
  void Fail(std::string message, Steady now, bool test, std::chrono::seconds delay);
  bool enabled_ = false, credential_ = false, auth_rejected_ = false;
  bool test_pending_ = false, in_flight_ = false, in_flight_test_ = false;
  bool in_flight_probe_ = false;
  std::uint64_t next_id_ = 0, in_flight_id_ = 0;
  std::string model_{"gfs"};
  std::optional<ForecastQuery> query_, in_flight_query_;
  std::string area_key_, fetched_key_, fetched_model_;
  std::optional<Steady> last_attempt_, last_success_;
  Steady not_before_{}, rate_limited_until_{};
  unsigned failures_ = 0;
  std::optional<Cache> cache_;
  std::string error_;  // Last failure, credential-free.
  ConnectionTest last_test_;
};
} // namespace opennav::weather
