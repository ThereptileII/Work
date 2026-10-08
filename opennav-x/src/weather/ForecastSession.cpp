#include "weather/ForecastSession.h"
#include "weather/GribStream.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace opennav::weather {
namespace {
using std::chrono::seconds;
std::string Minutes(std::chrono::seconds value) {
  const auto minutes = std::max<long long>(0, (value.count() + 59) / 60);
  return std::to_string(minutes) + " min";
}
std::string Age(std::chrono::seconds age) {
  const long long minutes = std::max<long long>(0, age.count() / 60);
  if (minutes < 60) return std::to_string(minutes) + " min";
  return std::to_string(minutes / 60) + " h " + std::to_string(minutes % 60) + " min";
}
// Area identity at forecast-grid resolution (0.25°): a vessel moving inside
// one model cell, or a sub-cell chart pan, does not count as a new area.
std::string CoarseKey(const ForecastQuery &q) {
  std::vector<std::pair<long long, long long>> cells;
  for (const auto &p : q.points)
    cells.emplace_back(std::llround(p.latitude_deg * 4), std::llround(p.longitude_deg * 4));
  std::sort(cells.begin(), cells.end());
  cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
  std::string key;
  for (const auto &c : cells) key += std::to_string(c.first) + "," + std::to_string(c.second) + ";";
  return key;
}
} // namespace

std::optional<std::chrono::seconds> ParseRetryAfter(std::string_view value) {
  while (!value.empty() && value.front() == ' ') value.remove_prefix(1);
  while (!value.empty() && (value.back() == ' ' || value.back() == '\r')) value.remove_suffix(1);
  if (value.empty() || value.size() > 9) return std::nullopt;
  long long total = 0;
  for (char c : value) {
    if (c < '0' || c > '9') return std::nullopt;
    total = total * 10 + (c - '0');
  }
  return std::chrono::seconds(total);
}

void ForecastSession::SetEnabled(bool enabled) { enabled_ = enabled; }
void ForecastSession::SetCredentialPresent(bool present) {
  credential_ = present;
  // A new or removed token clears a previous rejection and failure backoff.
  auth_rejected_ = false;
  failures_ = 0;
  not_before_ = {};
  error_.clear();
  if (!present) cache_.reset();
}
void ForecastSession::SetModel(std::string model) {
  if (gribstream::ValidModel(model)) model_ = std::move(model);
}
void ForecastSession::SetQuery(const ForecastQuery &query, WallTime now) {
  query_ = gribstream::Clamp(query, now);
  area_key_ = query_ ? CoarseKey(*query_) : std::string{};
}
void ForecastSession::RequestTest() { test_pending_ = true; }

std::optional<ForecastSession::Request> ForecastSession::Next(Steady now, WallTime wall) {
  if (in_flight_) return std::nullopt;
  if (now < rate_limited_until_) {
    if (test_pending_) {
      test_pending_ = false;
      last_test_ = {false, "GRIBstream quota reached; try again in about " +
                               Minutes(std::chrono::duration_cast<seconds>(rate_limited_until_ - now))};
    }
    return std::nullopt;
  }
  std::optional<ForecastQuery> query;
  bool test = false, probe = false;
  if (test_pending_) {
    test_pending_ = false;
    if (!credential_) {
      last_test_ = {false, "No GRIBstream API token stored"};
      return std::nullopt;
    }
    test = true;
    if (query_) query = gribstream::Clamp(*query_, wall);
    if (!query) {
      // Smallest possible request; the result is never cached or displayed.
      ForecastQuery single;
      single.points = {{0.0, 0.0}};
      single.from = wall;
      single.until = wall + std::chrono::hours(1);
      query = gribstream::Clamp(single, wall);
      probe = true;
    }
  } else {
    if (!enabled_ || !credential_ || auth_rejected_ || !query_ || now < not_before_)
      return std::nullopt;
    bool due = true;
    if (last_success_) {
      if (fetched_key_ != area_key_ || fetched_model_ != model_)
        due = !last_attempt_ || now - *last_attempt_ >= kAreaChangeMinimum;
      else
        due = now - *last_success_ >= kRefreshEvery;
    }
    if (!due) return std::nullopt;
    query = gribstream::Clamp(*query_, wall);
  }
  if (!query) return std::nullopt;
  Request r;
  r.id = ++next_id_;
  r.url = gribstream::TimeseriesUrl(model_);
  r.body = gribstream::RequestBody(*query);
  r.test = test;
  in_flight_ = true;
  in_flight_id_ = r.id;
  in_flight_test_ = test;
  in_flight_probe_ = probe;
  in_flight_query_ = std::move(query);
  last_attempt_ = now;
  return r;
}

void ForecastSession::CredentialMissing(std::uint64_t id, bool unreadable) {
  if (!in_flight_ || id != in_flight_id_) return;
  in_flight_ = false;
  credential_ = false;
  cache_.reset();
  error_ = unreadable ? "GRIBstream token storage unavailable" : "No GRIBstream API token stored";
  if (in_flight_test_) last_test_ = {false, error_};
}

void ForecastSession::Fail(std::string message, Steady now, bool test,
                           std::chrono::seconds delay) {
  error_ = std::move(message);
  ++failures_;
  not_before_ = now + delay;
  if (test) last_test_ = {false, error_};
}

void ForecastSession::Complete(std::uint64_t id, const HttpResult &result, Steady now,
                               WallTime wall) {
  if (!in_flight_ || id != in_flight_id_) return;
  in_flight_ = false;
  const bool test = in_flight_test_, probe = in_flight_probe_;
  const auto query = std::move(in_flight_query_);
  in_flight_query_.reset();
  const auto backoff = [this] {
    const unsigned shift = std::min(failures_, 5u);  // 1,2,4,8,16,30 min
    return std::min<seconds>(seconds(60LL << shift), kMaxBackoff);
  };
  if (!result.transport_ok) {
    Fail(result.too_large ? "GRIBstream response too large; discarded"
                          : "GRIBstream unreachable (network, TLS or timeout)",
         now, test, backoff());
    return;
  }
  const int status = result.status;
  if (status == 200) {
    auto parsed = gribstream::ParseCsv(result.body, *query);
    if (!parsed.ok) {
      Fail(parsed.error, now, test, backoff());
      return;
    }
    failures_ = 0;
    auth_rejected_ = false;
    not_before_ = {};
    error_.clear();
    if (test) last_test_ = {true, "Connection OK; GRIBstream forecast received"};
    if (probe) return;
    cache_ = Cache{std::move(parsed.winds), wall, model_};
    fetched_key_ = CoarseKey(*query);
    fetched_model_ = model_;
    last_success_ = now;
    return;
  }
  if (status == 401 || status == 403) {
    auth_rejected_ = true;
    Fail(status == 401
             ? "GRIBstream rejected the API token (invalid or expired); update it in Settings"
             : "GRIBstream refused this token (HTTP 403); check the account plan or permissions",
         now, test, kMaxBackoff);
    return;
  }
  if (status == 429) {
    auto delay = result.retry_after.value_or(kDefaultRateLimit);
    delay = std::clamp<seconds>(delay, seconds(60), std::chrono::hours(6));
    rate_limited_until_ = now + delay;
    Fail("GRIBstream quota reached; next attempt in about " + Minutes(delay), now, test, delay);
    return;
  }
  if (status >= 400 && status < 500) {
    Fail("GRIBstream rejected the forecast request (HTTP " + std::to_string(status) + ")",
         now, test, kMaxBackoff);
    return;
  }
  if (status >= 500 && status < 600) {
    Fail("GRIBstream service error (HTTP " + std::to_string(status) + "); retrying later",
         now, test, backoff());
    return;
  }
  Fail("Unexpected GRIBstream response (HTTP " + std::to_string(status) + ")", now, test,
       backoff());
}

void ForecastSession::Prune(WallTime wall) {
  if (cache_ && wall - cache_->fetched_at >= kCacheFor) cache_.reset();
}

ForecastSnapshot ForecastSession::Snapshot(WallTime now) const {
  ForecastSnapshot s;
  s.model = cache_ ? cache_->model : model_;
  if (!enabled_) {
    s.state = ForecastState::Disabled;
    s.status = "Weather forecast off";
    return s;
  }
  if (!credential_) {
    s.state = ForecastState::NoCredential;
    s.status = error_.empty() ? "Add your GRIBstream API token in Settings" : error_;
    return s;
  }
  const std::string problem = error_.empty() ? std::string{} : "; last update failed: " + error_;
  if (cache_) {
    const auto age = std::chrono::duration_cast<seconds>(now - cache_->fetched_at);
    if (age < kCacheFor) {
      s.fetched_at = cache_->fetched_at;
      s.winds = cache_->winds;
      for (const auto &w : s.winds)
        if (!s.model_run || *s.model_run < w.model_run) s.model_run = w.model_run;
      if (age.count() < 0) {
        s.state = ForecastState::Stale;
        s.status = "Forecast fetch time is ahead of the system clock; treat as historical" + problem;
      } else if (age < kLiveFor) {
        s.state = ForecastState::Live;
        s.status = "Forecast fetched " + Age(age) + " ago" + problem;
      } else {
        s.state = ForecastState::Stale;
        s.status = "STALE forecast fetched " + Age(age) + " ago; historical, not current" + problem;
      }
      return s;
    }
  }
  if (!error_.empty()) {
    s.state = ForecastState::Error;
    s.status = error_;
  } else {
    s.state = ForecastState::Loading;
    s.status = query_ ? "Fetching forecast" : "Waiting for a position, route or chart area";
  }
  return s;
}
} // namespace opennav::weather
