// SCRUM-324..331 weather provider: codec, lifecycle, query bounds, worker.
#include "weather/ForecastService.h"
#include "weather/ForecastSession.h"
#include "weather/GribStream.h"
#include "weather/QueryBuilder.h"
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

using namespace opennav;
using namespace opennav::weather;
using namespace std::chrono_literals;
namespace gs = opennav::weather::gribstream;

int checks = 0;
#define CHECK(v)                                                                   \
  do {                                                                             \
    ++checks;                                                                      \
    if (!(v))                                                                      \
      throw std::runtime_error("weather check at line " + std::to_string(__LINE__)); \
  } while (false)

namespace {
const WallTime wall0 = *gs::ParseUtc("2026-10-08T12:20:00Z");
const Steady steady0 = Steady{} + 1000h;
const std::string kToken = "SECRET-gribstream-TOKEN-0123456789";

bool Near(double a, double b, double eps = 1e-6) { return std::abs(a - b) <= eps; }
bool Contains(const std::string &s, const std::string &part) {
  return s.find(part) != std::string::npos;
}
ForecastQuery Query(std::vector<Coordinate> points) {
  ForecastQuery q;
  q.points = std::move(points);
  q.from = wall0;
  q.until = wall0 + 48h;
  return q;
}
// CSV for clamped query points p0..pN at from+2h and from+1h.
std::string Csv(const ForecastQuery &q) {
  std::string csv = "forecasted_at,forecasted_time,lat,lon,name,u,v\n";
  for (int h : {2, 1})  // Deliberately unordered.
    for (std::size_t i = 0; i < q.points.size(); ++i)
      csv += "2026-10-08T06:00:00Z," + gs::FormatUtc(q.from + std::chrono::hours(h)) + "," +
             std::to_string(q.points[i].latitude_deg) + "," +
             std::to_string(q.points[i].longitude_deg) + ",p" + std::to_string(i) +
             ",0,-5\n";
  return csv;
}
HttpResult Ok(std::string body) {
  HttpResult r;
  r.transport_ok = true;
  r.status = 200;
  r.body = std::move(body);
  return r;
}
HttpResult Status(int status, std::optional<std::chrono::seconds> retry = std::nullopt) {
  HttpResult r;
  r.transport_ok = true;
  r.status = status;
  r.retry_after = retry;
  r.body = "{\"error\":\"echo " + kToken + "\"}";  // Untrusted; must never surface.
  return r;
}
void NoToken(const ForecastSnapshot &s) {
  CHECK(!Contains(s.status, kToken));
  CHECK(!Contains(s.status, "echo"));
}

void codec() {
  // U/V -> meteorological FROM direction.
  auto w = gs::FromUV(0, -10);  // Blowing toward south = from north.
  CHECK(w && Near(w->speed_mps, 10) && Near(w->direction_from_true_deg, 0));
  w = gs::FromUV(-10, 0);  // Toward west = from east.
  CHECK(w && Near(w->direction_from_true_deg, 90));
  w = gs::FromUV(0, 10);
  CHECK(w && Near(w->direction_from_true_deg, 180));
  w = gs::FromUV(10, 0);
  CHECK(w && Near(w->direction_from_true_deg, 270));
  w = gs::FromUV(3, 4);
  CHECK(w && Near(w->speed_mps, 5) && w->direction_from_true_deg >= 0 &&
        w->direction_from_true_deg < 360);
  w = gs::FromUV(-1e-7, -5);  // Just west of north must stay < 360.
  CHECK(w && w->direction_from_true_deg >= 0 && w->direction_from_true_deg < 360);
  w = gs::FromUV(0, 0);
  CHECK(w && w->speed_mps == 0 && w->direction_from_true_deg == 0);
  CHECK(!gs::FromUV(std::nan(""), 1));
  CHECK(!gs::FromUV(1, INFINITY));
  CHECK(!gs::FromUV(150, 0));
  CHECK(!gs::FromUV(80, 80));  // Speed > 100 m/s.

  // Strict UTC.
  CHECK(gs::ParseUtc("2026-10-08T12:00:00Z"));
  CHECK(gs::ParseUtc("2026-10-08T12:00:00+00:00") == gs::ParseUtc("2026-10-08T12:00:00Z"));
  CHECK(*gs::ParseUtc("2026-10-08T12:00:00.5Z") - *gs::ParseUtc("2026-10-08T12:00:00Z") == 500ms);
  CHECK(gs::ParseUtc("2024-02-29T00:00:00Z"));
  CHECK(!gs::ParseUtc("2026-02-29T00:00:00Z"));
  CHECK(!gs::ParseUtc("2026-10-08 12:00:00Z"));
  CHECK(!gs::ParseUtc("2026-10-08T12:00:00"));
  CHECK(!gs::ParseUtc("2026-10-08T12:00:00+02:00"));
  CHECK(!gs::ParseUtc("2026-10-08T24:00:00Z"));
  CHECK(!gs::ParseUtc("2026-10-08T12:00:00.Z"));
  CHECK(!gs::ParseUtc("2026-1O-08T12:00:00Z"));
  CHECK(gs::FormatUtc(wall0) == "2026-10-08T12:20:00Z");
  CHECK(gs::FormatUtc(std::chrono::system_clock::from_time_t(0)) == "1970-01-01T00:00:00Z");

  // Model validation and URL.
  CHECK(gs::TimeseriesUrl("gfs") == "https://gribstream.com/api/v2/gfs/timeseries");
  CHECK(!gs::ValidModel("gfs/../x") && !gs::ValidModel("") && !gs::ValidModel("GFS?a=1"));
  CHECK(gs::TimeseriesUrl("../evil") == "https://gribstream.com/api/v2/gfs/timeseries");

  // Clamp.
  ForecastQuery many = Query({});
  for (int i = 0; i < 100; ++i) many.points.push_back({50 + i * 0.1, 10});
  many.points.insert(many.points.begin(), {{std::nan(""), 1}, {91, 0}, {50, 10}, {50, 10.00001}});
  auto c = gs::Clamp(many, wall0);
  CHECK(c && c->points.size() == kMaxForecastPoints);
  CHECK(Near(c->points[0].latitude_deg, 50) && Near(c->points[1].latitude_deg, 50.1));
  CHECK(c->from == *gs::ParseUtc("2026-10-08T12:00:00Z"));
  CHECK(c->until - c->from == std::chrono::hours(kMaxForecastSteps - 1));
  auto wrap = gs::Clamp(Query({{10, 190}, {10, -180}, {10, 180}}), wall0);
  CHECK(wrap && wrap->points.size() == 2 && Near(wrap->points[0].longitude_deg, -170) &&
        Near(wrap->points[1].longitude_deg, -180));
  auto window = Query({{1, 1}});
  window.from = wall0 - 72h;  // History is not requested.
  window.until = wall0 + 200h;
  c = gs::Clamp(window, wall0);
  CHECK(c && c->from == *gs::ParseUtc("2026-10-08T11:00:00Z") &&
        c->until == c->from + std::chrono::hours(kMaxForecastSteps - 1));
  window.until = window.from;
  CHECK(!gs::Clamp(window, wall0));
  CHECK(!gs::Clamp(Query({{std::nan(""), 0}}), wall0));

  // Request JSON (locale-free fixed decimals, names p0..pN).
  auto q = *gs::Clamp(Query({{59.12345, 18.5}, {-0.5, -0.05}}), wall0);
  const auto body = gs::RequestBody(q);
  CHECK(Contains(body, "\"fromTime\":\"2026-10-08T12:00:00Z\""));
  CHECK(Contains(body, "\"untilTime\":\"2026-10-10T12:00:00Z\""));
  CHECK(Contains(body, "{\"lat\":59.1235,\"lon\":18.5000,\"name\":\"p0\"}"));
  CHECK(Contains(body, "{\"lat\":-0.5000,\"lon\":-0.0500,\"name\":\"p1\"}"));
  CHECK(Contains(body, "\"name\":\"UGRD\",\"level\":\"10 m above ground\",\"alias\":\"u\""));
  CHECK(Contains(body, "\"name\":\"VGRD\",\"level\":\"10 m above ground\",\"alias\":\"v\""));
  CHECK(!Contains(body, "e+") && !Contains(body, ",5000"));

  // CSV: unordered rows are sorted; malformed rows are skipped, not invented.
  q = *gs::Clamp(Query({{60, 18}, {59, 18}}), wall0);
  std::string csv =
      "forecasted_at,forecasted_time,lat,lon,name,u,v,native_lat,native_lon\r\n"
      "2026-10-08T06:00:00Z,2026-10-08T14:00:00Z,60.0,18.0,p0,0,-5,60.0,18.0\r\n"
      "2026-10-08T06:00:00Z,2026-10-08T13:00:00Z,59.0,18.0,p1,10,0,59.0,18.0\r\n"
      "2026-10-08T06:00:00Z,2026-10-08T13:00:00Z,60.0,18.0,p0,-10,0,60.0,18.0\r\n"
      "2026-10-08T00:00:00Z,2026-10-08T13:00:00Z,60.0,18.0,p0,1,1,60.0,18.0\r\n"  // older run
      "garbage\r\n"
      "2026-10-08T06:00:00Z,2026-10-08T13:00:00Z,60.0,18.0,p7,1,1,60.0,18.0\r\n"   // unknown name
      "2026-10-08T06:00:00Z,2026-10-08T13:00:00Z,60.0,18.0,p01,1,1,60.0,18.0\r\n"  // non-canonical
      "2026-10-08T06:00:00Z,2026-10-08T13:00:00Z,61.0,18.0,p0,1,1,60.0,18.0\r\n"   // wrong coordinate
      "2026-10-08T06:00:00Z,2026-10-08 13:00:00,60.0,18.0,p0,1,1,60.0,18.0\r\n"    // bad time
      "2026-10-08T06:00:00Z,2026-10-08T15:00:00Z,60.0,18.0,p0,nan,1,60.0,18.0\r\n" // NaN
      "2026-10-08T06:00:00Z,2026-10-08T15:00:00Z,60.0,18.0,p0,500,1,60.0,18.0\r\n" // implausible
      "2026-10-08T06:00:00Z,2026-10-20T15:00:00Z,60.0,18.0,p0,1,1,60.0,18.0\r\n"   // out of window
      "2026-10-08T06:00:00Z,2026-10-08T15:00:00Z,60.0,18.0,p0,1,1\r\n";            // short row
  auto parsed = gs::ParseCsv(csv, q);
  CHECK(parsed.ok && parsed.winds.size() == 3);
  CHECK(parsed.rejected_rows == 9);
  CHECK(parsed.winds[0].valid_time == *gs::ParseUtc("2026-10-08T13:00:00Z"));
  CHECK(Near(parsed.winds[0].requested.latitude_deg, 59));  // latitude order
  CHECK(Near(parsed.winds[0].direction_from_true_deg, 270));
  CHECK(Near(parsed.winds[1].requested.latitude_deg, 60));
  CHECK(Near(parsed.winds[1].direction_from_true_deg, 90));  // newest run kept
  CHECK(parsed.winds[1].model_run == *gs::ParseUtc("2026-10-08T06:00:00Z"));
  CHECK(parsed.winds[1].grid && Near(parsed.winds[1].grid->latitude_deg, 60));
  CHECK(parsed.winds[2].valid_time == *gs::ParseUtc("2026-10-08T14:00:00Z"));
  CHECK(Near(parsed.winds[2].direction_from_true_deg, 0));

  // Column order is taken from the header, native columns are optional.
  parsed = gs::ParseCsv("name,u,v,forecasted_time,forecasted_at\n"
                        "p1,0,5,2026-10-08T13:00:00Z,2026-10-08T06:00:00Z\n", q);
  CHECK(parsed.ok && parsed.winds.size() == 1 && !parsed.winds[0].grid &&
        Near(parsed.winds[0].direction_from_true_deg, 180));
  parsed = gs::ParseCsv("forecasted_at,forecasted_time,name,u\n", q);
  CHECK(!parsed.ok && Contains(parsed.error, "missing required columns"));
  parsed = gs::ParseCsv("", q);
  CHECK(!parsed.ok);
  parsed = gs::ParseCsv("forecasted_at,forecasted_time,name,u,v\n", q);
  CHECK(!parsed.ok && parsed.winds.empty());
  parsed = gs::ParseCsv("forecasted_at,forecasted_time,name,u,v\nx,y,p0,1,1\n", q);
  CHECK(!parsed.ok && parsed.rejected_rows == 1);
  CHECK(!gs::ParseCsv(std::string(gs::kMaxResponseBytes + 1, 'a'), q).ok);

  // Steps are bounded even if the provider returns more.
  ForecastQuery long_window = *gs::Clamp(Query({{60, 18}}), wall0);
  std::string many_steps = "forecasted_at,forecasted_time,name,u,v\n";
  for (int h = 0; h < 60; ++h)
    many_steps += "2026-10-08T06:00:00Z," +
                  gs::FormatUtc(long_window.from + std::chrono::hours(h)) + ",p0,1,1\n";
  parsed = gs::ParseCsv(many_steps, long_window);
  CHECK(parsed.ok && parsed.winds.size() <= kMaxForecastSteps);

  CHECK(ParseRetryAfter("120") == std::chrono::seconds(120));
  CHECK(ParseRetryAfter(" 30 ") == std::chrono::seconds(30));
  CHECK(!ParseRetryAfter("Wed, 21 Oct 2015 07:28:00 GMT"));
  CHECK(!ParseRetryAfter(""));
  CHECK(!ParseRetryAfter("-5"));
}

void session() {
  ForecastSession s;
  CHECK(s.Snapshot(wall0).state == ForecastState::Disabled);
  CHECK(!s.Next(steady0, wall0));
  s.SetEnabled(true);
  CHECK(s.Snapshot(wall0).state == ForecastState::NoCredential);
  CHECK(!s.Next(steady0, wall0));
  s.SetCredentialPresent(true);
  CHECK(s.Snapshot(wall0).state == ForecastState::Loading);
  CHECK(!s.Next(steady0, wall0));  // No query yet.

  const auto query = Query({{60, 18}, {59, 18}});
  const auto clamped = *gs::Clamp(query, wall0);
  s.SetQuery(query, wall0);
  auto r = s.Next(steady0, wall0);
  CHECK(r && !r->test && r->url == "https://gribstream.com/api/v2/gfs/timeseries");
  CHECK(Contains(r->body, "\"name\":\"p1\""));
  CHECK(!s.Next(steady0, wall0));  // One request in flight at most.
  s.Complete(r->id + 99, Ok(Csv(clamped)), steady0, wall0);  // Stale id ignored.
  CHECK(s.Snapshot(wall0).state == ForecastState::Loading);
  s.Complete(r->id, Ok(Csv(clamped)), steady0, wall0);
  auto snap = s.Snapshot(wall0 + 1min);
  CHECK(snap.state == ForecastState::Live && snap.winds.size() == 4);
  CHECK(snap.fetched_at == wall0 && snap.model == "gfs" && snap.provider == "GRIBstream");
  CHECK(snap.model_run == *gs::ParseUtc("2026-10-08T06:00:00Z"));
  CHECK(snap.winds[0].valid_time <= snap.winds.back().valid_time);

  // Same area: no refetch before 30 min.
  s.SetQuery(query, wall0 + 1min);
  CHECK(!s.Next(steady0 + 29min, wall0 + 29min));
  // Vessel drift within one 0.25° cell is the same area.
  s.SetQuery(Query({{60.05, 18.05}, {59, 18}}), wall0 + 1min);
  CHECK(!s.Next(steady0 + 3min, wall0 + 3min));
  r = s.Next(steady0 + 30min, wall0 + 30min);
  CHECK(r);

  // Outage keeps the last forecast; age decides Live vs Stale, never valid_time.
  s.Complete(r->id, HttpResult{}, steady0 + 30min, wall0 + 30min);
  snap = s.Snapshot(wall0 + 31min);
  CHECK(snap.state == ForecastState::Live && !snap.winds.empty());
  CHECK(Contains(snap.status, "unreachable"));
  CHECK(!s.Next(steady0 + 30min + 59s, wall0 + 31min));  // 60 s backoff.
  r = s.Next(steady0 + 31min + 1s, wall0 + 31min);
  CHECK(r);
  s.Complete(r->id, HttpResult{}, steady0 + 32min, wall0 + 32min);
  CHECK(!s.Next(steady0 + 33min + 59s, wall0 + 34min));  // 120 s backoff.
  snap = s.Snapshot(wall0 + kLiveFor - 1s);
  CHECK(snap.state == ForecastState::Live);
  snap = s.Snapshot(wall0 + kLiveFor);
  CHECK(snap.state == ForecastState::Stale && !snap.winds.empty());
  CHECK(Contains(snap.status, "STALE") && Contains(snap.status, "historical"));
  snap = s.Snapshot(wall0 + kCacheFor);
  CHECK(snap.winds.empty() && snap.state == ForecastState::Error && !snap.fetched_at);
  // Clock moved backwards: never claim Live.
  snap = s.Snapshot(wall0 - 10min);
  CHECK(snap.state == ForecastState::Stale);
  s.Prune(wall0 + kCacheFor);
  CHECK(s.Snapshot(wall0 + 1min).winds.empty());

  // 401 / 403: understandable, credential-free, no automatic retries.
  ForecastSession a;
  a.SetEnabled(true);
  a.SetCredentialPresent(true);
  a.SetQuery(query, wall0);
  r = a.Next(steady0, wall0);
  a.Complete(r->id, Status(401), steady0, wall0);
  snap = a.Snapshot(wall0);
  CHECK(snap.state == ForecastState::Error && Contains(snap.status, "invalid or expired"));
  NoToken(snap);
  CHECK(!a.Next(steady0 + 5h, wall0 + 5h));
  a.RequestTest();  // The user may retry explicitly.
  r = a.Next(steady0 + 5h, wall0 + 5h);
  CHECK(r && r->test && a.TestPending());
  a.Complete(r->id, Status(403), steady0 + 5h, wall0 + 5h);
  CHECK(!a.LastTest().ok && Contains(a.LastTest().message, "403"));
  CHECK(!Contains(a.LastTest().message, kToken));
  a.SetCredentialPresent(true);  // New token clears the rejection.
  r = a.Next(steady0 + 5h, wall0 + 5h);
  CHECK(r);
  a.Complete(r->id, Ok(Csv(*gs::Clamp(query, wall0 + 5h))), steady0 + 5h, wall0 + 5h);
  CHECK(a.Snapshot(wall0 + 5h).state == ForecastState::Live);
  a.SetCredentialPresent(false);  // Token removed.
  CHECK(a.Snapshot(wall0 + 5h).state == ForecastState::NoCredential);
  CHECK(a.Snapshot(wall0 + 5h).winds.empty());
  a.RequestTest();
  CHECK(!a.Next(steady0 + 6h, wall0 + 6h));
  CHECK(!a.LastTest().ok && Contains(a.LastTest().message, "No GRIBstream API token"));

  // 429 honours Retry-After, including for Test.
  ForecastSession b;
  b.SetEnabled(true);
  b.SetCredentialPresent(true);
  b.SetQuery(query, wall0);
  r = b.Next(steady0, wall0);
  b.Complete(r->id, Status(429, 600s), steady0, wall0);
  CHECK(Contains(b.Snapshot(wall0).status, "quota") && Contains(b.Snapshot(wall0).status, "10 min"));
  NoToken(b.Snapshot(wall0));
  CHECK(!b.Next(steady0 + 599s, wall0));
  b.RequestTest();
  CHECK(!b.Next(steady0 + 599s, wall0));
  CHECK(!b.LastTest().ok && Contains(b.LastTest().message, "quota"));
  CHECK(b.Next(steady0 + 600s, wall0 + 600s));
  ForecastSession b2;
  b2.SetEnabled(true);
  b2.SetCredentialPresent(true);
  b2.SetQuery(query, wall0);
  r = b2.Next(steady0, wall0);
  b2.Complete(r->id, Status(429), steady0, wall0);
  CHECK(!b2.Next(steady0 + kDefaultRateLimit - 1s, wall0));
  CHECK(b2.Next(steady0 + kDefaultRateLimit, wall0));

  // Server errors, bad request and bad payload are distinct and bounded.
  ForecastSession e;
  e.SetEnabled(true);
  e.SetCredentialPresent(true);
  e.SetQuery(query, wall0);
  r = e.Next(steady0, wall0);
  e.Complete(r->id, Status(503), steady0, wall0);
  CHECK(Contains(e.Snapshot(wall0).status, "503"));
  NoToken(e.Snapshot(wall0));
  r = e.Next(steady0 + 61s, wall0);
  e.Complete(r->id, Ok("<html>not csv</html>"), steady0 + 61s, wall0);
  CHECK(e.Snapshot(wall0).state == ForecastState::Error);
  CHECK(!Contains(e.Snapshot(wall0).status, "html"));
  r = e.Next(steady0 + 61s + 121s, wall0);
  CHECK(r);
  e.Complete(r->id, Status(400), steady0 + 182s, wall0);
  CHECK(Contains(e.Snapshot(wall0).status, "rejected the forecast request"));
  CHECK(!e.Next(steady0 + 182s + kMaxBackoff - 1s, wall0));

  // A changed area refetches, but not more often than kAreaChangeMinimum.
  ForecastSession m;
  m.SetEnabled(true);
  m.SetCredentialPresent(true);
  m.SetQuery(query, wall0);
  r = m.Next(steady0, wall0);
  m.Complete(r->id, Ok(Csv(clamped)), steady0, wall0);
  m.SetQuery(Query({{40, -70}}), wall0);
  CHECK(!m.Next(steady0 + kAreaChangeMinimum - 1s, wall0));
  r = m.Next(steady0 + kAreaChangeMinimum, wall0);
  CHECK(r && Contains(r->body, "\"lat\":40.0000"));

  // Test without a query uses a single uncached probe, even when disabled.
  ForecastSession t;
  t.SetCredentialPresent(true);
  t.RequestTest();
  r = t.Next(steady0, wall0);
  CHECK(r && r->test && Contains(r->body, "\"lat\":0.0000"));
  const auto probe = *gs::Clamp(Query({{0, 0}}), wall0);
  t.Complete(r->id, Ok(Csv(probe)), steady0, wall0);
  CHECK(t.LastTest().ok && Contains(t.LastTest().message, "Connection OK"));
  CHECK(t.Snapshot(wall0).state == ForecastState::Disabled && t.Snapshot(wall0).winds.empty());
  t.SetEnabled(true);
  CHECK(t.Snapshot(wall0).winds.empty());  // Probe data never displayed.

  // Unreadable token storage is reported without the token.
  ForecastSession u;
  u.SetEnabled(true);
  u.SetCredentialPresent(true);
  u.SetQuery(query, wall0);
  r = u.Next(steady0, wall0);
  u.CredentialMissing(r->id, true);
  CHECK(u.Snapshot(wall0).state == ForecastState::NoCredential);
  CHECK(Contains(u.Snapshot(wall0).status, "storage unavailable"));
}

void query() {
  auto route = SampleRoute({{59, 18}, {59, 19}}, kMaxRouteSamples);
  CHECK(route.size() >= 2 && route.size() <= kMaxRouteSamples);
  CHECK(Near(route.front().longitude_deg, 18) && Near(route.back().longitude_deg, 19));
  route = SampleRoute({{0, 0}, {10, 0}, {10, 10}, {std::nan(""), 1}}, kMaxRouteSamples);
  CHECK(route.size() == kMaxRouteSamples && Near(route.back().latitude_deg, 10) &&
        Near(route.back().longitude_deg, 10));
  route = SampleRoute({{0, 179.9}, {0, -179.9}}, kMaxRouteSamples);  // Short antimeridian leg.
  CHECK(route.size() == 2);
  CHECK(SampleRoute({{59, 18}}, kMaxRouteSamples).size() == 1);
  CHECK(SampleRoute({}, kMaxRouteSamples).empty());

  auto grid = ChartGrid({59, 60, 18, 19.5}, 36);
  CHECK(!grid.empty() && grid.size() <= 36);
  for (const auto &c : grid)
    CHECK(Near(std::remainder(c.latitude_deg, 0.25), 0) &&
          Near(std::remainder(c.longitude_deg, 0.25), 0));
  CHECK(ChartGrid({59, 60, 18, 19.5}, 3).empty());
  CHECK(ChartGrid({-89, 89, -180, 180}, 36).size() <= 36);
  grid = ChartGrid({-10, 10, 170, 190}, 36);  // Unwrapped antimeridian box.
  CHECK(!grid.empty());
  for (const auto &c : grid) CHECK(c.longitude_deg >= -180 && c.longitude_deg <= 180);
  CHECK(ChartGrid({59.0, 59.01, 18.0, 18.01}, 36).size() >= 1);  // Zoomed in.
  CHECK(ChartGrid({std::nan(""), 1, 2, 3}, 36).empty());

  QueryInputs in;
  in.now = wall0;
  in.vessel = Coordinate{59.3, 18.1};
  for (int i = 0; i < 200; ++i) in.route.push_back({59 + i * 0.05, 18 + i * 0.05});
  in.chart = ChartBox{-80, 80, -170, 170};
  auto q = BuildForecastQuery(in);
  CHECK(q.points.size() <= kMaxForecastPoints);
  CHECK(Near(q.points[0].latitude_deg, 59.3));
  CHECK(q.from == *gs::ParseUtc("2026-10-08T12:00:00Z") && q.until - q.from == 48h);
  auto c = gs::Clamp(q, wall0);
  CHECK(c && c->points.size() <= kMaxForecastPoints &&
        c->until - c->from <= std::chrono::hours(kMaxForecastSteps - 1));
  in.vessel.reset();
  in.route.clear();
  in.chart.reset();
  CHECK(BuildForecastQuery(in).points.empty());
}

class FakeCredentials final : public ais::IAisCredentials {
public:
  ais::CredentialResult Read() const override {
    ais::CredentialResult r;
    if (present) {
      r.status = ais::CredentialStatus::Ready;
      r.key.Assign(kToken);
    }
    return r;
  }
  ais::CredentialStatus Store(const ais::Secret &) override { return ais::CredentialStatus::ReadOnly; }
  ais::CredentialStatus Remove() override { return ais::CredentialStatus::ReadOnly; }
  bool present = true;
};
class FakeTransport final : public IForecastTransport {
public:
  struct Shared {
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<int> posts{0};
    bool block = false, cancelled = false, token_ok = false;
    std::string body;
  };
  explicit FakeTransport(std::shared_ptr<Shared> s) : s_(std::move(s)) {}
  HttpResult Post(const std::string &, const std::string &request,
                  const ais::Secret &token) override {
    std::unique_lock<std::mutex> lock(s_->mutex);
    ++s_->posts;
    s_->token_ok = token.View() == kToken;
    if (s_->block) {
      s_->cv.wait(lock, [this] { return s_->cancelled; });
      return {};
    }
    (void)request;
    return Ok(s_->body);
  }
  void Cancel() override {
    std::lock_guard<std::mutex> lock(s_->mutex);
    s_->cancelled = true;
    s_->cv.notify_all();
  }

private:
  std::shared_ptr<Shared> s_;
};

template <typename F> bool WaitFor(F &&f) {
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < end) {
    if (f()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

void service() {
  auto shared = std::make_shared<FakeTransport::Shared>();
  const auto now = std::chrono::system_clock::now();
  ForecastQuery q;
  q.points = {{60, 18}};
  q.from = now;
  q.until = now + 48h;
  auto clamped = *gs::Clamp(q, now);
  shared->body = "forecasted_at,forecasted_time,name,u,v\n" +
                 gs::FormatUtc(clamped.from - 6h) + "," + gs::FormatUtc(clamped.from + 1h) +
                 ",p0,0,-5\n";
  {
    ForecastService service(std::make_unique<FakeCredentials>(),
                            std::make_unique<FakeTransport>(shared));
    CHECK(service.Read(now).state == ForecastState::Disabled);
    service.SetQuery(q);
    service.CredentialChanged(true);
    std::this_thread::sleep_for(50ms);
    CHECK(shared->posts == 0);  // Constructing/configuring never fetches while off.
    service.SetEnabled(true);
    CHECK(WaitFor([&] { return service.Read(std::chrono::system_clock::now()).state ==
                               ForecastState::Live; }));
    CHECK(shared->token_ok && shared->posts == 1);
    const auto snap = service.Read(std::chrono::system_clock::now());
    CHECK(snap.winds.size() == 1 && Near(snap.winds[0].direction_from_true_deg, 0));
    NoToken(snap);
    service.Test();
    CHECK(WaitFor([&] { return !service.TestPending() && service.LastTest().ok; }));
    CHECK(shared->posts == 2);
    CHECK(!Contains(service.LastTest().message, kToken));
  }
  // Shutdown during a stuck request cancels it and returns promptly.
  auto stuck = std::make_shared<FakeTransport::Shared>();
  stuck->block = true;
  const auto started = std::chrono::steady_clock::now();
  {
    ForecastService service(std::make_unique<FakeCredentials>(),
                            std::make_unique<FakeTransport>(stuck));
    service.SetQuery(q);
    service.CredentialChanged(true);
    service.SetEnabled(true);
    CHECK(WaitFor([&] { return stuck->posts == 1; }));
    // UI reads never wait for the network.
    const auto read_start = std::chrono::steady_clock::now();
    CHECK(service.Read(std::chrono::system_clock::now()).state == ForecastState::Loading);
    CHECK(std::chrono::steady_clock::now() - read_start < 500ms);
  }
  CHECK(stuck->cancelled && std::chrono::steady_clock::now() - started < 3s);
  // Missing token from storage.
  auto none = std::make_shared<FakeTransport::Shared>();
  auto creds = std::make_unique<FakeCredentials>();
  creds->present = false;
  ForecastService missing(std::move(creds), std::make_unique<FakeTransport>(none));
  missing.SetQuery(q);
  missing.CredentialChanged(true);
  missing.SetEnabled(true);
  CHECK(WaitFor([&] { return missing.Read(std::chrono::system_clock::now()).state ==
                             ForecastState::NoCredential; }));
  CHECK(none->posts == 0);
}
} // namespace

int main(int argc, char **argv) {
  const std::string group = argc > 1 ? argv[1] : "all";
  try {
    if (group == "codec" || group == "all") codec();
    if (group == "session" || group == "all") session();
    if (group == "query" || group == "all") query();
    if (group == "service" || group == "all") service();
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  std::cout << "weather " << group << ": " << checks << " checks passed\n";
  return 0;
}
