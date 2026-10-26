// SCRUM-44 / SCRUM-369 SKAGER Cloud: wire format and the background worker
// (pairing, account status, sharing, revocation) against a fake service.
#include "cloud/CloudProtocol.h"
#include "cloud/CloudService.h"
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace opennav;
using namespace opennav::cloud;
using namespace std::chrono_literals;

int checks = 0;
#define CHECK(v)                                                                 \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(v))                                                                    \
      throw std::runtime_error("cloud check at line " + std::to_string(__LINE__)); \
  } while (false)

namespace {
const std::string kToken = "skg1_" + std::string(64, 'a');

vessel::Sample Measured(double value, vessel::Time at, const char *source = "GPS 1") {
  vessel::Sample s;
  s.value = value;
  s.source = source;
  s.observed_at = at;
  s.validity = vessel::Validity::Measured;
  return s;
}

void protocol() {
  const auto now = vessel::Clock::now();
  vessel::VesselState state;
  // Nothing measured: nothing is sent, never a row of zeros.
  CHECK(!EncodeTelemetry(state, now));
  state.navigation.sog_kn = Measured(5.25, now - 400ms);
  state.navigation.latitude_deg = Measured(59.3, now - 1s);
  state.environment.depth_below_transducer_m = Measured(12.5, now - 30s, "DST\x01""800");
  state.battery.soc_percent = Measured(140, now);           // Outside the documented range.
  state.battery.current_native_a = Measured(-12, now);      // Raw sign is never uploaded.
  state.wind.true_speed_kn = Measured(10, now - 2h - 25h);  // Older than a day.
  const auto body = EncodeTelemetry(state, now);
  CHECK(body);
  rapidjson::Document doc;
  doc.Parse(body->c_str());
  CHECK(!doc.HasParseError() && doc["items"].IsObject());
  const auto &items = doc["items"];
  CHECK(items.MemberCount() == 3);
  CHECK(items["navigation.sog"]["v"].GetDouble() == 5.25);
  CHECK(std::string(items["navigation.sog"]["q"].GetString()) == "live");
  CHECK(items["navigation.sog"]["age"].GetDouble() == 0.4);
  CHECK(std::string(items["environment.depth"]["q"].GetString()) == "stale");
  CHECK(std::string(items["environment.depth"]["src"].GetString()) == "DST800");
  CHECK(!items.HasMember("battery.soc") && !items.HasMember("wind.true_speed"));
  // Simulated and replayed state is never shared.
  state.simulated = true;
  CHECK(!EncodeTelemetry(state, now));
  state.simulated = false;
  state.replayed = true;
  CHECK(!EncodeTelemetry(state, now));

  CHECK(ValidToken(kToken) && !ValidToken("skg1_" + std::string(64, 'A')) && !ValidToken(kToken + "0"));
  CHECK(FormatUserCode("BCDF2345") == "BCDF-2345");

  const auto offer = ParseLinkOffer(R"({"deviceCode":")" + std::string(64, 'c') +
      R"(","userCode":"BCDF2345","verificationUrl":"https://beta.skager.app/link/","expiresIn":600,"interval":1})");
  CHECK(offer && offer->user_code == "BCDF2345" && offer->interval == 2s);
  CHECK(!ParseLinkOffer(R"({"deviceCode":")" + std::string(64, 'c') +
      R"(","userCode":"BCDF2345","verificationUrl":"https://evil.example/link/"})"));
  CHECK(ParsePoll(R"({"state":"approved","token":")" + kToken + R"("})").token == kToken);
  CHECK(ParsePoll(R"({"state":"approved","token":"short"})").state == PollState::Invalid);
  CHECK(ParsePoll(R"({"state":"pending"})").state == PollState::Pending);
  CHECK(ParsePoll("not json").state == PollState::Invalid);
  const auto status = ParseStatus(R"({"linked":true,"account":{"email":"skipper@example.test"},"device":{"name":"Aurora"},)"
      R"("cloud":{"active":true,"status":"active","renewsAt":"2026-11-19T10:00:00Z","cancelAtPeriodEnd":false}})");
  CHECK(status && status->email == "skipper@example.test" && status->cloud_active && status->device_name == "Aurora");
  CHECK(!ParseStatus(R"({"cloud":{"active":true}})"));
  const auto request = EncodeLinkRequest("Aurora\nhelm", "windows", "0.4.0");
  CHECK(request == R"({"name":"Aurorahelm","platform":"windows","version":"0.4.0"})");
  CHECK(EncodeLinkRequest("", "linux", "0.4.0 (dev)") == R"({"name":"SKAGER","platform":"linux","version":"0.4.0dev"})");
}

class MemoryCredentials final : public ais::IAisCredentials {
public:
  explicit MemoryCredentials(std::shared_ptr<std::string> value,
                             std::shared_ptr<std::atomic<bool>> fail_remove = {})
      : value_(std::move(value)), fail_remove_(std::move(fail_remove)) {}
  ais::CredentialResult Read() const override {
    ais::CredentialResult r;
    if (!value_->empty() && r.key.Assign(*value_)) r.status = ais::CredentialStatus::Ready;
    return r;
  }
  ais::CredentialStatus Store(const ais::Secret &key) override {
    *value_ = std::string(key.View());
    return ais::CredentialStatus::Ready;
  }
  ais::CredentialStatus Remove() override {
    if (fail_remove_ && *fail_remove_) return ais::CredentialStatus::Unavailable;
    value_->clear();
    return ais::CredentialStatus::Removed;
  }

private:
  std::shared_ptr<std::string> value_;
  std::shared_ptr<std::atomic<bool>> fail_remove_;
};

// A minimal stand-in for /api/device/*. The handler runs on the worker thread.
struct FakeService {
  std::mutex mutex;
  std::vector<std::string> calls;
  std::map<std::string, int> count;
  bool approved = false, cloud = true, revoked = false, offline = false;
  std::string last_upload, last_token;
  std::vector<std::string> event_bodies;
  HttpResult Handle(const std::string &method, const std::string &path, const std::string &body,
                    const ais::Secret &token) {
    std::lock_guard<std::mutex> lock(mutex);
    calls.push_back(method + " " + path);
    ++count[path];
    last_token = std::string(token.View());
    if (offline) return {};
    const bool authorized = std::string(token.View()) == kToken && !revoked;
    if (path == "/api/device/link")
      return {true, 200, R"({"deviceCode":")" + std::string(64, 'c') +
                             R"(","userCode":")" + (count[path] == 1 ? "BCDF2345" : "GHJK6789") +
                             R"(","verificationUrl":"https://beta.skager.app/link/","expiresIn":600,"interval":5})"};
    if (path == "/api/device/link/poll")
      return {true, 200, approved ? R"({"state":"approved","token":")" + kToken + R"("})" : R"({"state":"pending"})"};
    if (!authorized) return {true, 401, R"({"linked":false})"};
    if (path == "/api/device/status")
      return {true, 200, std::string(R"({"account":{"email":"skipper@example.test"},"device":{"name":"Aurora"},"cloud":{"active":)") +
                             (cloud ? "true" : "false") + "}}"};
    if (path == "/api/device/telemetry") {
      if (!cloud) return {true, 402, "{}"};
      last_upload = body;
      return {true, 202, R"({"state":"stored"})"};
    }
    if (path == "/api/device/events") {
      if (!cloud) return {true, 402, "{}"};
      event_bodies.push_back(body);
      return {true, 202, R"({"state":"stored","new":1})"};
    }
    if (path == "/api/device/telemetry/stop") last_upload.clear();
    return {true, 200, "{}"};
  }
};
class FakeTransport final : public ICloudTransport {
public:
  explicit FakeTransport(std::shared_ptr<FakeService> s) : s_(std::move(s)) {}
  HttpResult Request(const std::string &m, const std::string &p, const std::string &b,
                     const ais::Secret &t) override { return s_->Handle(m, p, b, t); }
  void Cancel() override {}

private:
  std::shared_ptr<FakeService> s_;
};

// Hold one completed response so tests exercise actions while a request is in
// flight, without real network access or timing-dependent service behavior.
struct ResponseGate {
  explicit ResponseGate(std::string p) : path(std::move(p)) {}
  std::string path;
  std::atomic<bool> entered{false}, released{false};
};
class GatedTransport final : public ICloudTransport {
public:
  GatedTransport(std::shared_ptr<FakeService> service, std::shared_ptr<ResponseGate> gate)
      : service_(std::move(service)), gate_(std::move(gate)) {}
  HttpResult Request(const std::string &m, const std::string &p, const std::string &b,
                     const ais::Secret &t) override {
    auto result = service_->Handle(m, p, b, t);
    if (p == gate_->path && !gate_->entered.exchange(true))
      while (!gate_->released) std::this_thread::sleep_for(1ms);
    return result;
  }
  void Cancel() override { gate_->released = true; }
private:
  std::shared_ptr<FakeService> service_;
  std::shared_ptr<ResponseGate> gate_;
};

int Count(const std::shared_ptr<FakeService> &service, const std::string &path) {
  std::lock_guard<std::mutex> lock(service->mutex);
  return service->count[path];
}

bool WaitFor(const std::function<bool()> &condition) {
  for (int i = 0; i < 400; ++i) {
    if (condition()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}

CloudOptions FastOptions() {
  CloudOptions o;
  o.version = "0.4.0-test";
  o.poll_interval = 10ms;
  o.retry = 20ms;
  o.status_period = 10min;
  o.telemetry_interval = 0ms;
  o.event_retry = 20ms;
  return o;
}

void service() {
  auto fake = std::make_shared<FakeService>();
  auto stored = std::make_shared<std::string>();
  vessel::VesselState state;
  {
    CloudService cloud(std::make_unique<MemoryCredentials>(stored), std::make_unique<FakeTransport>(fake), FastOptions());
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::Unlinked; }));
    // Not linked: observing data sends nothing at all.
    cloud.SetShareEnabled(true);
    state.navigation.sog_kn = Measured(4.0, vessel::Clock::now());
    cloud.Observe(state, vessel::Clock::now());
    std::this_thread::sleep_for(30ms);
    CHECK(fake->calls.empty());

    cloud.StartLink("Aurora helm");
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::WaitingForApproval; }));
    CHECK(cloud.Read().user_code == "BCDF2345");
    CHECK(WaitFor([&] { std::lock_guard<std::mutex> l(fake->mutex); return fake->count["/api/device/link/poll"] >= 2; }));
    CHECK(cloud.Read().link == LinkState::WaitingForApproval && stored->empty());
    {
      std::lock_guard<std::mutex> l(fake->mutex);
      // Pairing requests never carry a token.
      CHECK(fake->last_token.empty());
      fake->approved = true;
    }
    CHECK(WaitFor([&] { return cloud.Read().account.has_value(); }));
    CHECK(*stored == kToken);
    CHECK(cloud.Read().link == LinkState::Linked && cloud.Read().account->cloud_active);
    CHECK(!cloud.Read().share_enabled);  // Pairing never inherits an earlier link's consent.
    cloud.SetShareEnabled(true);

    // Sharing: the newest observation is uploaded with the token.
    state.navigation.sog_kn = Measured(5.5, vessel::Clock::now());
    cloud.Observe(state, vessel::Clock::now());
    CHECK(WaitFor([&] { return cloud.Read().last_shared.has_value(); }));
    {
      std::lock_guard<std::mutex> l(fake->mutex);
      CHECK(fake->last_upload.find("\"navigation.sog\":{\"v\":5.5") != std::string::npos);
      CHECK(fake->last_token == kToken);
    }
    // Switching sharing off tells the service once.
    cloud.SetShareEnabled(false);
    CHECK(WaitFor([&] { std::lock_guard<std::mutex> l(fake->mutex); return fake->count["/api/device/telemetry/stop"] == 1; }));
    const int uploads = fake->count["/api/device/telemetry"];
    cloud.Observe(state, vessel::Clock::now());
    std::this_thread::sleep_for(30ms);
    CHECK(fake->count["/api/device/telemetry"] == uploads);
  }
  // Restart: the stored link is used without pairing again.
  {
    CloudService cloud(std::make_unique<MemoryCredentials>(stored), std::make_unique<FakeTransport>(fake), FastOptions());
    CHECK(WaitFor([&] { return cloud.Read().account.has_value(); }));
    CHECK(fake->count["/api/device/link"] == 1);
    // Cloud lapses: uploads are refused and stop until the status changes.
    {
      std::lock_guard<std::mutex> l(fake->mutex);
      fake->cloud = false;
    }
    cloud.SetShareEnabled(true);
    state.navigation.sog_kn = Measured(6.0, vessel::Clock::now());
    cloud.Observe(state, vessel::Clock::now());
    CHECK(WaitFor([&] { auto s = cloud.Read(); return s.account && !s.account->cloud_active; }));
    // Revoked in the web account: the next request forgets the local link.
    {
      std::lock_guard<std::mutex> l(fake->mutex);
      fake->revoked = true;
    }
    cloud.RefreshStatus();
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::Unlinked; }));
    CHECK(stored->empty() && !cloud.Read().account);
  }
  // Unlink removes the local credential even when the service is unreachable.
  {
    *stored = kToken;
    fake->revoked = false;
    fake->offline = true;
    CloudService cloud(std::make_unique<MemoryCredentials>(stored), std::make_unique<FakeTransport>(fake), FastOptions());
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::Linked; }));
    cloud.Unlink();
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::Unlinked; }));
    CHECK(stored->empty());
  }
  // Pairing without a connection reports it and returns to Unlinked.
  {
    CloudService cloud(std::make_unique<MemoryCredentials>(stored), std::make_unique<FakeTransport>(fake), FastOptions());
    cloud.StartLink("Aurora helm");
    CHECK(WaitFor([&] { auto s = cloud.Read(); return s.link == LinkState::Unlinked && !s.message.empty(); }));
    CHECK(cloud.Read().message.find("No connection") != std::string::npos);
  }
}

void service_edges() {
  // A queued observation ages while status is blocked. The eventual upload
  // must describe its actual age/quality, not the freshness at enqueue time.
  {
    auto fake = std::make_shared<FakeService>();
    auto stored = std::make_shared<std::string>(kToken);
    auto gate = std::make_shared<ResponseGate>("/api/device/status");
    CloudService cloud(std::make_unique<MemoryCredentials>(stored),
                       std::make_unique<GatedTransport>(fake, gate), FastOptions());
    CHECK(WaitFor([&] { return gate->entered.load(); }));
    cloud.SetShareEnabled(true);
    vessel::VesselState state;
    const auto observed = vessel::Clock::now();
    state.navigation.sog_kn = Measured(5.0, observed);
    state.navigation.sog_kn.freshness = {10ms, 20ms};
    cloud.Observe(state, observed);
    std::this_thread::sleep_for(120ms);
    gate->released = true;
    CHECK(WaitFor([&] { return cloud.Read().last_shared.has_value(); }));
    std::lock_guard<std::mutex> lock(fake->mutex);
    rapidjson::Document doc;
    doc.Parse(fake->last_upload.c_str());
    CHECK(doc["items"]["navigation.sog"]["age"].GetDouble() >= 0.1);
    CHECK(std::string(doc["items"]["navigation.sog"]["q"].GetString()) == "stale");
  }
  // OFF is local immediately and keeps retrying remote cleanup after an outage.
  {
    auto fake = std::make_shared<FakeService>();
    auto stored = std::make_shared<std::string>(kToken);
    CloudService cloud(std::make_unique<MemoryCredentials>(stored),
                       std::make_unique<FakeTransport>(fake), FastOptions());
    CHECK(WaitFor([&] { return Count(fake, "/api/device/telemetry/stop") >= 1; }));
    cloud.SetShareEnabled(true);
    vessel::VesselState state;
    state.navigation.sog_kn = Measured(4, vessel::Clock::now());
    cloud.Observe(state, vessel::Clock::now());
    CHECK(WaitFor([&] { return cloud.Read().last_shared.has_value(); }));
    const auto before = Count(fake, "/api/device/telemetry/stop");
    { std::lock_guard<std::mutex> lock(fake->mutex); fake->offline = true; }
    cloud.SetShareEnabled(false);
    CHECK(!cloud.Read().share_enabled);
    CHECK(WaitFor([&] { return Count(fake, "/api/device/telemetry/stop") > before; }));
    { std::lock_guard<std::mutex> lock(fake->mutex); fake->offline = false; }
    CHECK(WaitFor([&] { std::lock_guard<std::mutex> lock(fake->mutex); return fake->last_upload.empty(); }));
    CHECK(Count(fake, "/api/device/telemetry/stop") >= before + 2);
  }
  // Failed OS deletion cannot claim unlink success or permit further sharing.
  // Once it recovers, a later account link still needs a new sharing choice.
  {
    auto fake = std::make_shared<FakeService>();
    auto stored = std::make_shared<std::string>(kToken);
    auto fail_remove = std::make_shared<std::atomic<bool>>(true);
    CloudService cloud(std::make_unique<MemoryCredentials>(stored, fail_remove),
                       std::make_unique<FakeTransport>(fake), FastOptions());
    CHECK(WaitFor([&] { return cloud.Read().account.has_value(); }));
    cloud.SetShareEnabled(true);
    cloud.Unlink();
    CHECK(WaitFor([&] { return cloud.Read().message.find("could not be removed") != std::string::npos; }));
    CHECK(cloud.Read().unlink_pending && cloud.Read().link == LinkState::Linked);
    CHECK(!cloud.Read().share_enabled && *stored == kToken);
    cloud.SetShareEnabled(true);
    CHECK(!cloud.Read().share_enabled);
    *fail_remove = false;
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::Unlinked; }));
    CHECK(stored->empty() && !cloud.Read().unlink_pending);
    { std::lock_guard<std::mutex> lock(fake->mutex); fake->approved = true; }
    cloud.StartLink("Another account");
    CHECK(WaitFor([&] { return cloud.Read().account.has_value(); }));
    CHECK(!cloud.Read().share_enabled);
    CHECK(Count(fake, "/api/device/telemetry") == 0);
  }
  // Cancelling and immediately starting another link must discard the first
  // request's eventual response, even though the state is Starting again.
  {
    auto fake = std::make_shared<FakeService>();
    auto stored = std::make_shared<std::string>();
    auto gate = std::make_shared<ResponseGate>("/api/device/link");
    CloudService cloud(std::make_unique<MemoryCredentials>(stored),
                       std::make_unique<GatedTransport>(fake, gate), FastOptions());
    cloud.StartLink("First");
    CHECK(WaitFor([&] { return gate->entered.load(); }));
    cloud.CancelLink();
    cloud.StartLink("Second");
    gate->released = true;
    CHECK(WaitFor([&] { return cloud.Read().link == LinkState::WaitingForApproval; }));
    CHECK(Count(fake, "/api/device/link") == 2);
    CHECK(cloud.Read().user_code == "GHJK6789");
  }
  // Cancelling an approved poll also prevents the late token being persisted.
  {
    auto fake = std::make_shared<FakeService>();
    fake->approved = true;
    auto stored = std::make_shared<std::string>();
    auto gate = std::make_shared<ResponseGate>("/api/device/link/poll");
    CloudService cloud(std::make_unique<MemoryCredentials>(stored),
                       std::make_unique<GatedTransport>(fake, gate), FastOptions());
    cloud.StartLink("Cancelled");
    CHECK(WaitFor([&] { return gate->entered.load(); }));
    cloud.CancelLink();
    gate->released = true;
    CHECK(WaitFor([&] { return !cloud.Read().busy; }));
    CHECK(cloud.Read().link == LinkState::Unlinked && stored->empty());
  }
}
// SCRUM-387: the anchor watch and boat events for SKAGER Companion.
void anchor_protocol() {
  const auto now = vessel::Clock::now();
  vessel::VesselState state;
  state.navigation.sog_kn = Measured(0.3, now - 1s);
  AnchorReport anchor;
  anchor.armed = true;
  anchor.alarm = true;
  anchor.latitude_deg = 59.312345678;
  anchor.longitude_deg = 18.1;
  anchor.radius_m = -45.0;  // OpenCPN: alarm when inside.
  anchor.distance_m = 61.27;
  anchor.distance_observed_at = now - 2s;
  anchor.source = "GPS\n1";
  const auto body = EncodeTelemetry(state, now, anchor);
  CHECK(body);
  rapidjson::Document doc;
  doc.Parse(body->c_str());
  const auto &a = doc["anchor"];
  CHECK(a["armed"].GetBool() && a["alarm"].GetBool());
  CHECK(a["lat"].GetDouble() == 59.312346 && a["radius_m"].GetDouble() == -45.0);
  CHECK(a["distance_m"].GetDouble() == 61.3 && a["age"].GetDouble() == 2.0);
  CHECK(std::string(a["src"].GetString()) == "GPS1");
  // Without a current position there is no distance; nonsense values are left out.
  anchor.distance_observed_at = {};
  anchor.radius_m = 0.0;
  anchor.latitude_deg = 123.0;
  doc.Parse(EncodeTelemetry(state, now, anchor)->c_str());
  CHECK(!doc["anchor"].HasMember("distance_m") && !doc["anchor"].HasMember("age"));
  CHECK(!doc["anchor"].HasMember("radius_m") && !doc["anchor"].HasMember("lat") && !doc["anchor"].HasMember("lon"));
  // Off is explicit; no report means the field is absent (unknown).
  doc.Parse(EncodeTelemetry(state, now, AnchorReport{})->c_str());
  CHECK(doc["anchor"].MemberCount() == 1 && !doc["anchor"]["armed"].GetBool());
  doc.Parse(EncodeTelemetry(state, now)->c_str());
  CHECK(!doc.HasMember("anchor"));
  // Simulated state never leaves the boat, anchor or not.
  state.simulated = true;
  CHECK(!EncodeTelemetry(state, now, anchor));

  std::vector<BoatEvent> events(3);
  events[0] = {NewEventId(), EventKind::AnchorAlarm, now - 3s, "ignored", 61.0, 50.0};
  events[1] = {NewEventId(), EventKind::AlertCritical, now, "GPS position lost", 10.0, {}};
  events[2] = {NewEventId(), EventKind::AnchorArmed, now - std::chrono::hours(24 * 8), "", {}, {}};
  CHECK(events[0].id.size() == 32 && events[0].id != events[1].id);
  doc.Parse(EncodeEvents(events, now)->c_str());
  const auto &list = doc["events"];
  CHECK(list.Size() == 2);  // Older than 7 days: left out.
  CHECK(std::string(list[0]["kind"].GetString()) == "anchor_alarm" && list[0]["age"].GetDouble() == 3.0);
  CHECK(!list[0].HasMember("alert") && list[0]["data"]["distance_m"].GetDouble() == 61.0);
  CHECK(std::string(list[1]["alert"].GetString()) == "GPS position lost" && !list[1].HasMember("data"));
  CHECK(!EncodeEvents({events[2]}, now));
}

void anchor_service() {
  auto fake = std::make_shared<FakeService>();
  auto stored = std::make_shared<std::string>(kToken);
  CloudService cloud(std::make_unique<MemoryCredentials>(stored), std::make_unique<FakeTransport>(fake), FastOptions());
  CHECK(WaitFor([&] { return cloud.Read().account.has_value(); }));
  // Every event sent so far, in order, whether batched or not.
  const auto events = [&] {
    std::lock_guard<std::mutex> l(fake->mutex);
    std::vector<std::string> out;
    for (const auto &body : fake->event_bodies) {
      rapidjson::Document doc;
      doc.Parse(body.c_str());
      for (const auto &e : doc["events"].GetArray()) {
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        e.Accept(writer);
        out.emplace_back(buffer.GetString());
      }
    }
    return out;
  };
  AnchorReport anchor;
  anchor.armed = true;
  anchor.radius_m = 50.0;
  // Sharing off: transitions are tracked but nothing is queued or sent.
  cloud.ObserveAnchor(anchor, vessel::Clock::now());
  anchor.alarm = true;
  cloud.ObserveAnchor(anchor, vessel::Clock::now());
  std::this_thread::sleep_for(30ms);
  CHECK(events().empty() && cloud.Read().pending_events == 0);

  cloud.SetShareEnabled(true);
  anchor.alarm = false;
  cloud.ObserveAnchor(anchor, vessel::Clock::now());  // Alarm ended while not sharing:
  anchor.distance_m = 64.0;                           // the next change is reported normally.
  anchor.distance_observed_at = vessel::Clock::now();
  anchor.alarm = true;
  cloud.ObserveAnchor(anchor, vessel::Clock::now());
  CHECK(WaitFor([&] { return events().size() == 2; }));
  CHECK(events()[0].find("anchor_alarm_cleared") != std::string::npos);
  CHECK(events()[1].find("\"kind\":\"anchor_alarm\"") != std::string::npos);
  CHECK(events()[1].find("\"distance_m\":64") != std::string::npos);
  // Repeating the same state is not news.
  cloud.ObserveAnchor(anchor, vessel::Clock::now());
  std::this_thread::sleep_for(30ms);
  CHECK(events().size() == 2);

  // Offline: the alarm waits in the queue and goes out when the connection returns.
  { std::lock_guard<std::mutex> l(fake->mutex); fake->offline = true; }
  anchor.armed = false;
  anchor.alarm = false;
  const int attempts = Count(fake, "/api/device/events");
  cloud.ObserveAnchor(anchor, vessel::Clock::now());
  CHECK(WaitFor([&] { return Count(fake, "/api/device/events") >= attempts + 2; }));  // retried
  CHECK(cloud.Read().pending_events == 2);  // alarm cleared + watch off
  { std::lock_guard<std::mutex> l(fake->mutex); fake->offline = false; }
  CHECK(WaitFor([&] { return cloud.Read().pending_events == 0; }));
  CHECK(events().size() == 4 && events()[2].find("anchor_alarm_cleared") != std::string::npos
        && events()[3].find("anchor_disarmed") != std::string::npos);

  // Critical alerts: a new one and its end are each reported once.
  cloud.ObserveAlerts({{"position-lat", "GPS position lost"}}, vessel::Clock::now());
  cloud.ObserveAlerts({{"position-lat", "GPS position lost"}}, vessel::Clock::now());
  CHECK(WaitFor([&] { return events().size() == 5; }));
  CHECK(events()[4].find("\"alert\":\"GPS position lost\"") != std::string::npos);
  cloud.ObserveAlerts({}, vessel::Clock::now());
  CHECK(WaitFor([&] { return events().size() == 6; }));
  CHECK(events()[5].find("alert_cleared") != std::string::npos);

  // The snapshot carries the anchor watch.
  vessel::VesselState state;
  state.navigation.sog_kn = Measured(0.2, vessel::Clock::now());
  cloud.Observe(state, vessel::Clock::now());
  CHECK(WaitFor([&] { std::lock_guard<std::mutex> l(fake->mutex); return fake->last_upload.find("\"anchor\":{\"armed\":false}") != std::string::npos; }));

  // Switching sharing off discards anything unsent.
  { std::lock_guard<std::mutex> l(fake->mutex); fake->offline = true; }
  anchor.armed = true;
  cloud.ObserveAnchor(anchor, vessel::Clock::now());
  CHECK(WaitFor([&] { return cloud.Read().pending_events == 1; }));
  cloud.SetShareEnabled(false);
  CHECK(cloud.Read().pending_events == 0);
}
}  // namespace

int main(int argc, char **argv) {
  const std::string group = argc > 1 ? argv[1] : "all";
  if (group == "emit") {
    // Prints one upload with every key filled, for the server contract check
    // (web/edge validTelemetry must accept exactly what the app sends).
    const auto now = vessel::Clock::now();
    vessel::VesselState state;
    for (const auto &key : TelemetryKeys()) {
      auto &sample = const_cast<vessel::Sample &>(key.sample(state));
      sample = Measured(key.min + (key.max - key.min) * 0.25, now - 1s, "contract");
    }
    AnchorReport anchor;
    anchor.armed = anchor.alarm = true;
    anchor.latitude_deg = 59.3;
    anchor.longitude_deg = 18.1;
    anchor.radius_m = 50.0;
    anchor.distance_m = 61.5;
    anchor.distance_observed_at = now - 1s;
    anchor.source = "contract";
    std::cout << EncodeTelemetry(state, now, anchor).value_or("") << "\n";
    return 0;
  }
  if (group == "emit-events") {
    const auto now = vessel::Clock::now();
    std::vector<BoatEvent> events{{NewEventId(), EventKind::AnchorAlarm, now - 2s, "", 61.5, 50.0},
                                  {NewEventId(), EventKind::AlertCritical, now - 1s, "GPS position lost", {}, {}}};
    std::cout << EncodeEvents(events, now).value_or("") << "\n";
    return 0;
  }
  try {
    if (group == "protocol" || group == "all") protocol();
    if (group == "service" || group == "all") { service(); service_edges(); }
    if (group == "anchor" || group == "all") { anchor_protocol(); anchor_service(); }
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  std::cout << "cloud " << group << ": " << checks << " checks passed\n";
  return 0;
}
