// SCRUM-44 / SCRUM-369 SKAGER Cloud: wire format and the background worker
// (pairing, account status, sharing, revocation) against a fake service.
#include "cloud/CloudProtocol.h"
#include "cloud/CloudService.h"
#include <rapidjson/document.h>
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
  explicit MemoryCredentials(std::shared_ptr<std::string> value) : value_(std::move(value)) {}
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
    value_->clear();
    return ais::CredentialStatus::Removed;
  }

private:
  std::shared_ptr<std::string> value_;
};

// A minimal stand-in for /api/device/*. The handler runs on the worker thread.
struct FakeService {
  std::mutex mutex;
  std::vector<std::string> calls;
  std::map<std::string, int> count;
  bool approved = false, cloud = true, revoked = false, offline = false;
  std::string last_upload, last_token;
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
                             R"(","userCode":"BCDF2345","verificationUrl":"https://beta.skager.app/link/","expiresIn":600,"interval":5})"};
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
    std::cout << EncodeTelemetry(state, now).value_or("") << "\n";
    return 0;
  }
  try {
    if (group == "protocol" || group == "all") protocol();
    if (group == "service" || group == "all") service();
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  std::cout << "cloud " << group << ": " << checks << " checks passed\n";
  return 0;
}
