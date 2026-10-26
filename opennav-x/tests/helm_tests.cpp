// SCRUM-379/380 SKAGER Helm host: events from the Helm Server, the supervisor
// (start, restart, stop) against a fake server, and the real child process.
//   helm_tests events | host | process
// "process" also runs the real server when SKAGER_HELM_SERVER names a binary.
#include "helm/HelmHost.h"
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace opennav::helm;
using namespace std::chrono_literals;

int checks = 0;
#define CHECK(v)                                                                \
  do {                                                                          \
    ++checks;                                                                   \
    if (!(v))                                                                   \
      throw std::runtime_error("helm check at line " + std::to_string(__LINE__)); \
  } while (false)

namespace {

template <class F> bool Eventually(F f, std::chrono::milliseconds limit = 5s) {
  const auto end = std::chrono::steady_clock::now() + limit;
  while (std::chrono::steady_clock::now() < end) {
    if (f()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return f();
}

const char *kPairing =
    R"({"event":"pairing","key":"ABCD-EFGH-JKLM-NPQR","link":"app.skager.helm://pair?v=1&k=x","expires":"2026-10-26T21:30:00+01:00",)"
    R"("addresses":["192.168.1.20:43872"],"qr":{"size":21,"rows":[)";

std::string PairingLine() {
  std::string line = kPairing;
  for (int i = 0; i < 21; ++i) line += std::string(i ? "," : "") + "\"" + std::string(21, i % 2 ? '1' : '0') + "\"";
  return line + "]}}";
}

void Events() {
  HelmSnapshot s;
  const auto now = std::chrono::steady_clock::now();
  std::atomic<std::int64_t> input{0};
  CHECK(ApplyEvent(s, R"({"event":"ready","port":43872,"addresses":["192.168.1.20:43872","10.0.0.5:43872"]})", now));
  CHECK(s.state == HostState::Ready && s.port == 43872 && s.addresses.size() == 2);
  CHECK(ApplyEvent(s, PairingLine(), now));
  CHECK(s.pairing && s.pairing->key == "ABCD-EFGH-JKLM-NPQR" && s.pairing->qr_size == 21 && s.pairing->qr_rows.size() == 21);
  // 21:30 at +01:00 is 20:30 UTC.
  CHECK(std::chrono::duration_cast<std::chrono::seconds>(s.pairing->expires.time_since_epoch()).count() == 1793046600);
  CHECK(ApplyEvent(s, R"({"event":"paired","device":{"id":"d1","name":"Kristoffer's iPhone"}})", now));
  CHECK(!s.pairing && s.message == "Paired with Kristoffer's iPhone.");
  CHECK(ApplyEvent(s, R"({"event":"devices","devices":[{"id":"d1","name":"Kristoffer's iPhone","paired_at":"2026-10-26T20:00:00Z"},{"id":"d2","name":"iPad"}],"connected":"d2"})", now));
  CHECK(s.devices.size() == 2 && s.devices[1].connected && s.connected_name == "iPad");
  CHECK(ApplyEvent(s, R"({"event":"session","state":"ended","device":{"id":"d2"},"reason":"connection_lost"})", now));
  CHECK(s.connected_name.empty() && !s.devices[1].connected);
  CHECK(ApplyEvent(s, R"({"event":"screen","available":false})", now) && !s.screen_available);
  // A touch from a phone is recorded for the autopilot guard without a UI rebuild.
  const auto revision = s.revision;
  CHECK(ApplyEvent(s, R"({"event":"input"})", now, &input) && input.load() == now.time_since_epoch().count());
  CHECK(s.revision == revision);
  CHECK(!ApplyEvent(s, "not json", now));
  CHECK(!ApplyEvent(s, R"({"event":"pairing","key":"x","qr":{"size":3,"rows":["1"]}})", now));
  CHECK(!ApplyEvent(s, R"({"event":"surprise"})", now));
  CHECK(NameCommand("Saga \"II\"\n") == R"({"cmd":"name","name":"Saga \"II\"\n"})");
  CHECK(RevokeCommand("d1") == R"({"cmd":"revoke","device":"d1"})");
  CHECK(PilotCommand(R"({"state":"auto"})") == R"({"cmd":"pilot","status":{"state":"auto"}})");
}

// In-memory server: the test feeds its output and reads what SKAGER wrote.
struct FakeServer {
  std::mutex m;
  std::condition_variable cv;
  std::deque<std::string> out;
  std::vector<std::string> written;
  bool closed = false, stdin_closed = false, fail_start = false;
  int starts = 0;
  std::vector<std::string> args;
  void Emit(const std::string &line) { std::lock_guard<std::mutex> l(m); out.push_back(line); cv.notify_all(); }
  void Exit() { std::lock_guard<std::mutex> l(m); closed = true; cv.notify_all(); }
};

class FakeProcess final : public HelmProcess {
public:
  explicit FakeProcess(FakeServer &s) : s_(s) {}
  bool Start(const std::string &, const std::vector<std::string> &args, const std::string &, std::string &error) override {
    std::lock_guard<std::mutex> l(s_.m);
    ++s_.starts;
    s_.args = args;
    s_.closed = s_.stdin_closed = false;
    if (s_.fail_start) error = "test failure";
    return !s_.fail_start;
  }
  bool WriteLine(const std::string &line) override {
    std::lock_guard<std::mutex> l(s_.m);
    if (s_.stdin_closed) return false;
    s_.written.push_back(line);
    return true;
  }
  bool ReadLine(std::string &line) override {
    std::unique_lock<std::mutex> l(s_.m);
    s_.cv.wait(l, [&] { return !s_.out.empty() || s_.closed; });
    if (s_.out.empty()) return false;
    line = s_.out.front();
    s_.out.pop_front();
    return true;
  }
  void Stop() override {
    std::lock_guard<std::mutex> l(s_.m);
    s_.stdin_closed = s_.closed = true;  // The real server exits when stdin closes.
    s_.cv.notify_all();
  }

private:
  FakeServer &s_;
};

void Host() {
  FakeServer server;
  HelmHost host([&] { return std::make_unique<FakeProcess>(server); });
  HelmOptions o;
  o.executable = "skager-helm";
  o.state_dir = "/profile/helm";
  o.name = "Aurora";
  o.window = "0x1a2b";
  o.restart_delay = 20ms;
  o.max_restarts = 2;
  host.SetPilot(R"({"state":"standby"})");  // Remembered and sent once the server is ready.
  host.Start(o);
  CHECK(Eventually([&] { std::lock_guard<std::mutex> l(server.m); return server.starts == 1; }));
  {
    std::lock_guard<std::mutex> l(server.m);
    CHECK((server.args == std::vector<std::string>{"-control", "stdio", "-state-dir", "/profile/helm", "-name", "Aurora", "-window", "0x1a2b"}));
  }
  CHECK(host.Read().state == HostState::Starting);
  host.Pair();  // Not ready yet: refused with a message, nothing written.
  CHECK(host.Read().message == "SKAGER Helm is not running.");
  server.Emit(R"({"event":"ready","port":43872,"addresses":["192.168.1.20:43872"]})");
  CHECK(Eventually([&] { return host.Running(); }));
  CHECK(Eventually([&] { std::lock_guard<std::mutex> l(server.m); return !server.written.empty(); }));
  {
    std::lock_guard<std::mutex> l(server.m);
    CHECK(server.written[0] == R"({"cmd":"pilot","status":{"state":"standby"}})");
  }
  host.Pair();
  host.SetPilot(R"({"state":"standby"})");  // Unchanged: not sent again.
  host.SetPilot(R"({"state":"auto","heading":184})");
  host.Revoke("d1");
  {
    std::lock_guard<std::mutex> l(server.m);
    CHECK((server.written == std::vector<std::string>{R"({"cmd":"pilot","status":{"state":"standby"}})", R"({"cmd":"pair"})",
                                                       R"({"cmd":"pilot","status":{"state":"auto","heading":184}})",
                                                       R"({"cmd":"revoke","device":"d1"})"}));
  }
  server.Emit(PairingLine());
  CHECK(Eventually([&] { return host.Read().pairing.has_value(); }));
  CHECK(!host.RecentInput(1500ms));
  server.Emit(R"({"event":"input"})");
  CHECK(Eventually([&] { return host.RecentInput(1500ms); }));
  CHECK(!host.RecentInput(0ms) || host.RecentInput(1500ms));

  // The server dies: SKAGER shows it, restarts it, and the pilot status is re-sent.
  server.Exit();
  CHECK(Eventually([&] { return host.Read().state == HostState::Failed || host.Read().state == HostState::Starting; }));
  CHECK(Eventually([&] { std::lock_guard<std::mutex> l(server.m); return server.starts == 2; }));
  CHECK(!host.Read().pairing);
  server.Emit(R"({"event":"ready","port":43872})");
  CHECK(Eventually([&] { return host.Running(); }));
  CHECK(Eventually([&] { std::lock_guard<std::mutex> l(server.m); return server.written.back() == R"({"cmd":"pilot","status":{"state":"auto","heading":184}})"; }));

  // Switching off closes the server's stdin and ends at Off.
  host.Stop();
  CHECK(host.Read().state == HostState::Off);
  {
    std::lock_guard<std::mutex> l(server.m);
    CHECK(server.stdin_closed);
  }

  // A server that never starts is retried a few times, then given up with a message.
  server.fail_start = true;
  host.Start(o);
  CHECK(Eventually([&] { std::lock_guard<std::mutex> l(server.m); return server.starts == 2 + 1 + o.max_restarts; }));
  CHECK(Eventually([&] { return host.Read().message.find("could not start") != std::string::npos; }));
  std::this_thread::sleep_for(80ms);
  {
    std::lock_guard<std::mutex> l(server.m);
    CHECK(server.starts == 2 + 1 + o.max_restarts);
  }
  host.Stop();
  CHECK(host.Read().state == HostState::Off);
}

void Process() {
  // A missing executable is a clear start failure, not a hang.
  {
    auto p = CreateHelmProcess();
    std::string error;
    CHECK(!p->Start("/nonexistent/skager-helm", {}, "", error) && error.find("could not run") != std::string::npos);
  }
  const char *bin = std::getenv("SKAGER_HELM_SERVER");
  if (!bin || !*bin) {
    std::cout << "helm process: SKAGER_HELM_SERVER not set; real server not exercised\n";
    return;
  }
  const auto dir = std::filesystem::temp_directory_path() / ("helm-host-test-" + std::to_string(std::rand()));
  std::filesystem::create_directories(dir);
  HelmHost host;
  HelmOptions o;
  o.executable = bin;
  o.state_dir = dir.string();
  o.name = "Aurora";
  o.log_file = (dir / "skager-helm.log").string();
  o.extra_args = {"-source", "test", "-addr", "127.0.0.1:0"};
  host.Start(o);
  CHECK(Eventually([&] { return host.Running(); }, 10s));
  CHECK(host.Read().port > 0);
  host.Pair();
  CHECK(Eventually([&] { return host.Read().pairing.has_value(); }, 5s));
  const auto pairing = *host.Read().pairing;
  CHECK(pairing.key.size() == 19 && pairing.link.rfind("app.skager.helm://pair?", 0) == 0 && pairing.qr_size >= 25);
  host.CancelPair();
  CHECK(Eventually([&] { return !host.Read().pairing.has_value(); }, 5s));
  host.Stop();
  CHECK(host.Read().state == HostState::Off);
  CHECK(std::filesystem::exists(dir / "helm-devices.json"));
  std::filesystem::remove_all(dir);
}
}  // namespace

int main(int argc, char **argv) {
  const std::string group = argc > 1 ? argv[1] : "all";
  try {
    if (group == "events" || group == "all") Events();
    if (group == "host" || group == "all") Host();
    if (group == "process" || group == "all") Process();
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  std::cout << "helm " << group << ": " << checks << " checks passed\n";
  return 0;
}
