// SCRUM-44/369: exercise real profile persistence with an inert cloud service.
#include "integration/OnlineCloud.h"
#include <atomic>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <wx/fileconf.h>
#include <wx/filename.h>
#include <wx/init.h>

using namespace opennav;
using namespace std::chrono_literals;
namespace {
constexpr const char *key = "/OpenNav/Cloud/v1/ShareBoatData";
int checks = 0;
void Check(bool value, const char *message) {
  ++checks;
  if (!value) throw std::runtime_error(message);
}
bool Wait(const std::function<bool()> &condition) {
  for (int i = 0; i < 400; ++i) {
    if (condition()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}
struct TemporaryProfile {
  wxString path = wxFileName::CreateTempFileName("skager-cloud-preferences");
  ~TemporaryProfile() { wxRemoveFile(path); }
};
struct Config : wxFileConfig {
  explicit Config(const wxString &path) : wxFileConfig("", "", path, "", wxCONFIG_USE_LOCAL_FILE) {}
  bool fail = false;
  bool Flush(bool current_only = false) override {
    return !fail && wxFileConfig::Flush(current_only);
  }
};
struct Credentials : ais::IAisCredentials {
  explicit Credentials(bool linked) : present(linked) {}
  bool present;
  ais::CredentialResult Read() const override {
    ais::CredentialResult r;
    if (present) {
      r.key.Assign("skg1_" + std::string(64, 'a'));
      r.status = ais::CredentialStatus::Ready;
    }
    return r;
  }
  ais::CredentialStatus Store(const ais::Secret &) override { return ais::CredentialStatus::Ready; }
  ais::CredentialStatus Remove() override { present = false; return ais::CredentialStatus::Removed; }
};
struct Transport : cloud::ICloudTransport {
  explicit Transport(std::shared_ptr<std::atomic<bool>> revoked) : revoked(std::move(revoked)) {}
  std::shared_ptr<std::atomic<bool>> revoked;
  cloud::HttpResult Request(const std::string &, const std::string &path,
                            const std::string &, const ais::Secret &) override {
    if (*revoked) return {true, 401, "{}"};
    if (path == "/api/device/status")
      return {true, 200, R"({"account":{"email":"test@example.test"},"cloud":{"active":true}})"};
    return {true, 200, "{}"};
  }
  void Cancel() override {}
};
std::unique_ptr<integration::OnlineCloud> Make(Config &config,
    const std::shared_ptr<std::atomic<bool>> &revoked, bool linked = true) {
  cloud::CloudOptions options;
  options.retry = 20ms;
  return std::make_unique<integration::OnlineCloud>(config,
      std::make_unique<cloud::CloudService>(std::make_unique<Credentials>(linked),
                                           std::make_unique<Transport>(revoked), options));
}
void preferences() {
  TemporaryProfile profile;
  auto revoked = std::make_shared<std::atomic<bool>>(false);
  {
    Config config(profile.path);
    config.Write(key, "1");
    config.Write("/Settings/ChartPath", "existing-chart-path");
    Check(config.Flush(), "initial consent persistence");
    auto cloud = Make(config, revoked);
    Check(Wait([&] { return cloud->Read().account.has_value(); }), "stored link initialization");
    Check(cloud->Read().share_enabled, "explicit saved consent is restored");
    Check(cloud->Unlink().ok, "unlink request");
    Check(config.Read(key, "") == "0", "unlink clears consent immediately");
    Check(Wait([&] { return cloud->Read().link == cloud::LinkState::Unlinked; }), "unlink finishes");
  }
  {
    Config config(profile.path);
    Check(config.Read(key, "") == "0", "unlink OFF survives profile reopen");
    auto cloud = Make(config, revoked);  // Another valid device credential must not inherit consent.
    Check(Wait([&] { return cloud->Read().account.has_value(); }), "replacement link initialization");
    Check(!cloud->Read().share_enabled, "replacement link stays private");
    Check(cloud->Share(true).ok, "new explicit sharing choice");
    config.fail = true;
    Check(!cloud->Share(false).ok, "failed OFF persistence is reported");
    Check(!cloud->Read().share_enabled, "failed persistence still stops local sharing");
    config.fail = false;
    cloud->Read();
    Check(config.Read(key, "") == "0", "OFF persistence recovers");
    Check(cloud->Share(true).ok, "second explicit sharing choice");
    *revoked = true;
    cloud->Refresh();
    Check(Wait([&] { return cloud->Read().link == cloud::LinkState::Unlinked; }), "remote revocation");
    Check(config.Read(key, "") == "0", "revocation clears persisted consent");
    Check(config.Read("/Settings/ChartPath", "") == "existing-chart-path", "other profile settings preserved");
  }
  {
    Config config(profile.path);
    Check(config.Read(key, "") == "0", "revocation OFF survives profile reopen");
    config.Write(key, "1");
    Check(config.Flush(), "stale consent setup");
    auto cloud = Make(config, revoked, false);
    Check(Wait([&] { return cloud->Read().initialized; }), "missing credential initialization");
    Check(!cloud->Read().share_enabled && config.Read(key, "") == "0", "missing credentials withdraw stale consent");
  }
}
}  // namespace
int main() {
  wxInitializer init;
  try {
    Check(init.IsOk(), "wx initialization");
    preferences();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "cloud preferences: " << checks << " checks passed\n";
}
