#pragma once
// SKAGER Cloud background worker (SCRUM-44, SCRUM-369). All network I/O and
// credential access happen on its own thread; every public call returns at
// once and the UI reads copies. Cloud is optional: nothing here can block or
// change navigation, and losing the network only delays sharing.
#include "ais/Credentials.h"
#include "cloud/CloudProtocol.h"
#include <memory>

namespace opennav::cloud {
using WallTime = std::chrono::system_clock::time_point;

struct HttpResult {
  bool transport_ok = false;  // false: DNS/connect/TLS/timeout/cancel.
  int status = 0;
  std::string body;           // Bounded by the transport.
};

class ICloudTransport {
public:
  virtual ~ICloudTransport() = default;
  // Blocking HTTPS request to kOrigin + path on the worker thread. An empty
  // token sends no Authorization header. Never logs the token or body.
  virtual HttpResult Request(const std::string &method, const std::string &path,
                             const std::string &body, const ais::Secret &token) = 0;
  virtual void Cancel() = 0;  // Any thread; aborts the current request.
};

enum class LinkState { Unlinked, Starting, WaitingForApproval, Linked };

struct CloudSnapshot {
  LinkState link = LinkState::Unlinked;
  // While waiting for approval: the code to type at the verification page.
  std::string user_code, verification_url;
  WallTime code_expires{};
  // Known once a linked device has reached the service.
  std::optional<AccountStatus> account;
  bool share_enabled = false;
  std::optional<WallTime> last_shared;
  // Plain-language outcome of the latest action, for the settings page.
  std::string message;
  bool busy = false;  // A request is in flight.
  bool initialized = false;  // Platform credential read has completed.
  bool unlink_pending = false;  // A failed credential deletion is still retried.
};

struct CloudOptions {
  std::string version = "0.0.0";
  std::string platform = "windows";
  std::chrono::milliseconds status_period{std::chrono::minutes(10)};
  std::chrono::milliseconds retry{std::chrono::seconds(60)};
  // Tests shorten the server-supplied poll interval; production uses it.
  std::optional<std::chrono::milliseconds> poll_interval;
  std::chrono::milliseconds telemetry_interval{kTelemetryInterval};
};

class CloudService final {
public:
  CloudService(std::unique_ptr<ais::IAisCredentials> credentials,
               std::unique_ptr<ICloudTransport> transport, CloudOptions options);
  ~CloudService();  // Cancels the transport and joins.
  CloudService(const CloudService &) = delete;
  CloudService &operator=(const CloudService &) = delete;

  void StartLink(const std::string &device_name);
  void CancelLink();
  void Unlink();
  void RefreshStatus();
  void SetShareEnabled(bool enabled);
  // Application thread. Copies at most once per telemetry interval and only
  // while linked with sharing on; the newest snapshot replaces an unsent one.
  void Observe(const vessel::VesselState &state, vessel::Time now);
  CloudSnapshot Read() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace opennav::cloud
