#pragma once
// SCRUM-379/380: SKAGER Helm host. SKAGER starts the Helm Server
// (tools/helm-server, skager-helm[.exe]) as a child process and talks to it
// over stdin/stdout, one JSON object per line. The server owns pairing, the
// screen stream and touch input; this side only shows its state and relays
// the owner's choices. When SKAGER exits, the pipe closes and Helm stops.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace opennav::helm {

enum class HostState {
  Off,       // Switched off in SKAGER.
  Missing,   // skager-helm is not part of this installation.
  Starting,
  Ready,     // Listening; phones can pair and connect.
  Failed,    // Stopped unexpectedly; retried automatically.
};

struct HelmDevice {
  std::string id, name, paired_at, last_seen;
  bool connected = false;
};

struct HelmPairing {
  std::string key;   // ABCD-EFGH-JKLM-NPQR
  std::string link;  // app.skager.helm://pair?...
  int qr_size = 0;
  std::vector<std::string> qr_rows;  // '1' dark, '0' light; no quiet zone
  std::chrono::system_clock::time_point expires{};
};

struct HelmSnapshot {
  HostState state = HostState::Off;
  std::string message;  // Last problem or outcome, for the owner.
  int port = 0;
  std::vector<std::string> addresses;  // host:port the phone can use
  std::optional<HelmPairing> pairing;
  std::vector<HelmDevice> devices;
  std::string connected_name;  // Phone showing SKAGER now, if any.
  bool screen_available = true;
  std::string version;
  std::uint64_t revision = 0;  // Changes whenever anything above changes.
};

struct HelmOptions {
  std::string executable;  // Full path to skager-helm(.exe).
  std::string state_dir;   // Profile directory for paired devices.
  std::string name = "SKAGER";
  std::string window;      // HWND or X11 window id to show; empty = primary screen.
  std::string log_file;    // Server diagnostics (stderr); empty = discarded.
  std::vector<std::string> extra_args;
  std::chrono::milliseconds restart_delay{3000};
  int max_restarts = 5;  // In a row without reaching Ready.
};

// One running skager-helm process. Separate so tests can use a fake server.
class HelmProcess {
public:
  virtual ~HelmProcess() = default;
  virtual bool Start(const std::string &executable, const std::vector<std::string> &args,
                     const std::string &log_file, std::string &error) = 0;
  virtual bool WriteLine(const std::string &line) = 0;
  // Blocks for the next line; false once the process has closed its output.
  virtual bool ReadLine(std::string &line) = 0;
  // Closes stdin (Helm then exits), waits briefly, then terminates it.
  virtual void Stop() = 0;
};
std::unique_ptr<HelmProcess> CreateHelmProcess();

// Applies one event line from the server to the snapshot. Returns false for an
// unreadable line. "input" events set *input_at (used by the autopilot guard).
bool ApplyEvent(HelmSnapshot &snapshot, const std::string &line,
                std::chrono::steady_clock::time_point now,
                std::atomic<std::int64_t> *input_at = nullptr);

// The command lines SKAGER sends.
std::string PairCommand();
std::string CancelPairCommand();
std::string RevokeCommand(const std::string &device_id);
std::string DisconnectCommand();
std::string NameCommand(const std::string &name);
std::string PilotCommand(const std::string &status_json);

class HelmHost {
public:
  using Factory = std::function<std::unique_ptr<HelmProcess>()>;
  explicit HelmHost(Factory factory = CreateHelmProcess);
  ~HelmHost();
  HelmHost(const HelmHost &) = delete;
  HelmHost &operator=(const HelmHost &) = delete;

  // Starts (or restarts with new options) the server in the background.
  void Start(const HelmOptions &options);
  // Stops the server; connected phones are told SKAGER Helm stopped.
  void Stop();
  HelmSnapshot Read() const;
  bool Running() const;

  void Pair();
  void CancelPair();
  void Revoke(const std::string &device_id);
  void Disconnect();
  void SetName(const std::string &name);
  // Autopilot status for the phone (read-only there). Sent when it changes.
  void SetPilot(const std::string &status_json);

  // True when the latest touch or key from a Helm phone arrived within the
  // window: SKAGER refuses autopilot commands that may have come from Helm.
  bool RecentInput(std::chrono::milliseconds window) const;

private:
  void Supervise(HelmOptions options);
  bool Send(const std::string &line);
  void Update(const std::function<void(HelmSnapshot &)> &change);

  Factory factory_;
  mutable std::mutex mutex_;
  std::condition_variable wake_;
  HelmSnapshot snapshot_;
  std::shared_ptr<HelmProcess> process_;  // Shared so Stop() and writes never use a freed process.
  std::thread worker_;
  bool stopping_ = false;
  std::string pilot_;
  std::atomic<std::int64_t> input_at_{0};
};

}  // namespace opennav::helm
