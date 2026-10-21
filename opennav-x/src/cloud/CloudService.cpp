#include "cloud/CloudService.h"
#include <condition_variable>
#include <mutex>
#include <thread>

namespace opennav::cloud {
namespace {
using Steady = std::chrono::steady_clock;
}

class CloudService::Impl {
public:
  Impl(std::unique_ptr<ais::IAisCredentials> credentials,
       std::unique_ptr<ICloudTransport> transport, CloudOptions options)
      : credentials_(std::move(credentials)), transport_(std::move(transport)),
        options_(std::move(options)) {
    worker_ = std::thread([this] { Run(); });
  }
  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    transport_->Cancel();
    cv_.notify_all();
    worker_.join();
  }

  void StartLink(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_.link != LinkState::Unlinked) return;
    link_name_ = name;
    snapshot_.link = LinkState::Starting;
    snapshot_.message = "Requesting a pairing code...";
    cv_.notify_all();
  }
  void CancelLink() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_.link != LinkState::Starting && snapshot_.link != LinkState::WaitingForApproval) return;
    ClearLinkLocked("Pairing cancelled.");
  }
  void Unlink() {
    std::lock_guard<std::mutex> lock(mutex_);
    unlink_ = true;
    cv_.notify_all();
  }
  void RefreshStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    status_due_ = Steady::now();
    cv_.notify_all();
  }
  void SetShareEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_.share_enabled == enabled) return;
    snapshot_.share_enabled = enabled;
    payload_.reset();
    last_encoded_ = {};
    // Tell the service once so the web stops showing a live boat.
    if (!enabled && snapshot_.link == LinkState::Linked) stop_sharing_ = true;
    cv_.notify_all();
  }
  void Observe(const vessel::VesselState &state, vessel::Time now) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!snapshot_.share_enabled || snapshot_.link != LinkState::Linked ||
          (snapshot_.account && !snapshot_.account->cloud_active))
        return;
      if (last_encoded_ != vessel::Time{} && now - last_encoded_ < options_.telemetry_interval) return;
      last_encoded_ = now;
    }
    // Encoding is cheap and touches only the caller's copy of the state.
    auto encoded = EncodeTelemetry(state, now);
    if (!encoded) return;
    std::lock_guard<std::mutex> lock(mutex_);
    payload_ = std::move(encoded);
    cv_.notify_all();
  }
  CloudSnapshot Read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

private:
  enum class Job { None, Unlink, StartLink, Poll, Status, StopSharing, Upload };

  void ClearLinkLocked(const std::string &message) {
    snapshot_.link = LinkState::Unlinked;
    snapshot_.user_code.clear();
    snapshot_.verification_url.clear();
    device_code_.clear();
    snapshot_.message = message;
  }
  // Token rejected by the service: the device was unlinked in the account.
  void ForgetLocked(const std::string &message) {
    token_.Clear();
    credentials_->Remove();
    snapshot_.account.reset();
    snapshot_.last_shared.reset();
    payload_.reset();
    stop_sharing_ = false;
    ClearLinkLocked(message);
  }

  Job NextLocked(Steady::time_point now, Steady::time_point &wake) {
    if (unlink_) return Job::Unlink;
    if (snapshot_.link == LinkState::Starting) return Job::StartLink;
    if (snapshot_.link == LinkState::WaitingForApproval) {
      if (std::chrono::system_clock::now() >= snapshot_.code_expires) {
        ClearLinkLocked("The pairing code expired. Choose Link again for a new code.");
        return Job::None;
      }
      if (now >= next_poll_) return Job::Poll;
      wake = std::min(wake, next_poll_);
      return Job::None;
    }
    if (snapshot_.link != LinkState::Linked) return Job::None;
    if (now >= status_due_) return Job::Status;
    wake = std::min(wake, status_due_);
    if (stop_sharing_) return Job::StopSharing;
    if (payload_ && snapshot_.share_enabled && snapshot_.account && snapshot_.account->cloud_active) {
      if (now >= upload_after_) return Job::Upload;
      wake = std::min(wake, upload_after_);
    }
    return Job::None;
  }

  void Run() {
    {
      // Startup: a stored token means this SKAGER is already linked.
      auto stored = credentials_->Read();
      std::lock_guard<std::mutex> lock(mutex_);
      if (stored.status == ais::CredentialStatus::Ready && ValidToken(stored.key.View())) {
        token_ = std::move(stored.key);
        snapshot_.link = LinkState::Linked;
        status_due_ = Steady::now();
      }
    }
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
      const auto now = Steady::now();
      auto wake = now + std::chrono::hours(1);
      const auto job = NextLocked(now, wake);
      if (job == Job::None) {
        cv_.wait_until(lock, wake);
        continue;
      }
      snapshot_.busy = true;
      Execute(job, lock);
      snapshot_.busy = false;
    }
  }

  // Called and returns with the lock held; released around network requests.
  HttpResult Call(std::unique_lock<std::mutex> &lock, const char *method, const std::string &path,
                  const std::string &body, bool authorized) {
    ais::Secret token;
    if (authorized) token.Assign(token_.View());
    lock.unlock();
    auto result = transport_->Request(method, path, body, token);
    token.Clear();
    lock.lock();
    return result;
  }

  void Execute(Job job, std::unique_lock<std::mutex> &lock) {
    switch (job) {
    case Job::Unlink: {
      unlink_ = false;
      if (snapshot_.link == LinkState::Linked) {
        // Best effort: the local credential is removed even when offline.
        Call(lock, "POST", "/api/device/unlink", "{}", true);
      }
      ForgetLocked("This SKAGER is no longer linked to an account.");
      return;
    }
    case Job::StartLink: {
      const auto body = EncodeLinkRequest(link_name_, options_.platform, options_.version);
      const auto result = Call(lock, "POST", "/api/device/link", body, false);
      if (snapshot_.link != LinkState::Starting || stop_) return;  // Cancelled meanwhile.
      const auto offer = result.transport_ok && result.status == 200 ? ParseLinkOffer(result.body)
                                                                      : std::nullopt;
      if (!offer) {
        ClearLinkLocked(result.transport_ok ? "SKAGER Cloud could not start pairing. Try again later."
                                            : "No connection to SKAGER Cloud. Check the internet connection.");
        return;
      }
      device_code_ = offer->device_code;
      poll_interval_ = options_.poll_interval.value_or(offer->interval);
      next_poll_ = Steady::now() + poll_interval_;
      snapshot_.link = LinkState::WaitingForApproval;
      snapshot_.user_code = offer->user_code;
      snapshot_.verification_url = offer->verification_url;
      snapshot_.code_expires = std::chrono::system_clock::now() + offer->expires_in;
      snapshot_.message = "Waiting for approval in your SKAGER account.";
      return;
    }
    case Job::Poll: {
      const auto result = Call(lock, "POST", "/api/device/link/poll", EncodePoll(device_code_), false);
      if (snapshot_.link != LinkState::WaitingForApproval || stop_) return;
      next_poll_ = Steady::now() + poll_interval_;
      if (!result.transport_ok) return;  // Retry until the code expires.
      const auto poll = result.status == 200 || result.status == 429 ? ParsePoll(result.body) : PollResult{};
      switch (poll.state) {
      case PollState::Pending: return;
      case PollState::SlowDown: poll_interval_ += std::chrono::seconds(5); return;
      case PollState::Denied: ClearLinkLocked("Pairing was declined in the account."); return;
      case PollState::Expired:
        ClearLinkLocked("The pairing code expired. Choose Link again for a new code.");
        return;
      case PollState::Invalid: return;
      case PollState::Approved: break;
      }
      ais::Secret token;
      token.Assign(poll.token);
      const auto stored = credentials_->Store(token);
      token_ = std::move(token);
      snapshot_.link = LinkState::Linked;
      snapshot_.user_code.clear();
      snapshot_.verification_url.clear();
      device_code_.clear();
      snapshot_.message = stored == ais::CredentialStatus::Ready
          ? "Linked to your SKAGER account."
          : "Linked for this session. The link could not be stored securely and will be lost on restart.";
      status_due_ = Steady::now();
      return;
    }
    case Job::Status: {
      status_due_ = Steady::now() + options_.retry;
      const auto result = Call(lock, "GET", "/api/device/status", {}, true);
      if (snapshot_.link != LinkState::Linked || stop_ || unlink_) return;
      if (result.transport_ok && result.status == 401) {
        ForgetLocked("This SKAGER was unlinked from the account.");
        return;
      }
      const auto status = result.transport_ok && result.status == 200 ? ParseStatus(result.body)
                                                                       : std::nullopt;
      if (!status) return;  // Keep the last known account; retry later.
      snapshot_.account = status;
      status_due_ = Steady::now() + options_.status_period;
      return;
    }
    case Job::StopSharing: {
      stop_sharing_ = false;
      Call(lock, "POST", "/api/device/telemetry/stop", "{}", true);
      if (snapshot_.link == LinkState::Linked) snapshot_.message = "Boat data sharing is off.";
      return;
    }
    case Job::Upload: {
      auto body = std::move(*payload_);
      payload_.reset();
      const auto result = Call(lock, "POST", "/api/device/telemetry", body, true);
      if (snapshot_.link != LinkState::Linked || stop_ || unlink_) return;
      if (!result.transport_ok) {
        upload_after_ = Steady::now() + options_.retry;
        snapshot_.message = "Boat data will be sent when the internet connection returns.";
        return;
      }
      if (result.status == 202) {
        snapshot_.last_shared = std::chrono::system_clock::now();
        snapshot_.message = "Sharing boat data with SKAGER Cloud.";
      } else if (result.status == 401) {
        ForgetLocked("This SKAGER was unlinked from the account.");
      } else if (result.status == 402) {
        if (snapshot_.account) snapshot_.account->cloud_active = false;
        snapshot_.message = "Boat data needs an active SKAGER Cloud subscription.";
        status_due_ = Steady::now() + options_.retry;
      } else if (result.status != 429) {
        upload_after_ = Steady::now() + options_.retry;
        snapshot_.message = "SKAGER Cloud did not accept the boat data. It will try again.";
      }
      return;
    }
    case Job::None: return;
    }
  }

  std::unique_ptr<ais::IAisCredentials> credentials_;
  std::unique_ptr<ICloudTransport> transport_;
  const CloudOptions options_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  CloudSnapshot snapshot_;
  ais::Secret token_;
  std::string link_name_, device_code_;
  std::chrono::milliseconds poll_interval_{5000};
  Steady::time_point next_poll_{}, status_due_{}, upload_after_{};
  std::optional<std::string> payload_;
  vessel::Time last_encoded_{};
  bool stop_ = false, unlink_ = false, stop_sharing_ = false;
  std::thread worker_;
};

CloudService::CloudService(std::unique_ptr<ais::IAisCredentials> credentials,
                           std::unique_ptr<ICloudTransport> transport, CloudOptions options)
    : impl_(std::make_unique<Impl>(std::move(credentials), std::move(transport), std::move(options))) {}
CloudService::~CloudService() = default;
void CloudService::StartLink(const std::string &name) { impl_->StartLink(name); }
void CloudService::CancelLink() { impl_->CancelLink(); }
void CloudService::Unlink() { impl_->Unlink(); }
void CloudService::RefreshStatus() { impl_->RefreshStatus(); }
void CloudService::SetShareEnabled(bool enabled) { impl_->SetShareEnabled(enabled); }
void CloudService::Observe(const vessel::VesselState &state, vessel::Time now) { impl_->Observe(state, now); }
CloudSnapshot CloudService::Read() const { return impl_->Read(); }
}  // namespace opennav::cloud
