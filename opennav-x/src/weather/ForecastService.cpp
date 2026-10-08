#include "weather/ForecastService.h"
#include <condition_variable>
#include <mutex>
#include <thread>

namespace opennav::weather {
class ForecastService::Impl {
public:
  Impl(std::unique_ptr<ais::IAisCredentials> credentials,
       std::unique_ptr<IForecastTransport> transport)
      : credentials_(std::move(credentials)), transport_(std::move(transport)),
        worker_([this] { Run(); }) {}
  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    transport_->Cancel();
    changed_.notify_all();
    worker_.join();
  }
  template <typename F> void Mutate(F &&f) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      f(session_);
      wake_ = true;
    }
    changed_.notify_all();
  }
  void Disable() {
    Mutate([](ForecastSession &s) { s.SetEnabled(false); });
  }
  ForecastSnapshot Read(WallTime now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_.Snapshot(now);
  }
  ConnectionTest LastTest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_.LastTest();
  }
  bool TestPending() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_.TestPending();
  }

private:
  void Run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!stop_) {
      session_.Prune(std::chrono::system_clock::now());
      auto request = session_.Next(std::chrono::steady_clock::now(),
                                   std::chrono::system_clock::now());
      if (request) {
        lock.unlock();
        auto credential = credentials_->Read();
        HttpResult result;
        const bool ready = credential.status == ais::CredentialStatus::Ready;
        if (ready) result = transport_->Post(request->url, request->body, credential.key);
        credential.key.Clear();
        lock.lock();
        if (!ready)
          session_.CredentialMissing(request->id,
                                     credential.status != ais::CredentialStatus::Missing);
        else
          session_.Complete(request->id, result, std::chrono::steady_clock::now(),
                            std::chrono::system_clock::now());
        continue;
      }
      changed_.wait_for(lock, std::chrono::seconds(5), [this] { return stop_ || wake_; });
      wake_ = false;
    }
  }
  std::unique_ptr<ais::IAisCredentials> credentials_;
  std::unique_ptr<IForecastTransport> transport_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  ForecastSession session_;
  bool stop_ = false, wake_ = false;
  std::thread worker_;
};

ForecastService::ForecastService(std::unique_ptr<ais::IAisCredentials> credentials,
                                 std::unique_ptr<IForecastTransport> transport)
    : impl_(std::make_unique<Impl>(std::move(credentials), std::move(transport))) {}
ForecastService::~ForecastService() = default;
void ForecastService::SetEnabled(bool enabled) {
  impl_->Mutate([enabled](ForecastSession &s) { s.SetEnabled(enabled); });
}
void ForecastService::SetModel(const std::string &model) {
  impl_->Mutate([&model](ForecastSession &s) { s.SetModel(model); });
}
void ForecastService::SetQuery(const ForecastQuery &query) {
  impl_->Mutate([&query](ForecastSession &s) {
    s.SetQuery(query, std::chrono::system_clock::now());
  });
}
void ForecastService::CredentialChanged(bool present) {
  impl_->Mutate([present](ForecastSession &s) { s.SetCredentialPresent(present); });
}
void ForecastService::Test() {
  impl_->Mutate([](ForecastSession &s) { s.RequestTest(); });
}
ForecastSnapshot ForecastService::Read(WallTime now) const { return impl_->Read(now); }
ConnectionTest ForecastService::LastTest() const { return impl_->LastTest(); }
bool ForecastService::TestPending() const { return impl_->TestPending(); }
} // namespace opennav::weather
