#include "ais/AisStreamProvider.h"
#include "ais/AisStreamSession.h"
#include "ais/Credentials.h"
#include <condition_variable>
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <mutex>
#include <thread>

namespace opennav::ais {
namespace {
void Erase(std::string &value) {
  volatile char *p = value.empty() ? nullptr : &value[0];
  for (std::size_t i = 0; i < value.size(); ++i)
    p[i] = 0;
  value.clear();
}
unsigned Entropy() {
  return static_cast<unsigned>(vessel::Clock::now().time_since_epoch().count());
}
ConnectionObservation Observe(const ix::SocketConnectionInfo &input,
                              std::uint64_t generation) {
  ConnectionObservation result;
  if (input.family == ix::SocketConnectionInfo::Family::IPv4)
    result.family = AddressFamily::IPv4;
  else if (input.family == ix::SocketConnectionInfo::Family::IPv6)
    result.family = AddressFamily::IPv6;
  else
    return result;
  result.local = {input.local.address, input.local.port, input.local.scope};
  result.remote = {input.remote.address, input.remote.port, input.remote.scope};
  result.captured_at = input.capturedAt;
  result.generation = generation;
  return result;
}
} // namespace
class AisStreamProvider::Impl {
public:
  Impl(std::unique_ptr<IAisCredentials> credentials, std::string url,
       std::string ca)
      : credentials_(std::move(credentials)), url_(std::move(url)),
        ca_(std::move(ca)), worker_([this] { Run(); }) {}
  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
      session_.Enable(false, vessel::Clock::now());
      ++generation_;
    }
    changed_.notify_all();
    worker_.join();
  }
  void Enable(bool value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_.Enable(value, vessel::Clock::now()))
      ++generation_; // actual intent transition invalidates callbacks/credential reads
    Wake();
  }
  bool ViewportChanged(Viewport viewport) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool accepted = session_.ObserveViewport(viewport);
    if (accepted)
      Wake();
    return accepted;
  }
  void CredentialsChanged() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++generation_; // also invalidate a credential read while disabled/offline
    session_.RetryCredentials(vessel::Clock::now());
    Wake();
  }
#ifdef OPENNAV_AIS_TEST_TRANSPORT
  // Set once before ForTest hands the disabled provider to its caller.
  void SetOpenGateForTest(std::function<void()> gate) { open_gate_ = std::move(gate); }
#endif
  ProviderSnapshot Read(vessel::Time now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_.Read(now);
  }

private:
  void Wake() {
    wake_ = true;
    changed_.notify_all();
  }
  void Message(std::uint64_t generation,
               const ix::WebSocketMessagePtr &message) {
#ifdef OPENNAV_AIS_TEST_TRANSPORT
    // Real IX has completed TLS/HTTP and released its transport lock here.
    // Dedicated tests can delay delivery without holding the provider mutex.
    if (message->type == ix::WebSocketMessageType::Open && open_gate_)
      open_gate_();
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_ || generation != generation_)
      return;
    const auto now = vessel::Clock::now();
    try {
      switch (message->type) {
      case ix::WebSocketMessageType::Open:
        session_.Opened(now, Observe(message->openInfo.connectionInfo, generation));
        break;
      case ix::WebSocketMessageType::Message:
        // AISStream's UTF-8 JSON is carried in binary frames. The codec
        // validates UTF-8/schema after transport has already bounded
        // wire/inflated bytes.
        session_.Receive(message->str, now, std::chrono::system_clock::now(),
                         Entropy());
        break;
      case ix::WebSocketMessageType::Close:
      case ix::WebSocketMessageType::Error:
        // Never retain/log peer close text or the library's error string.
        session_.Disconnected(now, Entropy());
        break;
      default:
        break;
      }
    } catch (...) {
      session_.Disconnected(now, Entropy());
    }
    Wake();
  }
  void Run() {
    const bool network_ready = ix::initNetSystem();
    std::unique_ptr<ix::WebSocket> socket;
    Secret key;
    while (true) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (stop_)
        break;
      const auto now = vessel::Clock::now();
      session_.Tick(now, Entropy());
      if (socket && session_.ShouldClose()) {
        ++generation_;
        lock.unlock();
        socket->stop(); // never joins a socket thread while holding the state
                        // mutex
        socket.reset();
        key.Clear();
        continue;
      }
      if (!socket && session_.NeedsConnection(now)) {
        const auto credential_generation = generation_;
        lock.unlock();
        auto credential = credentials_->Read();
        lock.lock();
        if (stop_)
          break;
        if (credential_generation != generation_ ||
            !session_.NeedsConnection(vessel::Clock::now()))
          continue;
        if (credential.status != CredentialStatus::Ready) {
          session_.CredentialMissing(vessel::Clock::now());
          continue;
        }
        session_.Connecting(vessel::Clock::now());
        if (!network_ready) {
          session_.Disconnected(vessel::Clock::now(), Entropy());
          continue;
        }
        key = std::move(credential.key);
        socket = std::make_unique<ix::WebSocket>();
        socket->setUrl(url_);
        socket->setUntrustedClientLimits(65536);
        socket
            ->disableAutomaticReconnection(); // the bounded policy owns retries
        socket->setHandshakeTimeout(10);
        socket->setPingInterval(30);
        socket->enablePerMessageDeflate();
        ix::SocketTLSOptions tls;
        tls.caFile =
            ca_; // SYSTEM in production; never NONE, never ws downgrade
        tls.disable_hostname_validation = false;
        socket->setTLSOptions(tls);
        const auto generation = ++generation_;
        socket->setOnMessageCallback(
            [this, generation](const ix::WebSocketMessagePtr &message) {
              Message(generation, message);
            });
        lock.unlock();
        socket->start();
        continue;
      }
      if (socket) {
        const auto area = session_.PendingSubscription(now);
        if (!area.empty()) {
          auto wire = AisStreamSubscription(key.View(), area);
          if (!wire)
            session_.CredentialMissing(now);
          else {
            // The small complete subscription is sent as one message. Keeping
            // the state lock here makes viewport selection and Sent coherent.
            const auto sent = socket->sendText(*wire);
            Erase(*wire);
            if (sent.success && !sent.compressionError)
              session_.SubscriptionSent(vessel::Clock::now());
            else
              session_.Disconnected(vessel::Clock::now(), Entropy());
          }
        }
      }
      changed_.wait_for(lock, std::chrono::milliseconds(100),
                        [this] { return stop_ || wake_; });
      wake_ = false;
    }
    if (socket)
      socket->stop();
    key.Clear();
    if (network_ready)
      ix::uninitNetSystem();
  }
  std::unique_ptr<IAisCredentials> credentials_;
  std::string url_, ca_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  AisStreamSession session_;
  bool stop_ = false, wake_ = false;
  std::uint64_t generation_ = 0;
  std::thread worker_;
#ifdef OPENNAV_AIS_TEST_TRANSPORT
  std::function<void()> open_gate_;
#endif
};
AisStreamProvider::AisStreamProvider()
    : AisStreamProvider(CreateAisCredentials(),
                        "wss://stream.aisstream.io/v0/stream", "SYSTEM") {}
AisStreamProvider::AisStreamProvider(
    std::unique_ptr<IAisCredentials> credentials, std::string url,
    std::string ca)
    : impl_(std::make_unique<Impl>(std::move(credentials), std::move(url),
                                   std::move(ca))) {}
AisStreamProvider::~AisStreamProvider() = default;
void AisStreamProvider::SetEnabled(bool enabled) { impl_->Enable(enabled); }
bool AisStreamProvider::ObserveViewport(Viewport viewport) {
  return impl_->ViewportChanged(viewport);
}
void AisStreamProvider::CredentialChanged() { impl_->CredentialsChanged(); }
ProviderSnapshot AisStreamProvider::Read(vessel::Time now) const {
  return impl_->Read(now);
}
#ifdef OPENNAV_AIS_TEST_TRANSPORT
std::unique_ptr<AisStreamProvider>
AisStreamProvider::ForTest(std::unique_ptr<IAisCredentials> credentials,
                           const std::string &url, const std::string &ca,
                           std::function<void()> before_open) {
  if (!credentials ||
      (url.find("wss://127.0.0.1:") != 0 && url.find("wss://localhost:") != 0))
    return {};
  auto provider = std::unique_ptr<AisStreamProvider>(
      new AisStreamProvider(std::move(credentials), url, ca));
  provider->impl_->SetOpenGateForTest(std::move(before_open));
  return provider;
}
#endif
} // namespace opennav::ais
