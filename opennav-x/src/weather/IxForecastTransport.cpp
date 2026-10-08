#include "weather/IxForecastTransport.h"
#include "weather/GribStream.h"
#include <ixwebsocket/IXHttpClient.h>
#include <ixwebsocket/IXNetSystem.h>
#include <mutex>

namespace opennav::weather {
namespace {
void Erase(std::string &value) {
  volatile char *p = value.empty() ? nullptr : &value[0];
  for (std::size_t i = 0; i < value.size(); ++i) p[i] = 0;
  value.clear();
}
class IxForecastTransport final : public IForecastTransport {
public:
  IxForecastTransport() : network_ready_(ix::initNetSystem()) {}
  ~IxForecastTransport() override {
    if (network_ready_) ix::uninitNetSystem();
  }
  HttpResult Post(const std::string &url, const std::string &body,
                  const ais::Secret &token) override {
    HttpResult result;
    if (!network_ready_ || token.Empty()) return result;
    ix::HttpClient client(false);
    ix::SocketTLSOptions tls;
    tls.caFile = "SYSTEM";  // Never NONE; never an http downgrade.
    tls.disable_hostname_validation = false;
    client.setTLSOptions(tls);
    auto args = client.createRequest(url, ix::HttpClient::kPost);
    args->connectTimeout = 10;
    args->transferTimeout = 30;
    args->followRedirects = false;
    args->compress = false;
    args->verbose = false;
    args->extraHeaders["Content-Type"] = "application/json";
    args->extraHeaders["Accept"] = "text/csv";
    args->extraHeaders["User-Agent"] = "SKAGER-weather/1";
    std::string authorization = "Bearer ";
    authorization.append(token.View().data(), token.View().size());
    args->extraHeaders["Authorization"] = authorization;
    Erase(authorization);
    std::string received;
    bool too_large = false;
    // A chunk callback keeps IX from buffering (or pre-reserving from an
    // untrusted Content-Length) the body itself; the bound is enforced here.
    auto *raw = args.get();
    args->onChunkCallback = [&received, &too_large, raw](const std::string &chunk) {
      if (too_large) return;
      if (received.size() + chunk.size() > gribstream::kMaxResponseBytes) {
        too_large = true;
        received.clear();
        raw->cancel = true;
        return;
      }
      received += chunk;
    };
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cancelled_) {
        Erase(args->extraHeaders["Authorization"]);
        return result;
      }
      current_ = args;
    }
    ix::HttpResponsePtr response;
    try {
      response = client.request(url, ix::HttpClient::kPost, body, args);
    } catch (...) {
      response.reset();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_.reset();
    }
    Erase(args->extraHeaders["Authorization"]);
    if (!response || too_large) {
      result.too_large = too_large;
      return result;
    }
    // IX error strings may echo request details; they are never retained.
    if (response->errorCode != ix::HttpErrorCode::Ok) return result;
    result.transport_ok = true;
    result.status = response->statusCode;
    const auto retry = response->headers.find("Retry-After");
    if (retry != response->headers.end()) result.retry_after = ParseRetryAfter(retry->second);
    if (result.status == 200) result.body = std::move(received);
    return result;
  }
  void Cancel() override {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelled_ = true;
    if (current_) current_->cancel = true;
  }

private:
  bool network_ready_;
  std::mutex mutex_;
  bool cancelled_ = false;
  ix::HttpRequestArgsPtr current_;
};
} // namespace
std::unique_ptr<IForecastTransport> CreateIxForecastTransport() {
  return std::make_unique<IxForecastTransport>();
}
} // namespace opennav::weather
