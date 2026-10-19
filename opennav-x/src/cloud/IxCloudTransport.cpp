#include "cloud/IxCloudTransport.h"
#include <ixwebsocket/IXHttpClient.h>
#include <ixwebsocket/IXNetSystem.h>
#include <mutex>

namespace opennav::cloud {
namespace {
constexpr std::size_t kMaxResponseBytes = 16384;
void Erase(std::string &value) {
  volatile char *p = value.empty() ? nullptr : &value[0];
  for (std::size_t i = 0; i < value.size(); ++i) p[i] = 0;
  value.clear();
}
class IxCloudTransport final : public ICloudTransport {
public:
  IxCloudTransport() : network_ready_(ix::initNetSystem()) {}
  ~IxCloudTransport() override {
    if (network_ready_) ix::uninitNetSystem();
  }
  HttpResult Request(const std::string &method, const std::string &path, const std::string &body,
                     const ais::Secret &token) override {
    HttpResult result;
    if (!network_ready_ || path.empty() || path[0] != '/') return result;
    const std::string url = std::string(kOrigin) + path;
    ix::HttpClient client(false);
    ix::SocketTLSOptions tls;
    tls.caFile = "SYSTEM";  // Never NONE; never an http downgrade.
    tls.disable_hostname_validation = false;
    client.setTLSOptions(tls);
    const auto verb = method == "GET" ? ix::HttpClient::kGet : ix::HttpClient::kPost;
    auto args = client.createRequest(url, verb);
    args->connectTimeout = 10;
    args->transferTimeout = 20;
    args->followRedirects = false;
    args->compress = false;
    args->verbose = false;
    args->extraHeaders["Accept"] = "application/json";
    args->extraHeaders["User-Agent"] = "SKAGER-cloud/1";
    if (verb == ix::HttpClient::kPost) args->extraHeaders["Content-Type"] = "application/json";
    if (!token.Empty()) {
      std::string authorization = "Bearer ";
      authorization.append(token.View().data(), token.View().size());
      args->extraHeaders["Authorization"] = authorization;
      Erase(authorization);
    }
    std::string received;
    bool too_large = false;
    auto *raw = args.get();
    args->onChunkCallback = [&received, &too_large, raw](const std::string &chunk) {
      if (too_large) return;
      if (received.size() + chunk.size() > kMaxResponseBytes) {
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
      response = client.request(url, verb, body, args);
    } catch (...) {
      response.reset();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_.reset();
    }
    Erase(args->extraHeaders["Authorization"]);
    // IX error strings may echo request details; they are never retained.
    if (!response || too_large || response->errorCode != ix::HttpErrorCode::Ok) return result;
    result.transport_ok = true;
    result.status = response->statusCode;
    result.body = std::move(received);
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
}  // namespace
std::unique_ptr<ICloudTransport> CreateIxCloudTransport() {
  return std::make_unique<IxCloudTransport>();
}
}  // namespace opennav::cloud
