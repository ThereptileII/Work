// Dedicated local-server test driver, never linked into the installed product.
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXSocket.h>
#include <ixwebsocket/IXNetSystem.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
int main(int argc, char **argv) {
  if (argc != 5) return 2;
  ix::initNetSystem();
  ix::Socket unavailable;
  if (unavailable.getConnectionInfo().family != ix::SocketConnectionInfo::Family::Unavailable)
    return 7;
  ix::WebSocket client;
  const std::string url(argv[1]);
  // Only this dedicated driver permits local URLs/trust anchors.
  if (url.find("wss://127.0.0.1:") != 0 && url.find("wss://localhost:") != 0)
    return 3;
  client.setUrl(url);
  client.setUntrustedClientLimits(65536);
  client.disableAutomaticReconnection();
  client.enablePerMessageDeflate();
  if (std::string(argv[2]) != "SYSTEM") {
    ix::SocketTLSOptions tls;
    tls.caFile = argv[2];
    client.setTLSOptions(tls);
  }
  std::mutex mutex;
  std::condition_variable changed;
  bool ended = false, invalidObservation = false;
  unsigned messages = 0, opened = 0, errors = 0, code = 0;
  size_t maximum = 0;
  const unsigned expected = std::stoul(argv[3]);
  const unsigned expectedCode = std::stoul(argv[4]);
  client.setOnMessageCallback([&](const ix::WebSocketMessagePtr &msg) {
    std::lock_guard<std::mutex> lock(mutex);
    if (msg->type == ix::WebSocketMessageType::Open) {
      ++opened;
      const auto &connection = msg->openInfo.connectionInfo;
      invalidObservation = connection.family != ix::SocketConnectionInfo::Family::IPv4 ||
          !connection.local.port || !connection.remote.port ||
          connection.capturedAt <= std::chrono::steady_clock::time_point{} ||
          connection.capturedAt > std::chrono::steady_clock::now();
    } else if (msg->openInfo.connectionInfo.family != ix::SocketConnectionInfo::Family::Unavailable)
      invalidObservation = true;
    if (msg->type == ix::WebSocketMessageType::Message) {
      ++messages;
      maximum = (std::max)(maximum, msg->str.size());
      if (messages >= expected && expected > 0) ended = true;
    }
    if (msg->type == ix::WebSocketMessageType::Error) { ++errors; ended = true; }
    if (msg->type == ix::WebSocketMessageType::Close) {
      code = msg->closeInfo.code;
      ended = true;
    }
    changed.notify_all();
  });
  client.start();
  bool finished;
  {
    std::unique_lock<std::mutex> lock(mutex);
    finished = changed.wait_for(lock, std::chrono::seconds(6), [&] { return ended; });
  }
  client.stop();
  std::cout << "messages=" << messages << " maximum=" << maximum << " opened=" << opened
            << " errors=" << errors << " close=" << code << '\n';
  ix::uninitNetSystem();
  if (!finished || messages != expected || maximum > 65536 || invalidObservation) return 4;
  if (expectedCode && code != expectedCode) return 5;
  if (!expected && !expectedCode && (opened || !errors)) return 6;
}
