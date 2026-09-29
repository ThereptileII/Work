#pragma once
#include "integration/BuildFeatures.h"
#include <string>
#include <utility>

namespace opennav::integration {
// Copied from the actual OpenCPN transport, never inferred from an interface
// name, configuration permission, environment or a UI callback.
struct PilotOutputEndpoint {
  bool tcp = false, bidirectional = false, enabled = false;
  bool connected = false, actisense = false;
  std::string configured_address, peer_address;
  int configured_port = 0, peer_port = 0;
};
inline bool PilotOutputPermitted(const PilotOutputEndpoint &e) {
  return PilotLoopbackTestsEnabled() && e.tcp && e.bidirectional && e.enabled &&
         e.connected && e.actisense && e.configured_address == "127.0.0.1" &&
         e.peer_address == "127.0.0.1" && e.configured_port > 0 &&
         e.configured_port <= 65535 && e.peer_port == e.configured_port;
}
template <class Send>
bool DispatchPilotOutput(const PilotOutputEndpoint &endpoint, Send &&send) {
  // This final sink guard is independent of adapter capabilities and session
  // enablement. Product builds cannot invoke even a permissive callback.
  if (!PilotOutputPermitted(endpoint)) return false;
  return std::forward<Send>(send)();
}
} // namespace opennav::integration
