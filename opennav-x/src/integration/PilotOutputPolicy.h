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
  bool serial = false;
};
inline bool PilotOutputPermitted(const PilotOutputEndpoint &e) {
  if (e.serial)
    return PilotManualSerialEnabled() && !e.tcp && e.bidirectional && e.enabled &&
           e.connected && e.actisense;
  return PilotLoopbackTestsEnabled() && e.tcp && e.bidirectional && e.enabled &&
         e.connected && e.actisense && e.configured_address == "127.0.0.1" &&
         e.peer_address == "127.0.0.1" && e.configured_port > 0 &&
         e.configured_port <= 65535 && e.peer_port == e.configured_port;
}
inline std::string PilotControlBlocker(const PilotOutputEndpoint &e,
                                      const std::string &iface) {
  // Offering setup must obey the same transport boundary as the final sink.
  // An empty reason permits only the confirmation/binding flow, never output.
  if (!e.enabled || !e.connected)
    return "Pilot connection unavailable: " + iface;
  if (!e.bidirectional)
    return "Turn on \"send data\" for the " + iface +
           " NMEA 2000 connection in OpenCPN Connections";
  if (!PilotOutputPermitted(e))
    return "Control unavailable on " + iface +
           ": this transport is not qualified for pilot output";
  return {};
}
template <class Send>
bool DispatchPilotOutput(const PilotOutputEndpoint &endpoint, Send &&send) {
  // This final sink guard is independent of adapter capabilities and session
  // enablement. Only the qualified serial class or isolated test peer can pass.
  if (!PilotOutputPermitted(endpoint)) return false;
  return std::forward<Send>(send)();
}
} // namespace opennav::integration
