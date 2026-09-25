#pragma once
#include "integration/MarineDecoder.h"
#include "integration/N2kSourceIdentity.h"
#include "adapters/BoatN2k.h"
#include "observable.h"
#include <algorithm>
#include <memory>

namespace opennav::integration {
// Subscriptions are owned/destroyed on the GUI thread. No connection ownership,
// output messages, retained upstream objects or high-rate raw logging.
class MarineBridge final {
public:
  MarineBridge();
  vessel::SensorRegistry &Sources() { return sources_; }
  const vessel::SensorRegistry &Sources() const { return sources_; }
  vessel::VesselState Merge(vessel::VesselState navigation,
                            vessel::Time now) const;
  std::vector<N2kIdentity> Identities() const {
    return identities_.Observations();
  }
  void SetBoatBridge(const adapters::BoatN2kBinding &binding);
  std::vector<vessel::SourceHealth> Health(vessel::Time now) const;
  std::string BoatBridgeStatus(vessel::Time now) const { return boat_.Status(now); }
  void SetBindings(std::vector<SignalKBinding> bindings) {
    application::ValidateSignalKMappings(bindings);
    const bool same =
        bindings.size() == bindings_.size() &&
        std::equal(bindings.begin(), bindings.end(), bindings_.begin(),
                   [](const auto &a, const auto &b) {
                     return a.path == b.path && a.quantity == b.quantity &&
                            a.scale == b.scale && a.offset == b.offset;
                   });
    if (!same)
      sources_.Clear(); // Never retain values interpreted by an old mapping.
    bindings_ = std::move(bindings);
  }

private:
  void Accept(std::vector<vessel::SensorObservation> samples, vessel::Time now);
  bool CheckConnection(const std::string &interface_id, vessel::Time observed);
  std::vector<std::unique_ptr<ObsListener>> listeners_;
  vessel::SensorRegistry sources_;
  N2kSourceIdentity identities_;
  adapters::BoatN2k boat_;
  adapters::BoatN2kBinding boat_binding_;
  std::map<std::string, std::uint64_t> network_generations_;
  std::vector<SignalKBinding> bindings_;
  std::optional<vessel::Time> last_received_;
};
} // namespace opennav::integration
