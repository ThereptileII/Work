#pragma once
#include "integration/MarineDecoder.h"
#include "observable.h"
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
  void SetBindings(std::vector<SignalKBinding> bindings) {
    bindings_ = std::move(bindings);
  }

private:
  void Accept(std::vector<vessel::SensorObservation> samples, vessel::Time now);
  std::vector<std::unique_ptr<ObsListener>> listeners_;
  vessel::SensorRegistry sources_;
  std::vector<SignalKBinding> bindings_;
  std::optional<vessel::Time> last_received_;
};
} // namespace opennav::integration
