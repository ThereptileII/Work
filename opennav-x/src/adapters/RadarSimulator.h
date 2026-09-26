#pragma once
#include "adapters/Radar.h"

namespace opennav::adapters {
// Tests capability/status transitions only; never fabricates radar echoes.
class RadarStatusSimulator final : public IRadar {
public:
  void Observe(bool available, vessel::Time at) {
    state_.available = available;
    state_.observed_at = at;
    state_.capabilities = {true, true, true, false, true};
    state_.source = "DEMO radar status simulator";
    state_.status =
        available ? "DEMO status only; no radar image" : "DEMO disconnected";
    if (!available)
      state_.presentation = RadarPresentation::Off;
  }
  RadarState GetState() const override { return state_; }
  bool SetPresentation(RadarPresentation p) override {
    if (p != RadarPresentation::Off &&
        (!state_.available ||
         (p != RadarPresentation::Overlay && p != RadarPresentation::Focus)))
      return false;
    state_.presentation = p;
    return true;
  }

private:
  RadarState state_;
};
} // namespace opennav::adapters
