#pragma once
#include "vessel/VesselState.h"

namespace opennav::adapters {
enum class RadarPresentation { Off, Overlay, Focus };
struct RadarCapabilities {
  bool overlay = false, focus = false, receive = false, control = false,
       simulated = false;
};
struct RadarState {
  RadarCapabilities capabilities;
  RadarPresentation presentation = RadarPresentation::Off;
  std::string source, status;
  vessel::Time observed_at{};
  bool available = false;
};
class IRadar {
public:
  virtual ~IRadar() = default;
  virtual RadarState GetState() const = 0;
  virtual bool SetPresentation(RadarPresentation) = 0;
};
class UnavailableRadar final : public IRadar {
public:
  RadarState GetState() const override {
    RadarState s;
    s.status = "No compatible radar adapter connected";
    return s;
  }
  bool SetPresentation(RadarPresentation p) override {
    return p == RadarPresentation::Off;
  }
};
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
