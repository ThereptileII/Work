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
} // namespace opennav::adapters
