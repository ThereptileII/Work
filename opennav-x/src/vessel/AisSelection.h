#pragma once
#include "vessel/AisState.h"
namespace opennav::vessel {
// One copied target only; no AIS decoder/object lifetime escapes integration.
class AisSelection {
public:
  bool Select(int mmsi, const AisState &state, Time now);
  void Observe(const AisState &state, Time now);
  void Clear() { target_.reset(); }
  int Selected(Time now) const;
  static bool CurrentPosition(const AisTarget &, Time now);

private:
  std::optional<AisTarget> target_;
};
} // namespace opennav::vessel
