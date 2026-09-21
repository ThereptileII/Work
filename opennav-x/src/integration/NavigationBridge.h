#pragma once

#include "vessel/NavigationInput.h"
#include "observable.h"

#include <functional>

namespace opennav {

// Read-only subscription to OpenCPN's priority-selected decoded navigation bus.
// Own and destroy on the GUI thread before the shell/frame is destroyed.
class NavigationBridge final {
 public:
  explicit NavigationBridge(std::function<void(const vessel::VesselState&)> receive);
  const vessel::Navigation& PositionState() const { return input_.State().navigation; }
 private:
  vessel::NavigationInput input_;
  std::function<void(const vessel::VesselState&)> receive_;
  ObsListener listener_;
};

}  // namespace opennav
