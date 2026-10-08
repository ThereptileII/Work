#pragma once
#include "application/NavigationObjects.h"

namespace opennav::application {
struct RouteContextView {
  std::string id, name, status, departure, destination;
  std::size_t points = 0;
  bool available = false, active = false, can_view = false;
  // Explicit human actions only; the card never infers navigation intent.
  bool can_activate = false, can_stop = false;
};
RouteContextView PresentRouteContext(const std::string &selected_id,
                                    const std::optional<Route> &route);
// Native rollover timers can fire again while the pointer has not moved.
// Keep this guard across dismissal; an explicit selection always bypasses it.
class RouteHoverGate {
public:
  bool Accept(const std::string &id, int x, int y) {
    if (id.empty() || (id == id_ && x == x_ && y == y_)) return false;
    id_ = id; x_ = x; y_ = y;
    return true;
  }
private:
  std::string id_;
  int x_ = 0, y_ = 0;
};
} // namespace opennav::application
