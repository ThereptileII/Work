#pragma once
#include "integration/RouteProgressInput.h"
class Routeman;

namespace opennav::integration {
// Call only on the application thread. No pointer survives this function.
RouteCopy CopyActiveRoute(Routeman* manager);
RouteRead ReadRouteProgress(const vessel::Navigation& position);
}  // namespace opennav::integration
