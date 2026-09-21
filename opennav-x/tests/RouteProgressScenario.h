#pragma once
#include "vessel/RouteProgress.h"
#include <string>
namespace opennav::test {
// Compiled only into explicit integration-test builds, not release builds.
void EnableRouteScenario(const std::string& isolated_profile);
void RouteScenarioStep(const vessel::RouteProgress& snapshot);
}
