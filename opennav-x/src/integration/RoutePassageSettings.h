#pragma once
#include "application/RoutePassage.h"
#include <string>

class wxFileConfig;
class Route;
class RoutePoint;
namespace opennav::application {
struct Route;
struct CommandResult;
}
namespace opennav::integration {
// Initialize before navigation in XNav, Legacy and Safe; clear before config
// teardown. All calls require the application thread. No upstream pointers are
// retained. This profile metadata does not travel with GPX exports.
void ConfigureRoutePassage(wxFileConfig* config);
application::RoutePassageSettings ReadRoutePassage(const std::string& route_id);
application::CommandResult EditRoutePassage(
    const application::Route& selected, const application::RoutePassageSettings& value);
bool PassageVisible();
application::CommandResult SetPassageVisible(bool visible);
// nullopt: exact stock behavior. A negative result withholds arrival. Positive
// results are opt-in radial distances using normal upstream range, never XTE.
std::optional<double> ResolveRouteArrivalRadiusNm(
    const ::Route& route, const ::RoutePoint& point, double upstream_radius);
namespace detail {
// Storage boundary; EditRoutePassage resolves identity/editability first.
application::CommandResult StoreRoutePassage(
    const std::string& route_id, const application::RoutePassageSettings& value);
}
} // namespace opennav::integration
