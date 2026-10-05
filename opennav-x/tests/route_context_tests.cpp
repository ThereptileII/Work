#include "application/RouteContext.h"
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace opennav::application;
namespace { void Check(bool value, const char *why) { if (!value) throw std::runtime_error(why); } }
int main() {
  try {
    RouteHoverGate hover;
    Check(hover.Accept("route-a", 10, 20), "First verified hover is accepted");
    Check(!hover.Accept("route-a", 10, 20),
          "Unchanged native timer cannot reopen dismissed route context");
    Check(hover.Accept("route-a", 11, 20) && hover.Accept("route-b", 11, 20),
          "Pointer movement or different true route identity allows a new hover");
    Check(!hover.Accept("", 12, 20), "Empty hover identity cannot clear dismissal guard");
    Route route;
    route.id = "verified-route"; route.name = "Observed name";
    route.visible = route.editable = true;
    Waypoint first, last;
    first.name = "Observed departure"; first.latitude_deg = 57; first.longitude_deg = 16;
    last.name = "Observed destination"; route.points = {first, last};
    auto view = PresentRouteContext(route.id, route);
    Check(view.available && view.can_view && !view.active && view.points == 2,
          "Current route snapshot provides read-only action availability");
    Check(view.name == route.name && view.departure == first.name && view.destination == last.name,
          "Route identity and endpoint names come from actual supplied route");
    route.active = true;
    Check(PresentRouteContext(route.id, route).status == "ACTIVE ROUTE", "Active state is observed");
    Check(!PresentRouteContext("wrong-id", route).available &&
          !PresentRouteContext(route.id, {}).can_view,
          "Removed or mismatched route cannot retain available actions");
    route.visible = route.editable = false;
    Check(PresentRouteContext(route.id, route).status.find("HIDDEN / READ ONLY") != std::string::npos,
          "Hidden/protected route state is explicit");
    route.points.front().latitude_deg = std::numeric_limits<double>::quiet_NaN();
    Check(!PresentRouteContext(route.id, route).can_view,
          "Invalid coordinates cannot enable chart centering");
    route.points.clear(); route.name.clear();
    view = PresentRouteContext(route.id, route);
    Check(view.name == "Unnamed route" && view.points == 0 && !view.can_view && view.available,
          "Empty route remains inspectable without inventing endpoints");
    std::cout << "Route context presentation tests passed\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
