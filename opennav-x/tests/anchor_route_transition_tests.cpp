#include "application/AnchorRouteTransition.h"
#include <iostream>
#include <stdexcept>

using namespace opennav::application;
namespace {
int checks = 0;
void Check(bool ok, const char *reason) {
  ++checks;
  if (!ok) throw std::runtime_error(reason);
}
Waypoint Mark(const std::string &id) {
  Waypoint mark;
  mark.id = id;
  mark.revision = "unchanged " + id;
  return mark;
}
struct Fixture {
  Route route;
  AnchorWatchSelection current{true, {Mark("anchor-1"), Mark("anchor-2")}, {}};
  NavigationActions actions;
  bool saved = true, active = false;
  int persists = 0, activations = 0;
  Fixture() {
    route.id = "retained-route";
    route.revision = "retained-route-revision";
    route.points = {Mark("route-a"), Mark("route-b")};
    actions.anchor_watches = [this] { return current; };
    const auto run = [this](const Route &selected,
                            const AnchorWatchSelection *confirmed) {
      // Native Resolve applies this same identity/revision precondition.
      if (selected.id != route.id || selected.revision != route.revision)
        return CommandResult{false, "Route changed"};
      return CommitRouteActivation(current, confirmed, [this] {
        ++persists;
        return saved;
      }, [this] {
        ++activations;
        active = true;
        current.watches.clear();
        return CommandResult{true, "Activated", route.id};
      });
    };
    actions.activate = [run](const Route &selected) { return run(selected, nullptr); };
    actions.activate_after_anchor = [run](const Route &selected,
                                          const AnchorWatchSelection &confirmed) {
      return run(selected, &confirmed);
    };
  }
};
void CancelAndAccept() {
  for (int count = 0; count <= 2; ++count) {
    Fixture f;
    f.current.watches.resize(count);
    const auto before = f.current;
    const auto canceled = ConfirmRouteActivation(f.route, f.actions, [&](bool stop) {
      Check(stop == (count != 0), "confirmation describes actual watch transition");
      return false;
    });
    Check(!canceled && f.persists == 0 && f.activations == 0,
          "cancel cannot persist, stop a watch or activate navigation");
    Check(SameAnchorWatchSelection(before, f.current), "cancel preserves exact watch set");
    const auto accepted = ConfirmRouteActivation(f.route, f.actions, [](bool) { return true; });
    Check(accepted && accepted->ok && f.active && f.persists == 1 && f.activations == 1,
          "accept persists before a single transition");
    Check(f.current.watches.empty(), "both watch slots stop on accepted transition");
    Check(f.route.id == "retained-route" && f.route.points.size() == 2,
          "transition preserves selected route data");
  }
}
void ChangedDuringConfirmation() {
  for (int change = 0; change < 6; ++change) {
    Fixture f;
    const auto selected_route = f.route;
    const auto result = ConfirmRouteActivation(selected_route, f.actions, [&](bool) {
      switch (change) {
      case 0: f.current.watches[0].revision += " moved"; break;
      case 1: f.current.watches[1].id = "replacement"; break;
      case 2: f.current.watches.pop_back(); break;
      case 3: f.current.watches.clear(); break;
      case 4: f.current.available = false; break;
      case 5: f.route.revision += " edited"; break;
      }
      return true;
    });
    Check(result && !result->ok && f.persists == 0 && f.activations == 0,
          "stale selection fails before persistence or watch mutation");
  }
  Fixture f;
  f.current.watches.clear();
  const auto result = ConfirmRouteActivation(f.route, f.actions, [&](bool stopping) {
    Check(!stopping, "ordinary route prompt has no stop-watch consent");
    f.current.watches.push_back(Mark("new-watch"));
    return true;
  });
  Check(result && !result->ok && f.persists == 0 && f.activations == 0,
        "new watch during ordinary route confirmation cannot be silently stopped");
}
void PersistenceAndReplay() {
  Fixture f;
  f.saved = false;
  const auto before = f.current;
  const auto result = ConfirmRouteActivation(f.route, f.actions, [](bool) { return true; });
  Check(result && !result->ok && f.persists == 1 && f.activations == 0,
        "route persistence failure never dispatches watch stop or activation");
  Check(SameAnchorWatchSelection(before, f.current), "failed save preserves watches");
  bool live = true;
  f.actions = GuardNavigationChanges(std::move(f.actions), [&] { return live; });
  const auto replay = ConfirmRouteActivation(f.route, f.actions, [&](bool) {
    live = false;
    return true;
  });
  Check(replay && !replay->ok && f.persists == 1 && f.activations == 0,
        "replay beginning during modal blocks the consent callback");
  f.current.available = false;
  bool prompted = false;
  const auto unavailable = ConfirmRouteActivation(f.route, f.actions, [&](bool) {
    prompted = true;
    return true;
  });
  Check(unavailable && !unavailable->ok && !prompted,
        "unavailable watch set cannot offer an actionable confirmation");
}
void AnchorPersistenceOrder() {
  for (int fail = 0; fail < 4; ++fail) {
    std::string order;
    bool mark_saved = false, route_active = true, armed = false;
    const auto result = CommitAnchorWatch([&] {
      order += 'S';
      return mark_saved = fail != 1;
    }, [&] {
      order += 'D';
      if (fail >= 2) return false;
      route_active = false;
      return true;
    }, [&] {
      order += 'R';
      if (fail == 3) return false;
      mark_saved = false;
      return true;
    }, [&] {
      order += 'A';
      Check(!route_active && mark_saved, "arm requires persisted mark and stopped route");
      armed = true;
    });
    Check(order == (fail == 0 ? "SDA" : fail == 1 ? "S" : "SDR"),
          "anchor transition persists before deactivation and rolls back failed stop");
    Check(armed == (fail == 0), "failed transition never arms competing watch");
    Check(result == (fail == 0 ? AnchorStartResult::Started
                     : fail == 1 ? AnchorStartResult::SaveFailed
                     : fail == 2 ? AnchorStartResult::StopFailed
                                 : AnchorStartResult::RollbackFailed),
          "anchor save, stop and rollback failures are distinguished");
    Check(fail >= 2 || !mark_saved || armed, "saved mark belongs to accepted watch");
    Check(fail != 2 || !mark_saved, "successful rollback leaves no orphan mark");
  }
}
} // namespace
int main() {
  try {
    CancelAndAccept();
    ChangedDuringConfirmation();
    PersistenceAndReplay();
    AnchorPersistenceOrder();
    std::cout << "PASS " << checks << " anchor/route transition checks\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
