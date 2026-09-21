#include "integration/StartupMode.h"

#include <iostream>
#include <stdexcept>
#include <string>

using namespace opennav::integration;

void Require(bool condition, const char* message) {
  // Unlike assert(), this remains active in Release/MSVC builds.
  if (!condition) throw std::runtime_error(message);
}

int main() {
  try {
    const auto initial = ResolveStartup({});
    Require(initial.mode == StartupMode::XNav, "First-run default must be XNav");
    Require(initial.source == SelectionSource::Default, "Default provenance");
    Require(ResolveStartup({}, InterfaceMode::Legacy).mode == StartupMode::Legacy,
            "Persisted Legacy must survive restart");
    Require(ResolveStartup({}, InterfaceMode::XNav).source == SelectionSource::Persisted,
            "Persisted provenance");
    Require(ResolveStartup({true, false, false}, InterfaceMode::Legacy).mode == StartupMode::XNav,
            "Explicit XNav overrides stored Legacy");
    Require(ResolveStartup({false, true, false}, InterfaceMode::XNav).mode == StartupMode::Legacy,
            "Explicit Legacy overrides stored XNav");
    for (const auto persisted : {InterfaceMode::XNav, InterfaceMode::Legacy}) {
      for (const bool xnav : {false, true}) {
        for (const bool legacy : {false, true}) {
          const auto safe = ResolveStartup({xnav, legacy, true}, persisted);
          Require(safe.mode == StartupMode::Safe, "Safe always takes precedence");
          Require(!OpenNavModulesAllowed(safe.mode), "Safe must disable OpenNav modules");
          Require(ResolveStartup({}, persisted).mode != StartupMode::Safe,
                  "Safe must not become the persisted startup choice");
        }
      }
    }
    bool conflict_rejected = false;
    try { (void)ResolveStartup({true, true, false}); }
    catch (const std::invalid_argument&) { conflict_rejected = true; }
    Require(conflict_rejected, "Ambiguous command line must be rejected");
    for (const auto bad : {"", "safe", "safe-mode", "XNav", "legacy ", "unknown"}) {
      Require(!ParseInterfaceMode(bad), "Do not accept invalid persisted values");
    }
    for (const auto mode : {InterfaceMode::XNav, InterfaceMode::Legacy}) {
      Require(ParseInterfaceMode(ToConfigValue(mode)) == mode, "Config round trip");
    }
    Require(!OpenNavModulesAllowed(StartupMode::Legacy), "Legacy isolates new modules");
    Require(!OpenNavModulesAllowed(static_cast<StartupMode>(99)), "Unknown mode fails closed");
    std::cout << "Startup policy contract passed; runtime acceptance is a separate integration gate.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
