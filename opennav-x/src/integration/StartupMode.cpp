#include "integration/StartupMode.h"

#include <stdexcept>

namespace opennav::integration {

std::optional<InterfaceMode> ParseInterfaceMode(std::string_view value) {
  if (value == "xnav") return InterfaceMode::XNav;
  if (value == "legacy") return InterfaceMode::Legacy;
  return std::nullopt;
}

std::string_view ToConfigValue(InterfaceMode mode) {
  switch (mode) {
    case InterfaceMode::XNav: return "xnav";
    case InterfaceMode::Legacy: return "legacy";
  }
  throw std::invalid_argument("Invalid interface mode");
}

StartupSelection ResolveStartup(StartupFlags flags,
                                std::optional<InterfaceMode> persisted) {
  if (flags.safe) return {StartupMode::Safe, SelectionSource::CommandLine};
  if (flags.xnav && flags.legacy) {
    throw std::invalid_argument("Choose either --xnav or --legacy");
  }
  if (flags.xnav) return {StartupMode::XNav, SelectionSource::CommandLine};
  if (flags.legacy) return {StartupMode::Legacy, SelectionSource::CommandLine};
  if (persisted) {
    switch (*persisted) {
      case InterfaceMode::XNav:
        return {StartupMode::XNav, SelectionSource::Persisted};
      case InterfaceMode::Legacy:
        return {StartupMode::Legacy, SelectionSource::Persisted};
    }
    throw std::invalid_argument("Invalid persisted interface mode");
  }
  return {StartupMode::XNav, SelectionSource::Default};
}

bool OpenNavModulesAllowed(StartupMode mode) {
  switch (mode) {
    case StartupMode::XNav: return true;
    case StartupMode::Legacy:
    case StartupMode::Safe: return false;
  }
  return false;
}

void ValidateTestStartup(TestStartupFlags requested, StartupFlags mode) {
  const unsigned count = static_cast<unsigned>(requested.demo) +
                         static_cast<unsigned>(requested.route) +
                         static_cast<unsigned>(requested.objects);
  if (!count) return;
  if (!TestFixturesEnabled())
    throw std::invalid_argument("Synthetic test input is not available in this product build");
  if (!mode.xnav || mode.legacy || mode.safe)
    throw std::invalid_argument("Test input requires explicit XNav in a developer build");
  if (count != 1)
    throw std::invalid_argument("Choose only one isolated test input");
}

}  // namespace opennav::integration
