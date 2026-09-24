#pragma once

#include <optional>
#include <string_view>

namespace opennav::integration {

// Startup policy shared by the integration lifecycle and portable tests.
enum class InterfaceMode { XNav, Legacy };
enum class StartupMode { XNav, Legacy, Safe };
enum class SelectionSource { Default, Persisted, CommandLine };

struct StartupFlags {
  bool xnav = false;
  bool legacy = false;
  bool safe = false;
};

struct StartupSelection {
  StartupMode mode;
  SelectionSource source;
};

// Only normal interface modes are persistable. "safe" is deliberately invalid.
std::optional<InterfaceMode> ParseInterfaceMode(std::string_view value);
std::string_view ToConfigValue(InterfaceMode mode);

// Safe overrides all other flags. Conflicting normal flags are an error.
// Selecting a startup mode never writes configuration.
StartupSelection ResolveStartup(
    StartupFlags flags, std::optional<InterfaceMode> persisted = std::nullopt);

// A hard upper bound for future module creation. This grants no hardware
// command permission: actual adapters will require feedback and interlocks.
bool OpenNavModulesAllowed(StartupMode mode);

}  // namespace opennav::integration
