#include "integration/StartupMode.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace opennav::integration;

namespace {
void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void SetEnvironment(const char* name, const char* value) {
#ifdef _WIN32
  Check(_putenv_s(name, value) == 0, "Set isolated test environment");
#else
  Check(setenv(name, value, 1) == 0, "Set isolated test environment");
#endif
}
bool Permitted(TestStartupFlags request, StartupFlags mode) {
  try {
    ValidateTestStartup(request, mode);
    return true;
  } catch (const std::invalid_argument&) {
    return false;
  }
}
} // namespace

int main() {
  try {
    const bool fixtures = TestFixturesEnabled();
    Check(BuildPurpose() == (fixtures ? "DEVELOPER TEST BUILD" : "INSTALLED PRODUCT"),
          "Build purpose states capability truthfully");
    for (const auto mode : {StartupFlags{}, StartupFlags{true, false, false},
                            StartupFlags{false, true, false}, StartupFlags{false, false, true}})
      Check(Permitted({}, mode), "Normal mode policy is unchanged");
    for (unsigned request = 1; request < 8; ++request) {
      const TestStartupFlags flags{bool(request & 1), bool(request & 2), bool(request & 4)};
      const bool exactly_one = request == 1 || request == 2 || request == 4;
      for (unsigned mode = 0; mode < 8; ++mode) {
        const StartupFlags startup{bool(mode & 1), bool(mode & 2), bool(mode & 4)};
        Check(Permitted(flags, startup) == (fixtures && exactly_one && mode == 1),
              "Only one test source in explicit developer XNav is allowed");
      }
    }
    for (const auto* name : {"XNAV_ENABLE_TEST_FIXTURES", "XNAV_ENABLE_DEMO",
                             "OPENNAV_ROUTE_TESTS", "OPENNAV_ENABLE_ROUTE_SCENARIO"}) {
      SetEnvironment(name, "1");
      Check(TestFixturesEnabled() == fixtures, "Environment cannot upgrade build capability");
      Check(Permitted({true, false, false}, {true, false, false}) == fixtures,
            "Environment cannot enable synthetic input in installed product");
    }
    Check(!ParseInterfaceMode("demo") && !ParseInterfaceMode("xnav-demo"),
          "Saved configuration cannot enable synthetic input");
    std::cout << BuildPurpose() << ": all startup combinations and environment bypass attempts passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
