#pragma once

#include <string_view>

// No environment variable, profile setting or runtime switch can enable this.
// Only an explicitly configured developer/CI build may contain generators.
#ifndef XNAV_ENABLE_TEST_FIXTURES
#define XNAV_ENABLE_TEST_FIXTURES 0
#endif
#if XNAV_ENABLE_TEST_FIXTURES != 0 && XNAV_ENABLE_TEST_FIXTURES != 1
#error XNAV_ENABLE_TEST_FIXTURES must be 0 or 1
#endif

namespace opennav::integration {
constexpr bool TestFixturesEnabled() {
  return XNAV_ENABLE_TEST_FIXTURES == 1;
}
constexpr std::string_view BuildPurpose() {
  return TestFixturesEnabled() ? "DEVELOPER TEST BUILD" : "INSTALLED PRODUCT";
}
} // namespace opennav::integration
