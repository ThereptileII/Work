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

#ifndef XNAV_ENABLE_PILOT_LOOPBACK_TESTS
#define XNAV_ENABLE_PILOT_LOOPBACK_TESTS 0
#endif
#if XNAV_ENABLE_PILOT_LOOPBACK_TESTS != 0 && XNAV_ENABLE_PILOT_LOOPBACK_TESTS != 1
#error XNAV_ENABLE_PILOT_LOOPBACK_TESTS must be 0 or 1
#endif
#if XNAV_ENABLE_PILOT_LOOPBACK_TESTS && !XNAV_ENABLE_TEST_FIXTURES
#error Pilot loopback output requires a non-installable test-fixture build
#endif

namespace opennav::integration {
constexpr bool TestFixturesEnabled() {
  return XNAV_ENABLE_TEST_FIXTURES == 1;
}
constexpr std::string_view BuildPurpose() {
  return TestFixturesEnabled() ? "DEVELOPER TEST BUILD" : "INSTALLED PRODUCT";
}
constexpr bool PilotLoopbackTestsEnabled() {
  return XNAV_ENABLE_PILOT_LOOPBACK_TESTS == 1;
}
// No public product capability permits XNav-owned physical equipment output.
// Stock OpenCPN connections/plugins are separate and preserve their semantics.
constexpr std::string_view HardwareOutputPolicy() {
  return PilotLoopbackTestsEnabled() ? "test-loopback-only" : "status-only";
}
} // namespace opennav::integration
