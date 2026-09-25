// Link wrappers observe ownership in the actual pinned discovery implementation.
// They do not replace enumeration data or open an independent device stack.
#include <gtest/gtest.h>
#include <libudev.h>
#include <wx/arrstr.h>
#include "model/ser_ports.h"
#include <memory>

namespace {
struct Counts {
  int contexts = 0, contexts_freed = 0;
  int scans = 0, scans_freed = 0;
  int devices = 0, devices_freed = 0;
  bool active = false, fail_context = false, fail_scan = false;
} counts;
struct Observe {
  Observe() { counts = {}; counts.active = true; }
  ~Observe() { counts.active = false; }
};
}
extern "C" {
udev* __real_udev_new();
udev* __real_udev_unref(udev*);
udev_enumerate* __real_udev_enumerate_new(udev*);
udev_enumerate* __real_udev_enumerate_unref(udev_enumerate*);
udev_device* __real_udev_device_new_from_syspath(udev*, const char*);
udev_device* __real_udev_device_unref(udev_device*);
udev* __wrap_udev_new() {
  if (counts.active && counts.fail_context) return nullptr;
  auto* result = __real_udev_new();
  if (counts.active && result) ++counts.contexts;
  return result;
}
udev* __wrap_udev_unref(udev* value) {
  if (counts.active && value) ++counts.contexts_freed;
  return __real_udev_unref(value);
}
udev_enumerate* __wrap_udev_enumerate_new(udev* context) {
  if (counts.active && counts.fail_scan) return nullptr;
  auto* result = __real_udev_enumerate_new(context);
  if (counts.active && result) ++counts.scans;
  return result;
}
udev_enumerate* __wrap_udev_enumerate_unref(udev_enumerate* value) {
  if (counts.active && value) ++counts.scans_freed;
  return __real_udev_enumerate_unref(value);
}
udev_device* __wrap_udev_device_new_from_syspath(udev* context, const char* path) {
  auto* result = __real_udev_device_new_from_syspath(context, path);
  if (counts.active && result) ++counts.devices;
  return result;
}
udev_device* __wrap_udev_device_unref(udev_device* value) {
  if (counts.active && value) ++counts.devices_freed;
  return __real_udev_device_unref(value);
}
}
TEST(OpenNavDiscovery, RepeatedScansReleaseAllOwnedReferences) {
  Observe observing;
  for (int i = 0; i < 8; ++i) {
    std::unique_ptr<wxArrayString> ports(EnumerateSerialPorts());
    ASSERT_NE(ports, nullptr);
    EXPECT_EQ(counts.contexts, counts.contexts_freed);
    EXPECT_EQ(counts.scans, counts.scans_freed);
    EXPECT_EQ(counts.devices, counts.devices_freed);
  }
  EXPECT_EQ(counts.contexts, 8);
  EXPECT_EQ(counts.scans, 8);
}
TEST(OpenNavDiscovery, ContextFailureReturnsUnavailableCatalog) {
  Observe observing;
  counts.fail_context = true;
  std::unique_ptr<wxArrayString> ports(EnumerateSerialPorts());
  ASSERT_NE(ports, nullptr);
  EXPECT_TRUE(ports->empty());
  EXPECT_EQ(counts.scans, 0);
}
TEST(OpenNavDiscovery, ScanFailureReleasesContext) {
  Observe observing;
  counts.fail_scan = true;
  std::unique_ptr<wxArrayString> ports(EnumerateSerialPorts());
  ASSERT_NE(ports, nullptr);
  EXPECT_TRUE(ports->empty());
  EXPECT_EQ(counts.contexts, 1);
  EXPECT_EQ(counts.contexts_freed, 1);
}
