// Compile the reviewed discovery implementation with test-only Win32 API spies.
// The application uses the same source without these aliases. Real SetupAPI
// lists and HKCU query handles are released by the production ownership guards.
#include <winsock2.h>
#include <windows.h>
#include <setupapi.h>
#include <gtest/gtest.h>
#include <wx/arrstr.h>
#include "model/garmin_protocol_mgr.h"
#include <memory>
#include <set>
#include <string>

namespace {
struct DiscoveryObservation {
  int attempts = 0, acquired = 0, released = 0;
  int keys_acquired = 0, keys_released = 0;
  bool fail_lists = false, fixture_port = false;
  std::set<HDEVINFO> lists;
  std::set<HKEY> keys;
  std::wstring key_path;
} observed;

struct Observe {
  Observe() { observed = {}; }
  ~Observe() {
    // A red test must not contaminate another test or leave user registry data.
    for (auto handle : observed.lists) SetupDiDestroyDeviceInfoList(handle);
    for (auto handle : observed.keys) RegCloseKey(handle);
    if (!observed.key_path.empty())
      RegDeleteKeyW(HKEY_CURRENT_USER, observed.key_path.c_str());
  }
  void Port(bool value) {
    observed.fixture_port = true;
    observed.key_path = L"Software\\OpenNavX-DiscoveryFixture-" +
        std::to_wstring(GetCurrentProcessId()) + L"-" +
        std::to_wstring(GetTickCount64());
    HKEY key = nullptr;
    ASSERT_EQ(RegCreateKeyExW(HKEY_CURRENT_USER, observed.key_path.c_str(), 0,
        nullptr, REG_OPTION_VOLATILE, KEY_SET_VALUE, nullptr, &key, nullptr), ERROR_SUCCESS);
    if (value) {
      const wchar_t name[] = L"COM250";
      EXPECT_EQ(RegSetValueExW(key, L"PortName", 0, REG_SZ,
          reinterpret_cast<const BYTE*>(name), sizeof(name)), ERROR_SUCCESS);
    }
    EXPECT_EQ(RegCloseKey(key), ERROR_SUCCESS);
  }
};

HDEVINFO WINAPI ObserveGetClassDevs(const GUID* guid, PCWSTR enumerator,
                                   HWND parent, DWORD flags) {
  ++observed.attempts;
  if (observed.fail_lists) return INVALID_HANDLE_VALUE;
  auto result = observed.fixture_port ? SetupDiCreateDeviceInfoList(nullptr, nullptr)
      : SetupDiGetClassDevsW(guid, enumerator, parent, flags);
  if (result != INVALID_HANDLE_VALUE) {
    ++observed.acquired;
    EXPECT_TRUE(observed.lists.insert(result).second);
  }
  return result;
}
BOOL WINAPI ObserveDestroy(HDEVINFO handle) {
  EXPECT_EQ(observed.lists.erase(handle), 1u);
  ++observed.released;
  return SetupDiDestroyDeviceInfoList(handle);
}
BOOL WINAPI ObserveEnum(HDEVINFO list, PSP_DEVINFO_DATA device, const GUID* guid,
                        DWORD index, PSP_DEVICE_INTERFACE_DATA result) {
  if (!observed.fixture_port)
    return SetupDiEnumDeviceInterfaces(list, device, guid, index, result);
  if (index != 0) { SetLastError(ERROR_NO_MORE_ITEMS); return FALSE; }
  result->cbSize = sizeof(*result);
  return TRUE;
}
BOOL WINAPI ObserveDetail(HDEVINFO list, PSP_DEVICE_INTERFACE_DATA item,
                          PSP_DEVICE_INTERFACE_DETAIL_DATA_W detail, DWORD size,
                          PDWORD required, PSP_DEVINFO_DATA device) {
  if (!observed.fixture_port)
    return SetupDiGetDeviceInterfaceDetailW(list, item, detail, size, required, device);
  if (device) device->cbSize = sizeof(*device);
  SetLastError(ERROR_INSUFFICIENT_BUFFER);
  return FALSE;
}
BOOL WINAPI ObserveProperty(HDEVINFO list, PSP_DEVINFO_DATA device, DWORD property,
                            PDWORD type, PBYTE buffer, DWORD bytes, PDWORD required) {
  if (!observed.fixture_port)
    return SetupDiGetDeviceRegistryPropertyW(list, device, property, type, buffer, bytes, required);
  SetLastError(ERROR_INVALID_DATA);
  return FALSE;
}
HKEY WINAPI ObserveOpenKey(HDEVINFO list, PSP_DEVINFO_DATA device, DWORD scope,
                           DWORD profile, DWORD type, REGSAM access) {
  HKEY key = nullptr;
  if (observed.fixture_port) {
    if (RegOpenKeyExW(HKEY_CURRENT_USER, observed.key_path.c_str(), 0,
                     KEY_QUERY_VALUE, &key) != ERROR_SUCCESS)
      return reinterpret_cast<HKEY>(INVALID_HANDLE_VALUE);
  } else {
    key = SetupDiOpenDevRegKey(list, device, scope, profile, type, access);
  }
  if (key != reinterpret_cast<HKEY>(INVALID_HANDLE_VALUE)) {
    ++observed.keys_acquired;
    EXPECT_TRUE(observed.keys.insert(key).second);
  }
  return key;
}
LSTATUS WINAPI ObserveCloseKey(HKEY key) {
  EXPECT_EQ(observed.keys.erase(key), 1u);
  ++observed.keys_released;
  return RegCloseKey(key);
}
}

#define SetupDiGetClassDevsW ObserveGetClassDevs
#define SetupDiDestroyDeviceInfoList ObserveDestroy
#define SetupDiEnumDeviceInterfaces ObserveEnum
#define SetupDiGetDeviceInterfaceDetailW ObserveDetail
#define SetupDiGetDeviceRegistryPropertyW ObserveProperty
#define SetupDiOpenDevRegKey ObserveOpenKey
#define RegCloseKey ObserveCloseKey
#define EnumerateSerialPorts ObservePinnedSerialPorts
#include "model/src/ser_ports.cpp"
#undef EnumerateSerialPorts
#undef RegCloseKey
#undef SetupDiOpenDevRegKey
#undef SetupDiGetDeviceRegistryPropertyW
#undef SetupDiGetDeviceInterfaceDetailW
#undef SetupDiEnumDeviceInterfaces
#undef SetupDiDestroyDeviceInfoList
#undef SetupDiGetClassDevsW

TEST(OpenNavWindowsDiscovery, RepeatedNativeListsAreReleased) {
  Observe scope;
  for (int i = 0; i < 8; ++i) {
    std::unique_ptr<wxArrayString> ports(ObservePinnedSerialPorts());
    ASSERT_NE(ports, nullptr);
    EXPECT_TRUE(observed.lists.empty());
    EXPECT_TRUE(observed.keys.empty());
    EXPECT_EQ(observed.acquired, observed.released);
    EXPECT_EQ(observed.keys_acquired, observed.keys_released);
  }
  EXPECT_EQ(observed.attempts, 16);
  EXPECT_GT(observed.acquired, 0);
}
TEST(OpenNavWindowsDiscovery, FailedListsAreNotReleasedAsHandles) {
  Observe scope;
  observed.fail_lists = true;
  std::unique_ptr<wxArrayString> ports(ObservePinnedSerialPorts());
  ASSERT_NE(ports, nullptr);
  EXPECT_EQ(observed.attempts, 2);
  EXPECT_EQ(observed.acquired, 0);
  EXPECT_EQ(observed.released, 0);
}
TEST(OpenNavWindowsDiscovery, PortQueryReleasesActualRegistryHandle) {
  Observe scope;
  scope.Port(true);
  std::unique_ptr<wxArrayString> ports(ObservePinnedSerialPorts());
  ASSERT_NE(ports, nullptr);
  bool found = false;
  for (const auto& port : *ports) found |= port.StartsWith("COM250 ");
  EXPECT_TRUE(found);
  EXPECT_EQ(observed.keys_acquired, 1);
  EXPECT_EQ(observed.keys_released, 1);
  EXPECT_TRUE(observed.keys.empty());
  EXPECT_TRUE(observed.lists.empty());
}
TEST(OpenNavWindowsDiscovery, MissingPortValueStillReleasesRegistryHandle) {
  Observe scope;
  scope.Port(false);
  std::unique_ptr<wxArrayString> ports(ObservePinnedSerialPorts());
  ASSERT_NE(ports, nullptr);
  EXPECT_EQ(observed.keys_acquired, 1);
  EXPECT_EQ(observed.keys_released, 1);
  EXPECT_TRUE(observed.keys.empty());
  EXPECT_TRUE(observed.lists.empty());
}
TEST(OpenNavWindowsDiscovery, ActualGarminQueriesDoNotAccumulateHandles) {
  for (int i = 0; i < 4; ++i) GarminProtocolHandler::IsGarminPlugged();
  DWORD before = 0, after = 0;
  ASSERT_TRUE(GetProcessHandleCount(GetCurrentProcess(), &before));
  for (int i = 0; i < 64; ++i) GarminProtocolHandler::IsGarminPlugged();
  ASSERT_TRUE(GetProcessHandleCount(GetCurrentProcess(), &after));
  EXPECT_LE(after, before + 2);  // Allow unrelated lazy runtime bookkeeping.
}
