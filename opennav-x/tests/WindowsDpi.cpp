// Disposable Windows CI desktop helper only; never linked into OpenCPN/shipped.
// QueryDisplayConfig/DisplayConfig{Get,Set}DeviceInfo use the SDK structures.
// The -3/-4 source DPI packets are undocumented and isolated to this test tool.
// Their observed layout is described by the original investigation:
// https://github.com/lihas/windows-DPI-scaling-sample/blob/master/DPIHelper/DpiHelper.h
// A requested setting is not evidence: the harness must check GetDpiForWindow.
#define WIN32_LEAN_AND_MEAN
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <windows.h>
struct DpiGet {
  DISPLAYCONFIG_DEVICE_INFO_HEADER header;
  INT32 minimum, current, maximum;
};
struct DpiSet {
  DISPLAYCONFIG_DEVICE_INFO_HEADER header;
  INT32 relative;
};
static_assert(sizeof(DpiGet) == 32 && sizeof(DpiSet) == 24,
              "Unexpected source DPI packet layout");
DpiGet Get(const DISPLAYCONFIG_PATH_SOURCE_INFO &source) {
  DpiGet p{};
  p.header.type = static_cast<DISPLAYCONFIG_DEVICE_INFO_TYPE>(-3);
  p.header.size = sizeof(p);
  p.header.adapterId = source.adapterId;
  p.header.id = source.id;
  auto result = DisplayConfigGetDeviceInfo(&p.header);
  if (result != ERROR_SUCCESS)
    throw std::runtime_error("DPI query unavailable, Windows error " +
                             std::to_string(result));
  if (p.minimum > 0 || p.current < p.minimum || p.current > p.maximum ||
      p.minimum < -11 || p.maximum - p.minimum > 11)
    throw std::runtime_error("Unrecognized DPI scale range");
  return p;
}
int main(int argc, char **argv) {
  try {
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    if (argc == 4 && std::string(argv[1]) == "--tap") {
      const auto permit = std::getenv("OPENNAV_DISPOSABLE_DESKTOP");
      if (!permit || std::string(permit) != "1")
        throw std::runtime_error(
            "Touch injection requires a disposable desktop");
      const int x = std::stoi(argv[2]), y = std::stoi(argv[3]);
      if (x < 0 || y < 0 || x >= GetSystemMetrics(SM_CXSCREEN) ||
          y >= GetSystemMetrics(SM_CYSCREEN))
        throw std::runtime_error("Touch outside primary test desktop");
      if (!InitializeTouchInjection(1, TOUCH_FEEDBACK_NONE))
        throw std::runtime_error("Touch injection unavailable, error " +
                                 std::to_string(GetLastError()));
      POINTER_TOUCH_INFO contact{};
      contact.pointerInfo.pointerType = PT_TOUCH;
      contact.pointerInfo.pointerId = 1;
      contact.pointerInfo.ptPixelLocation = {x, y};
      contact.pointerInfo.pointerFlags =
          POINTER_FLAG_DOWN | POINTER_FLAG_INRANGE | POINTER_FLAG_INCONTACT;
      contact.touchMask =
          TOUCH_MASK_CONTACTAREA | TOUCH_MASK_ORIENTATION | TOUCH_MASK_PRESSURE;
      contact.rcContact = {x - 2, y - 2, x + 2, y + 2};
      contact.orientation = 90;
      contact.pressure = 512;
      if (!InjectTouchInput(1, &contact))
        throw std::runtime_error("Touch down failed, error " +
                                 std::to_string(GetLastError()));
      Sleep(100);
      contact.pointerInfo.pointerFlags = POINTER_FLAG_UP;
      if (!InjectTouchInput(1, &contact))
        throw std::runtime_error("Touch up failed, error " +
                                 std::to_string(GetLastError()));
      std::cout << "{\"touch_injected\":true}\n";
      return 0;
    }
    UINT32 paths_count = 0, modes_count = 0;
    auto result = GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS,
                                              &paths_count, &modes_count);
    if (result != ERROR_SUCCESS || paths_count == 0 || paths_count > 64 ||
        modes_count > 256)
      throw std::runtime_error(
          "No supported active WDDM display configuration, error " +
          std::to_string(result));
    std::vector<DISPLAYCONFIG_PATH_INFO> paths(paths_count);
    std::vector<DISPLAYCONFIG_MODE_INFO> modes(modes_count);
    result =
        QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &paths_count, paths.data(),
                           &modes_count, modes.data(), nullptr);
    if (result != ERROR_SUCCESS || paths_count == 0)
      throw std::runtime_error("Display topology query failed");
    const auto source = paths.front().sourceInfo;
    auto packet = Get(source);
    const std::vector<int> scales{100, 125, 150, 175, 200, 225,
                                  250, 300, 350, 400, 450, 500};
    const int before = scales.at(packet.current - packet.minimum);
    if (argc == 2) {
      const auto permit = std::getenv("OPENNAV_DISPOSABLE_DESKTOP");
      if (!permit || std::string(permit) != "1")
        throw std::runtime_error(
            "DPI changes require a disposable test desktop");
      const std::string text = argv[1];
      if (text != "100" && text != "125" && text != "150")
        throw std::runtime_error(
            "Only 100/125/150 percent test scales permitted");
      const int percent = std::stoi(text);
      const auto i = std::find(scales.begin(), scales.end(), percent);
      const auto relative =
          static_cast<INT32>(i - scales.begin()) + packet.minimum;
      if (relative > packet.maximum)
        throw std::runtime_error(
            "Requested DPI unsupported by this display mode");
      DpiSet p{};
      p.header.type = static_cast<DISPLAYCONFIG_DEVICE_INFO_TYPE>(-4);
      p.header.size = sizeof(p);
      p.header.adapterId = source.adapterId;
      p.header.id = source.id;
      p.relative = relative;
      result = DisplayConfigSetDeviceInfo(&p.header);
      if (result != ERROR_SUCCESS)
        throw std::runtime_error("DPI change failed, Windows error " +
                                 std::to_string(result));
      for (int attempt = 0; attempt < 12; ++attempt) {
        packet = Get(source);
        if (packet.current == relative)
          break;
        Sleep(250);
      }
      if (packet.current != relative)
        throw std::runtime_error("DPI change was not observed");
    } else if (argc != 1)
      throw std::runtime_error(
          "Use zero arguments to query or a permitted percent to set");
    std::cout << "{\"before_percent\":" << before
              << ",\"percent\":" << scales.at(packet.current - packet.minimum)
              << ",\"source_id\":" << source.id
              << ",\"minimum_relative\":" << packet.minimum
              << ",\"maximum_relative\":" << packet.maximum << "}\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
