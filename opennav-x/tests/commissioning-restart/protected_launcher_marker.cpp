// Inert stand-in for the protected launcher. It records the command delivered
// by the real restart helper; it contains no broker, profile or marine code.
#include "AtomicMarker.h"

#include <windows.h>

#include <cwchar>
#include <cstdint>
#include <iostream>
#include <string>

namespace {
std::string Utf8(const std::wstring &value) {
  if (value.empty()) return {};
  const int size = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS,
      value.data(), static_cast<int>(value.size()), nullptr, 0, nullptr, nullptr);
  if (size <= 0) return {};
  std::string result(size, '\0');
  if (!WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, value.data(),
      static_cast<int>(value.size()), result.data(), size, nullptr, nullptr))
    return {};
  return result;
}
} // namespace

int wmain(int argc, wchar_t **argv) {
  if (argc == 2 && std::wcscmp(argv[1], L"--marker-self-test") == 0) {
    std::cout << "{\"contract\":\"OpenNavX.ProtectedRestart.LauncherMarker.1\","
                 "\"broker\":false,\"profile_accessed\":false,"
                 "\"marine_code\":false}\n";
    return std::cout ? 0 : 70;
  }
  if (argc < 1 || argc > 8) return 71;
  std::string marker = std::to_string(argc) + '\n';
  for (int index = 1; index < argc; ++index) {
    const auto argument = Utf8(argv[index]);
    if (argument.empty() || argument.find('\n') != std::string::npos ||
        argument.find('\r') != std::string::npos)
      return 72;
    marker += argument + '\n';
  }
  FILETIME created{}, exited{}, kernel{}, user{};
  if (!GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user))
    return 74;
  const auto created_ticks = (std::uint64_t(created.dwHighDateTime) << 32) |
                             created.dwLowDateTime;
  marker += std::to_string(created_ticks) + '\n';
  std::string error;
  if (!opennav::tests::PublishMarker("protected-launcher-args.txt", marker, error)) {
    std::cerr << "Protected launcher marker failed: " << error << '\n';
    return 73;
  }
  return 0;
}
