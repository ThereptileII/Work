#include "platform/PlatformIntegration.h"
#include "platform/windows/WindowsArguments.h"

#include <windows.h>
#include <filesystem>

namespace opennav::platform {
namespace {
std::wstring Wide(const std::string& text) {
  const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
                                       static_cast<int>(text.size()), nullptr, 0);
  if (count <= 0) return {};
  std::wstring result(count, L'\0');
  MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
                      static_cast<int>(text.size()), result.data(), count);
  return result;
}
}  // namespace

bool RestartAfterExit(const std::string& executable,
                      const std::vector<std::string>& arguments) {
  const auto exe = Wide(executable);
  const auto helper = (std::filesystem::path(exe).parent_path() / L"opennav-restart.exe").wstring();
  if (!std::filesystem::exists(helper)) return false;
  std::wstring command = QuoteWindowsArgument(helper) + L" " +
      std::to_wstring(GetCurrentProcessId()) + L" " + QuoteWindowsArgument(exe);
  for (const auto& value : arguments) command += L" " + QuoteWindowsArgument(Wide(value));
  STARTUPINFOW startup{};
  startup.cb = sizeof(startup);
  PROCESS_INFORMATION process{};
  const bool ok = CreateProcessW(helper.c_str(), command.data(), nullptr, nullptr,
      FALSE, CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process) != FALSE;
  if (ok) { CloseHandle(process.hThread); CloseHandle(process.hProcess); }
  return ok;
}

}  // namespace opennav::platform
