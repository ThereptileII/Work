#include "platform/PlatformIntegration.h"
#include "platform/windows/CommissioningRestartNative.h"
#include "platform/windows/ProtectedRestartPolicy.h"
#include "platform/windows/WindowsArguments.h"

// Keep Windows SDK prerequisites before the KnownFolder GUID declarations.
#include <windows.h>
#include <shlobj.h>
#include <knownfolders.h>

#include <cstring>
#include <cwchar>
#include <filesystem>
#include <memory>
#include <stdexcept>

namespace opennav::platform {
namespace {
std::wstring Wide(const std::string &text) {
  const int count =
      MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
                          static_cast<int>(text.size()), nullptr, 0);
  if (count <= 0)
    return {};
  std::wstring result(count, L'\0');
  MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
                      static_cast<int>(text.size()), result.data(), count);
  return result;
}
// These retained handles corroborate routing against the actual running image.
// They do not grant image/history authority; that remains the broker's job.
struct RestartPin {
  HANDLE handle = INVALID_HANDLE_VALUE;
  BY_HANDLE_FILE_INFORMATION info{};
  FILE_ID_INFO id{};
  bool full_id = false;
  std::wstring native;
  RestartPin(const std::wstring &path, bool directory,
             bool strict_file = true) {
    handle = CreateFileW(
        path.c_str(), directory ? FILE_READ_ATTRIBUTES : GENERIC_READ,
        directory ? FILE_SHARE_READ | FILE_SHARE_WRITE : FILE_SHARE_READ,
        nullptr, OPEN_EXISTING,
        ((directory || strict_file) ? FILE_FLAG_OPEN_REPARSE_POINT : 0) |
            (directory ? FILE_FLAG_BACKUP_SEMANTICS : 0),
        nullptr);
    if (handle == INVALID_HANDLE_VALUE)
      throw std::runtime_error("Restart routing file unavailable");
    try {
      full_id = GetFileInformationByHandleEx(handle, FileIdInfo, &id,
                                             sizeof(id)) != FALSE;
      if (!GetFileInformationByHandle(handle, &info) ||
          ((directory || strict_file) && !full_id) ||
          ((directory || strict_file) &&
           (info.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT)) ||
          bool(info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != directory ||
          (!directory && strict_file && info.nNumberOfLinks != 1))
        throw std::runtime_error("Ambiguous restart routing file");
      native.resize(32768);
      const DWORD used = GetFinalPathNameByHandleW(
          handle, native.data(), static_cast<DWORD>(native.size()),
          FILE_NAME_NORMALIZED | VOLUME_NAME_NT);
      if (!used || used >= native.size())
        throw std::runtime_error("Restart native path unavailable");
      native.resize(used);
    } catch (...) {
      CloseHandle(handle);
      handle = INVALID_HANDLE_VALUE;
      throw;
    }
  }
  ~RestartPin() {
    if (handle != INVALID_HANDLE_VALUE)
      CloseHandle(handle);
  }
  RestartPin(const RestartPin &) = delete;
  RestartPin &operator=(const RestartPin &) = delete;
};
std::wstring RestartImage(DWORD flags) {
  std::wstring path(32768, L'\0');
  DWORD size = static_cast<DWORD>(path.size());
  if (!QueryFullProcessImageNameW(GetCurrentProcess(), flags, path.data(),
                                  &size) ||
      !size)
    throw std::runtime_error("Actual restart image unavailable");
  path.resize(size);
  return path;
}
bool SameRestartPath(const std::wstring &left, const std::wstring &right) {
  return CompareStringOrdinal(left.c_str(), -1, right.c_str(), -1, TRUE) ==
         CSTR_EQUAL;
}
bool SameRestartFile(const RestartPin &left, const RestartPin &right) {
  // Older ordinary portable filesystems may expose only the classic native ID.
  // Protected routing still requires full IDs on all retained original images.
  return left.info.dwVolumeSerialNumber == right.info.dwVolumeSerialNumber &&
         left.info.nFileIndexHigh == right.info.nFileIndexHigh &&
         left.info.nFileIndexLow == right.info.nFileIndexLow &&
         left.info.ftCreationTime.dwHighDateTime ==
             right.info.ftCreationTime.dwHighDateTime &&
         left.info.ftCreationTime.dwLowDateTime ==
             right.info.ftCreationTime.dwLowDateTime &&
         left.full_id == right.full_id &&
         (!left.full_id ||
          (left.id.VolumeSerialNumber == right.id.VolumeSerialNumber &&
           !std::memcmp(&left.id.FileId, &right.id.FileId,
                        sizeof(left.id.FileId)))) &&
         SameRestartPath(left.native, right.native);
}
std::vector<wchar_t> ProtectedRestartEnvironment() {
  LPWCH block = GetEnvironmentStringsW();
  if (!block)
    throw std::runtime_error("Protected restart environment unavailable");
  struct Cleanup {
    LPWCH value;
    ~Cleanup() { FreeEnvironmentStringsW(value); }
  } cleanup{block};
  std::vector<wchar_t> result;
  for (const auto *entry = block; *entry; entry += std::wcslen(entry) + 1) {
    const std::wstring value(entry);
    if (!ProtectedRestartEnvironmentEntryAllowed(value) ||
        result.size() + value.size() + 2 > (1u << 20))
      throw std::runtime_error("Protected restart fenced environment refused");
    result.insert(result.end(), value.begin(), value.end());
    result.push_back(L'\0');
  }
  if (result.empty())
    result.push_back(L'\0');
  result.push_back(L'\0');
  return result;
}
std::wstring RestartProgramFiles() {
  PWSTR value = nullptr;
  const auto status =
      SHGetKnownFolderPath(FOLDERID_ProgramFilesX86, 0, nullptr, &value);
  struct Cleanup {
    PWSTR value;
    ~Cleanup() { CoTaskMemFree(value); }
  } cleanup{value};
  if (FAILED(status) || !value)
    throw std::runtime_error("Native restart ProgramFilesX86 unavailable");
  return std::wstring(value);
}
// Keep every protected descendant and both companions pinned through helper
// creation. The unchanged helper's PID wait is scheduling only; it never proves
// the broker's retained original application owner exited, selects a
// generation, or authorizes a direct application launch in this protected
// route.
void PinProtectedRestart(const std::wstring &image, const std::wstring &root,
                         const std::wstring &native_root,
                         std::vector<std::unique_ptr<RestartPin>> &pins) {
  const auto folded_image = RestartPathFold(image);
  const auto folded_root = RestartPathFold(root);
  if (!RestartPathBelow(folded_image, folded_root))
    throw std::runtime_error("Protected restart DOS routing differs");
  std::wstring dos = root, physical = native_root;
  auto offset = root.size() + 1;
  while (true) {
    const auto end = image.find(L'\\', offset);
    if (end == std::wstring::npos)
      break; // original executable is pinned separately
    const auto part = image.substr(offset, end - offset);
    dos += L"\\" + part;
    physical += L"\\" + part;
    auto pin = std::make_unique<RestartPin>(dos, true);
    if (!SameRestartPath(pin->native, physical))
      throw std::runtime_error("Protected restart ancestry differs");
    pins.push_back(std::move(pin));
    offset = end + 1;
  }
}
} // namespace

bool RestartAfterExit(const std::string &executable,
                      const std::vector<std::string> &arguments) {
  try {
    const auto exe = Wide(executable);
    if (exe.empty() || exe.find(L'\0') != std::wstring::npos)
      return false;
    const auto helper =
        (std::filesystem::path(exe).parent_path() / L"opennav-restart.exe")
            .wstring();
    if (!std::filesystem::exists(helper))
      return false;
    const auto actual_path = RestartImage(0);
    RestartPin actual(actual_path, false, false), requested(exe, false, false);
    const auto native_image = RestartImage(PROCESS_NAME_NATIVE);
    if (native_image.compare(0, 8, L"\\Device\\") != 0)
      return false;
    // Process creation can retain an 8.3 spelling. Reopen only to corroborate
    // against our retained image: unresolved reported spelling is not identity.
    RestartPin reported(L"\\\\?\\GLOBALROOT" + native_image, false, false);
    if (!SameRestartFile(actual, requested) ||
        !SameRestartFile(actual, reported))
      return false;
    const auto program_files = RestartProgramFiles();
    RestartPin root(program_files, true);
    const auto route =
        ProtectedRestartPolicy(actual.native, root.native, arguments);
    if (route.kind == ProtectedRestartKind::Refuse)
      return false;
    std::wstring target = exe;
    std::wstring launch_helper = helper;
    std::vector<std::wstring> child_arguments;
    std::vector<std::unique_ptr<RestartPin>> pins;
    std::vector<wchar_t> environment;
    if (route.kind == ProtectedRestartKind::BrokerCurrent) {
      // Qualification fencing cannot turn a protected restart into a direct
      // application launch. Never clear/strip an existing qualification
      // binding.
      if (commissioning::StartupBinding().state !=
              commissioning::GuardState::Unarmed ||
          !actual.full_id || !requested.full_id || !reported.full_id ||
          actual.info.nNumberOfLinks != 1)
        return false;
      // Use canonical DOS spelling resolved from the SAME actual native image,
      // never a caller path or environment-derived installation root.
      std::wstring canonical(32768, L'\0');
      const DWORD used = GetFinalPathNameByHandleW(
          actual.handle, canonical.data(), static_cast<DWORD>(canonical.size()),
          FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
      if (!used || used >= canonical.size())
        return false;
      canonical.resize(used);
      if (canonical.compare(0, 4, L"\\\\?\\") != 0)
        return false;
      canonical.erase(0, 4);
      // Independently normalize the KnownFolder spelling (8.3 components are
      // permitted only when these retained native identities corroborate them).
      std::wstring canonical_root(32768, L'\0');
      const DWORD root_used =
          GetFinalPathNameByHandleW(root.handle, canonical_root.data(),
                                    static_cast<DWORD>(canonical_root.size()),
                                    FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
      if (!root_used || root_used >= canonical_root.size())
        return false;
      canonical_root.resize(root_used);
      if (canonical_root.compare(0, 4, L"\\\\?\\") != 0)
        return false;
      canonical_root.erase(0, 4);
      if (ProtectedRestartPolicy(canonical, canonical_root, arguments).kind !=
          ProtectedRestartKind::BrokerCurrent)
        return false;
      PinProtectedRestart(canonical, canonical_root, root.native, pins);
      const auto app = std::filesystem::path(canonical).parent_path();
      target = (app / L"skager-start.exe").wstring();
      launch_helper = (app / L"opennav-restart.exe").wstring();
      for (const auto *name : {L"opennav-restart.exe", L"skager-start.exe"}) {
        auto pin = std::make_unique<RestartPin>((app / name).wstring(), false);
        if (!SameRestartPath(
                pin->native,
                std::filesystem::path(actual.native).parent_path().wstring() +
                    L"\\" + name))
          return false;
        pins.push_back(std::move(pin));
      }
      // Dispatch through the corroborated canonical sibling, not a caller's
      // alternate/reparse spelling after its image identity check.
      // Pass a fixed snapshot rather than re-inheriting a concurrently mutated
      // environment in the helper. Presence of qualification/commissioning keys
      // is denial only; no variable grants routing or launch authority.
      environment = ProtectedRestartEnvironment();
      child_arguments.push_back(route.argument);
    } else {
      // A malformed/redirected protected-looking spelling cannot escape into
      // the legacy path merely because its physical target is elsewhere.
      if (LooksProtectedRestartNamespace(exe, program_files) ||
          LooksProtectedRestartNamespace(actual_path, program_files))
        return false;
      if (commissioning::StartupBinding().state !=
          commissioning::GuardState::Unarmed)
        return commissioning::SpawnGuardedHelper(helper, exe, arguments);
      for (const auto &value : arguments)
        child_arguments.push_back(Wide(value));
    }
    std::wstring command = QuoteWindowsArgument(launch_helper) + L" " +
                           std::to_wstring(GetCurrentProcessId()) + L" " +
                           QuoteWindowsArgument(target);
    for (const auto &value : child_arguments)
      command += L" " + QuoteWindowsArgument(value);
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    PROCESS_INFORMATION process{};
    const bool ok =
        CreateProcessW(
            launch_helper.c_str(), command.data(), nullptr, nullptr, FALSE,
            CREATE_NO_WINDOW |
                (environment.empty() ? 0 : CREATE_UNICODE_ENVIRONMENT),
            environment.empty() ? nullptr : environment.data(), nullptr,
            &startup, &process) != FALSE;
    if (ok) {
      CloseHandle(process.hThread);
      CloseHandle(process.hProcess);
    }
    return ok;
  } catch (...) {
    return false;
  }
}

} // namespace opennav::platform
