#pragma once
#ifdef _WIN32
#include "integration/Win32PluginTlsImports.h"
#include "integration/PluginTlsBytePolicy.h"
#include <wx/msw/wrapwin.h>
#include <wx/filename.h>
#include <wx/string.h>

namespace opennav::integration {
// Hold a read-only, non-write/non-delete-shared handle across the original
// module lifetime, including forwarded exports, declared delay loads and
// detach callbacks. No profile/plugin mutation, DLL execution or dependency
// search-order mutation is performed by inspection. Declared candidates in the
// original plugin, executable and actual upstream plugin roots share one walk.
// This is not a hostile-filesystem or complete Windows loader security boundary.
class OriginalPluginTlsGuard {
 public:
  explicit OriginalPluginTlsGuard(const wxString& path,
      const std::vector<std::string>& plugin_roots = {}) {
    // Upstream passes its actual Libdirs(), never the process PATH. Bound that
    // handoff before allocating/opening any additional directory candidates.
    if (plugin_roots.size() > PluginTlsClosureLimits{}.max_nodes - 2) {
      refusal_ = "Plugin import closure node budget exceeded";
      return;
    }
    file_ = CreateFileW(path.wc_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file_ == INVALID_HANDLE_VALUE) {
      refusal_ = "Cannot open plugin for import inspection";
      return;
    }
    LARGE_INTEGER size{};
    if (!GetFileSizeEx(file_, &size) || size.QuadPart < 64 ||
        size.QuadPart > 128ll * 1024 * 1024) {
      refusal_ = "Unsupported plugin size for import inspection";
      return;
    }
    try {
      std::vector<unsigned char> bytes(static_cast<std::size_t>(size.QuadPart));
      DWORD count = 0;
      if (!ReadFile(file_, bytes.data(), static_cast<DWORD>(bytes.size()),
                    &count, nullptr) || count != bytes.size()) {
        refusal_ = "Cannot read complete plugin for import inspection";
        return;
      }
      // Get the executable path directly; do not use the process current
      // directory, PATH, or a plugin-supplied host location as authority.
      std::vector<wchar_t> executable(32768);
      const DWORD length = GetModuleFileNameW(nullptr, executable.data(),
                                             static_cast<DWORD>(executable.size()));
      if (!length || length >= executable.size()) {
        refusal_ = "Cannot identify host executable for import inspection";
        return;
      }
      std::vector<wxString> requested{
          wxFileName(path).GetPath(),
          wxFileName(wxString(executable.data(), length)).GetPath()};
      for (const auto& directory : plugin_roots)
        requested.push_back(wxFileName::DirName(wxString(directory)).GetPath());
      std::vector<wxString> directories;
      std::vector<std::string> identities;
      for (std::size_t index = 0; index < requested.size(); ++index) {
        std::string identity;
        refusal_ = IdentifyDirectory(requested[index], identity, index >= 2);
        if (!refusal_.empty()) return;
        // Libdirs may include a currently absent user plugin directory. There
        // is no existing candidate to inspect there; do not create it. This
        // does not claim exclusion of later filesystem additions.
        if (identity.empty()) continue;
        directories.push_back(requested[index]);
        identities.push_back(std::move(identity));
      }
      refusal_ = RefuseLegacyPluginTlsClosure(bytes, identities,
          [&](std::size_t index, const std::string& module) {
            return ReadCandidate(directories[index], module);
          }, RefuseKnownObsoletePluginTlsBytes);
    } catch (const std::exception&) {
      refusal_ = "Cannot allocate plugin import inspection";
    }
  }
  ~OriginalPluginTlsGuard() {
    for (auto handle : dependencies_) CloseHandle(handle);
    if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_);
  }
  OriginalPluginTlsGuard(const OriginalPluginTlsGuard&) = delete;
  OriginalPluginTlsGuard& operator=(const OriginalPluginTlsGuard&) = delete;
  const std::string& Refusal() const { return refusal_; }
 private:
  void KeepHandle(HANDLE handle) {
    try {
      dependencies_.push_back(handle);
    } catch (...) {
      CloseHandle(handle);
      throw;
    }
  }
  std::string IdentifyDirectory(const wxString& directory, std::string& identity,
                                bool permit_missing) {
    if (directory.empty() || !wxFileName::DirName(directory).IsAbsolute())
      return "Cannot identify absolute plugin import directory";
    // Follow a directory alias only to obtain its actual identity. Keeping this
    // read-attributes handle also prevents write/delete sharing on that directory
    // while custody lasts; ancestors and hostile filesystem races remain outside
    // this policy's claimed boundary.
    HANDLE handle = CreateFileW(directory.wc_str(), FILE_READ_ATTRIBUTES,
                                FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                                FILE_FLAG_BACKUP_SEMANTICS, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
      const auto error = GetLastError();
      if (permit_missing &&
          (error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND))
        return {};
      return "Cannot open plugin import directory for inspection";
    }
    KeepHandle(handle);
    BY_HANDLE_FILE_INFORMATION information{};
    if (!GetFileInformationByHandle(handle, &information) ||
        !(information.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
      return "Cannot identify plugin import directory";
    identity = std::to_string(information.dwVolumeSerialNumber) + ":" +
               std::to_string(information.nFileIndexHigh) + ":" +
               std::to_string(information.nFileIndexLow);
    return {};
  }
  PluginTlsLocalModule ReadCandidate(const wxString& directory,
                                     const std::string& module) {
    const wxString candidate =
        wxFileName(directory, wxString::FromUTF8(module.c_str())).GetFullPath();
    HANDLE handle = CreateFileW(candidate.wc_str(), GENERIC_READ, FILE_SHARE_READ,
                                nullptr, OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT,
                                nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
      const auto error = GetLastError();
      if (error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND)
        return {};
      return {false, {}, "Cannot inspect local plugin dependency"};
    }
    KeepHandle(handle);
    BY_HANDLE_FILE_INFORMATION information{};
    if (!GetFileInformationByHandle(handle, &information) ||
        (information.dwFileAttributes &
         (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT)))
      return {false, {}, "Unsupported local plugin dependency path"};
    LARGE_INTEGER size{};
    if (!GetFileSizeEx(handle, &size) || size.QuadPart < 64 ||
        size.QuadPart > 128ll * 1024 * 1024)
      return {false, {}, "Unsupported local plugin dependency size"};
    std::vector<unsigned char> bytes(static_cast<std::size_t>(size.QuadPart));
    DWORD count = 0;
    if (!ReadFile(handle, bytes.data(), static_cast<DWORD>(bytes.size()),
                  &count, nullptr) || count != bytes.size())
      return {false, {}, "Cannot read complete local plugin dependency"};
    return {true, std::move(bytes), {}};
  }
  HANDLE file_ = INVALID_HANDLE_VALUE;
  std::vector<HANDLE> dependencies_;
  std::string refusal_;
};
}
#endif
