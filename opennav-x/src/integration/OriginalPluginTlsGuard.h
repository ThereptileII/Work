#pragma once
#ifdef _WIN32
#include "integration/Win32PluginTlsImports.h"
#include "integration/PluginTlsBytePolicy.h"
#include <wx/msw/wrapwin.h>
#include <wx/filename.h>
#include <wx/string.h>

namespace opennav::integration {
// Hold a read-only, non-write/non-delete-shared handle across the original
// LoadLibrary call. No profile/plugin mutation, DLL execution or dependency
// search is performed by the inspection. This is not a hostile-filesystem or
// complete Windows loader security boundary (see the architecture document).
class OriginalPluginTlsGuard {
 public:
  explicit OriginalPluginTlsGuard(const wxString& path) {
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
      const wxString directory = wxFileName(path).GetPathWithSep();
      if (directory.empty()) {
        refusal_ = "Cannot identify plugin import directory";
        return;
      }
      refusal_ = RefuseLegacyPluginLocalTlsClosure(bytes,
          [&](const std::string& module) { return ReadSibling(directory, module); },
          RefuseKnownObsoletePluginTlsBytes);
    } catch (const std::exception&) {
      refusal_ = "Cannot allocate plugin import inspection";
    }
  }
  ~OriginalPluginTlsGuard() {
    for (auto handle : siblings_) CloseHandle(handle);
    if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_);
  }
  OriginalPluginTlsGuard(const OriginalPluginTlsGuard&) = delete;
  OriginalPluginTlsGuard& operator=(const OriginalPluginTlsGuard&) = delete;
  const std::string& Refusal() const { return refusal_; }
 private:
  PluginTlsLocalModule ReadSibling(const wxString& directory,
                                   const std::string& module) {
    const wxString candidate = directory + wxString::FromUTF8(module.c_str());
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
    try {
      siblings_.push_back(handle);
    } catch (...) {
      CloseHandle(handle);
      throw;
    }
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
  std::vector<HANDLE> siblings_;
  std::string refusal_;
};
}
#endif
