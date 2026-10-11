#pragma once
#ifdef _WIN32
#include "integration/Win32PluginTlsImports.h"
#include <wx/msw/wrapwin.h>
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
      refusal_ = RefuseLegacyPluginTlsImports(bytes);
    } catch (const std::exception&) {
      refusal_ = "Cannot allocate plugin import inspection";
    }
  }
  ~OriginalPluginTlsGuard() {
    if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_);
  }
  OriginalPluginTlsGuard(const OriginalPluginTlsGuard&) = delete;
  OriginalPluginTlsGuard& operator=(const OriginalPluginTlsGuard&) = delete;
  const std::string& Refusal() const { return refusal_; }
 private:
  HANDLE file_ = INVALID_HANDLE_VALUE;
  std::string refusal_;
};
}
#endif
