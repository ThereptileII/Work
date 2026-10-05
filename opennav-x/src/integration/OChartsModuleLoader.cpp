#include "integration/OChartsModuleLoader.h"
#include "integration/PluginPresentationFallback.h"
#include <wx/filename.h>
#include <wx/thread.h>
#include <algorithm>
#include <cstring>
#include <iterator>
#ifdef _WIN32
#include <windows.h>
#include "picosha2.h"
#include <array>
#endif
namespace opennav::integration {
bool ValidOChartsStatus(const SkagerChartPresentationStatusV1 &value) {
  if (value.structBytes != sizeof(value) || value.version != SKAGER_CHART_BINDING_VERSION ||
      std::any_of(std::begin(value.reserved), std::end(value.reserved),
                  [](uint32_t word) { return word != 0; })) return false;
  switch (value.state) {
    case SKAGER_CHART_UNBOUND: return value.reason == SKAGER_CHART_REASON_UNBOUND;
    case SKAGER_CHART_BOUND_PENDING_INITIALIZATION:
    case SKAGER_CHART_SELECTED: return value.reason == SKAGER_CHART_REASON_NONE;
    case SKAGER_CHART_STANDARD_FALLBACK:
      return value.reason == SKAGER_CHART_REASON_RESOURCE_VERIFICATION ||
             value.reason == SKAGER_CHART_REASON_RENDERER_INITIALIZATION;
    default: return false;
  }
}

namespace {
#ifdef _WIN32
class LockedFile {
public:
  explicit LockedFile(const wxString &path) {
    // The installed application owns an ordinary local file. Refuse links and
    // redirected parent paths instead of extending the executable trust root.
    wxFileName node(path);
    if (!node.IsAbsolute() || path.StartsWith("\\\\")) return;
    wxString checked = node.GetFullPath();
    for (;;) {
      const auto attributes = GetFileAttributesW(checked.wc_str());
      if (attributes == INVALID_FILE_ATTRIBUTES ||
          (attributes & FILE_ATTRIBUTE_REPARSE_POINT)) return;
      wxFileName parent(checked);
      const auto next = parent.GetPath();
      if (next.empty() || next == checked) break;
      checked = next;
    }
    file_ = CreateFileW(path.wc_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (file_ == INVALID_HANDLE_VALUE) return;
    // Bind the hash to the actual opened local path, including a concurrent
    // redirect between directory inspection and opening. Never resolve a DLL
    // through an unexpected mount/junction after verifying a different path.
    std::array<wchar_t, 32768> final_path{}, absolute_path{};
    const auto final_size = GetFinalPathNameByHandleW(file_, final_path.data(),
        static_cast<DWORD>(final_path.size()), FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
    const auto absolute_size = GetFullPathNameW(path.wc_str(),
        static_cast<DWORD>(absolute_path.size()), absolute_path.data(), nullptr);
    if (!final_size || final_size >= final_path.size() || !absolute_size ||
        absolute_size >= absolute_path.size()) {
      CloseHandle(file_);
      file_ = INVALID_HANDLE_VALUE;
      return;
    }
    wxString resolved(final_path.data(), final_size);
    if (resolved.StartsWith("\\\\?\\")) resolved = resolved.Mid(4);
    const wxString expected(absolute_path.data(), absolute_size);
    wxFileName root(expected);
    const auto drive = root.GetVolume() + ":\\";
    if (resolved.CmpNoCase(expected) != 0 ||
        GetDriveTypeW(drive.wc_str()) != DRIVE_FIXED) {
      CloseHandle(file_);
      file_ = INVALID_HANDLE_VALUE;
    }
  }
  ~LockedFile() { if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_); }
  LockedFile(const LockedFile &) = delete;
  LockedFile &operator=(const LockedFile &) = delete;
  bool Matches(const char *digest, std::uint64_t expected_bytes = 0) {
    if (file_ == INVALID_HANDLE_VALUE || !digest || std::strlen(digest) != 64)
      return false;
    LARGE_INTEGER size{};
    if (!GetFileSizeEx(file_, &size) || size.QuadPart <= 0 ||
        size.QuadPart > 128ll * 1024 * 1024 ||
        (expected_bytes && static_cast<std::uint64_t>(size.QuadPart) != expected_bytes))
      return false;
    picosha2::hash256_one_by_one hash;
    std::array<unsigned char, 16384> bytes{};
    std::uint64_t remaining = static_cast<std::uint64_t>(size.QuadPart);
    while (remaining) {
      DWORD count = 0;
      const auto wanted = static_cast<DWORD>((std::min)(remaining, std::uint64_t(bytes.size())));
      if (!ReadFile(file_, bytes.data(), wanted, &count, nullptr) || count != wanted)
        return false;
      hash.process(bytes.begin(), bytes.begin() + count);
      remaining -= count;
    }
    hash.finish();
    return picosha2::get_hash_hex_string(hash) == digest;
  }
private:
  HANDLE file_ = INVALID_HANDLE_VALUE;
};
#endif
}
OChartsModuleResult LoadOChartsModule(wxDynamicLibrary &library,
    const OChartsModuleRequest &request, const PluginCompatibilityCheck &compatible) {
#ifdef _WIN32
  if (!request.main_thread || !wxIsMainThread() || request.safe_mode ||
      request.resources.empty() ||
      wxFileName(request.original).GetFullName().CmpNoCase("o-charts_pi.dll") != 0)
    return {};
  const auto refused = [](const char *reason) {
    return OChartsModuleResult{false, true, reason};
  };
  if (library.IsLoaded()) return refused("destination module is already loaded");
  const auto &original = request.original;
  const auto &adapter = request.adapter;
  const auto &resources = request.resources;
  constexpr auto original_hash = OriginalOChartsSha256;
  if (!request.package_available)
    return refused("private chart presentation is not included in this build");
  {
    LockedFile original_file(original), adapter_file(adapter);
    if (!original_file.Matches(original_hash))
      return refused("original plugin identity is unsupported");
    if (!adapter_file.Matches(request.adapter_sha256.c_str(), request.adapter_bytes))
      return refused("private adapter is missing or changed");
  }
  // Upstream's PE inspection opens with exclusive sharing. Inspect only after
  // hash verification, then reacquire locks and rehash before executing code.
  if (!compatible || !compatible(adapter))
    return refused("private adapter ABI is incompatible");
  LockedFile original_file(original), adapter_file(adapter);
  if (!original_file.Matches(original_hash) ||
      !adapter_file.Matches(request.adapter_sha256.c_str(), request.adapter_bytes))
    return refused("plugin changed during qualification");
  SkagerChartBindingV1 binding{};
  binding.structBytes = sizeof(binding);
  binding.version = SKAGER_CHART_BINDING_VERSION;
  const auto utf8 = resources.utf8_str();
  if (!utf8 || !utf8.length() || utf8.length() >= sizeof(binding.resourceDirectory))
    return refused("presentation path cannot be represented");
  std::memcpy(binding.resourceDirectory, utf8.data(), utf8.length());
  if (!library.Load(adapter)) return refused("private adapter did not load");
  const auto bind = reinterpret_cast<SkagerBindChartPresentationV1>(
      library.GetSymbol(SKAGER_CHART_BINDING_EXPORT));
  const auto query = reinterpret_cast<SkagerGetChartPresentationStatusV1>(
      library.GetSymbol(SKAGER_CHART_STATUS_EXPORT));
  if (!bind || !query || !library.HasSymbol("create_pi") ||
      !library.HasSymbol("destroy_pi") || bind(&binding) != 1) {
    UnloadPluginModuleChecked(library);
    return refused("private adapter rejected presentation binding");
  }
  SkagerChartPresentationStatusV1 observation{};
  observation.structBytes = sizeof(observation);
  observation.version = SKAGER_CHART_BINDING_VERSION;
  if (query(&observation) != 1 || !ValidOChartsStatus(observation) ||
      observation.state != SKAGER_CHART_BOUND_PENDING_INITIALIZATION) {
    UnloadPluginModuleChecked(library);
    return refused("private adapter returned invalid pre-initialization state");
  }
  return {true, true, {}};
#else
  (void)library; (void)request; (void)compatible;
  return {};
#endif
}
}
