#include "plugin-adapters/ocharts/BindingState.h"
#include <cstdio>
#include <memory>
#include <windows.h>
#include <winver.h>

namespace {
skager::ocharts::BindingState binding;
void Stage(const char* value) {
  std::fprintf(stderr, "stage=%s\n", value);
  std::fflush(stderr);
}
LONG WINAPI ObserveException(EXCEPTION_POINTERS* exception) {
  HMODULE module = nullptr;
  wchar_t path[32768]{};
  char utf8[32768]{};
  const auto address = exception->ExceptionRecord->ExceptionAddress;
  if (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                        reinterpret_cast<LPCWSTR>(address), &module)) {
    GetModuleFileNameW(module, path, 32768);
    WideCharToMultiByte(CP_UTF8, 0, path, -1, utf8, 32768, nullptr, nullptr);
  }
  std::fprintf(stderr, "exception=%08lX module=%s rva=%llu\n",
      exception->ExceptionRecord->ExceptionCode, utf8,
      static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(address) -
                                     reinterpret_cast<uintptr_t>(module)));
  std::fflush(stderr);
  return EXCEPTION_CONTINUE_SEARCH;  // Observe; never suppress the crash.
}
bool Runtime(const wchar_t* name, const char* label) {
  const auto module = GetModuleHandleW(name);
  wchar_t path[32768]{};
  char utf8[32768]{};
  const DWORD count = GetModuleFileNameW(module, path, 32768);
  if (!module || !count || count >= 32768 ||
      !WideCharToMultiByte(CP_UTF8, 0, path, -1, utf8, 32768, nullptr, nullptr)) return false;
  DWORD ignored = 0;
  const DWORD bytes = GetFileVersionInfoSizeW(path, &ignored);
  if (!bytes || bytes > 1024 * 1024) return false;
  const auto data = std::make_unique<unsigned char[]>(bytes);
  VS_FIXEDFILEINFO* version = nullptr;
  UINT size = 0;
  if (!GetFileVersionInfoW(path, 0, bytes, data.get()) ||
      !VerQueryValueW(data.get(), L"\\", reinterpret_cast<void**>(&version), &size) ||
      size < sizeof(*version) || version->dwSignature != 0xfeef04bd) return false;
  std::printf("runtime|%s|%s|%u.%u.%u.%u\n", label, utf8,
      HIWORD(version->dwFileVersionMS), LOWORD(version->dwFileVersionMS),
      HIWORD(version->dwFileVersionLS), LOWORD(version->dwFileVersionLS));
  std::fflush(stdout);
  return true;
}
SkagerChartPresentationStatusV1 Status() {
  SkagerChartPresentationStatusV1 result{};
  result.structBytes = sizeof(result);
  result.version = SKAGER_CHART_BINDING_VERSION;
  return result;
}
}  // namespace

int main() {
  SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
  if (!AddVectoredExceptionHandler(1, ObserveException)) return 2;
  std::printf("compiler=%d.%d\n", _MSC_VER, _MSC_FULL_VER);
#ifdef _DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR
  std::puts("guard=enabled");
#else
  std::puts("guard=disabled");
#endif
  if (!Runtime(L"msvcp140.dll", "msvcp140.dll") ||
      !Runtime(L"vcruntime140.dll", "vcruntime140.dll")) return 3;
  SkagerChartBindingV1 request{};
  request.structBytes = sizeof(request);
  request.version = SKAGER_CHART_BINDING_VERSION;
  constexpr char path[] = "C:\\SKAGER-binding-proof";
  std::memcpy(request.resourceDirectory, path, sizeof(path));
  Stage("before-bind");
  if (!binding.Bind(&request)) return 4;
  Stage("after-bind");
  auto status = Status();
  Stage("before-status");
  if (!binding.ReadStatus(&status) || status.state != SKAGER_CHART_BOUND_PENDING_INITIALIZATION ||
      status.reason != SKAGER_CHART_REASON_NONE) return 5;
  Stage("after-status");
  Stage("before-initialization");
  const auto copied = binding.BeginInitialization();
  if (std::strcmp(copied.data(), path) != 0) return 6;
  Stage("after-initialization");
  Stage("before-complete");
  binding.Complete(true, SKAGER_CHART_REASON_NONE);
  Stage("after-complete");
  status = Status();
  Stage("before-selected-status");
  if (!binding.ReadStatus(&status) || status.state != SKAGER_CHART_SELECTED ||
      status.reason != SKAGER_CHART_REASON_NONE) return 7;
  Stage("after-selected-status");
  Stage("before-repeat-bind");
  if (binding.Bind(&request)) return 8;
  Stage("after-repeat-bind");
  Stage("complete");
  return 0;
}
