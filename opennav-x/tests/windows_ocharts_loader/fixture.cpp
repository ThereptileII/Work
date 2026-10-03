// Deliberately tiny test DLL. No OpenCPN, chart, helper, transport or UI code.
#include "plugin-adapters/ocharts/BindingState.h"
#include <windows.h>
#include <array>
#include <cstring>
#include <cwchar>

namespace {
skager::ocharts::BindingState binding;
void Event(const char* text) {
  wchar_t path[32768]{};
  const auto n=GetEnvironmentVariableW(L"SKAGER_LOADER_EVENTS",path,32768);
  if(!n || n>=32768) return;
  HANDLE file=CreateFileW(path,FILE_APPEND_DATA,FILE_SHARE_READ|FILE_SHARE_WRITE,
                          nullptr,OPEN_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
  if(file==INVALID_HANDLE_VALUE) return;
  DWORD wrote=0;
  WriteFile(file,text,static_cast<DWORD>(std::strlen(text)),&wrote,nullptr);
  WriteFile(file,"\n",1,&wrote,nullptr);CloseHandle(file);
}
bool Mode(const wchar_t* wanted) {
  wchar_t mode[128]{};
  const auto n=GetEnvironmentVariableW(L"SKAGER_LOADER_MODE",mode,128);
  return n && n<128 && !std::wcscmp(mode,wanted);
}
bool BoundOriginalLock(DWORD access) {
  wchar_t path[32768]{};
  const auto n=GetEnvironmentVariableW(L"SKAGER_LOADER_ORIGINAL",path,32768);
  if(!n || n>=32768) return false;
  HANDLE file=CreateFileW(path,access,FILE_SHARE_READ|FILE_SHARE_WRITE|FILE_SHARE_DELETE,
                          nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
  if(file!=INVALID_HANDLE_VALUE) {CloseHandle(file);return false;}
  return GetLastError()==ERROR_SHARING_VIOLATION;
}
}
BOOL WINAPI DllMain(HINSTANCE, DWORD reason, LPVOID) {
  // Bounded plain file evidence; no loader, CRT stream, thread or plugin calls.
  if(reason==DLL_PROCESS_ATTACH) Event("attach");
  if(reason==DLL_PROCESS_DETACH) Event("detach");
  return TRUE;
}
#ifndef OMIT_CREATE
extern "C" __declspec(dllexport) void* __cdecl create_pi(void*) {
  Event("UNEXPECTED_FACTORY");ExitProcess(97);return nullptr;
}
#endif
#ifndef OMIT_DESTROY
extern "C" __declspec(dllexport) void __cdecl destroy_pi(void*) {
  Event("UNEXPECTED_DESTROY");ExitProcess(98);
}
#endif
#ifndef OMIT_BIND
extern "C" __declspec(dllexport) int32_t SKAGER_CHART_CALL
skager_bind_chart_presentation_v1(const SkagerChartBindingV1* value) {
  Event("bind");
  if(!BoundOriginalLock(GENERIC_WRITE) || !BoundOriginalLock(DELETE)) {
    Event("UNEXPECTED_UNLOCKED_ORIGINAL");return 0;
  }
  Event("original-write-delete-locked");
  if(Mode(L"reject-bind")) return 0;
  wchar_t expected[4096]{};
  const auto n=GetEnvironmentVariableW(L"SKAGER_LOADER_RESOURCES",expected,4096);
  std::array<char,4096> utf8{};
  if(!n || n>=4096 || !WideCharToMultiByte(CP_UTF8,WC_ERR_INVALID_CHARS,expected,-1,
       utf8.data(),static_cast<int>(utf8.size()),nullptr,nullptr) || !value ||
     std::memcmp(value->resourceDirectory,utf8.data(),utf8.size())) {
    Event("UNEXPECTED_BINDING_PATH");return 0;
  }
  return binding.Bind(value) ? 1 : 0;
}
#endif
#ifndef OMIT_STATUS
extern "C" __declspec(dllexport) int32_t SKAGER_CHART_CALL
skager_chart_presentation_status_v1(SkagerChartPresentationStatusV1* out) {
  Event("status");
  if(!binding.ReadStatus(out)) return 0;
  if(Mode(L"status-return")) return 0;
  if(Mode(L"status-size")) --out->structBytes;
  if(Mode(L"status-version")) ++out->version;
  if(Mode(L"status-reserved")) out->reserved[7]=1;
  if(Mode(L"status-reason")) out->reason=SKAGER_CHART_REASON_UNBOUND;
  if(Mode(L"status-selected")) out->state=SKAGER_CHART_SELECTED;
  if(Mode(L"status-unknown")) out->state=99;
  return 1;
}
#endif
