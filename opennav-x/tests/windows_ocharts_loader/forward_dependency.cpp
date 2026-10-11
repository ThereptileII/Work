// Native forwarder targets only: no TLS, plugin factory, transport or UI code.
#include <windows.h>
#include <cstring>
#ifndef FORWARD_VALUE
#define FORWARD_VALUE 209
#endif
extern "C" int __cdecl fixture_value() { return FORWARD_VALUE; }
extern "C" int __cdecl fixture_other_value() { return FORWARD_VALUE + 1; }
BOOL WINAPI DllMain(HINSTANCE, DWORD reason, LPVOID) {
  if (reason != DLL_PROCESS_ATTACH && reason != DLL_PROCESS_DETACH) return TRUE;
  wchar_t path[32768]{};
  const auto n = GetEnvironmentVariableW(L"SKAGER_LOADER_EVENTS", path, 32768);
  if (!n || n >= 32768) return TRUE;
  HANDLE file = CreateFileW(path, FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE,
                            nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) return TRUE;
  const char* text = reason == DLL_PROCESS_ATTACH ? "forward-target-attach\n" : "forward-target-detach\n";
  DWORD wrote = 0;
  WriteFile(file, text, static_cast<DWORD>(std::strlen(text)), &wrote, nullptr);
  CloseHandle(file);
  return TRUE;
}
