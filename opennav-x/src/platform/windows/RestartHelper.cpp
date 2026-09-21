#include "platform/windows/WindowsArguments.h"
#include <windows.h>
#include <cwchar>

int wmain(int argc, wchar_t** argv) {
  if (argc < 3) return 2;
  wchar_t* end = nullptr;
  const unsigned long id = std::wcstoul(argv[1], &end, 10);
  if (!id || !end || *end != L'\0') return 2;
  HANDLE parent = OpenProcess(SYNCHRONIZE, FALSE, static_cast<DWORD>(id));
  if (parent) {
    const DWORD status = WaitForSingleObject(parent, 30000);
    CloseHandle(parent);
    if (status != WAIT_OBJECT_0) return 3;
  } else if (GetLastError() != ERROR_INVALID_PARAMETER) {
    return 4;
  }
  std::wstring command;
  for (int i = 2; i < argc; ++i) {
    if (i != 2) command += L' ';
    command += opennav::platform::QuoteWindowsArgument(argv[i]);
  }
  STARTUPINFOW startup{};
  startup.cb = sizeof(startup);
  PROCESS_INFORMATION process{};
  if (!CreateProcessW(argv[2], command.data(), nullptr, nullptr, FALSE, 0,
                       nullptr, nullptr, &startup, &process)) return 5;
  CloseHandle(process.hThread);
  CloseHandle(process.hProcess);
  return 0;
}
