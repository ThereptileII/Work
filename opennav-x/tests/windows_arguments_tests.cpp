#include "platform/windows/WindowsArguments.h"
#include <iostream>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif

int main() {
  using opennav::platform::QuoteWindowsArgument;
  if (QuoteWindowsArgument(L"") != L"\"\"") return 1;
  if (QuoteWindowsArgument(L"C:\\boat data\\") != L"\"C:\\boat data\\\\\"") return 2;
  if (QuoteWindowsArgument(L"a\"b") != L"\"a\\\"b\"") return 3;
#ifdef _WIN32
  const std::vector<std::wstring> values = {L"", L"plain", L"C:\\boat data\\",
      L"embedded\"quote", L"backslash\\\"quote", L"\u00c5ngstr\u00f6m", L"& exit $(no-shell)"};
  for (const auto& value : values) {
    const auto command = L"program.exe " + QuoteWindowsArgument(value);
    int count = 0;
    auto parsed = CommandLineToArgvW(command.c_str(), &count);
    if (!parsed) return 4;
    const bool equal = count == 2 && value == parsed[1];
    LocalFree(parsed);
    if (!equal) return 5;
  }
#endif
  std::cout << "Windows restart argument contract passed.\n";
}
