#pragma once
#include <string>

namespace opennav::platform {

// CommandLineToArgvW/MSVC escaping, not shell escaping.
inline std::wstring QuoteWindowsArgument(const std::wstring& argument) {
  std::wstring result = L"\"";
  std::size_t slashes = 0;
  for (const wchar_t ch : argument) {
    if (ch == L'\\') { ++slashes; continue; }
    if (ch == L'\"') result.append(slashes * 2 + 1, L'\\');
    else result.append(slashes, L'\\');
    slashes = 0;
    result += ch;
  }
  result.append(slashes * 2, L'\\');
  result += L'\"';
  return result;
}

}  // namespace opennav::platform
