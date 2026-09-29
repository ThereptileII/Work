#pragma once
// Test-only readiness reader. Never linked into the application/restart helper.
#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif

namespace opennav::tests {
inline std::string MarkerPathUtf8(const std::filesystem::path &path) {
  const auto bytes = path.u8string();
  // C++20 changes u8string's element type to char8_t. Preserve its UTF-8
  // representation without locale conversion in either language standard.
  return {reinterpret_cast<const char *>(bytes.data()), bytes.size()};
}

class MarkerReadError : public std::runtime_error {
public:
  MarkerReadError(const std::filesystem::path &path, const std::string &caller,
                  const std::string &stage_value, std::uint32_t code_value)
      : std::runtime_error("Fixture marker " + caller + " failed at " + stage_value +
                           ": " + MarkerPathUtf8(path) + " / " +
#ifdef _WIN32
                           "Win32=" +
#else
                           "errno=" +
#endif
                           std::to_string(code_value)), stage(stage_value), code(code_value) {}
  const std::string stage;
  const std::uint32_t code;
};

inline std::string ReadClosedMarker(const std::filesystem::path &path,
                                    const std::string &caller) {
  constexpr std::size_t maximum = 1024 * 1024;
#ifdef _WIN32
  // Rename can expose the final name while its DELETE handle remains open.
  // Sharing DELETE permits that completed rename, but omitting WRITE rejects
  // an unclosed content writer. One open; never retry a failed readiness read.
  const auto raw = CreateFileW(path.c_str(), GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL, nullptr);
  if (raw == INVALID_HANDLE_VALUE) {
    const DWORD code = GetLastError();
    throw MarkerReadError(path, caller, "CreateFileW", code);
  }
  struct Handle {
    HANDLE value;
    ~Handle() { if (value != INVALID_HANDLE_VALUE) CloseHandle(value); }
  } handle{raw};
  LARGE_INTEGER length{};
  if (!GetFileSizeEx(raw, &length)) {
    const DWORD code = GetLastError();
    throw MarkerReadError(path, caller, "GetFileSizeEx", code);
  }
  if (length.QuadPart < 0 || length.QuadPart > static_cast<LONGLONG>(maximum))
    throw MarkerReadError(path, caller, "size exceeds one MiB", ERROR_FILE_TOO_LARGE);
  std::string result(static_cast<std::size_t>(length.QuadPart), '\0');
  std::size_t offset = 0;
  while (offset < result.size()) {
    DWORD received = 0;
    if (!ReadFile(raw, result.data() + offset,
                  static_cast<DWORD>(result.size() - offset), &received, nullptr)) {
      const DWORD code = GetLastError();
      throw MarkerReadError(path, caller, "ReadFile", code);
    }
    if (!received)
      throw MarkerReadError(path, caller, "ReadFile premature EOF", ERROR_HANDLE_EOF);
    offset += received;
  }
  handle.value = INVALID_HANDLE_VALUE;
  if (!CloseHandle(raw)) {
    const DWORD code = GetLastError();
    throw MarkerReadError(path, caller, "CloseHandle", code);
  }
  return result;
#else
  errno = 0;
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) {
    const auto code = errno;
    throw MarkerReadError(path, caller, "open", code);
  }
  const auto length = input.tellg();
  if (length < 0) throw MarkerReadError(path, caller, "size", EIO);
  if (length > static_cast<std::streamoff>(maximum))
    throw MarkerReadError(path, caller, "size exceeds one MiB", EFBIG);
  input.seekg(0);
  std::string result(static_cast<std::size_t>(length), '\0');
  if (!result.empty()) input.read(result.data(), static_cast<std::streamsize>(result.size()));
  if (!input) throw MarkerReadError(path, caller, "read", EIO);
  return result;
#endif
}
} // namespace opennav::tests
