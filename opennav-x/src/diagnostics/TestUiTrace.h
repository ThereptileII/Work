#pragma once
#include "integration/BuildFeatures.h"

// Opt-in, bounded evidence for isolated native interaction tests. The installed
// product has neither the environment switch nor the trace implementation.
#if XNAV_ENABLE_TEST_FIXTURES
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
namespace opennav::diagnostics {
inline void TestUiTrace(const char *stage, std::uint64_t tick = 0,
                        std::int64_t detail = 0) {
  static const std::filesystem::path sink = [] {
    const auto *value = std::getenv("OPENNAV_TEST_UI_TRACE");
    const auto *file = std::getenv("OPENNAV_TEST_UI_TRACE_FILE");
    if (value && std::strcmp(value, "1") == 0 && file) {
      try {
        auto path = std::filesystem::u8path(file);
        if (path.is_absolute() && path.filename() == "opennav-ui-trace.log" &&
            std::filesystem::is_regular_file(path.parent_path() / "OPENNAV_TEST_PROFILE"))
          return path;
      } catch (const std::filesystem::filesystem_error &) {}
    }
    return std::filesystem::path{};
  }();
  static unsigned count = 0;
  if (sink.empty() || count >= 2048) return;
  ++count;
  const auto at = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  // GUI runtimes may close/reuse stderr. Use only the marked disposable
  // profile, closing after each event so even a stuck callback is observable.
  std::ofstream out(sink, std::ios::app | std::ios::binary);
  out << "OpenNav test-ui " << at << ' ' << stage << " tick=" << tick
      << " detail=" << detail << '\n';
}
} // namespace opennav::diagnostics
#define XNAV_TEST_UI_TRACE(...) ::opennav::diagnostics::TestUiTrace(__VA_ARGS__)
#else
#define XNAV_TEST_UI_TRACE(...) do {} while (false)
#endif
