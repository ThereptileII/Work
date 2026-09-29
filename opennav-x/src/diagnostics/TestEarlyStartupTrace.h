#pragma once
#include "integration/BuildFeatures.h"

// Pre-log evidence for an explicitly opted-in Linux/GTK fixture process only.
// No installed product or native Windows entry point is changed by this hook.
#if XNAV_ENABLE_TEST_FIXTURES && defined(__linux__) && defined(__WXGTK__) && \
    !defined(_WIN32) && !defined(__WXMSW__)
#define XNAV_TEST_EARLY_STARTUP 1
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace opennav::diagnostics {
enum class StartupStage {
  Entry,
  EntryReturn,
  Initialize,
  InitializeReturn,
  OnInit,
  ParserReturn,
  ParserDefinition,
  ParserParsed,
  ParserHelp,
  ParserError,
  OpenNavParserReturn,
  ModeConflict,
  RemoteConflict,
  PreviewPathFailure,
  FixturePolicyFailure,
  ConfigDirectory,
  ConfigDirectoryReturn,
  LogInitialize,
  LogInitializeReturn,
};

inline const char* StartupStageName(StartupStage stage) {
  switch (stage) {
    case StartupStage::Entry: return "entry";
    case StartupStage::EntryReturn: return "entry.return";
    case StartupStage::Initialize: return "initialize";
    case StartupStage::InitializeReturn: return "initialize.return";
    case StartupStage::OnInit: return "on_init";
    case StartupStage::ParserReturn: return "parser.return";
    case StartupStage::ParserDefinition: return "parser.definition";
    case StartupStage::ParserParsed: return "parser.parsed";
    case StartupStage::ParserHelp: return "parser.help";
    case StartupStage::ParserError: return "parser.error";
    case StartupStage::OpenNavParserReturn: return "opennav_parser.return";
    case StartupStage::ModeConflict: return "mode_conflict";
    case StartupStage::RemoteConflict: return "remote_conflict";
    case StartupStage::PreviewPathFailure: return "preview_path_failure";
    case StartupStage::FixturePolicyFailure: return "fixture_policy_failure";
    case StartupStage::ConfigDirectory: return "config_directory";
    case StartupStage::ConfigDirectoryReturn: return "config_directory.return";
    case StartupStage::LogInitialize: return "log_initialize";
    case StartupStage::LogInitializeReturn: return "log_initialize.return";
  }
  return "unknown";
}

inline void TestEarlyStartupTrace(StartupStage stage, int result = 0) noexcept {
  const int saved_errno = errno;
  try {
    static const std::filesystem::path sink = [] {
      const char* value = std::getenv("OPENNAV_TEST_EARLY_STARTUP_TRACE");
      const char* file = std::getenv("OPENNAV_TEST_EARLY_STARTUP_TRACE_FILE");
      if (value && std::strcmp(value, "1") == 0 && file) {
        try {
          const auto path = std::filesystem::u8path(file);
          std::error_code error;
          if (path.is_absolute() && path.filename() == "opennav-startup-trace.log" &&
              std::filesystem::is_regular_file(std::filesystem::symlink_status(
                  path.parent_path() / "OPENNAV_TEST_PROFILE", error)) && !error)
            return path;
        } catch (...) {}
      }
      return std::filesystem::path{};
    }();
    static unsigned count = 0;
    if (!sink.empty() && count < 64) {
      ++count;
      char record[160];
      const int length = std::snprintf(record, sizeof(record),
          "OpenNav startup pid=%ld stage=%s result=%d\n",
          static_cast<long>(getpid()), StartupStageName(stage), result);
      // GUI runtimes may close/reuse stderr. Open only the explicitly marked
      // disposable trace file. Never follow a sink symlink or block on a FIFO.
      // No fsync/retry; observation failure cannot change startup's result.
      if (length > 0 && static_cast<unsigned>(length) < sizeof(record)) {
        const int fd = ::open(sink.c_str(), O_WRONLY | O_CREAT | O_APPEND |
            O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK, 0600);
        if (fd >= 0) {
          struct stat status {};
          if (::fstat(fd, &status) == 0 && S_ISREG(status.st_mode))
            (void)::write(fd, record, static_cast<unsigned>(length));
          ::close(fd);
        }
      }
    }
  } catch (...) {
    // Allocation/path failures in optional observation must never prevent the
    // original startup call, alter its result, or escape across wx boundaries.
  }
  errno = saved_errno;
}

template<class Call>
inline auto ObserveStartupCall(StartupStage before, StartupStage after,
                               Call&& call) {
  TestEarlyStartupTrace(before);
  const auto result = call();  // exactly once, preserving exceptions and result
  TestEarlyStartupTrace(after, static_cast<int>(result));
  return result;
}
}  // namespace opennav::diagnostics
#define XNAV_EARLY_STARTUP_TRACE(stage, result) \
  ::opennav::diagnostics::TestEarlyStartupTrace( \
      ::opennav::diagnostics::StartupStage::stage, result)
#else
#define XNAV_TEST_EARLY_STARTUP 0
#define XNAV_EARLY_STARTUP_TRACE(...) do {} while (false)
#endif
