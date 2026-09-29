#include "diagnostics/TestEarlyStartupTrace.h"

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;
void Check(bool ok, const char* reason) {
  if (!ok) throw std::runtime_error(reason);
}
std::string Read(const fs::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
void Run(const fs::path& base, const std::string& scenario) {
  const auto folder = base / scenario;
  fs::create_directory(folder);
  const auto sink = folder / "opennav-startup-trace.log";
  const auto canary = folder / "untouched.txt";
  std::ofstream(canary) << "unrelated content must survive";
  if (scenario != "missing-marker")
    std::ofstream(folder / "OPENNAV_TEST_PROFILE") << "disposable test only";
  if (scenario == "symlink") fs::create_symlink(canary, sink);
  if (scenario == "directory") fs::create_directory(sink);
  if (scenario == "fifo") Check(::mkfifo(sink.c_str(), 0600) == 0, "create FIFO");
  if (scenario == "append") std::ofstream(sink) << "previous launch\n";
  const pid_t child = ::fork();
  Check(child >= 0, "fork child");
  if (child == 0) {
    try {
      setenv("OPENNAV_TEST_EARLY_STARTUP_TRACE", scenario == "disabled" ? "0" : "1", 1);
      if (scenario == "unset") unsetenv("OPENNAV_TEST_EARLY_STARTUP_TRACE");
      setenv("UNRELATED_SECRET_CANARY", "must-never-appear-in-trace", 1);
      const auto destination = scenario == "wrong-name" ? folder / "other.log" :
          scenario == "relative" ? fs::path("opennav-startup-trace.log") : sink;
      setenv("OPENNAV_TEST_EARLY_STARTUP_TRACE_FILE", destination.c_str(), 1);
      if (scenario == "relative") fs::current_path(folder);
#if XNAV_TEST_EARLY_STARTUP
      using namespace opennav::diagnostics;
      static_assert(noexcept(TestEarlyStartupTrace(StartupStage::Entry)));
      int calls = 0;
      errno = EALREADY;
      const int result = ObserveStartupCall(StartupStage::Entry,
          StartupStage::EntryReturn, [&] {
            Check(errno == EALREADY, "delegate receives unchanged errno");
            ++calls; errno = ERANGE; return -1;
          });
      Check(result == -1 && calls == 1 && errno == ERANGE,
            "delegate signed result, call count and errno preserved");
      const bool accepted = ObserveStartupCall(StartupStage::Initialize,
          StartupStage::InitializeReturn, [&] { ++calls; return false; });
      Check(!accepted && calls == 2, "boolean result preserved");
      bool caught = false;
      try {
        ObserveStartupCall(StartupStage::OnInit, StartupStage::ParserReturn,
            [&]() -> bool { ++calls; throw 17; });
      } catch (int value) { caught = value == 17; }
      Check(caught && calls == 3, "exception unchanged without retry");
      if (scenario == "bounded")
        for (int i = 0; i < 1000; ++i) TestEarlyStartupTrace(StartupStage::ParserParsed);
#else
      int evaluated = 0;
      XNAV_EARLY_STARTUP_TRACE(Entry, ++evaluated);
      Check(evaluated == 0, "disabled hook must not evaluate arguments");
#endif
      _exit(0);
    } catch (...) { _exit(1); }
  }
  int status = 0;
  Check(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
        "observational child succeeds");
  Check(Read(canary) == "unrelated content must survive", "canary unchanged");
#if XNAV_TEST_EARLY_STARTUP
  const bool allowed = scenario == "enabled" || scenario == "append" || scenario == "bounded";
  if (allowed) {
    auto contents = Read(sink);
    if (scenario == "append") {
      Check(contents.rfind("previous launch\n", 0) == 0, "append preserves prior launch");
      contents.erase(0, std::string("previous launch\n").size());
    }
    Check(contents.find("must-never-appear-in-trace") == std::string::npos,
          "unrelated environment never recorded");
    Check(contents.find(folder.string()) == std::string::npos, "paths never recorded");
    const std::regex record("OpenNav startup pid=" + std::to_string(child) +
        " stage=[a-z_.]+ result=-?[0-9]+\\n");
    const char* expected[] = {" stage=entry result=0\n", " stage=entry.return result=-1\n",
        " stage=initialize result=0\n", " stage=initialize.return result=0\n",
        " stage=on_init result=0\n"};
    std::size_t records = 0;
    while (!contents.empty()) {
      std::smatch match;
      Check(std::regex_search(contents, match, record) && match.position() == 0,
            "only fixed stage, PID and numeric result fields");
      if (records < 5)
        Check(match.str().find(expected[records]) != std::string::npos,
              "observed stage order and signed/boolean result truthful");
      contents.erase(0, match.length()); ++records;
    }
    Check(records == (scenario == "bounded" ? 64U : 5U), "bounded stage-only records");
  } else
#endif
  {
    if (scenario == "append") Check(Read(sink) == "previous launch\n", "disabled append unchanged");
    else if (scenario != "symlink" && scenario != "directory" && scenario != "fifo")
      Check(!fs::exists(sink), "refused sink not created");
    Check(!fs::exists(folder / "other.log"), "wrong filename not created");
  }
}

int main() {
  char pattern[] = "/tmp/opennav-startup-observation-XXXXXX";
  const char* created = ::mkdtemp(pattern);
  if (!created) return 1;
  const fs::path base(created);
  try {
    for (const char* scenario : {"unset", "disabled", "enabled", "append", "missing-marker",
         "wrong-name", "relative", "symlink", "directory", "fifo", "bounded"}) Run(base, scenario);
    fs::remove_all(base);
    std::cout << "11 startup-observation isolation/delegation scenarios passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    fs::remove_all(base); return 1;
  }
}
