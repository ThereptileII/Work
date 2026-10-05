#include <fstream>
#include <string>
#include <vector>
#include <wx/filefn.h>
#include <wx/filename.h>
#include "TrustProbeConsole.h"

namespace {
int destroyed_startup_logs = 0;
class ObservedStartupLog : public wxLogStderr {
 public:
  ~ObservedStartupLog() override {
    ++destroyed_startup_logs;
    opennav::TrustProbeConsole::Stage("startup-log-destroyed-by-wx");
  }
};
}

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const std::string mode(argv[1]);
  if (mode == "original-log") {
    opennav::TrustProbeConsole::Stage("before-original-log");
    wxLogMessage("native-original-log");
    opennav::TrustProbeConsole::Stage("after-original-log");
    return 0;
  }
  if (mode == "startup-log-ownership") {
    auto owner = std::make_unique<ObservedStartupLog>();
    auto* previous = wxLog::SetActiveTarget(owner.get());
    if (previous != nullptr) {
      wxLog::SetActiveTarget(previous);
      return 8;
    }
    opennav::TrustProbeConsole::Stage("before-wx-initialization");
    wxInitializer initializer;
    opennav::TrustProbeConsole::Stage("after-wx-initialization");
    if (!initializer.IsOk()) {
      if (destroyed_startup_logs) owner.release();
      else wxLog::SetActiveTarget(nullptr);
      return 3;
    }
    // wx3.2.8 deletes the target during initialization. This directly observes
    // the former double ownership without dereferencing or deleting freed data.
    if (destroyed_startup_logs != 1) {
      wxLog::SetActiveTarget(nullptr);
      return 9;
    }
    owner.release();
    opennav::TrustProbeConsole::Stage("former-owner-would-delete-again");
    // Deterministically reject the old ownership invariant rather than invoke
    // undefined behavior by deleting the observed dangling pointer again.
    return 87;
  }
  if (mode == "fixed-lifecycle") {
    for (int iteration = 0; iteration != 16; ++iteration) {
      {
        opennav::TrustProbeConsole console;
        if (!console.IsOk()) return 3;
        auto* active = wxLog::GetActiveTarget();
        std::vector<std::string> allocations(256, std::string(256, 'x'));
        wxLogMessage("native-fixed-lifecycle %d", iteration);
        if (wxLog::GetActiveTarget() != active) return 10;
      }
      opennav::TrustProbeConsole::Stage("fixed-scope-destroyed");
    }
    return 0;
  }
  if (mode != "fixed" && mode != "fixed-assert") return 2;
  opennav::TrustProbeConsole console;
  if (!console.IsOk()) return 3;
  opennav::TrustProbeConsole::Stage("initialized");
  if (mode == "fixed-assert") {
    opennav::TrustProbeConsole::Stage("before-assert");
    wxASSERT_MSG(false, "native-console-assert");
    opennav::TrustProbeConsole::Stage("after-assert");
    return 4;
  }
  wxLogMessage("native-fixed-message");
  wxLogWarning("native-fixed-warning");
  opennav::TrustProbeConsole::Stage("after-fixed-log");
  const wxString destination = wxString::FromUTF8(argv[2]);
  const auto prefix = wxFileName(destination).GetPathWithSep() + ".ocpn-download-";
  const auto partial = wxFileName::CreateTempFileName(prefix);
  if (partial.empty()) return 5;
  {
    std::ofstream stream(partial.ToStdString(), std::ios::binary | std::ios::trunc);
    stream << "native console staging payload\n";
    stream.close();
    if (!stream.good()) return 6;
  }
  opennav::TrustProbeConsole::Stage("staging-written");
  if (!wxRenameFile(partial, destination, true)) return 7;
  opennav::TrustProbeConsole::Stage("rename-complete");
  return 0;
}
