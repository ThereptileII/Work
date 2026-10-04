#include <fstream>
#include <string>
#include <wx/filefn.h>
#include <wx/filename.h>
#include "TrustProbeConsole.h"

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  const std::string mode(argv[1]);
  if (mode == "original-log") {
    opennav::TrustProbeConsole::Stage("before-original-log");
    wxLogMessage("native-original-log");
    opennav::TrustProbeConsole::Stage("after-original-log");
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
