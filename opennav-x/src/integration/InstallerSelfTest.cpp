#include "integration/InstallerSelfTest.h"
#include "OpenNavBuild.h"
#include "application/Version.h"
#include <filesystem>
#include <wx/cmdline.h>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/jsonwriter.h>
#include <wx/stdpaths.h>
namespace opennav::integration {
namespace {
wxString report_path;
bool requested = false;
} // namespace
void AddInstallerSelfTest(wxCmdLineParser &parser) {
  parser.AddOption(
      "", "opennav-self-test",
      "Read-only loader/resource check; new absolute JSON report path");
}
bool ParseInstallerSelfTest(wxCmdLineParser &parser) {
  requested = parser.Found("opennav-self-test", &report_path);
  return requested;
}
bool InstallerSelfTestRequested() { return requested; }
int RunInstallerSelfTest() {
  if (!requested)
    return 2;
  // Refuse overwrites and relative paths; the installer supplies a new staged
  // report.
  if (report_path.empty() || !wxFileName(report_path).IsAbsolute() ||
      wxFileName::Exists(report_path))
    return 2;
  const auto exe = wxStandardPaths::Get().GetExecutablePath();
  const auto directory = wxFileName(exe).GetPath();
#ifdef __WXMSW__
  const auto resources = directory;
#else
  const auto resources = directory + "/../share/opencpn";
#endif
  wxJSONValue report;
  report["contract"] = wxString("OpenNavX.LoaderSelfTest.1");
  report["commit"] = wxString(OPENNAV_BUILD_COMMIT);
  report["version"] = wxString::FromUTF8(application::Version);
  report["upstream"] = wxString("37fd0cddb7334fe489e9f18aa163977a9c5c84f7");
  report["compiler"] = wxString(OPENNAV_BUILD_COMPILER);
  report["profile_initialized"] = false;
  report["normal_config_directory"] = wxStandardPaths::Get().GetConfigDir();
  report["plugins_loaded"] = false;
  bool valid = true;
  for (const auto *relative :
       {"s57data/chartsymbols.xml", "s57data/s57objectclasses.csv",
        "uidata/styles.xml", "gshhs/poly-c-1.dat"}) {
    wxFileName file(resources + "/" + relative);
    const bool ok = file.FileExists() && file.GetSize() != wxInvalidSize &&
                    file.GetSize().GetValue() > 0;
    report["resources"][relative] = ok;
    valid = valid && ok;
  }
  report["passed"] = valid;
  wxString serialized;
  wxJSONWriter().Write(report, serialized);
  wxFile file;
  if (!file.Create(report_path, false) || !file.Write(serialized, wxConvUTF8) ||
      !file.Flush() || !file.Close())
    return 2;
  return valid ? 0 : 1;
}
} // namespace opennav::integration
