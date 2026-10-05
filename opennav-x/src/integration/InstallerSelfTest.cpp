#include "integration/InstallerSelfTest.h"
#include "integration/ChartModuleCheck.h"
#include "integration/BuildFeatures.h"
#include "OpenNavBuild.h"
#include "application/Version.h"
#include <filesystem>
#include <wx/cmdline.h>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/jsonwriter.h>
#include <wx/stdpaths.h>
#ifdef __WXMSW__
#include "platform/windows/CommissioningRestartNative.h"
#endif
namespace opennav::integration {
namespace {
wxString report_path;
bool requested = false, module_requested = false, invalid_module_option = false;
wxString original_module;
} // namespace
void AddInstallerSelfTest(wxCmdLineParser &parser) {
  parser.AddOption(
      "", "opennav-self-test",
      "Read-only loader/resource check; new absolute JSON report path");
  parser.AddOption("", "skager-chart-module-check",
      "With --opennav-self-test only: exact original DLL identity input; private module load/bind check");
}
bool ParseInstallerSelfTest(wxCmdLineParser &parser) {
  requested = parser.Found("opennav-self-test", &report_path);
  module_requested = parser.Found("skager-chart-module-check", &original_module);
  invalid_module_option = module_requested && !requested;
  // A misplaced diagnostic option must not fall through to normal startup.
  requested = requested || module_requested;
  return requested;
}
bool InstallerSelfTestRequested() { return requested; }
int RunInstallerSelfTest() {
  if (!requested || invalid_module_option)
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
  report["build_purpose"] = wxString::FromUTF8(BuildPurpose().data());
  report["test_fixtures"] = TestFixturesEnabled();
  report["xnav_hardware_output_policy"] = wxString::FromUTF8(HardwareOutputPolicy().data());
  report["upstream"] = wxString("37fd0cddb7334fe489e9f18aa163977a9c5c84f7");
  report["compiler"] = wxString(OPENNAV_BUILD_COMPILER);
  report["profile_initialized"] = false;
  report["normal_config_directory"] = wxStandardPaths::Get().GetConfigDir();
  report["plugins_loaded"] = false;
#ifdef __WXMSW__
  report["commissioning_restart_protocol"] =
      platform::commissioning::ProtocolCapability();
#else
  report["commissioning_restart_protocol"] = 0;
#endif
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
  if (module_requested) {
    report["plugin_loading_scope"]=wxString("private DLL module only; no plugin factory or Init");
    const bool checked=valid && CheckChartModule(original_module,directory,report["chart_module"]);
    valid=valid && checked;
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
