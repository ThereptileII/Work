#pragma once
#include <wx/string.h>
class wxCmdLineParser;
namespace opennav::integration {
// Loader/resource check only. Never initializes a profile, plugin or transport.
void AddInstallerSelfTest(wxCmdLineParser &parser);
bool ParseInstallerSelfTest(wxCmdLineParser &parser);
bool InstallerSelfTestRequested();
int RunInstallerSelfTest();
} // namespace opennav::integration
