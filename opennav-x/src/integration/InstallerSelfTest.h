#pragma once
#include <wx/string.h>
class wxCmdLineParser;
namespace opennav::integration {
// Default loader/resource check never loads plugins or initializes a profile/transport.
// The explicit companion module check loads only the compiled private DLL,
// without calling a plugin factory, Init, or original-module fallback.
void AddInstallerSelfTest(wxCmdLineParser &parser);
bool ParseInstallerSelfTest(wxCmdLineParser &parser);
bool InstallerSelfTestRequested();
int RunInstallerSelfTest();
} // namespace opennav::integration
