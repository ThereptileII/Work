#include "platform/windows/ProtectedRestartPolicy.h"
#include <iostream>
#include <stdexcept>
#include <utility>

using namespace opennav::platform;
namespace {
void Check(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}
void Tests() {
  for (const auto &entry : std::vector<std::wstring>{
           L"OPENNAV_COMMISSIONING_RESTART_SESSION=",
           L"opennav_commissioning_restart_record_sha256=hash",
           L"SKAGER_UPDATE_QUALIFY_PROFILE=1", L"SkAgEr_UpDaTe_PIPE=ignored",
           L"SKAGER_UPDATE_CHALLENGE="})
    Check(!ProtectedRestartEnvironmentEntryAllowed(entry),
          "Protected helper accepted fenced environment");
  for (const auto &entry :
       std::vector<std::wstring>{L"PATH=C:\\Windows", L"=C:=C:\\marine",
                                 L"USERNAME=SKAGER_UPDATE_PIPE",
                                 L"OPENNAV_OTHER=1", L"SKAGER_UPDATES=1"})
    Check(ProtectedRestartEnvironmentEntryAllowed(entry),
          "Ordinary environment entry changed");
  const std::wstring root = L"C:\\Program Files (x86)";
  const std::wstring generation(32, L'b');
  const std::wstring relative =
      L"\\SKAGER\\Applications\\S-1-5-21-1-2-3-1001\\generations\\" +
      generation + L"\\app\\opencpn.exe";
  const auto image = root + relative;
  for (const auto &tc : std::vector<std::pair<std::string, std::wstring>>{
           {"--xnav", L""},
           {"--legacy", L":legacy"},
           {"--safe-mode", L":safe"}}) {
    const auto route = ProtectedRestartPolicy(image, root, {tc.first});
    Check(route.kind == ProtectedRestartKind::BrokerCurrent,
          "Protected mode not routed");
    Check(route.generation == generation, "Sibling generation changed");
    Check(route.argument == L"--broker-current=" + generation + tc.second,
          "Protected mode produced non-closed arguments");
    // The native caller uses retained NT identities, not a drive/environment
    // spelling. The same routing policy must handle that native namespace.
    const std::wstring nt_root =
        L"\\Device\\HarddiskVolume7\\Program Files (x86)";
    const auto native =
        ProtectedRestartPolicy(nt_root + relative, nt_root, {tc.first});
    Check(native.argument == route.argument && native.kind == route.kind,
          "Native path changed routing");
  }
  for (const auto &args : std::vector<std::vector<std::string>>{
           {},
           {"--xnav", "--legacy"},
           {"--legacy", "--configdir", "C:\\profile"},
           {"--safe-mode", "--portable"},
           {"--xnav", "--no_opengl"},
           {"--fullscreen"},
           {"--broker-start=" + std::string(32, 'b')},
           {"--broker-current=" + std::string(32, 'b')},
           {"--safe"},
           {"--xnav --configdir=other"},
           {std::string("--xnav\0ignored", 14)}})
    Check(ProtectedRestartPolicy(image, root, args).kind ==
              ProtectedRestartKind::Refuse,
          "Protected profile/non-mode/extra arguments accepted");
  for (const auto &bad : std::vector<std::wstring>{
           root + L"\\SKAGER", root + L"\\SKAGER\\Applications",
           root + L"\\SKAGER\\other\\opencpn.exe", L"D:\\portable" + relative,
           root + L" other" + relative,
           root +
               L"\\SKAGER\\Applications\\S-1-5-21-01-2-3-1001\\generations\\" +
               generation + L"\\app\\opencpn.exe",
           root +
               L"\\SKAGER\\Applications\\S-1-5-21-4294967296-2-3-"
               L"1001\\generations\\" +
               generation + L"\\app\\opencpn.exe",
           root + L"\\SKAGER\\Applications\\S-1-5-21-1-2-3\\generations\\" +
               generation + L"\\app\\opencpn.exe",
           root +
               L"\\SKAGER\\Applications\\S-1-5-21-1-2-3-1001\\generations\\" +
               std::wstring(32, L'B') + L"\\app\\opencpn.exe",
           root +
               L"\\SKAGER\\Applications\\S-1-5-21-1-2-3-1001\\generations\\" +
               std::wstring(31, L'b') + L"\\app\\opencpn.exe",
           image + L":other", image + L"\\extra", root + relative + L".",
           root + L"\\SKAGER\\Applications\\..\\" + relative,
           root +
               L"\\SKAGER\\Applications\\S-1-5-21-1-2-3-1001\\generations\\" +
               generation + L"\\app \\opencpn.exe",
           root + L"/SKAGER/Applications/S-1-5-21-1-2-3-1001/generations/" +
               generation + L"/app/opencpn.exe"})
    Check(ProtectedRestartPolicy(bad, root, {"--xnav"}).kind ==
              ProtectedRestartKind::Refuse,
          "Malformed protected namespace fell back or routed");
  Check(ProtectedRestartPolicy(L"C:\\portable\\opencpn.exe", root,
                               {"--configdir", "C:\\profile", "--legacy"})
                .kind == ProtectedRestartKind::Ordinary,
        "Ordinary portable restart changed");
  Check(ProtectedRestartPolicy(L"C:\\Program Files (x86)\\OpenCPN\\opencpn.exe",
                               root, {"--portable", "--safe-mode"})
                .kind == ProtectedRestartKind::Ordinary,
        "Ordinary stock restart changed");
  Check(ProtectedRestartPolicy(
            L"c:\\program files "
            L"(x86)\\skager\\applications\\S-1-5-21-1-2-3-1001\\GENERATIONS\\" +
                generation + L"\\APP\\OPENCPN.EXE",
            root, {"--legacy"})
                .kind == ProtectedRestartKind::BrokerCurrent,
        "Windows fixed-name case changed routing");
}
} // namespace
int main() {
  try {
    Tests();
    std::cout << "Protected restart closed routing policy passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
