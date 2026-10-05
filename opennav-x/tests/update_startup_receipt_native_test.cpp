// Inert native fixture only. Links the exact production sender; the separate
// PowerShell harness owns the real authenticated receiver and kills this worker.
#include "integration/UpdateStartupReceipt.h"
#include "OpenNavBuild.h"

#include <windows.h>
#include <cstdlib>
#include <iostream>
#include <string>

int main() {
  using namespace opennav::integration;
  using namespace std::chrono_literals;
  const auto raw_mode = std::getenv("SKAGER_RECEIPT_FIXTURE_MODE");
  const std::string mode = raw_mode ? raw_mode : "";
  if (mode != "healthy" && mode != "no-marker" && mode != "interrupted" &&
      mode != "warning-agree" && mode != "warning-cancel" && mode != "warning-timeout" &&
      mode != "warning-no-checkpoint" && mode != "warning-fast") return 2;
  CaptureUpdateStartupReceipt();
  for (const auto name : {L"SKAGER_UPDATE_PIPE", L"SKAGER_UPDATE_GENERATION",
                          L"SKAGER_UPDATE_CHALLENGE"}) {
    wchar_t buffer[128];
    if (GetEnvironmentVariableW(name, buffer, 128) || _wgetenv(name)) {
      std::cerr << "FAIL update environment survived custody\n";
      return 3;
    }
  }
  std::cout << "PASS Win32 and CRT updater environment cleared; compiled commit "
            << OPENNAV_BUILD_COMMIT << '\n' << std::flush;
  const UpdateStartupReceiptState::Time now{100s};
  if (mode.rfind("warning-", 0) == 0) {
    NotifyUpdateNavigationWarning(true, false);
    // Only this inert fixture uses shortened receiver budgets. The real
    // sender must preserve WAIT ordering even with immediate acceptance.
    if (mode != "warning-fast") Sleep(1800);
    ObserveUpdateStartupHealth(true, now);
    NotifyUpdateStartupHealthy();
    ObserveUpdateStartupHealth(true, now + 31s);
    if (mode == "warning-timeout") { Sleep(10000); return 0; }
    NotifyUpdateNavigationWarning(false, mode != "warning-cancel");
    ObserveUpdateStartupHealth(true, now + 60s);
    if (mode != "warning-no-checkpoint") NotifyUpdateStartupHealthy();
    ObserveUpdateStartupHealth(true, now + 90s);
    Sleep(10000);
    return 0;
  }
  ObserveUpdateStartupHealth(true, now);
  if (mode == "interrupted") {
    ObserveUpdateStartupHealth(false, now + 29s);
    ObserveUpdateStartupHealth(true, now + 30s);
    NotifyUpdateStartupHealthy();
    ObserveUpdateStartupHealth(true, now + 59s);
  } else {
    if (mode != "no-marker") NotifyUpdateStartupHealthy();
    for (int repeat = 0; repeat < 8; ++repeat)
      ObserveUpdateStartupHealth(true, now + 31s);
  }
  // Real receiver checks this exact executable is still alive after EOF.
  Sleep(10000);
  return 0;
}
