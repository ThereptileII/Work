package main

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func installedCodeSetupSource(t *testing.T) string {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join("..", "..", "..", "..", "installer", "windows", "AlphaSetup.nsi"))
	if err != nil {
		t.Fatal(err)
	}
	return string(raw)
}
func installedCodeSetupBlock(t *testing.T, s, start, end string) string {
	t.Helper()
	a := strings.Index(s, start)
	if a < 0 {
		t.Fatal("missing installer boundary", start)
	}
	b := strings.Index(s[a+len(start):], end)
	if b < 0 {
		t.Fatal("missing installer boundary", end)
	}
	return s[a : a+len(start)+b]
}

// NSIS is the user-level routing shell. These source-contract checks guard
// fixed action/authority boundaries; they do not claim native wizard acceptance.
func TestInstalledCodeSetupComponentRoutesNeverReachLegacyMutation(t *testing.T) {
	s := installedCodeSetupSource(t)
	for _, route := range []struct{ name, section, result, missing string }{
		{"setup", `Section "SKAGER Beta integration"`, "setup_result:", "setup_components_missing:"},
		{"maintain", `Section "Uninstall"`, "maintenance_result:", "protected_components_missing:"},
	} {
		t.Run(route.name, func(t *testing.T) {
			section := installedCodeSetupBlock(t, s, route.section, "SectionEnd")
			component := installedCodeSetupBlock(t, section, `${If} $Action == "Update service components"`, `${EndIf}`)
			fixed := `nsExec::ExecToLog '"$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" --maintenance=code-upgrade-current'`
			if strings.Count(component, fixed) != 1 || !strings.Contains(component, `SetOutPath "$PROGRAMFILES32\SKAGER\UpdateBroker"`) {
				t.Fatal("component action lost fixed installed helper")
			}
			for _, foreign := range []string{"Lifecycle.ps1", "-PackageDirectory", "--trust-config", "--client-sid", "--provisioner-sha256", "legacy_maintenance"} {
				if strings.Contains(component, foreign) {
					t.Fatal("component route acquired caller authority or legacy fallback", foreign)
				}
			}
			destination := strings.TrimSuffix(route.result, ":")
			if strings.Count(component, "Goto "+destination) != 2 {
				t.Fatal("success and missing-helper paths must both terminate at dedicated result")
			}
			missing := installedCodeSetupBlock(t, component, route.missing, "Goto "+destination)
			if !strings.Contains(missing, "StrCpy $Result 1") {
				t.Fatal("missing/partial helper became success")
			}
			result := section[strings.Index(section, route.result):]
			fail := installedCodeSetupBlock(t, result, `${If} $Action == "Update service components"`, `${EndIf}`)
			if !strings.Contains(fail, "did not complete") || !strings.Contains(fail, "Abort") {
				t.Fatal("component failure reached completion")
			}
		})
	}
}
func TestInstalledCodeSetupDedicatedPreflightAndPageState(t *testing.T) {
	s := installedCodeSetupSource(t)
	check := installedCodeSetupBlock(t, s, "Function CheckSource", "FunctionEnd")
	branch := installedCodeSetupBlock(t, check, `${If} $Action == "Update service components"`, `${EndIf}`)
	if !strings.Contains(branch, "StrCpy $Result 1") || !strings.Contains(branch, "Return") || strings.Contains(branch, "nsExec") || strings.Contains(branch, "Lifecycle") {
		t.Fatal("component availability entered application preflight or executed maintenance early")
	}
	if !strings.Contains(check[len(branch):], "-Action Preflight") {
		t.Fatal("ordinary application preflight removed")
	}
	source := installedCodeSetupBlock(t, s, "Function UpdateSourceAction", "FunctionEnd")
	if !strings.Contains(source, `EnableWindow $StockControl 0`) || !strings.Contains(source, `EnableWindow $BrowseControl 0`) || !strings.Contains(source, `EnableWindow $StockControl 1`) || !strings.Contains(source, "normal SKAGER startup for application updates") || !strings.Contains(source, `${NSD_SetText} $StockControl "$StockPath"`) {
		t.Fatal("source action state or protected application guidance lost")
	}
	for _, page := range []string{"BackupPage", "OptionsPage"} {
		body := installedCodeSetupBlock(t, s, "Function "+page, "FunctionEnd")
		skip := installedCodeSetupBlock(t, body, `${If} $Action == "Update service components"`, `${EndIf}`)
		if !strings.Contains(skip, "Abort") {
			t.Fatal("component action showed unrelated installation page", page)
		}
	}
	ready := installedCodeSetupBlock(t, s, "Function ReadyPage", "FunctionEnd")
	component := installedCodeSetupBlock(t, ready, `${If} $Action == "Update service components"`, `${EndIf}`)
	if !strings.Contains(component, "Ready to check service components") || !strings.Contains(component, "Return") || strings.Contains(component, "publishes the new version") {
		t.Fatal("component Ready page claimed application installation")
	}
}
func TestInstalledCodeSetupCompletionIsDistinctAndOrdinaryGuardRemains(t *testing.T) {
	s := installedCodeSetupSource(t)
	if strings.Count(s, `${NSD_CB_AddString} $ActionControl "Update service components"`) != 2 {
		t.Fatal("dedicated action unavailable in Setup or Maintain")
	}
	if !strings.Contains(s, `${AndIf} $Action != "Update service components"`) {
		t.Fatal("Maintain action not admitted")
	}
	for _, fn := range []string{"SourceLeave", "un.MaintenanceLeave"} {
		body := installedCodeSetupBlock(t, s, "Function "+fn, "FunctionEnd")
		if !strings.Contains(body, `StrCpy $FinishTitle "Service components checked"`) || !strings.Contains(body, "Open SKAGER normally to check for an application update") {
			t.Fatal("finish state falsely conflates service and application updates", fn)
		}
	}
	if strings.Count(s, `!define MUI_FINISHPAGE_TITLE "$FinishTitle"`) != 2 || strings.Count(s, `!define MUI_FINISHPAGE_TEXT "$FinishText"`) != 2 {
		t.Fatal("dynamic finish copy not used by both wizards")
	}
	if !strings.Contains(s, `ExecShell "open" "$SMPROGRAMS\SKAGER\Skager.lnk"`) {
		t.Fatal("normal startup shortcut changed")
	}
	raw, err := os.ReadFile(filepath.Join("..", "..", "..", "..", "installer", "windows", "Lifecycle.ps1"))
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(raw), `if ($Action -ne 'Diagnostics') { Assert-NoProtectedBrokerEnrollment }`) {
		t.Fatal("legacy protected enrollment guard bypassed")
	}
}
