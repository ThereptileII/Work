; -*- coding: utf-8 -*-
Unicode true
!include "MUI2.nsh"
!include "nsDialogs.nsh"
!include "FileFunc.nsh"
!include "LogicLib.nsh"
!include "x64.nsh"
Name "OpenNav X Beta 1"
OutFile "${OUTPUT}"
RequestExecutionLevel user
SetCompressor /SOLID lzma
VIProductVersion "0.3.0.0"
VIAddVersionKey "ProductName" "OpenNav X Beta 1"
VIAddVersionKey "FileDescription" "Version-gated OpenNav X Beta setup"
VIAddVersionKey "FileVersion" "0.3.0-beta1"
VIAddVersionKey "LegalCopyright" "OpenNav X contributors; GPL"
Var StockPath
Var Action
Var ReportPath
Var FailurePoint
Var StockControl
Var ActionControl
Var Result
!define MUI_WELCOMEPAGE_TEXT "OpenNav X Beta 1 is for evaluation, not approved navigation.$\r$\n$\r$\nThis setup verifies the exact supported OpenCPN 5.12.4 executable, installs beside it for this user, and preserves its shared navigation profile. Close all OpenCPN modes first.$\r$\n$\r$\nThe original OpenCPN program remains unchanged. Back up your normal OpenCPN profile before testing Beta."
!insertmacro MUI_PAGE_WELCOME
Page custom SourcePage SourceLeave
!insertmacro MUI_PAGE_INSTFILES
!define MUI_FINISHPAGE_RUN
!define MUI_FINISHPAGE_RUN_TEXT "Launch OpenNav X Beta 1"
!define MUI_FINISHPAGE_RUN_FUNCTION LaunchXNav
!insertmacro MUI_PAGE_FINISH
UninstPage custom un.MaintenancePage un.MaintenanceLeave
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH
!insertmacro MUI_LANGUAGE "English"

Function .onInit
  ${IfNot} ${RunningX64}
    MessageBox MB_ICONSTOP "Beta 1 targets on Windows x64 with the supported x86 OpenCPN ABI."
    Abort
  ${EndIf}
  SetShellVarContext current
  InitPluginsDir
  StrCpy $Action "Install"
  StrCpy $StockPath ""
  StrCpy $ReportPath ""
  StrCpy $FailurePoint ""
  ${GetParameters} $0
  ${GetOptions} $0 "/ACTION=" $Action
  ${If} ${Errors}
    StrCpy $Action "Install"
  ${EndIf}
  ${GetOptions} $0 "/OPENCPN=" $StockPath
  ${GetOptions} $0 "/REPORT=" $ReportPath
  ${GetOptions} $0 "/FAILURE=" $FailurePoint
  SetOutPath "$PLUGINSDIR"
  File /oname=Lifecycle.ps1 "${ENGINE}"
  File /oname=package.json "${PACKAGE}\package.json"
  File /oname=payload.zip "${PACKAGE}\payload.zip"
  WriteUninstaller "$PLUGINSDIR\Maintain.exe"
FunctionEnd
Function SourcePage
  !insertmacro MUI_HEADER_TEXT "Verified OpenCPN installation" "Install, update or repair the side-by-side Beta integration."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 38u "Select the original installed OpenCPN 5.12.4 opencpn.exe. Leave blank to use registry discovery. Its SHA-256 must match the tested allowlist. Unknown builds are refused before installation changes."
  Pop $0
  ${NSD_CreateText} 0 45u 78% 14u "$StockPath"
  Pop $StockControl
  ${NSD_CreateBrowseButton} 80% 44u 20% 16u "Browse..."
  Pop $0
  ${NSD_OnClick} $0 BrowseStock
  ${NSD_CreateLabel} 0 73u 100% 12u "Action"
  Pop $0
  ${NSD_CreateDropList} 0 90u 100% 70u ""
  Pop $ActionControl
  ${NSD_CB_AddString} $ActionControl "Install"
  ${NSD_CB_AddString} $ActionControl "Update"
  ${NSD_CB_AddString} $ActionControl "Repair"
  ${NSD_CB_SelectString} $ActionControl "$Action"
  ${NSD_CreateLabel} 0 125u 100% 35u "No administrator rights are needed. All modes use the normal OpenCPN navigation profile. The portable package has a separate profile and remains a recovery/testing option."
  Pop $0
  nsDialogs::Show
FunctionEnd
Function BrowseStock
  nsDialogs::SelectFileDialog open "$StockPath" "OpenCPN executable|opencpn.exe"
  Pop $0
  ${If} $0 != ""
    StrCpy $StockPath $0
    ${NSD_SetText} $StockControl "$StockPath"
  ${EndIf}
FunctionEnd
Function SourceLeave
  ${NSD_GetText} $StockControl $StockPath
  ${NSD_GetText} $ActionControl $Action
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$PLUGINSDIR\Lifecycle.ps1" -Action Preflight -OpenCpn "$StockPath" -PackageDirectory "$PLUGINSDIR" -ManifestSha256 "${MANIFEST_SHA256}"'
  Pop $Result
  ${If} $Result != 0
    MessageBox MB_ICONSTOP "OpenCPN compatibility preflight failed. Check the selected executable and close OpenCPN. No installed program or profile files were changed. See setup details."
    Abort
  ${EndIf}
FunctionEnd
Section "OpenNav X Beta integration"
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$PLUGINSDIR\Lifecycle.ps1" -Action "$Action" -OpenCpn "$StockPath" -PackageDirectory "$PLUGINSDIR" -ManifestSha256 "${MANIFEST_SHA256}" -Report "$ReportPath" -FailurePoint "$FailurePoint"'
  Pop $Result
  ${If} $Result != 0
    SetErrorLevel 1
    MessageBox MB_ICONSTOP "OpenNav setup did not complete. Existing application generations and navigation data are preserved. Inspect setup details and %LOCALAPPDATA%\OpenNavXAlpha1\logs, then rerun Setup." /SD IDOK
    Abort
  ${EndIf}
  SetErrorLevel 0
SectionEnd
Function LaunchXNav
  ExecShell "open" "$SMPROGRAMS\OpenNav X Alpha 1\OpenNav X.lnk"
FunctionEnd
Function un.onInit
  SetShellVarContext current
  StrCpy $Action "Repair"
  StrCpy $ReportPath ""
  ${GetParameters} $0
  ${GetOptions} $0 "/ACTION=" $Action
  ${If} ${Errors}
    StrCpy $Action "Repair"
  ${EndIf}
  ${GetOptions} $0 "/REPORT=" $ReportPath
  ${If} $Action != "Uninstall"
  ${AndIf} $Action != "Rollback"
  ${AndIf} $Action != "Diagnostics"
  ${AndIf} $Action != "Repair"
    MessageBox MB_ICONSTOP "Use the original Setup download for Repair or Update."
    Abort
  ${EndIf}
FunctionEnd
Function un.MaintenancePage
  !insertmacro MUI_HEADER_TEXT "Maintain OpenNav X Beta 1" "Original OpenCPN and navigation data are preserved."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 45u "Repair restores OpenNav-owned files from the retained package. Rollback restores the prior application generation, or removes the first installation. Uninstall removes verified OpenNav-owned files and registration; modified/custom additions and diagnostics remain."
  Pop $0
  ${NSD_CreateDropList} 0 55u 100% 70u ""
  Pop $ActionControl
  ${NSD_CB_AddString} $ActionControl "Repair"
  ${NSD_CB_AddString} $ActionControl "Rollback"
  ${NSD_CB_AddString} $ActionControl "Uninstall"
  ${NSD_CB_AddString} $ActionControl "Diagnostics"
  ${NSD_CB_SelectString} $ActionControl "$Action"
  ${NSD_CreateLabel} 0 100u 100% 40u "Close all OpenCPN modes before repair, rollback or uninstall. Diagnostics writes a report in %LOCALAPPDATA%\OpenNavXAlpha1\logs. To update, download and run the newer Beta Setup."
  Pop $0
  GetDlgItem $0 $HWNDPARENT 1
  SendMessage $0 ${WM_SETTEXT} 0 "STR:Continue"
  nsDialogs::Show
FunctionEnd
Function un.MaintenanceLeave
  ${NSD_GetText} $ActionControl $Action
FunctionEnd
Section "Uninstall"
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$INSTDIR\Lifecycle.ps1" -Action "$Action" -Report "$ReportPath"'
  Pop $Result
  ${If} $Result != 0
    SetErrorLevel 1
    MessageBox MB_ICONSTOP "Maintenance failed. OpenCPN and retained recovery generations are preserved. Inspect the OpenNav installation logs." /SD IDOK
    Abort
  ${EndIf}
  SetErrorLevel 0
SectionEnd
