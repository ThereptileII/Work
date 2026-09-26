; -*- coding: utf-8 -*-
Unicode true
!include "MUI2.nsh"
!include "nsDialogs.nsh"
!include "FileFunc.nsh"
!include "LogicLib.nsh"
!include "x64.nsh"
Name "OpenNav X Beta 2"
OutFile "${OUTPUT}"
RequestExecutionLevel user
SetCompressor /SOLID lzma
VIProductVersion "0.4.0.0"
VIAddVersionKey "ProductName" "OpenNav X Beta 2"
VIAddVersionKey "FileDescription" "Version-gated OpenNav X Beta setup"
VIAddVersionKey "FileVersion" "0.4.0-beta2"
VIAddVersionKey "LegalCopyright" "OpenNav X contributors; GPL"
Var StockPath
Var Action
Var ReportPath
Var FailurePoint
Var StockControl
Var ActionControl
Var Result
Var StatusControl
Var ExistingVersion
Var DetectedVersion
Var InstallRoot
Var RecoveryRoot
Var LegacyShortcut
Var SafeShortcut
Var LegacyControl
Var SafeControl
Var ShortcutModes
!define MUI_WELCOMEPAGE_TITLE "Install OpenNav X"
!define MUI_WELCOMEPAGE_TEXT "Modern navigation interface for OpenCPN.$\r$\n$\r$\nSetup checks your existing OpenCPN, creates a recovery record and installs OpenNav X beside it. XNav, Legacy and Safe Mode use your real charts and OpenCPN profile.$\r$\n$\r$\nClose OpenCPN before continuing. Beta 2 is for evaluation and is not approved for navigation."
!insertmacro MUI_PAGE_WELCOME
Page custom SourcePage SourceLeave
Page custom BackupPage
Page custom OptionsPage OptionsLeave
Page custom ReadyPage
!insertmacro MUI_PAGE_INSTFILES
!define MUI_FINISHPAGE_RUN
!define MUI_FINISHPAGE_RUN_TEXT "Launch OpenNav X Beta 2"
!define MUI_FINISHPAGE_RUN_FUNCTION LaunchXNav
!insertmacro MUI_PAGE_FINISH
UninstPage custom un.MaintenancePage un.MaintenanceLeave
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_UNPAGE_FINISH
!insertmacro MUI_LANGUAGE "English"

Function .onInit
  ${IfNot} ${RunningX64}
    MessageBox MB_ICONSTOP "Beta 2 targets on Windows x64 with the supported x86 OpenCPN ABI."
    Abort
  ${EndIf}
  SetShellVarContext current
  InitPluginsDir
  StrCpy $Action "Install"
  StrCpy $LegacyShortcut ${BST_CHECKED}
  StrCpy $SafeShortcut ${BST_CHECKED}
  StrCpy $ShortcutModes ""
  StrCpy $StockPath ""
  StrCpy $ReportPath ""
  StrCpy $FailurePoint ""
  ${GetParameters} $0
  ${GetOptions} $0 "/ACTION=" $Action
  ${If} ${Errors}
    StrCpy $Action "Install"
    ReadRegStr $ExistingVersion HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenNavXAlpha1" "DisplayVersion"
    ${If} $ExistingVersion != ""
      StrCpy $Action "Update"
    ${EndIf}
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
Function CheckSource
  Delete "$PLUGINSDIR\preflight.ini"
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$PLUGINSDIR\Lifecycle.ps1" -Action Preflight -OpenCpn "$StockPath" -PackageDirectory "$PLUGINSDIR" -ManifestSha256 "${MANIFEST_SHA256}" -SummaryPath "$PLUGINSDIR\preflight.ini"'
  Pop $Result
  ${If} $Result == 0
    ReadINIStr $StockPath "$PLUGINSDIR\preflight.ini" "Preflight" "Stock"
    ReadINIStr $DetectedVersion "$PLUGINSDIR\preflight.ini" "Preflight" "StockVersion"
    ReadINIStr $InstallRoot "$PLUGINSDIR\preflight.ini" "Preflight" "Root"
    ReadINIStr $RecoveryRoot "$PLUGINSDIR\preflight.ini" "Preflight" "Recovery"
    ReadINIStr $ExistingVersion "$PLUGINSDIR\preflight.ini" "Preflight" "Existing"
    ReadINIStr $LegacyShortcut "$PLUGINSDIR\preflight.ini" "Preflight" "LegacyShortcut"
    ReadINIStr $SafeShortcut "$PLUGINSDIR\preflight.ini" "Preflight" "SafeShortcut"
  ${EndIf}
FunctionEnd
Function SourcePage
  !insertmacro MUI_HEADER_TEXT "OpenCPN detected" "Confirm the OpenCPN installation you use aboard."
  Call CheckSource
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 28u "Select the original installed OpenCPN application. Setup verifies its exact version and files before making installation changes."
  Pop $0
  ${NSD_CreateText} 0 35u 78% 14u "$StockPath"
  Pop $StockControl
  ${NSD_CreateBrowseButton} 80% 34u 20% 16u "Browse..."
  Pop $0
  ${NSD_OnClick} $0 BrowseStock
  ${NSD_CreateLabel} 0 60u 100% 28u "Choose a supported OpenCPN installation to continue. No files have been changed."
  Pop $StatusControl
  ${If} $Result == 0
    ${NSD_SetText} $StatusControl "OpenCPN $DetectedVersion - Compatible$\r$\nYour existing charts and settings will be used."
  ${EndIf}
  ${NSD_CreateLabel} 0 96u 100% 12u "Installation action"
  Pop $0
  ${NSD_CreateDropList} 0 113u 100% 65u ""
  Pop $ActionControl
  ${NSD_CB_AddString} $ActionControl "Install"
  ${NSD_CB_AddString} $ActionControl "Update"
  ${NSD_CB_AddString} $ActionControl "Repair"
  ${NSD_CB_SelectString} $ActionControl "$Action"
  ${NSD_CreateLabel} 0 145u 100% 25u "OpenNav X installs for this Windows user. Your original OpenCPN application stays available."
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
  Call CheckSource
  ${If} $Result != 0
    MessageBox MB_ICONSTOP "This OpenCPN installation is not supported, or required files are unavailable. No installation or profile files have been changed. Select the original supported OpenCPN application and retry. See setup details."
    Abort
  ${EndIf}
FunctionEnd
Function BackupPage
  !insertmacro MUI_HEADER_TEXT "Your recovery backup" "Your navigation data stays in OpenCPN."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 55u "Before installing, OpenNav X records the original program identity and the current integration version. Existing OpenNav application files are retained for rollback.$\r$\n$\r$\nThe original OpenCPN program is not replaced."
  Pop $0
  ${NSD_CreateLabel} 0 65u 100% 45u "Your charts, routes, tracks, waypoints, connections and settings are preserved. Rollback changes application files only; it never restores older navigation data over your recent work."
  Pop $0
  ${NSD_CreateLabel} 0 118u 100% 32u "Recovery location:$\r$\n$RecoveryRoot"
  Pop $0
  ${NSD_CreateLabel} 0 158u 100% 18u "Keep your normal independent OpenCPN profile backup too."
  Pop $0
  nsDialogs::Show
FunctionEnd
Function OptionsPage
  !insertmacro MUI_HEADER_TEXT "Start menu shortcuts" "Choose convenient ways to open OpenNav X."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateCheckbox} 0 10u 100% 18u "OpenNav X"
  Pop $0
  ${NSD_Check} $0
  EnableWindow $0 0
  ${NSD_CreateCheckbox} 0 40u 100% 18u "Legacy OpenCPN"
  Pop $LegacyControl
  ${NSD_SetState} $LegacyControl $LegacyShortcut
  ${NSD_CreateCheckbox} 0 70u 100% 18u "OpenNav Safe Mode"
  Pop $SafeControl
  ${NSD_SetState} $SafeControl $SafeShortcut
  ${NSD_CreateLabel} 0 112u 100% 45u "Legacy and Safe Mode remain available from OpenNav X even if you omit their shortcuts. Maintenance is always available through Windows Installed Apps."
  Pop $0
  nsDialogs::Show
FunctionEnd
Function OptionsLeave
  ${NSD_GetState} $LegacyControl $LegacyShortcut
  ${NSD_GetState} $SafeControl $SafeShortcut
  StrCpy $ShortcutModes "xnav"
  ${If} $LegacyShortcut == ${BST_CHECKED}
    StrCpy $ShortcutModes "$ShortcutModes,legacy"
  ${EndIf}
  ${If} $SafeShortcut == ${BST_CHECKED}
    StrCpy $ShortcutModes "$ShortcutModes,safe"
  ${EndIf}
FunctionEnd
Function ReadyPage
  !insertmacro MUI_HEADER_TEXT "Ready to install" "Files and startup prerequisites will be checked automatically."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 42u "Action: $Action OpenNav X Beta 2$\r$\nOpenCPN: $DetectedVersion$\r$\n$StockPath"
  Pop $0
  ${NSD_CreateLabel} 0 55u 100% 32u "Install location:$\r$\n$InstallRoot"
  Pop $0
  ${NSD_CreateLabel} 0 100u 100% 32u "Recovery location:$\r$\n$RecoveryRoot"
  Pop $0
  ${NSD_CreateLabel} 0 145u 100% 30u "Setup verifies every OpenNav file, checks the executable loader and only then publishes the new version. Close all OpenCPN windows before continuing."
  Pop $0
  nsDialogs::Show
FunctionEnd
Section "OpenNav X Beta integration"
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$PLUGINSDIR\Lifecycle.ps1" -Action "$Action" -OpenCpn "$StockPath" -PackageDirectory "$PLUGINSDIR" -ManifestSha256 "${MANIFEST_SHA256}" -Report "$ReportPath" -FailurePoint "$FailurePoint" -ShortcutModes "$ShortcutModes"'
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
  !insertmacro MUI_HEADER_TEXT "Maintain OpenNav X Beta 2" "Original OpenCPN and navigation data are preserved."
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
