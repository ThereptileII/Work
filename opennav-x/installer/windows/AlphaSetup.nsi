; -*- coding: utf-8 -*-
Unicode true
!include "MUI2.nsh"
!include "nsDialogs.nsh"
!include "FileFunc.nsh"
!include "LogicLib.nsh"
!include "x64.nsh"
Name "SKAGER Beta 2"
UninstallCaption "SKAGER Maintenance"
OutFile "${OUTPUT}"
RequestExecutionLevel user
SetCompressor /SOLID lzma
VIProductVersion "${PRODUCT_VERSION_NUMERIC}"
VIAddVersionKey "ProductName" "SKAGER Beta 2"
VIAddVersionKey "FileDescription" "Version-gated SKAGER Beta setup"
VIAddVersionKey "FileVersion" "${PRODUCT_VERSION}"
VIAddVersionKey "ProductVersion" "${PRODUCT_VERSION}"
VIAddVersionKey "LegalCopyright" "OpenCPN and SKAGER contributors; GPL"
Var StockPath
Var Action
Var SupervisedArgument
Var ReportPath
Var FailurePoint
Var StockControl
Var BrowseControl
Var SourceDescriptionControl
Var FinishTitle
Var FinishText
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
!define MUI_ICON "${BRAND_ICON}"
!define MUI_UNICON "${BRAND_ICON}"
!define MUI_WELCOMEPAGE_TITLE "Install SKAGER"
!define MUI_WELCOMEPAGE_TEXT "Modern navigation interface for OpenCPN.$\r$\n$\r$\nSetup checks your existing OpenCPN, creates a recovery record and installs SKAGER beside it. SKAGER, Legacy and Safe Mode use your real charts and OpenCPN profile.$\r$\n$\r$\nClose OpenCPN before continuing. Beta 2 is for evaluation and is not approved for navigation."
!insertmacro MUI_PAGE_WELCOME
Page custom SourcePage SourceLeave
Page custom BackupPage
Page custom OptionsPage OptionsLeave
Page custom ReadyPage
!insertmacro MUI_PAGE_INSTFILES
!define MUI_FINISHPAGE_RUN
!define MUI_FINISHPAGE_TITLE "$FinishTitle"
!define MUI_FINISHPAGE_TEXT "$FinishText"
!define MUI_FINISHPAGE_RUN_TEXT "Open SKAGER"
!define MUI_FINISHPAGE_RUN_FUNCTION LaunchXNav
!insertmacro MUI_PAGE_FINISH
UninstPage custom un.MaintenancePage un.MaintenanceLeave
!define MUI_PAGE_HEADER_TEXT "Applying SKAGER maintenance"
!define MUI_PAGE_HEADER_SUBTEXT "Original OpenCPN and navigation data are preserved."
!insertmacro MUI_UNPAGE_INSTFILES
!define MUI_FINISHPAGE_TITLE "$FinishTitle"
!define MUI_FINISHPAGE_TEXT "$FinishText"
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
  StrCpy $FinishTitle "SKAGER setup complete"
  StrCpy $FinishText "SKAGER setup has completed. Your original OpenCPN and navigation data are preserved."
  StrCpy $LegacyShortcut ${BST_CHECKED}
  StrCpy $SafeShortcut ${BST_CHECKED}
  StrCpy $ShortcutModes ""
  StrCpy $StockPath ""
  StrCpy $ReportPath ""
  StrCpy $FailurePoint ""
  StrCpy $SupervisedArgument ""
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
  StrCpy $1 ""
  ${GetOptions} $0 "/SUPERVISED=" $1
  ${If} $1 == "1"
    StrCpy $SupervisedArgument "-SupervisedUpdate"
  ${EndIf}
  SetOutPath "$PLUGINSDIR"
  File /oname=Lifecycle.ps1 "${ENGINE}"
  File /oname=UpdateTransaction.ps1 "${UPDATE_TRANSACTION}"
  File /oname=UpdateSupervisor.ps1 "${UPDATE_SUPERVISOR}"
  File /oname=package.json "${PACKAGE}\package.json"
  File /oname=payload.zip "${PACKAGE}\payload.zip"
  WriteUninstaller "$PLUGINSDIR\Maintain.exe"
FunctionEnd
Function CheckSource
  ; A component action never enters legacy application Preflight. Presence here
  ; is UI availability only; the fixed helper establishes all native authority.
  ${If} $Action == "Update service components"
    StrCpy $Result 1
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" 0 +2
      StrCpy $Result 0
    Return
  ${EndIf}
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
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 28u "OpenCPN must be installed separately. Select its original application. Setup verifies the exact version and files before making changes."
  Pop $SourceDescriptionControl
  ${NSD_CreateText} 0 35u 78% 14u "$StockPath"
  Pop $StockControl
  ${NSD_CreateBrowseButton} 80% 34u 20% 16u "Browse..."
  Pop $BrowseControl
  ${NSD_OnClick} $BrowseControl BrowseStock
  ${NSD_CreateLabel} 0 60u 100% 28u "Choose a supported OpenCPN installation to continue. No files have been changed."
  Pop $StatusControl
  ${NSD_CreateLabel} 0 96u 100% 12u "Installation action"
  Pop $0
  ${NSD_CreateDropList} 0 113u 100% 65u ""
  Pop $ActionControl
  ${NSD_CB_AddString} $ActionControl "Install"
  ${NSD_CB_AddString} $ActionControl "Update"
  ${NSD_CB_AddString} $ActionControl "Repair"
  ${NSD_CB_AddString} $ActionControl "Update service components"
  ${NSD_CB_SelectString} $ActionControl "$Action"
  ${NSD_OnChange} $ActionControl SourceActionChanged
  ${NSD_CreateLabel} 0 145u 100% 25u "Built on OpenCPN, with thanks to its developers. SKAGER installs for this Windows user; original OpenCPN stays available."
  Pop $0
  Call UpdateSourceAction
  nsDialogs::Show
FunctionEnd
Function SourceActionChanged
  Pop $0
  Call UpdateSourceAction
FunctionEnd
Function UpdateSourceAction
  ${NSD_GetText} $StockControl $StockPath
  ${NSD_GetText} $ActionControl $Action
  Call CheckSource
  ${If} $Action == "Update service components"
    !insertmacro MUI_HEADER_TEXT "Update service components" "Application updates are offered when you open SKAGER normally."
    ${NSD_SetText} $SourceDescriptionControl "Check the installed update service for a verified component update. Application updates are offered separately when you open SKAGER normally."
    EnableWindow $StockControl 0
    EnableWindow $BrowseControl 0
    ${If} $Result == 0
      ${NSD_SetText} $StatusControl "Installed service maintenance is available. Verification happens when you continue."
    ${Else}
      ${NSD_SetText} $StatusControl "The installed service helper is unavailable. Service component maintenance cannot continue."
    ${EndIf}
  ${Else}
    !insertmacro MUI_HEADER_TEXT "OpenCPN detected" "Confirm the OpenCPN installation you use aboard."
    ${NSD_SetText} $SourceDescriptionControl "OpenCPN must be installed separately. Select its original application. Setup verifies the exact version and files before making changes."
    EnableWindow $StockControl 1
    EnableWindow $BrowseControl 1
    ${NSD_SetText} $StockControl "$StockPath"
    ${If} $Result == 0
      ${NSD_SetText} $StatusControl "OpenCPN $DetectedVersion - Compatible$\r$\nYour existing charts and settings will be used."
    ${Else}
      ${NSD_SetText} $StatusControl "Application setup is unavailable. Protected installations use normal SKAGER startup for application updates, or Update service components for the service."
    ${EndIf}
  ${EndIf}
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
  ${If} $Action == "Update service components"
    ${If} $Result != 0
      MessageBox MB_ICONSTOP "The installed service helper is unavailable. Service component maintenance cannot continue. Preserve the installation for recovery."
      Abort
    ${EndIf}
    StrCpy $FinishTitle "Service components checked"
    StrCpy $FinishText "The service-component check completed. Open SKAGER normally to check for an application update. Your original OpenCPN and navigation data are preserved."
    Return
  ${EndIf}
  StrCpy $FinishTitle "SKAGER setup complete"
  StrCpy $FinishText "SKAGER setup has completed. Your original OpenCPN and navigation data are preserved."
  ${If} $Result != 0
    MessageBox MB_ICONSTOP "This OpenCPN installation is not supported, or required files are unavailable. No installation or profile files have been changed. For a protected service installation, open SKAGER normally for application updates or choose Update service components. Otherwise select the original supported OpenCPN application and retry. See setup details."
    Abort
  ${EndIf}
FunctionEnd
Function BackupPage
  ${If} $Action == "Update service components"
    Abort
  ${EndIf}
  !insertmacro MUI_HEADER_TEXT "Your recovery backup" "Your navigation data stays in OpenCPN."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 55u "Before installing, SKAGER records the original program identity and the current integration version. Existing application files are retained for rollback.$\r$\n$\r$\nThe original OpenCPN program is not replaced."
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
  ${If} $Action == "Update service components"
    Abort
  ${EndIf}
  !insertmacro MUI_HEADER_TEXT "Start menu shortcuts" "Choose convenient ways to open SKAGER."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateCheckbox} 0 10u 100% 18u "SKAGER"
  Pop $0
  ${NSD_Check} $0
  EnableWindow $0 0
  ${NSD_CreateCheckbox} 0 40u 100% 18u "Legacy OpenCPN"
  Pop $LegacyControl
  ${NSD_SetState} $LegacyControl $LegacyShortcut
  ${NSD_CreateCheckbox} 0 70u 100% 18u "SKAGER Safe Mode"
  Pop $SafeControl
  ${NSD_SetState} $SafeControl $SafeShortcut
  ${NSD_CreateLabel} 0 112u 100% 45u "Legacy and Safe Mode remain available from SKAGER even if you omit their shortcuts. Maintenance is always available through Windows Installed Apps."
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
  ${If} $Action == "Update service components"
    !insertmacro MUI_HEADER_TEXT "Ready to check service components" "Application versions and navigation data are preserved."
    nsDialogs::Create 1018
    Pop $0
    ${NSD_CreateLabel} 0 0 100% 45u "The installed update service will check for a verified component update and apply it if available. Windows may ask you to approve the change for this Windows account."
    Pop $0
    ${NSD_CreateLabel} 0 60u 100% 45u "Application versions, original OpenCPN and navigation data are preserved. Open SKAGER normally to check for an application update."
    Pop $0
    nsDialogs::Show
    Return
  ${EndIf}
  !insertmacro MUI_HEADER_TEXT "Ready to install" "Files and startup prerequisites will be checked automatically."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 42u "Action: $Action SKAGER Beta 2$\r$\nOpenCPN: $DetectedVersion$\r$\n$StockPath"
  Pop $0
  ${NSD_CreateLabel} 0 55u 100% 32u "Install location:$\r$\n$InstallRoot"
  Pop $0
  ${NSD_CreateLabel} 0 100u 100% 32u "Recovery location:$\r$\n$RecoveryRoot"
  Pop $0
  ${NSD_CreateLabel} 0 145u 100% 30u "Setup verifies every application file, checks the executable loader and only then publishes the new version. Close all OpenCPN windows before continuing."
  Pop $0
  nsDialogs::Show
FunctionEnd
Section "SKAGER Beta integration"
  ${If} $Action == "Update service components"
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" 0 setup_components_missing
    SetOutPath "$PROGRAMFILES32\SKAGER\UpdateBroker"
    nsExec::ExecToLog '"$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" --maintenance=code-upgrade-current'
    Pop $Result
    Goto setup_result
setup_components_missing:
    StrCpy $Result 1
    Goto setup_result
  ${EndIf}
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$PLUGINSDIR\Lifecycle.ps1" -Action "$Action" -OpenCpn "$StockPath" -PackageDirectory "$PLUGINSDIR" -ManifestSha256 "${MANIFEST_SHA256}" -Report "$ReportPath" -FailurePoint "$FailurePoint" -ShortcutModes "$ShortcutModes" $SupervisedArgument'
  Pop $Result
setup_result:
  ${If} $Result != 0
    SetErrorLevel 1
    ${If} $Action == "Update service components"
      MessageBox MB_ICONSTOP "The service-component check did not complete. Preserve the installation and recovery evidence; no application update was performed by Setup." /SD IDOK
      Abort
    ${EndIf}
    MessageBox MB_ICONSTOP "SKAGER setup did not complete. Existing application generations and navigation data are preserved. Inspect the setup details, then run Setup again. If SKAGER is already installed, choose Diagnostics in SKAGER Maintenance to collect a report." /SD IDOK
    Abort
  ${EndIf}
  SetErrorLevel 0
SectionEnd
Function LaunchXNav
  ExecShell "open" "$SMPROGRAMS\SKAGER\Skager.lnk"
FunctionEnd
Function un.onInit
  SetShellVarContext current
  StrCpy $Action "Repair"
  StrCpy $FinishTitle "SKAGER maintenance complete"
  StrCpy $FinishText "The selected maintenance action has completed. Your original OpenCPN and navigation data are preserved."
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
  ${AndIf} $Action != "Update service components"
    MessageBox MB_ICONSTOP "Use the original Setup download for Repair or Update."
    Abort
  ${EndIf}
FunctionEnd
Function un.MaintenancePage
  !insertmacro MUI_HEADER_TEXT "Maintain SKAGER" "Original OpenCPN and navigation data are preserved."
  nsDialogs::Create 1018
  Pop $0
  ${NSD_CreateLabel} 0 0 100% 45u "Repair restores owned files from the retained package. Rollback returns to a prior application release while preserving current navigation data. Uninstall removes verified owned files and registration; modified/custom additions and diagnostics remain."
  Pop $0
  ${NSD_CreateDropList} 0 55u 100% 70u ""
  Pop $ActionControl
  ${NSD_CB_AddString} $ActionControl "Repair"
  ${NSD_CB_AddString} $ActionControl "Rollback"
  ${NSD_CB_AddString} $ActionControl "Uninstall"
  ${NSD_CB_AddString} $ActionControl "Diagnostics"
  ${NSD_CB_AddString} $ActionControl "Update service components"
  ${NSD_CB_SelectString} $ActionControl "$Action"
  ${NSD_CreateLabel} 0 100u 100% 65u "Close all OpenCPN modes before repair, rollback or uninstall. Choose Diagnostics to save a SKAGER installation report. Choose Update service components to check the installed update service. For protected application updates, open SKAGER normally; other installations use the newer Setup."
  Pop $0
  GetDlgItem $0 $HWNDPARENT 1
  SendMessage $0 ${WM_SETTEXT} 0 "STR:Continue"
  nsDialogs::Show
FunctionEnd
Function un.MaintenanceLeave
  ${NSD_GetText} $ActionControl $Action
  ${If} $Action == "Update service components"
    StrCpy $FinishTitle "Service components checked"
    StrCpy $FinishText "The service-component check completed. Open SKAGER normally to check for an application update. Your original OpenCPN and navigation data are preserved."
  ${Else}
    StrCpy $FinishTitle "SKAGER maintenance complete"
    StrCpy $FinishText "The selected maintenance action has completed. Your original OpenCPN and navigation data are preserved."
  ${EndIf}
FunctionEnd
Section "Uninstall"
  ; The machine-protected helper owns its own same-account elevation and native
  ; maintenance conversation. This user-level wizard supplies only a fixed action,
  ; never an elevated script, approval hash, SID, package or cleanup destination.
  ${If} $Action == "Update service components"
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" 0 protected_components_missing
    SetOutPath "$PROGRAMFILES32\SKAGER\UpdateBroker"
    nsExec::ExecToLog '"$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" --maintenance=code-upgrade-current'
    Pop $Result
    Goto maintenance_result
protected_components_missing:
    StrCpy $Result 1
    Goto maintenance_result
  ${EndIf}
  ${If} $Action == "Repair"
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" protected_repair
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\*" protected_uninstall_missing legacy_maintenance
protected_repair:
    SetOutPath "$PROGRAMFILES32\SKAGER\UpdateBroker"
    nsExec::ExecToLog '"$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" --maintenance=repair-current'
    Pop $Result
    Goto maintenance_result
  ${EndIf}
  ${If} $Action == "Rollback"
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" protected_rollback
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\*" protected_uninstall_missing legacy_maintenance
protected_rollback:
    SetOutPath "$PROGRAMFILES32\SKAGER\UpdateBroker"
    nsExec::ExecToLog '"$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" --maintenance=rollback-current'
    Pop $Result
    Goto maintenance_result
  ${EndIf}
  ${If} $Action == "Uninstall"
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" protected_uninstall
    IfFileExists "$PROGRAMFILES32\SKAGER\UpdateBroker\*" protected_uninstall_missing legacy_maintenance
protected_uninstall:
    SetOutPath "$PROGRAMFILES32\SKAGER\UpdateBroker"
    nsExec::ExecToLog '"$PROGRAMFILES32\SKAGER\UpdateBroker\skager-update-provisioner.exe" --maintenance=uninstall-current'
    Pop $Result
    Goto maintenance_result
protected_uninstall_missing:
    StrCpy $Result 1
    Goto maintenance_result
  ${EndIf}
legacy_maintenance:
  nsExec::ExecToLog '"$SYSDIR\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -NonInteractive -ExecutionPolicy Bypass -File "$INSTDIR\Lifecycle.ps1" -Action "$Action" -Report "$ReportPath"'
  Pop $Result
maintenance_result:
  ${If} $Result != 0
    SetErrorLevel 1
    ${If} $Action == "Update service components"
      MessageBox MB_ICONSTOP "The service-component check did not complete. Preserve the installation and recovery evidence; no application update was performed by Maintenance." /SD IDOK
      Abort
    ${EndIf}
    MessageBox MB_ICONSTOP "Maintenance failed. OpenCPN and retained recovery generations are preserved. Inspect the SKAGER installation logs." /SD IDOK
    Abort
  ${EndIf}
  SetErrorLevel 0
SectionEnd
