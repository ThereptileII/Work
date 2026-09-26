# Guarded official upgrade wizard

This maintenance driver implements only the separately authorized boat
prerequisite upgrade documented in [stock-upgrade-5.12.2.md](stock-upgrade-5.12.2.md).
It cannot install OpenNav, authorize a different stock release, start OpenCPN,
repair the existing profile, or qualify Beta 2.

Run `tools/boat/upgrade-official-opencpn.ps1` in a fresh **64-bit Windows
PowerShell 5.1**, as the already logged-in user in an elevated interactive task.
The task must have an unlocked interactive desktop. Keep all other installers
and OpenCPN processes closed. Its dependencies are `Common.ps1`,
`OfficialUpgradePolicy.ps1` and `OfficialWizardNative.cs` in the same directory.
Python is not required.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File C:\XNav\scripts\upgrade-official-opencpn.ps1 `
  -Record <fresh-prepared.json> -ExpectedRecordSha256 <exact-prepared-record-hash> `
  -Output <new-private-evidence-directory>\wizard.json
```

Preparation must be less than one hour old. The destination is fixed to
`C:\Program Files (x86)\OpenCPN`; the source profile is fixed to
`C:\ProgramData\opencpn`. The driver rechecks the preparation/recovery records,
exact installer and original executable hashes, original uninstaller hash and
all original uninstall-registration values and types before launch and again
before Install. The installer file is held open without write/delete sharing
during execution. The exact source, setup and target hashes are fixed in code,
not supplied by configuration.

The observed Registry32 bare `OpenCPN` entry belongs to `OpenCPN rtlsdr_pi`
1.3.1, published by `opencpn.org`, with no installation location. The pinned
[`Check_Prev_Installs_Macro`](https://github.com/OpenCPN/OpenCPN/blob/37fd0cddb7334fe489e9f18aa163977a9c5c84f7/NSIS.template.in.in#L3681)
only counts OpenCPN keys longer than seven characters with a nonempty
`CompareVersion`; the seven-character bare plugin key is excluded. The
maintenance policy recognizes only this exact observed plugin exception and
still requires one versioned core registration. It retains and rechecks every
plugin registry value/type and the root-level `Uninstall rtlsdr_pi.exe`, whose
observed SHA-256 is
`0da3cfb79b1cf2085f6f53c095abe60300f76f95a9503c09ce467a9c833912fb`.
Unknown bare entries, multiple core registrations and changed plugin identities
remain blocking failures. No registry entry is removed to influence detection,
and the plugin uninstaller is never invoked by the maintenance tooling.

`start-official-upgrade.ps1` provides the repeatable Start/Collect wrapper for
SSH orchestration. It dispatches only this reviewed driver as the same logged-in
SID, using an elevated interactive task. It checks exact task ownership/action
and disables scheduler timeouts, battery stops and hard termination. Collect
removes only an identified completed task; a failed task without result evidence
is reported explicitly and retained for inspection.

Every native action belongs to the exact newly launched installer PID. There
is no `/S`, destination override, generic Next handler, separate uninstaller
invocation, reboot, forced process termination or remote-access change.
The state machine permits only:

```text
Language (optional) -> Welcome -> License -> Upgrade -> Components
-> Start Menu (optional) -> Ready -> Installing -> Finish
```

An install that finishes between observations can transition directly from
Ready to Finish. A fresh install, reinstall, downgrade, parallel installation,
new-directory/configuration page or unknown page cannot proceed. Ready must
explicitly report **Upgrade**, the exact destination, and **none** under
configuration deletion. Unexpected dialogs remain open with failure evidence;
there is no automatic retry or attempt to finish them blindly.

The Components page is inspected using normal Windows TreeView APIs. A bounded
temporary x86 `TVITEM` buffer reads every item, including collapsed children.
Checked deletion leaves are deselected through NSIS's normal Space-key handler,
which changes section flags; changing a checkbox's state image alone would not
change installer behavior. This distinction follows the
[NSIS component-page implementation](https://github.com/kichik/nsis/blob/master/Source/exehead/Ui.c).
All reset/delete items and any default-configuration writer must then be
confirmed unchecked. Unknown, mixed or duplicate destructive components fail
closed. The final summary independently validates that nothing will be deleted.

Finish is recognized only when completion text, Run and Show controls all
exist. Both options are explicitly cleared and read back before Finish is
clicked. Completion also requires the exact expected target executable and no
running OpenCPN process. An observed Finish plus exit status 0 or the previously
observed official-installer anomaly 1223 still requires independent
`upgrade-stock.ps1 -Action Verify` postflight; wizard evidence alone does not
establish successful preservation.

Only the installer's window is captured. It must own foreground, remain visible
and not minimized, and fit inside the virtual desktop. Foreground and bounds
are checked immediately before/after screen capture without reacquiring focus;
interrupted or moved captures are discarded. Mouse clicks likewise require the
exact owned control under the pointer immediately before input. Captures and
control text can contain local paths: retain them privately, outside the live
application/profile, and do not upload them as public CI evidence.

`test-official-upgrade-policy.ps1` exercises complete/incomplete page identity,
transition restrictions, exact final options, destructive checkbox states,
configuration writer restrictions and registry changes. It also compiles the
native interop helper without invoking Windows APIs. The 85-check pure suite
passes on Linux PowerShell and is wired into native Windows PowerShell 5.1 CI. Actual
wizard execution and complete postflight remain native operational gates.
