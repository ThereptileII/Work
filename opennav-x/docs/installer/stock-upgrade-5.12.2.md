# Authorized boat prerequisite upgrade: 5.12.2 to 5.12.4

This procedure upgrades **official OpenCPN**, before OpenNav integration. It
does not launch OpenCPN, restore an INI, alter sensor/plugin configuration, or
qualify Beta 2. The user authorized this prerequisite upgrade after inspection
found that the real boat installation was 5.12.2. The normal OpenNav installer
allowlist still accepts only the exact previously qualified 5.12.4 executable.

## Exact boundaries

| Item | SHA-256 |
| --- | --- |
| Inspected original 5.12.2 x86 EXE | `2fdcd6a2cdef7f730aa4c094fcd21302ed2a5d531a611ee180c06533f3a2cb48` |
| Official 5.12.4 Setup | `e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa` |
| Qualified 5.12.4 x86 EXE | `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c` |

Download the [official 5.12.4 installer](https://github.com/OpenCPN/OpenCPN/releases/download/Release_5.12.4/opencpn_5.12.4-0%2B3720.37fd0cd_setup.exe)
and verify its hash before execution. No alternate executable is authorized by
this procedure.

The actual old release source is
[`b69f44ce8e376d0d499164a1182dbd2ba3156e0f`](https://github.com/OpenCPN/OpenCPN/blob/b69f44ce8e376d0d499164a1182dbd2ba3156e0f/NSIS.template.in.in).
Its NSIS template SHA-256 is
`2932946902f0e4d0c7d20a9ff0653e9d25b2cf0c833f9ca6d18a2c344ee85f4f`.
Its text is identical to the pinned 5.12.4 template. Inspected boundaries:

- `RequestExecutionLevel highest` and `.onInit` require administrative access.
  UAC must not be disabled or bypassed by changing Windows policy.
- `Page_TypeInst_Leave` selects `INSTALL_TYPE=2` for visible **Upgrade**. This
  initialization is skipped by `/S`; silent installation is not a qualified
  substitute for the real upgrade flow.
- `SecTopmost` runs the previous uninstaller with `/S /type=1`. The old
  `un.onInit` clears profile-folder and configuration-registry deletion for
  this type. Generated program-file removal still executes.
- Configuration-reset sections are optional. Preserve them unchecked. An
  existing zero-filled INI is still an existing file; this operation must not
  repair or replace it.
- Finish offers **Run OpenCPN** by default. Explicitly uncheck Run and Show
  before Finish; no application launch is permitted in this operation.
- The official installer reapplies the profile directory's BUILTIN Users ACL.
  File contents must remain identical; the ACL before and after is recorded.
- `Check_Prev_Installs` counts versioned OpenCPN registry keys longer than seven
  characters with a `CompareVersion`; it excludes the observed bare `OpenCPN`
  key belonging to RTL-SDR 1.3.1. The guard recognizes only that exact plugin
  name, publisher, version and uninstall path alongside one valid core key.
  It preserves and rechecks every plugin registry value and type; other bare
  keys or ambiguous core/plugin registrations still fail. The plugin's
  application-root `Uninstall rtlsdr_pi.exe` has observed SHA-256
  `0da3cfb79b1cf2085f6f53c095abe60300f76f95a9503c09ce467a9c833912fb`.
  It is preserved like the plugin's complete ancillary tree and never invoked.

## Guarded procedure

1. Keep OpenCPN/XNav closed. Obtain the verified cold recovery record and its
   SHA-256. Run the read-only preflight (only private evidence is written):

   ```powershell
   .\upgrade-stock.ps1 -Action Preflight -Workspace C:\XNav `
     -Setup C:\XNav\downloads\official-opencpn-5.12.4-setup.exe `
     -BackupRecord <verified-recovery.json> `
     -ExpectedBackupRecordSha256 <exact-record-hash>
   ```

   This verifies both complete backup trees and current originals, the source
   EXE/PE/version, exact official installer, one old core registration,
   the narrowly identified retained plugin registration, and
   free-space margin. It emits a private `prepared.json` and its hash. It never
   invokes an installer. The record contains original file inventories and
   registration values for recovery; do not commit or upload it.
2. Give this exact preparation record/hash to the separately reviewed native
   wizard driver. Run the verified official installer in the interactive
   administrator session, select **Upgrade using previous settings**, verify
   destination and preserve every configuration-reset option. Stop on any
   unexpected page or dialog. Uncheck Run/Show before Finish. The driver must
   bind every action to the exact verified installer process/window and produce
   completed evidence. Never use the disposable CI helper's force-termination
   fallback on the boat.
3. Run postflight using the preparation record and the completed wizard report,
   each bound to its exact hash:

   ```powershell
   .\upgrade-stock.ps1 -Action Verify -Workspace C:\XNav `
     -Record <prepared.json> -ExpectedRecordSha256 <prepared-hash> `
     -WizardReport <wizard.json> -ExpectedWizardReportSha256 <wizard-hash>
   ```

   Require the target EXE/PE/version, resources, one correct target core registration,
   every original plugin registration value unchanged,
   and every profile file unchanged, including the damaged INI and valid-looking
   temporary siblings. Compare all non-bundled `plugins/` files, including
   executables, dependencies, scripts and licenses, plus the root plugin
   uninstaller identified above. If the official uninstaller
   removed these, rerun with `-RestoreMissingPluginFiles`: only missing files are
   copied atomically from the reverified recovery tree. Existing differing files
   are never overwritten. The four stock bundled plugin trees may legitimately
   update. Full resulting application inventory is retained privately.
4. No running OpenCPN process is permitted at completion. The original recovery
   set remains intact. The corrupt-profile question and output/plugin safety
   audit remain separate gates before any subsequent application launch.

An official wizard exit of 1223 has previously occurred after an observed
successful Finish on native CI. It is accepted only with exact completed-wizard
evidence and every independent postcondition, never based on the exit code alone.
Any other failure preserves evidence and requires inspection; this procedure does
not force-kill processes, reboot, change remote-access services, restore a profile,
or guess a rollback across an incomplete official installer transaction.
