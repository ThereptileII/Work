# Boat PC development and recovery

Use **`ssh boat`**, preserving the configured account quoting and key identity.
Do not construct a replacement user/host command. Tailscale, SSH, RustDesk and
their startup/access rules are outside application deployment scope. No script
here changes them, reboots the PC, elevates silently or sends actuator commands.

Initial inspection found a free, suitable `C:\XNav` workspace and the actual
normal profile at `C:\ProgramData\opencpn`. The original program is under
`C:\Program Files (x86)\OpenCPN`. Its initially observed **5.12.2** hash is
unsupported, so deployment is blocked pending the separately authorized
prerequisite decision. A version string cannot override the installer manifest.

## Repeatable sequence

1. Read `inspect.ps1` output privately. Never commit raw Desktop listings, account
   names, navigation data, chart paths or connection credentials.
2. Close OpenCPN normally. Run `backup-environment.ps1` with the inspected
   application/profile directories. It verifies every copied file, rescans the
   sources, then atomically publishes a versioned local recovery set. A `.partial`
   directory is an incomplete backup, never accepted recovery. Do not delete it
   before diagnosing a failure with `inspect-partial-backup.ps1`.
3. Place a reviewed `boat-target.json` in the workspace, starting with
   `boat-target.example.json`. Keep this machine-specific file private. All
   deployment/launch scripts independently reject unsupported stock hashes.
4. Commit, pass Linux/native Windows CI, and obtain the exact installer and its
   SHA-256 from the accepted artifact. Transfer via the established SSH alias.
   `install.ps1` or `update.ps1` requires that hash and exact expected commit;
   it does not fetch an unpinned latest release or launch the application.
5. Inspect real connection output directions, every discoverable plugin and route
   state. Record a short-lived read-only audit of the exact profile INI, plugin
   files and installed commit. Outputs from normal OpenCPN/plugins can exist
   independently of XNav's disabled autopilot controller. **Do not launch until
   these are reviewed.** The script will not alter connections to pass this gate.
   The audit uses actual JSON booleans (a string such as `"false"` is rejected),
   exact profile/build/plugin hashes, and a 24-hour expiry. It independently
   rejects corrupt/zero-filled INI files, output-enabled connections, restored
   active routes and ambiguous configuration. The configured profile must be
   the actual normal Windows profile, never an unrelated test INI.
   Pinned OpenCPN constructs plugin candidates before checking their enabled
   flag. Therefore **disabled plugin DLLs also require startup/idle review**.
   The complete recursive `*_pi.dll` inventory in both the app and current-user
   plugin directories must match `pluginFiles`; each entry requires
   `startupAndIdleReadOnly: true` after source/behavior inspection. An empty list
   is valid only when no candidates exist. Custom plugin directories and reparse
   paths are refused until their path semantics are explicitly supported.
   The same checks run again in the interactive task immediately before launch.
6. `run-xnav.ps1`, `run-legacy.ps1` and `run-safe.ps1` schedule a single limited
   task in the already logged-in user's desktop. This avoids starting an invisible
   GUI in the SSH service session. One matching interactive Explorer session is
   required. The temporary task is removed afterwards; the application stays open.
7. `capture-ui.ps1` captures only the native application rectangle, in physical
   pixels with actual window DPI. Captures can contain vessel position or licensed
   charts: retain them privately, and review before publishing cropped/redacted
   evidence. Display dimensions are measured, never assumed from the specification.
8. `smoke-test.ps1` starts XNav, checks response/startup marker, captures the chart
   and closes normally. It cannot accept an all-water/blank image: the result is
   explicitly **captured-review-required** until chart content is reviewed.
   `stop.ps1` requests normal close and refuses force termination on timeout.
9. `collect-logs.ps1` exports bounded deployment/process metadata only. For detailed
   source health use the product's explicit sanitized **Export Diagnostic Bundle**.
   The scripts do not copy raw navigation logs or the profile into reports.
10. Retest modes/chart content and record the tested commit, executable hash,
    screenshots, timings, data availability, and remaining physical limitations.

A normal application exit can change its INI. Renew the read-only audit after
reviewing those changes before the next launch; do not blindly refresh its hash.
No script clicks pilot commands, activates routes, injects sensor input or creates
synthetic AIS/radar. Native/boat visual review is separate from hardware control
acceptance.

## Maintenance and old versions

`repair.ps1`, `rollback.ps1` and `uninstall.ps1` execute the verified installed
transaction engine and recheck the untouched original executable. An unknown
modified generation is preserved for inspection. Recovery does not restore old
navigation data over recent work.

`retire-portable.ps1` removes an explicitly identified old portable folder from
its user-facing location by atomically moving it into local recovery. It requires
an inventory hash from the accepted old release, a portable marker, no reparse
entries and no running OpenCPN. All profile/chart/log additions are preserved. A durable planned record is written
before the move and a separate completion record follows verification; a power
interruption cannot erase the known recovery location.
It never guesses ownership from a folder name, deletes unrelated Desktop content,
or prunes the active/previous installed recovery generation. ZIPs/shortcuts outside
such a verified folder require separately inspected ownership before removal.

## Optional source/build operation

`update-source.ps1` fetches an exact commit into the owned workspace, requiring a
clean checkout and the expected origin. `build.ps1` is optional for a previously
provisioned native MSVC Win32 product build tree with test fixtures OFF. It runs
build/tests but does not qualify or install a new local binary. Normal deployment
uses the already-qualified CI installer.

The native disposable `test-boat-tools.ps1` covers cold-copy verification with
multiple, empty and zero-filled files, literal ampersand names, overlapping-path
refusal, atomic evidence publication, journaled portable retirement with
private user-data preservation, strict audit booleans, corrupt/malformed profile
rejection, connection directions and complete plugin inventories. These tests do not substitute for actual
boat screenshots, connections, installed lifecycle or physical touch validation.
