# Boat PC development and recovery

Use **`ssh boat`**, preserving the configured account quoting and key identity.
Do not construct a replacement user/host command. Tailscale, SSH, RustDesk and
their startup/access rules are outside application deployment scope. No script
here changes them, reboots the PC, elevates silently or sends actuator commands.

Initial inspection found a free, suitable `C:\XNav` workspace and the actual
normal profile at `C:\ProgramData\opencpn`. The original program is under
`C:\Program Files (x86)\OpenCPN`. Its initially observed **5.12.2** hash was
unsupported. The separately authorized official upgrade now passes exact 5.12.4
hash and full profile/plugin preservation checks; see
[the verified upgrade](evidence/beta2-boat-stock-5.12.4-upgrade.json).
A version string cannot override the installer manifest.

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
   installed deployment/launch scripts independently reject unsupported stock hashes.
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
   pixels with actual window DPI. It refuses hidden/minimized/off-screen windows,
   requires the exact process to own the foreground immediately before/after the
   capture, and discards pixels if the window moves; refused captures publish no
   PNG. Captures can contain vessel position or licensed
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

## Early development review versus release acceptance

After native functional, fixture-free package, installer lifecycle, DPI/touch
and public chart/plugin gates pass, CI may upload
`beta2-boat-review-pending-endurance-<commit>`. This is a separate development
bundle with explicit pending-qualification text, the same tested payload hashes
and source artifact. It allows supervised read-only boat iteration while the
three-hour endurance gate continues. It does not bypass compatibility, backups,
the real-profile plugin/output audit or the user prohibition on physical commands.

Record any such deployment as **development review**, including all gates still
pending. Never use it as release acceptance. The final candidate artifact and
named Beta 2 publication remain gated on complete same-commit CI and actual boat
evidence. A failed endurance or other remaining gate invalidates qualification
even if an earlier development review package exists.

## Preliminary display review while the installed prerequisite is blocked

A separate, fixture-free portable review can inspect the physical display without
starting or modifying unsupported stock OpenCPN or its damaged/unknown profile.
This is **preliminary display evidence only**. It does not qualify installation,
the actual user profile, the real nautical charts, connections or vessel hardware.
The installed launch checks above remain mandatory and have no override.

Only after the exact candidate passes its Linux/native Windows product gates:

1. Transfer the accepted `OpenNavX-Beta2-Portable-Recovery.zip` to `C:\XNav` and
   obtain its SHA-256 and commit from the same accepted artifact. Run
   `prepare-portable-review.ps1 -Archive <local-ZIP> -ExpectedArchiveSha256 <hash>
   -ExpectedCommit <commit>`. It requires OpenCPN closed, records a private hash
   inventory of normal OpenCPN application/profile/plugin files, extracts into a
   new workspace run, and verifies all package files and the actual executable's
   fixture-OFF loader identity before any GUI launch.
2. The helper keeps a fresh package-only profile, disabling initialization of the
   four audited bundled plugins. It never copies a real/Beta1 profile, plugins,
   navigation objects, chart database or connection. A nonempty marine connection,
   imported vessel settings, custom plugin path or navigation objects prevents
   launch. Reviewed plugin constructors are limited to their bundled resources
   and initial state; the enabled flag alone is not considered an isolation proof.
3. Retain the returned `record` and `recordSha256` privately. Use
   `portable-review.ps1 -Record <record> -ExpectedRecordSha256 <hash> -Action Launch
   -Mode XNav` (or `Legacy`/`Safe`). A limited interactive task repeats the package
   and profile checks and starts only the owned executable, fixed portable profile
   and software-rendering arguments. The process inherits an OS-only DLL PATH.
4. Use `-Action Capture -ProcessId <returned-PID> -Name navigation-day` to capture
   the actual window and DPI, and `-Action Close -ProcessId <PID>` for normal close.
   Capture and close remain available if profile changes now prevent another
   launch. After an in-app mode restart, identify the new process by its exact
   package executable path; never target an unrelated OpenCPN process.
5. Inspect Day/Dusk/Night, unavailable data, instruments, energy, settings,
   diagnostics and XNav/Legacy/Safe on the bundled real coastline. Do not activate
   navigation, import charts/plugins, configure sensors, request pilot discovery,
   enable control or click actuator commands. No synthetic vessel data is present.
   Do not change Windows display/remote-access settings merely to obtain a size.
   Record measured dimensions/DPI and the difference from the 1280×800 target.
6. `-Action Close` and `-Action Verify` compare the normal installation/profile
   inventory with the original hashes. Keep private native screenshots and logs
   under the review run. Any changed original file, unexpected live input, output
   driver or blocked close fails the review and requires inspection; the helper
   never force-kills the process or silently rewrites a profile to pass.

The portable integration forces profile/logs below its own package, refuses an
external `--configdir` or `--remote`, and does not start OpenCPN's REST/mDNS service.
Pinned OpenCPN creates marine drivers from configured connections; this review's
connection list is empty. Pilot control remains unconfigured/OFF. These reviewed
boundaries, exact package hashes and before/after normal-file checks support this
limited display test; they do not replace later real-boat commissioning.

## Maintenance and old versions

The separately authorized official 5.12.2 → 5.12.4 prerequisite upgrade follows
[the exact source/hash and visible-wizard procedure](installer/stock-upgrade-5.12.2.md).
`upgrade-stock.ps1` performs preflight and postflight only; it does not silently
install or launch an application. It verifies all cold-copy/current file hashes
and preserves the real profile verbatim, including pre-existing corruption.
Missing unbundled plugin files may be restored explicitly from verified backup;
changed files are never overwritten. This does not relax the normal installer
allowlist or the subsequent read-only profile/plugin launch audit.

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
or prunes the active/previous installed recovery generation. `retire-download.ps1`
archives a separately identified obsolete OpenNav ZIP only when its bytes match
the supplied accepted-release SHA-256; it also journals the location before its
same-volume move. Unrelated filenames/hashes are refused. External shortcuts
require separately inspected ownership before removal.

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
Its default requires native Windows CI. An explicit `-IsolatedLocal` invocation
allows the same temporary-file-only checks on native Windows with every OpenCPN
process closed, and labels the output as local filesystem evidence. It never
executes the dummy application/plugin files or reads/writes a real profile,
registry, service or hardware connection. Do not impersonate CI by setting an
environment variable on the boat.
