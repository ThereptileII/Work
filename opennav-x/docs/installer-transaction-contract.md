# Alpha installer transaction contract (candidate; native acceptance pending)

The NSIS setup runs as the current user. Windows PowerShell 5.1 implements the
transaction engine. The installer is deliberately side-by-side: the original
supported OpenCPN executable and its system DLLs are never replaced. XNav,
integrated Legacy and Safe use OpenCPN's normal shared profile; the portable
marker is excluded. No profile import/copy is performed by installation.

Before installation mutations, preflight checks the embedded manifest hash,
payload hash, original executable SHA-256, PE i386 architecture and 5.12.4 file
version. Registry locations only supply discovery hints. The public allowlist
remains empty during qualification. A clearly marked CI-only candidate manifest
uses the inspected official binary for disposable lifecycle tests; it does not
establish release support or permit arbitrary version-string matching.

`%LOCALAPPDATA%/OpenNavXAlpha1` contains an ownership marker, atomic state pointer,
transaction journal, immutable application generations and bounded per-action
logs. Setup stages files in a new generation, checks every payload hash, copies
unbundled system plugin additions, runs the actual executable loader/resource
check, records ownership/provenance, then atomically replaces `state.json`.
Start-menu shortcuts and the per-user uninstall entry follow that pointer. A
failure before commit leaves the previous pointer intact; a failure after commit
is recovered by rebuilding shortcuts from the committed state. Unpublished
staging directories remain available for diagnosis. No interrupted operation
restores an old navigation database over newer user work.

Repair uses a new generation with known-good owned files from its retained,
hash-verified package. The maintenance wizard offers Repair, Rollback, Uninstall
and Diagnostics; updating uses a newer Setup download. User additions are
carried forward separately from the managed payload; collisions fail closed.
Update retains the previous generation. Rollback verifies and restores it;
rolling back the first installation unregisters the integration. Uninstall
removes registration, shortcuts and hash-matching publisher-owned files. Modified
files, custom additions, unpublished staging and diagnostic logs are retained.
These items are documented for manual inspection/removal after verification.
The untouched stock shortcut remains usable.
This is not a claim of a final production uninstaller or crash-proof filesystem.

The engine refuses running OpenCPN processes, unknown ownership, redirected
paths, unsafe ZIP paths, duplicate payload names, oversized extraction and
changed stock hashes. A transaction file lock prevents concurrent maintenance.
The installer does not download or execute plugin installers. Compatible user
plugins remain subject to normal OpenCPN plugin behavior and separate testing.
No system-wide plugins or runtime installation are required by OpenNav.

The application `--opennav-self-test <new-absolute-json-path>` exits before
profile selection, OpenCPN startup and plugin/connection initialization. Normal
Windows DLL loading still occurs, and required S57/UI/coastline resources are
checked. Report overwrite is refused. The narrow guarded startup/exit bypass is
only active for this explicit mode. This loader check supplements, rather than
replaces, native application, chart and plugin smoke tests.

The native candidate gate installs the hash-pinned official stock setup in a
disposable runner, seeds its actual wx normal profile with disconnected fixtures,
checks byte preservation during maintenance, verifies all interface modes and
coastlines, repairs damage, updates, rolls back, injects failures around commit,
checks diagnostic hashes, uninstalls and launches original OpenCPN. Every case
must pass before populating the public compatibility manifest. Linux checks the
real loader path and profile non-mutation; PowerShell 7 parsing on Linux is a
syntax aid, not a substitute for native PowerShell 5.1 execution.

The stock setup fixture follows NSIS's special final, unquoted `/D=` argument
contract even for paths with spaces. Conventional uninstall tests wait for the
engine's durable result because NSIS normally copies the uninstaller into a
second temporary process. See [NSIS command-line contract](https://nsis.sourceforge.io/Docs/Chapter3.html).

The update gate compiles a distinct `0.2.0-alpha0-ci` application and packages it
as a private fixture. It installs/launches that executable, upgrades to the exact
Alpha candidate, verifies version/hash changes and prior-generation retention,
and then runs the remaining lifecycle. Fixture compilation restores the tracked
version header and original candidate executable in a `finally` block; the
fixture is excluded from release downloads. Loader checks verify product version
as well as build commit. Literal upstream resource names containing `&` are
accepted as filenames; they never enter a shell command.

NSIS `GetOptions` clears the output on an absent option. Normal wizard startup
therefore explicitly restores Install (Setup) or Repair (maintenance) defaults.
Native automation quotes option values (`/OPENCPN="path with spaces"`) rather
than the entire option, so the selected stock path is actually parsed. The
normal no-argument wizard is exercised and captured before cancellation; this
supplements the silent lifecycle suite. [NSIS GetOptions source](https://github.com/kichik/nsis/blob/master/Include/FileFunc.nsh).

Before native compilation, the Windows contract job executes the actual
filesystem-helper AST definitions under both 32-bit and 64-bit Windows
PowerShell 5.1. It checks literal resource names, path traversal/reserved names,
reparse ancestors, native atomic JSON replacement, owned-file corruption and PE
architecture. It never executes the installer entry point or writes outside its
new temporary fixture. These checks do not replace the full installed lifecycle.

Native PowerShell 5.1 exposed `$null` coercion to an empty backup-path string in
`File.Replace`. Atomic replacement now supplies `NullString.Value`, preserving
the no-backup-file .NET contract. The actual native replacement test remains the
acceptance gate; the failed candidate is retained in evidence.

The official OpenCPN prerequisite is installed only in the disposable fixture.
Its unmodified upstream setup requests administrator access; the harness uses
Windows ShellExecute RunAs and records the caller privilege, exit result and
installed hash. It does not lower UAC policy or patch the prerequisite. This
elevation belongs to installing stock OpenCPN, not to Alpha Setup, which remains
`RequestExecutionLevel user`. A failed prerequisite never qualifies an Alpha
installation or populates the compatibility allowlist.

The exact official prerequisite has a native exit-status anomaly: `149c41b`
completed every visible installation page including Finish yet returned 1223.
The fixture now validates explicit completion text, exact executable hash,
chart/UI resources and matching native uninstall registration before allowing
that specific status for this stock package. OpenNav setup/maintenance still
require zero. This replaces an incorrect exit-code-only fixture assumption; it
does not accept an aborted/missing wizard or unknown binary. The final lifecycle
also launches the untouched stock application and checks chart/profile data.
[Observed completion](evidence/installer-stock-exit-149c41b-observation.json).
