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
