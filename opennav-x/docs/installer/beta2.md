# Beta 2 setup and lifecycle

Beta 2 keeps the accepted installation architecture: an exact-hash-gated,
per-user application beside the original OpenCPN. The real OpenCPN profile,
charts, connections, routes and plugins remain shared by installed XNav, Legacy
and Safe. There is no profile import or duplicate chart database. The historical
`OpenNavXAlpha1` owner/root/registry identity remains intentional migration state.
Normal product captions use the active release version.

`OpenNavX-Beta2-Setup.exe` presents Welcome, detected OpenCPN, recovery backup,
shortcut options, Ready, installation/validation and Finish. Original OpenCPN
must match the established 5.12.4 i386 executable SHA-256; its version label or
registry registration alone does not authorize installation. Unsupported copies
are refused. Install/Update/Repair remain explicit actions. The original
application is never overwritten, so setup normally needs no administrator rights.

Before staging, a durable timestamped recovery record captures the original
executable identity, old state pointer, target version and target commit. Previous
application generations are the application backup. Navigation data is never
rolled back. A separate cold boat-PC backup preserves the original application
and actual profile locally before any first deployment or prerequisite upgrade.

Every staged file is hash checked. The actual loader self-test verifies commit,
version, resources and normal-profile readability without initialization. Beta 2
additionally requires `test_fixtures=false` and `build_purpose=INSTALLED PRODUCT`;
an executable containing CI/demo fixtures cannot be installed as Beta 2. Earlier
accepted Beta 1 remains valid as the explicitly retained upgrade/rollback fixture.

The wizard offers optional Legacy and Safe shortcuts, with XNav and maintenance
always available. Shortcut choices are saved with the application generation and
restored on rollback. Normal updates/repairs preserve them unless the wizard
explicitly changes them. Windows Installed Apps provides maintenance/uninstall.
All previously accepted ownership, reparse, payload-bound, concurrent transaction,
locked-file, interrupted-copy, atomic-publication and modified-file retention
protections remain mandatory.

## Qualification

The prior-release fixture now fetches **the actual accepted Beta 1 Setup**, using
`tools/accepted-beta1.lock.json`. Outer archive and extracted Setup bytes are
independently hash pinned. Expired/modified artifacts fail the gate; the candidate
is never relabeled to simulate an older release. Native lifecycle tests add all
wizard pages, the recovery-before-update record, Beta 1 to Beta 2, Beta 2 repair,
uninstall/reinstall, same-version generation update/rollback, shortcut preference
persistence, and rejection of an actual fixture-enabled native executable even
when its package and manifest hashes are internally consistent. Existing
original-stock/profile/plugin-preservation and failure cases are retained.

A same-version generation test does not prove two distinct Beta 2 builds are
compatible. That comparison requires retaining a subsequently accepted earlier
Beta 2 artifact and must be recorded separately if exercised. Native tests and
boat results are authoritative; PowerShell parsing or Linux NSIS compilation
alone does not qualify installation.

## Actual boat preflight

The initial boat inspection found OpenCPN **5.12.2**, x86, executable SHA-256
`2fdcd6a2cdef7f730aa4c094fcd21302ed2a5d531a611ee180c06533f3a2cb48`.
This is unsupported. Beta 2 installation is blocked until an explicitly authorized
and verified supported prerequisite is installed. No exception has been added to
the allowlist. See the current project status for the latest boat decision and
backup evidence; do not infer deployment from the existence of these scripts.
