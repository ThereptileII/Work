# Boat deployment and maintenance ordering

Complete installation, update or repair before preparing a read-only commissioning
session. After a session, close OpenCPN and its reviewed helpers normally, inspect
the saved differences, and complete the exact commissioning restoration before
any further installation, repair, rollback or uninstall. Changing an installed
generation during quarantine would invalidate the paths and identities required
for restoration.

The boat `install.ps1` and `maintain.ps1` wrappers refuse all mutating actions while
`commissioning-active.json` exists, even if that record is incomplete. They check
before target access or evidence creation and again before invoking maintenance.
Do not delete this marker to bypass the check. Read-only diagnostics remain
available. Operations must be serialized; these wrappers are not a substitute
for coordinating separate Windows setup processes.

`repair.ps1` and `uninstall.ps1` can inspect an owned installation when its managed
`app/opencpn.exe` is missing or corrupt. This exception only permits offline
Repair/Uninstall: it never authorizes running that executable. The state owner,
generation identifier, ownership record and expected executable hash record must
still be valid, and `maintain.ps1` independently verifies the exact retained
`Lifecycle.ps1` before invoking it. A damaged maintenance engine requires the
verified original Setup. Normal launches, captures and commissioning identity
checks continue to require the exact installed executable hash.

The underlying lifecycle engine stages and verifies replacement files, then
publishes a new generation. It retains modified files during uninstall and never
restores old navigation data over the current shared profile. Silent boat setup
does not run the Finish-page launch action. Its staged loader self-test exits
before profile initialization or plugin loading.

`test-boat-tools.ps1 -PortableContracts` passes five Linux maintenance groups.
The complete native suite passes 27 groups, retaining the original 21 and adding six, including
all six mutating wrapper entrypoints refusing active commissioning before any
target/process access. The boat's native temporary-file run is recorded privately
in `evidence/local/boat-beta2/native-maintenance-repair-casing-final.json`; the existing
launch-verification suite also passes eleven Linux groups. These temporary
fixtures do not qualify a real installation or hardware session.

## Explicit cold reconciliation after a later user installation (SCRUM-17)

If a later user installation changed the selected generation while an older
commissioning transaction remained active, normal Restore, adoption and session
preservation continue to refuse the context mismatch. Do not delete the active
marker, rewrite old records or roll the installation back to imitate its old
state. `reconcile-commissioning.ps1` has a separate three-phase path:

```powershell
.\reconcile-commissioning.ps1 -Action Inspect -Workspace C:\XNav `
  -Record <original-prepared.json> -ExpectedRecordSha256 <exact-original-hash>
.\reconcile-commissioning.ps1 -Action Plan -Workspace C:\XNav `
  -Inspection <returned-inspection.json> -ExpectedInspectionSha256 <exact-hash> `
  -PreservationReview <independent-review.json> -ExpectedReviewSha256 <exact-hash>
.\reconcile-commissioning.ps1 -Action Commit -Workspace C:\XNav `
  -Plan <returned-plan.json> -ExpectedPlanSha256 <exact-hash>
```

Inspect and Plan change only private evidence. They retain a complete current
profile backup, loader-tree/ACL observations and the old prepared record, plan,
inventory and DLL backups. Old and current generation contexts remain distinct.
Both generations must still be owned and retained; all old loader trees must be
unchanged except the independently verified original quarantine entries. Every
original plugin destination must be absent, with its original directory present.
Unknown, occupied, redirected, retired or missing paths refuse. No directory is
recreated and no old plugin is placed in the current generation.

The independent review uses the existing `OpenNavX.SessionPreservationReview.1`
schema and exact per-key before/after decisions, with `parentPreparedSha256`,
`inspectionSha256` and an additional exact `connectionDisposition`:

- `preserve-current-original-direction`: the entire current DataConnections value
  already equals the original baseline. All current profile bytes remain exact.
- `restore-original-temporary-direction`: it still equals the original temporary
  input-only value. Commit reverses only that identified COM8 direction byte.

No other connection value/schema is admitted. A separately saved
`normalized-input.ini` permits the existing bounded preservation policy to check
other settings; it is never substituted for the live profile. Every actual
current difference, including an already-restored connection direction, still
requires an explicit review decision. Cross-generation resource-path changes,
unknown keys and manual-pilot binding changes remain unsupported and refuse;
this is not general profile migration or new launch authority.

Commit reacquires the existing profile/transaction locks and checks the exact
unchanged current identity, marker, current profile, both loader inventories,
ACLs and original backups. An unstarted plan expires after 24 hours. A durable
parent intent pins one plan before mutation; individual original-path move
journals permit recovery after profile publication or partial DLL restoration.
Retry only Commit with the same plan/hash. Changed bytes, conflicting journals
or other target choices refuse. Neither phase launches an application or sends
physical output; restoring an output-capable stored connection is not permission
to launch it.

Completion and `reconciled-baseline.json` use distinct
`explicit-cross-generation-reconciliation;current-user-state-origin-unverified`
provenance. They preserve the original context and independently recorded current
context; they never relabel the old proof as current. Only after verifying durable
completion does Commit remove its exact active marker. The completed baseline
can then be explicitly selected by fresh Inventory/Prepare after the next
qualified installation, with a new source/plugin review and launch audit.

`test-commissioning-reconciliation.ps1 -PortableContracts` exercises disposable
entrypoints; Windows CI or explicit `-IsolatedLocal` additionally exercises native
profile locks, ACL preservation and file operations with inert identity/process
adapters. The existing session-preservation suite includes it, so the registered
`skager-profile-preservation.yml` needs no workflow change. Portable results do
not qualify native Windows behavior, the actual boat profile or any launch.
