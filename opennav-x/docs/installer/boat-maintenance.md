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
requires an explicit review decision. Version 1 continues to refuse
cross-generation resource-path changes and manual-pilot binding changes.

An independently reviewed **version 2 cold retention** may instead retain the
entire already-existing original-direction profile without publishing profile
bytes or granting authority. Use `schema: 2`, owner
`OpenNavX.ColdReconciliationRetentionReview.2` and mode
`retain-existing-original-direction-no-grants`, with the same exact hashes,
per-key decisions and `connectionDisposition: preserve-current-original-direction`.
It additionally requires all eight actual boolean acknowledgments under
`retentionAcknowledgments`: `retainExistingBytes`, `originUnverified`,
`pilotIdentityUnverified`, `savedPermissionIsNotAuthorization`,
`noProfilePublication`, `noApplicationLaunch`, `noCommands` and
`freshCommissioningRequired`. `wmmRetention` is null unless the resource path
changed. For a change it contains the exact `Get-ReconciliationWmmRetention`
two-generation inventory: retained ownership bytes/hash, exact generation and
commit, canonical location, and all three source-pinned resource hashes. Plan
and Commit recheck both live generations; historical baseline reads validate
the frozen evidence without requiring retired generations to remain installed.

This mode still refuses unknown keys, malformed encoding, mappings, calibration,
unowned resources and unsupported values. Its extra bounded policies cover the
old-to-current owned WMM location, known base vessel setup/display fields and
stored COM8 pilot address/NAME/permission scalars, AIS radius 1–200 NM, weather's
exact 0/1 flag, and a finite monotonic Dashboard `SumLogNM`. A stored manual
permission is retained data with unverified origin; it never supplies manual
child proof or a qualified pilot. The plan, completion and baseline record
`pilotBindingQualified: false`, `commandsAuthorized: false` and
`profilePublished: false`. This restriction survives subsequent cold-baseline,
adoption, preservation and ordinary reconciliation lineage.

Commit reacquires the existing profile/transaction locks and checks the exact
unchanged current identity, marker, current profile, both loader inventories,
ACLs and original backups. An unstarted plan expires after 24 hours. A durable
parent intent pins one plan before mutation; individual original-path move
journals permit recovery after profile publication or partial DLL restoration.
Before profile publication, a separate journal pins the plan-derived staged path,
its exact target bytes and native metadata. Creation time is stored as a decimal
tick string so Windows PowerShell 5.1 JSON cannot round it to milliseconds; the
existing exact metadata/ACL comparison remains unchanged. A retry admits only
that validated stage into the otherwise unchanged complete profile inventory
and publishes it
through the same atomic replacement API. Changed/incomplete staging bytes,
changed metadata, a substituted stage path and arbitrary extra files refuse;
keep the evidence for investigation instead of deleting or ignoring them.
Retry only Commit with the same plan/hash. Changed bytes, conflicting journals
or other target choices refuse. Neither phase launches an application or sends
physical output; restoring an output-capable stored connection is not permission
to launch it.

Completion and `reconciled-baseline.json` use distinct
`explicit-cross-generation-reconciliation;current-user-state-origin-unverified`
provenance. They preserve the original context and independently recorded current
context; they never relabel the old proof as current. Only after verifying durable
completion does Commit remove its exact active marker. If the process stops
immediately after removal, repeating Commit with the same pinned plan validates
the existing completion/baseline lineage, original plugin paths, exact current
profile, contexts and inventories under both locks, then returns the existing
receipt without mutation. A changed plan or live state, missing completion or an
unknown new marker refuses; an absent marker alone never proves success.
The completed baseline can then be explicitly selected by fresh Inventory/Prepare
after the next qualified installation, with a new source/plugin review and launch
audit.

For an unqualified retained baseline, the existing private review plan must also
contain `passiveLaunchReview.evidencePath` and `evidenceSha256`, identifying a
JSON source-review artifact. Its schema is 1, owner is
`OpenNavX.ReadOnlyCommissioning.PassiveReview.1`, scope is
`passive-startup-idle-shutdown-no-commands`, and `reviewedUtc` is the actual review
time. Pin `sourceRevision`, `executableSha256`, `baselineSha256`, `inputSha256`
and `inventorySha256` to the new candidate and fresh inventory. `transport` is
exactly `COM8-serial-NMEA2000-input-only;gateway-management-traffic-allowed`.
Its five `boundaries` name `core-generic-send-call-sites`,
`retained-plugins-helpers-adjacent-libraries`, `skager-pilot-sink`,
`gateway-initialization` and `startup-idle-shutdown-route-state`; each needs
concrete `sourceReferences`, a specific `reason` and conclusion
`no-actuator-command-reachable-in-passive-scope`. This is a reviewed source
closure, not a generated assertion. A reachable actuator-producing path in the
intended scope must refuse or be separately quarantined/isolated.

Prepare validates and copies that exact artifact; Apply and actual launch
verification require it to be at most 24 hours old. Prepared/applied records
retain false pilot qualification and command authority, and the independent
read-only launch audit must bind `passiveReviewSha256` to the immutable prepared
copy. Existing profile, complete loader-tree, plugin/source, account, build,
route and transport checks still apply. COM8 IOSelect=0 withholds SKAGER's
qualified output path; generic OpenCPN/plugin writers require the explicit
source review above. Harmless serial gateway initialization/manufacturer queries
are allowed. No pilot identity query, command/UI test, route activation or
Send-to-GPS action follows from passive review. Physical bidirectional pilot
qualification is separate and is not a prerequisite for this passive scope.

The artifact also pins `pluginReviewSha256`, computed by
`Get-CommissioningPluginReviewHash` over the existing per-plugin path, binary
hash, disposition, source revision/boundary, startup/idle conclusion and evidence
hash. This binds the actual retained/quarantined subset without a cyclic hash of
the whole plan. Changed decisions need a new passive source review; copied
evidence paths may change while their exact content hashes remain fixed.

`test-commissioning-reconciliation.ps1 -PortableContracts` exercises disposable
entrypoints; Windows CI or explicit `-IsolatedLocal` additionally exercises native
profile locks, ACL preservation and file operations with inert identity/process
adapters. The existing session-preservation suite includes it, so the registered
`skager-profile-preservation.yml` needs no workflow change. Portable results do
not qualify native Windows behavior, the actual boat profile or any launch.
