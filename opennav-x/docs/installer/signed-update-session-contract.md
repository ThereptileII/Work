# Signed boat update session contract (SCRUM-311)

This contract covers a **pure receipt model**, a separately qualified
non-resumable journal store, authenticated acquisition and concrete restoration
components. It is not an enabled boat update path.
`tools/boat/SignedUpdateSession.ps1` records four ordered transitions and rejects
missing identities, reused requests, skipped steps, expired sessions, mutated
receipt chains and transferred commissioning evidence. It performs no file,
network, process, profile, signature, commissioning or launch operation. Receipt
hashes are content bindings, not signatures and not launch permission.

`SignedUpdateSessionStore.ps1` adds exclusive same-process custody around that
reducer. It creates one private, write-through armed journal with its owner and
protected Windows ACL supplied atomically. Held file and ancestor handles reject
replacement, reparse paths and multiple file links. Each flushed frame contains
the complete state and head; the live owner retains the exact prior bytes/hash.
Duplicate or case-folded JSON keys, invalid UTF-8, oversized input, reordered or
replayed transitions, changed identity, expiry and storage faults refuse.

This store deliberately cannot resume a disk journal after owner loss. A whole
disk rollback cannot be detected from disk records alone. Close or failure leaves
the armed record in place; there is no delete, disarm or reopen API. Returned
receipts record consumed transitions and are not reusable launch tokens. Native
Windows PowerShell 5.1 x86/x64 passed 90 inert store checks and the existing 49
reducer checks at helper `43972f2be2cf5a809b7edaa5dc9d6ca759491678`,
[run 37429822450](https://github.com/ThereptileII/Work/actions/runs/37429822450).
The store does not implement IPC peer authentication, actual artifact verification,
installer/spawn hooks, selected-directory policy or interrupted-update recovery.
Those integrations must reject missing/abandoned custody once armed, including
ordinary startup; the current product does not yet call this store.

### Installed armed-session denial boundary

Installed arming uses the explicit `-InstallationRoot` option on
`New-SignedUpdateStore`. The private journal directory remains separate and
retains its original protected ACL requirement. While holding the installation's
existing exclusive `transaction.lock`, the store checks the owned root and
selected previous generation, then atomically creates and flushes
`<InstallationRoot>/signed-update-armed.jsonl` with its protected owner/SYSTEM
file ACL **before** creating the private journal. The store retains that root
file and opened ancestor handles; current-store checks also verify the held
marker bytes. The maintenance lock is released only after initial arming is
durable. Any later refusal, partial write or close leaves the root marker in
place. There is no delete, disarm, reopen, resume or bypass API.

The installed launcher, supervisor and mutating Lifecycle entry points reserve
that exact root filename. Any present leaf, including empty/corrupt bytes or a
directory, denies ordinary startup and maintenance. Lookup uncertainty also
denies; the implementation must prove leaf absence rather than interpret a
failed `File.Exists`/`Test-Path` as permission. Checks at the existing lock and
creation boundaries narrow races but do not claim to pin absence or authenticate
a broker. Marker contents are never parsed into launch or rollback authority.
The final ordinary Go creation checks derive the installation root from the held
native executable and exact managed-generation suffix, under its existing
ancestor custody. Remapping the original requested DOS/SUBST alias cannot select
another root's absent marker for that held image. Other Go lookups capture the
drive once before checking physical ancestry and the fixed leaf. PowerShell
checks likewise resolve their own physical drive for each lookup; per-call
resolution alone is not proof of identity binding to a previously held image.
The boat direct launcher (`InteractiveJob.ps1`), startup launcher and
commissioning restart broker also retain the existing native image owner via
`InstalledLaunchBarrier.ps1`. Their final marker lookup derives its root from
that retained native image and exact managed-generation suffix, then corroborates
the original requested image spelling. Checks precede actual process creation
and both restart permit publication and ALLOW. The installed custody module is
executed from the same bounded held bytes whose ownership hash was verified.
These restrictions preserve existing creator paths and do not grant signed
preparation authority or continuously pin absence. Portable focused checks pass;
native fixtures and actual-entry integration remain qualification gates.
Normal installations with proven absent markers keep their existing behavior.
Fresh-install/preflight checks may separately prove a missing root component
under checked existing ancestors. They do not authorize mutation of an existing
root: after root creation, strict marker-absence checks run before and under
`transaction.lock`, before ownership, recovery or installed state writes.

Retained native directory owners request `FILE_LIST_DIRECTORY` as well as
metadata access while omitting share-delete. A metadata-only handle does not
provide the claimed Windows rename exclusion. Empty-directory native tests
verify sharing refusal while held and successful rename after release, without
letting a descendant image handle mask an ineffective ancestor pin. This applies
to Go ordinary/armed launch ancestry, the C# barrier/image owner, and the new
broker store/image/stock ancestry. It does not continuously pin a DOS alias,
freeze reparse metadata, exclude earlier writable handles or prove loader
custody. See the [exact-source native evidence](../evidence/scrum311-directory-pins-native-20261012/README.md).

The option without an installation root remains an inert/private store API; it
does not arm installed entry points or authorize a product operation. The
installed option itself supplies **only denial and persistence**, not proof of
the previous commit/package/image, reviewed commissioning, restored user data,
source approval or permission to perform a stage. The future trusted broker must
independently verify those identities and use its own authenticated operation
path; the ordinary entry points deliberately have no exception for it yet.
The four-stage broker and actual restoration/installation/spawn hooks below
remain required. No live signed update is enabled by this increment.

`test-installed-signed-update-store.ps1` is included in the existing signed-session
gate. Portable execution covers actual temporary-file persistence, ordering,
lock conflict, malformed ownership, selected-generation mismatch and injected
root/journal write failures, with explicit substitutions for unsupported Windows
handle/ACL operations. The exact native store ACL/path checks are recorded below;
combined installed entry-point qualification remains pending. Earlier image-owner
passes do not qualify that combined boundary.

### Physical installed-store paths

Installed arming captures the requested DOS drive once and obtains the native
volume path from that retained handle. It verifies the held volume is local,
fixed and NTFS, then pins its complete physical ancestry, including ancestors
above a SUBST target. Subsequent transaction-lock, owner/state-record and marker
opens use that physical path. Private-journal creation follows the same rule.
The original DOS spellings remain diagnostic fields; they are not recaptured
as mutation authority. Marker/journal checks also verify the held file is the
expected child of its held directory. Native record opens avoid .NET Framework
normalization of `GLOBALROOT` paths.

Failed initialization attempts all independent cleanup even if stream disposal
also fails, preserving the original error and any created denial marker. The
focused portable results are 236 private-store and 38 installed-store checks;
the latter includes injected cleanup failures. Windows-only fixtures remap a
temporary SUBST alias after capture, verify the lock/marker or journal belongs
to physical A rather than new target B, and refuse renaming an ancestor above
the captured target. Those fixtures have now executed on the authorized boat PC:
42 installed-store and 245 private-store checks passed under each native x64/x86
Windows PowerShell host. All five transferred source hashes matched `d55641cf`.
See the [exact-source native receipt](../evidence/scrum311-store-boat-native-20261011/README.md).
This validates the tested store behavior, not full loader custody, application
qualification or the separate previous-restoration operation. Portable adapters
alone do not qualify native behavior.

This binds these store operations to retained physical directories. It does
not prevent additions throughout a loader namespace or supply the missing
authenticated signed-update broker. Directory-change notifications are not
exclusive custody, and temporarily changing a DACL does not revoke rights from
pre-existing handles. No `nativeCustodyQualified` claim or signed spawn is
enabled by this store change.

## Authenticated Recovery and exact installer identity

`tools/update-verifier/recovery.go` adds `PrepareRecovery`. It repeats the real
protected-state signed update check and exact eligible-upgrade consent comparison
before downloading the policy's Recovery artifact. A caller-supplied policy or
archive path cannot establish authenticity. The ZIP reader uses the retained
authenticated file handle and verifies the outer signed hash/length again,
bounded archive structure, safe Windows paths, unique names, complete inventory,
CRC and actual member hashes. It derives the executable, PRODUCT_BUILD and
inventory identities from those bytes. The focused signed TLS fixture tests
passed seven top-level groups and 49 subcases; Windows/x86 compilation is not
native execution or product qualification.

The actual installer package manifest is distinct from the outer Setup/Recovery
hashes. After successful NSIS generation and Setup checksum, the producer
`tools/installer_package_receipt.py` retains the **unchanged raw** `package.json`
at Recovery root `installer/package.json`, outside app/docs. It checks the exact
selected compatibility identities, payload and package file list against actual
Recovery bytes, PRODUCT_BUILD and executable/helper hashes, then rebuilds the
Recovery inventory, ZIP and checksum before sealing/signing. App/docs, payload,
Setup and the manifest's bytes remain unchanged; keeping the sidecar outside the
payload avoids a hash cycle. Staged publication failures restore prior outputs,
but this is not a multi-file crash-atomic transaction or hostile namespace custody.
Nineteen focused producer tests pass and are included in the existing release
manifest gate; no extra full application build is requested by that registration.

The reader validates the inventoried sidecar's exact schema, complete app/docs
file list and signed compatibility set, then reports its raw-byte SHA-256 as
`LifecycleManifestSHA256`. Older archives without the sidecar explicitly report
`LifecycleManifestAvailable=false`; callers must require availability for exact
Lifecycle binding. `DeclaredPayloadSHA256` is an authenticated declaration,
because payload.zip is not contained in Recovery. It is not a verified payload
download. Keep the artifact handle alive through separately authorized use.

The installed launcher's actual `Run` preparation now calls `PrepareUpdateSet`
(`7f9ca7d3`). After the user's Update Now choice, it performs one fresh
authenticated check against the exact confirmed policy, validates both artifact
origins, then acquires Recovery before Setup. Both downloads use that same
authenticated policy; a later server offer cannot mix their identities. The set
requires `LifecycleManifestAvailable` before downloading Setup. An older Recovery
without the exact sidecar therefore refuses preparation, preserving the existing
preparation-failure startup behavior rather than inventing a manifest identity.

The preparation owns both read-only artifact handles and the derived Recovery
identity until the existing installer/launcher operation finishes. Installer
timeout retains both handles until that exact child exits. Failure/cancellation
and caller cleanup attempt both independent releases and preserve cleanup
failures. Each artifact's close attempts removal once; repeated closes return
the original result without deleting a later file at the same spelling. This
does not reconcile artifacts left after process loss. The actual launcher must
distinguish uncertain artifact cleanup from
a clean refusal: `e75a2c55` propagates fixed `ErrArtifactCleanup` identity through
progress redaction and withholds startup at `run`. Error text alone is not the
classification. Clean cancellation, offline transport and missing old sidecars
retain ordinary-start fallback; private paths/transport details are not surfaced.
Three additional combined-root tests and seven actual-caller subcases pass;
the deliberately violated Unix pathname-removal fixture does not claim native
Windows sharing qualification. Existing `PrepareUpdate` and `PrepareRecovery` APIs remain available;
neither an arbitrary archive nor a caller-provided policy can enter this set.
Five new combined-root tests and nine subcases pass, including a signed server
offer change during acquisition, invalid/old Recovery, Setup failure, cancellation
and independent cleanup failures. Windows/x86 compilation is not native execution.

The acquisition APIs authenticate and identify material. The separately added
`PreparedUpdateSet.StageRecovery` below performs bounded extraction; neither
operation installs, commissions or provides continuous loader custody.
The product caller now uses the acquisition set, but does not supply
signed-session authority or prove Setup's embedded manifest from its sidecar.
Exact newly built archive/Setup correspondence and installed native/boat updater
behavior remain required gates; isolated fixture results cannot substitute for them.

## Protected candidate staging — source increment, not enabled updater

`PreparedUpdateSet.StageRecovery` can be called only on the original live set
returned by actual authenticated acquisition. A private owner-bound seal binds
the exact artifacts, consent, policy and Recovery identity. Forged/copied sets,
changed public fields and closed artifacts refuse. It rereads the retained
signed outer ZIP, complete inventory, CRC, product identity and exact Lifecycle
sidecar before streaming files. Only the manifest's installed app/docs set is
sent, in lexical order; the portable marker, profile, sidecar and inventory are
not installed files. Every sink call must consume the exact verified length
synchronously. Sink failure, cancellation, incomplete consumption, changed bytes
or identity returns no successful identity. A caller-defined sink is not custody
authority. The production joining code uses the concrete store below.

The original two sidecars are now retained separately through
`RecoveryEvidenceSink`, which must complete original evidence binding before
sealing. The three fixed `installer/engine/{Lifecycle.ps1,UpdateSupervisor.ps1,
UpdateTransaction.ps1}` members are preserved when inventoried. NSIS and Recovery
receive the same fresh pre-compilation snapshots; post-compilation validation
refuses changed bytes. ZIP helper members are nonempty and bounded to 4 MiB each.
The shared validator admits either historical zero helpers or the complete exact
three-member set, and binds every present helper digest to retained observations.
No helper is installed as app/docs or regenerated. Candidate/installed native
namespace and reopen checks preserve this exact distinction. Reconciliation
recognizes only fixed inert residue, including partial helpers, without adoption
or deletion. Historical zero-helper evidence cannot authorize helper execution.
These producer/custody checks neither prove Setup embedded bytes independently
nor supply signing, provisioning, installation, transaction or launch authority.
[Source and focused native/component evidence](../evidence/scrum311-engine-preservation-20261012/README.md)
record the implementation and remaining gates.


The new `brokerhost` operation and `cmd/skager-update-broker` are **source-only**:
the existing launcher and package producer do not enable this service. The SCM
own-process service must run as `NT SERVICE\SkagerUpdateBroker`. There is no
interactive/elevated console fallback, arbitrary root or service CLI installer.
Its fixed executable location uses Windows' actual ProgramFilesX86 KnownFolder
plus `SKAGER\UpdateBroker\skager-update-broker.exe`. The protected data location
uses actual ProgramData plus `SKAGER\UpdateBroker`; environment variables and
client requests cannot redirect either root.

The operation consumes only `prepare-and-stage`, exact update consent and a
fresh 16-byte request ID encoded as 32 lowercase hex characters. Actual local
pipe token/process identity selects `registrations/<SID>.json` in the protected
store. That registration supplies installed version/commit, stock OpenCPN path
and independent update trust/origins. Its bounded JSON rejects duplicate, unknown,
case-aliased and trailing fields. The measured stock executable must match the
existing independently qualified 5.12.4/x86 hash and pinned upstream commit.
It is read and hashed through a retained single-link file handle that denies
write/delete sharing; physical NTFS ancestor handles and DOS-resolution
corroboration remain held through the operation. No executable is run to discover
its version.

An exclusive durable CREATE_NEW candidate reservation occurs before acquisition.
The native store additionally retains a protected `store.lock` data-access handle
with no sharing for its entire lifetime, acquired after actual root validation
and before candidate/registration traversal. Every validation rechecks its exact
physical identity, protected descriptor, volume, regular-file type, single link
and zero length. A matching surviving file is a synchronization object only;
it grants no candidate adoption, cleanup or launch authority. Close attempts all
child/ancestor cleanup before releasing the lock last. This excludes concurrent
store instances. The acquisition-only reconciliation described below runs under
this lock; the lock itself grants no recovery authority.
The broker's private trust/artifact paths must already exist and pass the
existing verifier's protection checks; staging never initializes lost trust or
adopts a cache. One fresh real `PrepareBrokerUpdateSet` then supplies both artifacts
and `StageRecovery` streams to the concrete protected candidate. The store is
revalidated around acquisition, extraction, cleanup and sealing. Both artifact
cleanup attempts must succeed before a bounded staging receipt is published.
Owners are released and final validation passes before a success response.
`Candidate.Close` and `Store.Close` attempt their owned cleanup only once and
retain the complete original result. Repeated calls cannot turn cleanup
uncertainty into success. Concurrent store closes wait for the first drain to
finish and return that same result; the store becomes unavailable before its
children drain. These semantics release handles, not durable reservations, and
do not classify abandoned candidates or authorize their deletion/reuse.
Failures return fixed redacted refusal and no successful result. Partial
reservations remain protected and cannot be retried with the same ID; there is
no wildcard deletion or eviction. Candidate payload recovery/adoption remains
separate from the narrowly indexed disposable-artifact cleanup below.

### Broker-owned acquisition and interrupted-download cleanup

The actual broker passes the same newly reserved concrete native `Candidate` to
`PrepareBrokerUpdateSet`; a caller-selected directory or reconstructed receipt
cannot replace it. `BeginAcquisition` creates a private `.acquisition/lease.jsonl`
before metadata networking. After one authenticated check, exact consent and
both installed-origin checks, the lease binds policy hash, version, commit and
both signed artifact sizes/hashes. Fixed `recovery.zip` and `setup.exe` slots are
created under broker/SYSTEM-only protection before receiving bytes. Recovery's
exact Lifecycle sidecar is verified before Setup is acquired.

The bounded canonical journal records creation intent, actual full native file
identity and deletion intent/completion. Stage, client, parent and acquisition
directory identities remain bound throughout. The actual writer is replaced by
a read-only data handle with retained deletion custody; the borrowed reader must
not be closed or removed independently. Cleanup deletes through that verified
native handle and retains all close/deletion uncertainty across repeated drains.

Actual store startup inspects only indexed fixed acquisition slots under its
exclusive lifetime lock. It opens and validates both present artifact owners
before deleting either. Matching indexed partial files may be deleted; an
already absent file requires prior durable delete intent. Unknown files,
creation without durable actual identity, altered ACLs, hardlinks, mismatched
identities or uncertain journals refuse readiness. There is no recursive sweep,
payload deletion, candidate adoption, request-ID reuse or execution. Completed
lease journals remain inert tombstones and reservations still count toward the
existing cap.

Focused native temporary-file fixtures cover these primitives, including three
interrupted states and six refusal cases. They use the current test account and
do not establish actual service-account/ProgramData startup, privileged
provisioning, authenticated installed acquisition or updater acceptance. See
[exact source and component evidence](../evidence/scrum311-broker-acquisition-20261012/README.md).

The only successful state is **staged-awaiting-qualification**. It is neither
installation nor readiness, launch, startup success or a healthy receipt. The
operation has no candidate-code execution, selected-generation mutation,
elevated Lifecycle call, user-plugin/PATH import or actuator path. A receipt
after a later close/namespace failure still describes only staged bytes; it
cannot authorize downstream execution. Transport request replay memory is not
durable authority; the protected candidate reservation provides ID non-reuse.

Production source `67665e18` joins the original independently approved registration
to a concrete protected initial baseline. Fresh retained operator enrollment
publishes the fixed SID request and baseline ZIP; the stopped service authenticates
and imports the exact manifest app/docs payload before registration completion.
`brokerstore.OpenInstalledBaseline` validates the completed creation record,
original sidecars/product bytes, independently approved installed pins and exact
physical namespace. The host retains that read owner through candidate staging.
This replaces the earlier version/commit-only installed baseline boundary; it
does not establish a selected previous generation or transaction destination owner.
Returning a StageID after closing the candidate cannot reconstruct that custody.
The existing per-user Lifecycle and supervisor refuse armed mutation/launch;
their CurrentUser DPAPI health proof cannot be adopted by the non-impersonated
broker account. The missing installation bridge must establish those native
bindings through authenticated provisioning and retain the original owners.
Neither invoking per-user Lifecycle as the service nor adding a general armed
exception supplies that authority. SCRUM-311 remains open for this integration.

Originally protected provisioning is mandatory: inspecting today's DACL cannot
exclude handles acquired before protection. Actual native service identity,
cross-account process/token query rights, IPC, creation-time ACLs, namespace
behavior and integrated extraction require Windows qualification. The fixed DOS
verifier paths are corroborated under the distinct broker account; their DOS
namespace is not claimed continuously pinned. Missing/ambiguous setup refuses.
Service shutdown cancels and drains work; an undrained timeout exits the own
process without closing owners concurrently with the active handler. Authenticated
service provisioning/upgrade/removal, launcher activation, full loader dependency
custody, actual installation and exact-source Windows/boat update qualification
remain separate unfinished SCRUM-311 gates.

## Authenticated initial installed baseline — source and inert component evidence

The initial baseline path is fixed to
`ProgramData/SKAGER/UpdateBroker/users/<SID>/installed/<generation>`. A permanent
reservation precedes fresh creation; existing/partial trees are never adopted,
deleted or repaired. Final payload descriptors grant only broker/SYSTEM mutation
and approved client read/execute. Native owners retain every directory/member,
check exact namespace and creation identity, and refuse additions, replacement,
reparse points, hardlinks and aliases. The completed marker binds the independently
approved request and exact durable completion record; close errors are retained.
Late cancellation after successful durable publication cannot mean non-commit.
A real publication/custody failure may be uncertain even if valid durable bytes
exist and must not trigger automatic retry or rollback.

`ImportApprovedBaseline` reparses the exact original request against the separately
authenticated approval hash and streams only the original manifest's app/docs
payload, excluding the portable marker. Original `FILE_SHA256.json`,
`installer/package.json` and product evidence keep their original byte identity.
Shared `baselineevidence.Validate` checks these semantics during import and
retained reads. `Installed.PackageSHA256` means the original Lifecycle package
manifest hash; archive copy hashes and a declared payload hash cannot silently
replace it. Recovery launcher/profile/root members do not become installed
payload. Mutable LocalAppData state is neither approval nor proof of the running
application. The stopped `register-client` path takes only SID and approval hash,
returns before `Running` or pipe announcement, and has no caller-selected path.

[Exact source and component evidence](../evidence/scrum311-installed-baseline-20261012/README.md)
records production `67665e18`, test-only Windows mask fix `f2a9e051`, **131 top-level
+ 477 nested Linux component passes**, **68 session-reported Python packaging
passes**, Windows/x86 component compilation, and **10 top-level + 6 nested native
inert passes**. Both original compiler and local evidence-extraction failures are
preserved; correcting local extraction did not rerun native tests. The running
OpenCPN process stayed responsive; no owned test process remained.

This increment provides no selected/pending generation transition, successor
promotion, interactive user-side authenticated baseline/launch handoff,
commissioning or healthy-start handshake. A baseline directory is informational,
not launch authority. Actual privileged first execution/SCM provisioning and
real update/fallback transaction qualification remain required. SCRUM-311 and
release acceptance stay open; passing inert helpers cannot arm Lifecycle,
substitute broker account custody for per-user DPAPI proof, or authorize launch.

## Protected application materialization boundary

The executable application view is separate from the private broker metadata:
`ProgramFilesX86/SKAGER/Applications/<SID>/generations/<StageID>`. Fresh machine
enrollment creates the fixed `Applications` boundary with final protected
Administrator/SYSTEM/broker mutation and authenticated-user read/traverse rights.
It does not broaden the broker executable's permissions or expose private
ProgramData ancestors. A missing or previously unproven boundary is not repaired
by normal startup.

Materialization takes the **live retained** authenticated candidate and original
approved baseline, not paths or reconstructed receipts. It copies only the
complete original Recovery payload, inventory, manifest and execution helpers
into fresh protected objects. The signed-candidate completion binds its policy,
Recovery and Setup identities separately from the original operator approval;
these hashes are not interchangeable. Partial attempts remain reserved. The
broker operation performs the copy before releasing baseline/candidate custody.

This completion describes copied application bytes. It does **not** select a
generation, alter the real profile, commission hardware, authorize a child, or
prove healthy startup. The interactive transaction must still persist pending
state before selection, consume spawn authority before suspended creation,
retain loader/process custody through authenticated readiness, and preserve the
real restoration/commissioning sequence for candidate and fallback. The older
per-user Lifecycle mutation path cannot stand in for that integration.

### One-way read-only generation transfer

`InstalledGeneration.TransitionReadOnly` now closes each original data writer
and opens a distinct read-only native file object. Retained metadata anchors,
physical parent identities and original creation-time protection span the gap;
the anchor is not claimed to be a write fence. Protected-at-birth broker/SYSTEM
mutation rights and serialized store writers supply that boundary. The transition
also transfers original private history, reservation and completion files. Exact
IDs, contents, descriptors and inventory are rechecked before and after transfer.

The original writer is permanently revoked. A live `ReadOnlyGeneration` supplies
bounded, revocable member reads and informational identities, with no raw native
handle or executable path. Store/original-owner closure revokes all views. A
failed transition poisons/drains once, retaining cleanup uncertainty and completed
bytes; it cannot be retried or reconstructed from a completion receipt.

[Native component evidence](../evidence/scrum311-generation-readonly-20261012/README.md)
proves actual image-mapping/read-execute compatibility and new write/delete
refusal on temporary objects, not an application launch. The immutable snapshot
of sibling history is still too restrictive to retain an older loading owner
through later generation publication. Authenticated history evolution and the
initial approved baseline public view remain necessary, along with the actual
private selection/interactive commissioning/startup/fallback transaction.

## Inactive broker package prerequisite

`tools/package-update-broker.py` is a native-Windows-only producer for the fixed
Win32 broker binary and build record. `tools/broker_package.py` verifies the
copied binary, actual embedded Go metadata, GUI PE format and unchanged
same-revision launcher source descriptor. The native probe uses a separate
`broker-install` copy; the original launcher-only output and application
package transfer remain unchanged. This does not install or enable a service.

The existing source archive includes the broker command, host, build scripts
and required shared updater verifier. Its retained provisioning checksum file
supports an offline cache audit; the actual compile remains in the clean
product module. No second source archive or dependency download is required.
Go's pinned trimpath build omits linker flags from embedded metadata: the fixed
recipe and actual PE subsystem are checked separately rather than requiring
an impossible metadata field. Local hashes and metadata are consistency only;
the caller must authenticate the successful same-revision producer. See the
[source/component evidence](../evidence/scrum311-broker-package-source-20261012/README.md).

## Shared commissioning profile exclusion

`CommissioningProfileLock.cs` supplies a native, machine-wide mutex for the
cooperating boat tools. Its key uses the held physical profile directory's full
volume/file identity (`FILE_ID_INFO`) plus the fixed `opencpn.ini` leaf. The
workspace, DOS drive alias and replaceable INI file identity are not keys.
Acquisition holds plain, non-reparse directory ancestors and a protected
owner/SYSTEM mutex without modifying profile bytes or filesystem permissions.
Missing/ambiguous ancestry, contention, in-process recursive acquisition and an
observed abandoned acquisition refuse.

Actual Apply, InspectRestore and Restore take this exclusion before their final
context checks and retain it through completion/failure. Installed direct cold
launch and startup-launch observation retain it from before the final audit
through child readiness; the guarded restart broker holds it after proven normal
parent exit through audit, ALLOW and child identity receipt. Stock coexistence
launch also uses it. The new C# dependency is pinned in each fresh restart tool
inventory. The existing transaction-specific restore lock and all durable
commissioning checks remain required.

This is stable-path exclusion among these cooperating entry points, **not**
complete audit-to-spawn custody or signed-flow completion. Callers still use DOS
paths for their I/O; malicious retargeting after acquisition is not prevented by
this mutex. Directory handles do not exclude new loader children, and other
processes/entry points that do not take this lock are not serialized. An observed
abandoned acquisition is refused, but Windows mutex abandonment is transient:
a later successful acquisition does not prove recovery or authorize launch.
Existing durable commissioning/armed markers must still be inspected, never
inferred away from lock state. This adds no signed-session bypass or recovery
permission.

The existing commissioning gate now invokes
`test-commissioning-profile-lock.ps1`. Its native fixtures cover independent
process contention, same-process recursion, distinct profiles, atomic INI
replacement, directory replacement refusal, SUBST alias identity, reparse and
malformed/missing paths, owner loss, original bytes and unchanged directory ACL.
Only its original inert PowerShell fixture child may be force-cleaned on a
fixture timeout; product launch/close policy is unchanged. Linux compiles the
exact C# and verifies unsupported-platform refusal, with native behavior
explicitly unexecuted until the combined Windows gate.

## Four transitions

| Receipt | Required real operation before recording it | Operation that may follow only through the still-unimplemented native boundary |
| --- | --- | --- |
| `PreviousRestored` | Close the old application normally; finish its exact InspectRestore/Restore transaction with reviewed settings-preservation evidence, while its generation remains selected. Verify restored trees/profile and removal of the active marker. | Run the exact authenticated installer for the confirmed successor. |
| `CandidateCommissioned` | With the actual successor selected, inventory the resulting loader trees, complete fresh source review/Inventory/Prepare/Apply, and obtain a new independent target audit and approval. The new baseline must equal the just-restored profile. | Create the exact successor application process once, with the audited working directory and PATH. |
| `CandidateRestored` | After unsuccessful startup closes normally, inspect and restore that candidate's own commissioning transaction, preserving reviewed current user state, while the candidate is still selected. | Change the pointer back through the exact guarded rollback transaction. |
| `FallbackCommissioned` | With the previous generation selected again, perform a new inventory/commissioning/audit/approval over its current profile and complete actual loader trees. Its baseline must equal the just-restored candidate profile. | Create the exact fallback application process once. |

### Concrete previous-generation restoration

`SignedUpdatePreviousRestore.ps1` now implements the first operation through the
existing `commission-read-only.ps1` engine. `Inspect-SignedUpdatePreviousRestore`
requires a fresh live operation-event store and the exact prepared transaction
selected in its binding. It verifies the currently selected owned generation,
context and physical installed root before retaining the real engine inspection.
`Invoke-SignedUpdatePreviousRestore` requires that inspection and its exact
reviewed current-INI hash. Substantive user-setting changes require the existing
independently reviewed session-preservation proposal; they cannot be erased by
restoring an old baseline automatically.

The real Restore runs under the installation transaction lock. After the engine
returns, the operation reacquires shared profile exclusion and independently
checks its completion, restored profile bytes and permissions, complete profile
and plugin trees, returned quarantine files, absence of the active commissioning
marker and unchanged selected generation/state/ownership. Only then does it
derive and append `PreviousRestored` to the live store. Failure after starting
Restore poisons that store and retains the installed denial marker; even an
actually completed restoration cannot publish a success event after failed
postchecks or journal persistence. A lock-release failure also poisons custody,
attempts both independent releases and withholds the success result. Any event
already appended remains consumed; cleanup cannot turn it into retry authority.

The portable suite passed 55 assertions across 12 actual temporary-file
transactions against the integrated store. It copies the exact transaction
scripts and substitutes only unsupported Windows identity/ACL/mutex/handle
boundaries; Restore and preservation/journal logic are real. It is registered
in the portable signed-session gate. A separate fresh-host native fixture now
passes 106 assertions under each Windows PowerShell 5.1 x86/x64 host on the boat
PC (212 total). It retains real native owner/store/handles/ACL/SACL/mutex and
transaction code, with explicitly synthetic bounded context, closed-process
observation and baseline. The unmodified public baseline refusal is checked
before that test-only substitution. See the [exact-source native receipt](../evidence/scrum311-previous-restore-native-20261011/README.md).
The native Staging qualifier registers that fixture in separate processes;
actual installed-context and integrated updater qualification remain pending.
The engine still uses DOS paths,
and its profile-lock release/reacquisition leaves a checked gap: this is not
continuous namespace custody. The operation is not connected to product
startup and cannot install, create/terminate a process or grant launch authority.

A healthy successor ends after receipt 2; it does not need a fictitious rollback.
Receipt 4 ends the fallback branch. The in-memory session permits no additional
transition. Refused requests leave it unchanged. Copies returned to the caller
cannot mutate the retained receipt. An installed version, prior healthy receipt,
disabled plugin flag or package signature never replaces commissioning evidence.

## Opt-in operation journal foundation

`SignedUpdateOperations.ps1` now wraps the unchanged four-receipt reducer with
one bounded event sequence and head. It models the denied-before-launch branch;
it does **not** establish a broker, authenticate an artifact, prove that no
process started, or perform restoration. Only a future trusted broker with
continuous native custody may supply evidence after verifying the real operation.
The existing receipt-only APIs remain available for their original inert model.
They must not be used as a substitute for the operation boundary during future
signed-flow integration.

`New-SignedUpdateStore(directory, bindingJson, -OperationEvents)` opts into
schema-2 frames. `Add-SignedUpdateStoreEvent(handle, requestJson)` then accepts a
closed request containing `session`, integer `sequence`, `action`, unique
`requestNonce`, `previousEventSha256`, and `evidence`. Every operation **and**
normal receipt advances that same event sequence/head. The receipt-only append
API refuses operation stores, and operation appends refuse receipt-only stores.
The original four receipt bodies keep their original sequence and hash chain
inside the operation state. A returned event is a consumed journal record, never
an allow token. The initial event head equals the binding hash; subsequent
requests bind the preceding returned `eventSha256`.

The only paths are:

| Current state | Permitted next event |
| --- | --- |
| New session | `PreviousRestored` (normal receipt 1) |
| Previous restored | `CandidatePreparationArmed` |
| Preparation armed | `CandidateCommissioned` (receipt 2), or irreversible `DeniedBeforeLaunch` |
| Candidate commissioned | `CandidateSpawnConsumed` |
| Candidate spawn consumed | `CandidateRestored` (receipt 3), after independently proven normal closure/restoration |
| Candidate restored | `FallbackCommissioned` (receipt 4) |
| Fallback commissioned | `FallbackSpawnConsumed`, terminal |
| Denied before launch | `DeniedRestoreIntent` |
| Denied restore intent | `CandidateDeniedRestored`, terminal |

A healthy successor needs no invented restoration after its consumed spawn.
For the narrow denied branch, even receipt 2 without a consumed spawn is refused:
it must use a separately designed recovery path, never be relabelled as denied.

Preparation binds the exact authenticated successor, selected state/ownership,
prepared transaction hash and bounded local transaction-directory spelling,
context, plan, inventory, quarantine, source review, and receipt 1's restored
profile baseline. It must be durably consumed **before Apply**. Receipt 2 must
match that preparation. Spawn consumption binds the exact commissioning receipt,
creation-request hash and audited environment; it must be durably consumed
**before any native creation or ALLOW transmission**.

Denial references the armed preparation and refusal evidence. Restore intent
references that denial and the same candidate/transaction, pins inspection,
current profile, target profile and complete target-tree hashes, and binds an
explicit preservation proof when choosing a different profile from the original
restored baseline. Completion must match those target/inspection hashes and
record removal of the active marker. The broker still has to inspect the actual
files, closed processes, locks, ACLs, selected pointer and marker absence: hash
strings and `activeMarkerAbsent=true` cannot independently prove them. No
successful Apply or commissioning receipt is fabricated for partial Apply.

A denied completion leaves only receipt 1 in the normal chain. It grants no
pointer rollback, fallback commissioning, launch, disarm or automatic retry.
Failure before an active transaction exists remains an attention case; this
foundation does not fabricate restoration of an unstarted mutation.

All requests replay-check the bounded history before mutation. Both append APIs
share the existing owner/ACL/handle verification, strict JSON parser, durable
write/flush and current disk-head checks. Failed writes poison live custody;
whole-journal rollback, changed owner, expiry and owner loss refuse. A consumed
spawn with a missing response or unrecorded child cannot become an unlaunched
denial. Losing any response does not permit replay of its nonce/head. Disk
inspection can replay events for verification, but cannot resume live custody.

The operation foundation has portable inert coverage in
`test-signed-update-operations.ps1` and the extended store tests. Its native
PowerShell 5.1 x86/x64 qualification remains pending; the earlier native store run
above predates this addition. Actual broker/IPC, shared commissioning mutation
exclusion, selected-generation custody, artifact verification, every spawn hook,
and interrupted-process recovery remain unimplemented. No product caller uses
this mode, and this change does not qualify live signed updates.

## Data contract

`New-SignedUpdateSession(binding, now)` pins a random 64-hex session identity,
previous generation/commit/package/executable hashes, its existing commissioning
record, confirmed release commit/package/executable/policy/source-review hashes,
a reviewed fallback source boundary, and a maximum four-hour lifetime. The
successor generation ID is assigned by installation; receipt 2 must bind that
new ID to the already confirmed release hashes.

`Add-SignedUpdateReceipt(session, request, now)` takes exactly the session ID,
next integer sequence, action, unique 64-hex request nonce, previous receipt
hash and evidence object. Restoration evidence binds the selected generation,
state/ownership hashes, exact prepared transaction, restoration completion,
inspection, restored profile and restored complete trees. Commissioning evidence
additionally binds its new prepared/applied records, preceding restored baseline,
current input-only profile, actual trees, source review, independent audit,
explicit approval, child environment and fresh review time. Receipt 4 cannot
reuse receipt 2's prepared/applied/audit/approval evidence or the original
commissioning transaction.

The transport parser must reject duplicate or case-folded JSON keys before
constructing these closed objects. The native caller must read/hash/verify every referenced artifact and its
contents. Merely supplying plausible hash strings is insufficient. In particular,
the approval must identify the exact current generation, profile, reviewed tree,
source evidence and environment represented by the request. It must not be a
boolean or an executable callback selected from untrusted metadata.

The reducer provides ordering and replay rejection against the **current
in-memory journal**. The native orchestrator must own an exclusive, ACL-protected,
crash-safe journal; pin its current head in the armed session; persist a consumed
transition before any allow/operation; authenticate caller SID/session/process
creation time/image and nonce; and reject stale on-disk journal replay. Missing
broker, binding, journal, proof or timeout after arming must remain a denial.
Restarting a launcher must not silently become an unguarded normal launch.

## Concrete integration hooks still required

Current source is `tools/update-verifier/launcher/launcher.go`,
`installer/windows/UpdateSupervisor.ps1` and `installer/windows/Lifecycle.ps1`.
The code names below are stable hook descriptions, not patched behavior.

1. In Go `run`, after successful authenticated `prepareWithProgress` and the
   final `assertCurrent`, but before invoking the installer, require receipt 1.
   Download failure or Later occurs earlier and must use the existing generation's
   complete read-only launch audit through a guarded `ordinary` path. Once old
   commissioning is restored, no old application may start without fresh setup.
2. In `Invoke-UpdateSupervision`'s `LaunchPending` branch, verify the exact
   candidate using `Get-SupervisedGeneration`, then complete receipt 2 **before
   `New-UpdateStartupSession` increments attempts and outside the catch block
   which automatically initiates rollback**. Approval refusal is an attention
   state, not a failed candidate launch. Retain pending identity without starting
   the application, counting an attempt or changing the pointer.
3. After `Stop-SupervisedProcess`/`Stop-InterruptedUpdateCandidate` proves normal
   closure, require receipt 3 before `Invoke-UpdateGuardedRollback`. Lifecycle
   must independently require the same restored commissioning lineage while its
   transaction lock is held, before its rollback `state.json` publication. The
   initial update publication similarly needs proof that the old transaction was
   restored. Direct maintenance calls must not bypass the armed boundary.
4. Gate **every** application creation: Go `ordinary`'s `p.start` (including both
   `launchInstalled` and `recoverStartup` fallback calls), and PowerShell
   `Start-SupervisedGeneration` (including `QualifyCurrent`). Assign the verified
   environment at the actual spawn, not after observing a child. Receipt 4 is
   required for fallback; an old known-good receipt supplies health identity only.

For each audit, reuse `Assert-ReadOnlyAudit` and
`verify-commissioning-launch.ps1`. They verify actual normal-profile input-only
bytes, independent boat target, active prepared/applied commissioning lineage,
source evidence, complete loader trees and quarantine, and return the exact
child working directory/PATH. The environment must be bound to the same
session, retained identities and one-use creation request. The product's
startup-health receipt remains a separate post-start identity/readiness proof.

## Important implementation constraints

- `RestartCommissioningBroker.ps1` is not a cross-generation broker.
  `Read-RestartSession` pins selected generation, build, executable/helper,
  product-build proof, target audit and whole context. Its one-use framing and
  peer verification can inform a new boundary; changing those pins in place
  would invalidate existing commissioning.
- `Lifecycle.ps1` calls `PreserveAdditions` for both stock plugins and the prior
  generation before writing successor ownership. Only that resulting inventory
  can be commissioned; the signed payload alone is incomplete.
- `Get-SupervisedGeneration` verifies **all** declared generation files.
  Commissioning may subsequently quarantine a plugin DLL. Preserve the full
  immutable-generation check **before** Apply/quarantine, then validate the
  complete reviewed tree minus only proven quarantine moves immediately before
  spawn. Do not generally waive missing owned files or rewrite ownership. Future
  startup/recovery must understand the exact still-active commissioning lineage
  or fail closed until it is restored.
- Old restoration returns `doNotAutoLaunch=true` and may restore original output
  configuration. Its success authorizes no application startup. Pointer changes
  invalidate the old commissioning context, so restoration must precede them.
- Installer self-tests execute a separate profile/plugin-free path. Their existing
  exact-package qualification remains necessary; this model does not authorize
  additional application processes under a generic 'self-test' exception.
- The bootstrap `StartupLauncher.ps1` trust/pending refusals and retained
  `state.json` handle remain unchanged. A separate armed signed-flow path needs
  bounded handoff of state custody, not removal of those protections.

## Focused verification and remaining gate

`pwsh -NoProfile -File tools/boat/test-signed-update-session.ps1` is the existing
portable/native helper entry point. It runs the receipt suite, then the operation
and store suites in separate script scopes. Receipt and operation checks are
in-memory; the store uses only disposable temporary files. No product build is
required. Coverage includes complete candidate/fallback order, identity and source
mismatch, restored-baseline continuity, missing proof, reused approval, replay,
skipped steps, unknown fields, expiry and unchanged-on-refusal behavior.

`tools/qualify-staging-windows.ps1` also invokes that entry point under both
native Windows PowerShell 5.1 x64 and x86 during Staging qualification and
retained-package retests. Each host must succeed; logs are retained in
`evidence/local/signed-update-custody/`. This wiring adds no installation or
application launch behavior and is not evidence that the native checks passed.

The 2026-10-11 helper integration onto monorepo `bf68b355` takes only the product
helper/contract deltas from `43972f2b` and `09e2d055`, plus the negative fixtures
from `e738891d`; it excludes their workflow changes and unrelated ancestors.
The combined entry point passed **61 receipt, 129 operation and 128 store checks**
on local PowerShell 7/Linux (`nativeWindows=False`). Portable store fixtures
substitute only the unsupported native boundary; these results do not qualify
Windows ACL/share/reparse custody. The historical `43972f2b` run above likewise
cannot qualify the later operation code or this combined revision. The added Go
launcher negatives were not run in that local check because a Go compiler was
unavailable; their execution is separate from the signed-helper results below.
Their separate verification still requires the module's pinned toolchain for
`go test ./launcher -run 'TestInstalled' -count=1` and `go test ./launcher`.

The later [retained Windows retest 38140837201](https://github.com/ThereptileII/Work/actions/runs/38140837201)
passed **61 receipt, 129 operation and 245 store checks on each of Windows
PowerShell 5.1 x64 and x86**, including the denied-recovery fault fixtures below.
It also passed **16 artifact-content tests** and the complete selected qualifier
(`scope=all`, `checks=[all:passed]`). These helper results belong to harness
`efec7a4e19d8e36298f4d23db175540b7b2ede10`; the retained compiled product remains
`5dd795be16081b4b0e521315bfc45f23a9af8251`. The retest did not rebuild or relabel
that product, and does not qualify later product revisions. See the
[retained-run provenance](../evidence/2026-10-11-retained-staging-retest.md).

Native broker/orchestration, durable-journal crash and replay tests, stale/racing
loader/profile proof tests, exact successor/fallback marker-process fixtures and
live signed flow, private HTTPS delivery, actual commissioning and boat
qualification remain unimplemented/unqualified as an integrated updater. The
component acquisition and restoration evidence above does not close those gates.
No claim of safe live Update Now
or automatic fallback follows from this model passing.

### Denied-recovery persistence fault fixtures (2026-10-11)

The existing store test entry point now adds eight disposable-journal cases:
`DeniedRestoreIntent` and `CandidateDeniedRestored`, each with failure before
write, a torn 17-byte append, failure after durable flush but before return, and
a successfully consumed result deliberately discarded by the caller. These
extend the earlier spawn-consumption faults to the denied recovery branch.

The assertions distinguish untouched, torn and complete journal bytes, retain
the exact prior frame prefix, replay complete frames for inspection only, and
require the normal receipt chain to remain at receipt 1. A storage exception
must poison live custody without publishing the proposed in-memory state.
Replayed requests, a fresh nonce at the current sequence, candidate/fallback
commissioning, and reopening the abandoned journal must all refuse. Closing
custody must retain the exact fault evidence. The already-existing successful
denied-branch fixture remains the positive control for intent then completion.

These are helper persistence fixtures, not process interruption or physical
restoration tests. They do not validate the artifact inspector's snapshots,
claim that no child started, perform Apply/Restore, or authorize recovery or
launch. The content inspector remains a separate read-only boundary without
native retained custody. No production caller or workflow changes were made.
Their initial [source-check record](../evidence/scrum311-denied-recovery-faults-20261011/results.json)
retains the historical local execution gap: PowerShell was unavailable and no
runtime was downloaded. The later retained retest above executed these cases
successfully as part of the 245 store checks on each native host. This resolves
that fixture execution gap at harness `efec7a4e`; it does not qualify live
interruption, restoration or launch behavior in the retained product.

### Source-bound implementation handoff

The actual signed boat boundary is deliberately still unhooked. Current
`ordinary` startup supplies no authenticated handoff for retaining the audited
profile/tree/environment through process creation; `Start-SupervisedGeneration`
consumes a startup attempt before such a review can finish. The same-generation
restart broker cannot transfer its proof to another generation. A finite private
broker needs native one-use audit/spawn custody and interrupted/denied review
handling: a candidate commissioning transaction can be partly applied before
receipt 2 exists, and must then be restored under the candidate pointer without
inventing a successful commissioning receipt. The four-receipt normal-branch
model alone cannot establish this missing recovery authority. The opt-in operation
journal above models denial/restoration ordering, but the native broker must still
prove uninterrupted no-spawn custody and validate the actual partial transaction.
No callback, state-handle release, generic approval flag or bootstrap guard
relaxation was added.


## Native supervisor image owner increment (2026-10-11)

`UpdateTransaction.ps1` now contains a reusable retained image/physical-ancestry
owner used by the real PowerShell supervisor's creation, authenticated health and
normal-cleanup path. This implements a bounded native image/CWD predicate;
it supplies no receipt-reducer approval, launch authorization or full
`nativeCustodyQualified` assertion. It uses suspended native creation and checks
identity before resume, rather than validating after user code can execute.
Original DOS record validation and all startup protocol checks remain in place;
known-good receipt creation additionally requires that live creator owner.

Distinct `nativeLaunchV1`/`processCreationFileTimeV1` fields retain uncertainty
across interruption without changing the meaning of historical
`processStartTicks`. An attempted historical or armed native record refuses
automatic recovery; current-host positively pre-create refusal may still take
the existing guarded rollback. Fresh interrupted checks only restrict normal
close and are not reconstructed historical custody.

Portable policy/source-compilation checks pass. Native run
[38163791374](https://github.com/ThereptileII/Work/actions/runs/38163791374)
passed acquired image/ancestor protections, then refused actual creation with
Win32 87 before resume. The physical application/CWD and other parameters need
the bounded suspended-only diagnostic; no cause or creation acceptance is inferred.
Actual Windows creation, alias-remap launch, health and cleanup remain pending.
The subsequent suspended-only diagnostic
[38166517938](https://github.com/ThereptileII/Work/actions/runs/38166517938)
separated application spelling from argv[0] and CWD: eight conventional DOS
application rows succeeded, five GLOBALROOT application rows failed with 87,
and all four held-file-derived volume-GUID application rows failed with 87.
All children remained suspended and were cleaned up. This rules out the tested
GUID replacement; it does not identify the exact Windows internal cause.

The reviewed compatibility adjustment (`7e133776`, `255fc30c`) selects exactly
one conventional DOS application spelling from the held image handle with
`GetFinalPathNameByHandle` flags 0. It never uses the caller's original alias as
a fallback and never retries creation with another name. The selected name must
have a safe local-drive grammar and match the held native volume suffix.
Every owner recheck corroborates its DOS root and image against the retained
physical NTFS root/image identities. The DOS namespace is **not pinned**:
suspended-child native identity, liveness and raw-handle checks remain mandatory
before the single resume. Physical GLOBALROOT argv[0]/CWD, held image/ancestor
locks, one-use state, authenticated health and uncertainty/cleanup policy remain
in force. This is a compatibility adjustment to application spelling, not
mapped-section, general DLL/resource or environment custody. Focused native
PowerShell 5.1 x64/x86 and Go Win32 qualification first ran in
[38167390164](https://github.com/ThereptileII/Work/actions/runs/38167390164):
Go passed, while the PowerShell test stopped at normal close of its hidden,
owned inert fixture window before health checks; the x86 driver failed before
starting the harness. [The audited failure](../evidence/scrum311-owner-native-38167390164/README.md)
is retained. Fixture-only correction `3130b3c9` exposes a discoverable unowned
window while retaining the actual normal-close gate; the workflow now gives the
x86 host a named `.ps1` file. In
[38167897619](https://github.com/ThereptileII/Work/actions/runs/38167897619),
both owner harnesses passed 79 assertions, including health and cleanup;
Go Win32 passed and the x64 transaction completed. The x86 transaction stopped
later at removal of its compiled inert application; no complete x86 transaction
receipt exists. [The audited partial result](../evidence/scrum311-owner-native-38167897619/README.md)
remains an overall failure. Test-only correction `8ed247a5` compiles that
transaction application explicitly as x86, verifies PE32/CLR 32BITREQUIRED and
preserves the first error if cleanup also fails. Native confirmation remains
pending at that increment; production owner/cleanup source is unchanged by this
correction. Final focused confirmation
[38168539396](https://github.com/ThereptileII/Work/actions/runs/38168539396)
passed at exact `b08f9a1cf3360ff10e075b8b0134e5c39ac15f73`: both PowerShell
host widths completed owner and transaction checks, and Go Win32 completed its
required actual-creator/refusal/SUBST fixtures without skips. The
[audited native receipt](../evidence/scrum311-owner-native-38168539396/README.md)
retains exact inputs and architecture evidence. This qualifies the selected
image-owner mechanics, not the broader commissioning/custody boundary below.
The separate [ordinary Go creator increment](../evidence/scrum311-held-ordinary-image-20261011/README.md)
covers ordinary/fallback source paths, with Linux/cross-compile evidence only.
Lifecycle self-test, helper/script/shortcut roots and loader/profile/environment
custody remain outside these increments; the authenticated four-stage commissioning and
mutation/restoration owner is still required. [Evidence and exact remaining
native checks](../evidence/scrum311-native-launch-identity-20261011/README.md).
