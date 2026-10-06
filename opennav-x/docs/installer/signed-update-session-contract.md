# Signed boat update session contract (SCRUM-311)

This is an implemented **pure receipt model** with a separately qualified,
non-resumable journal store, not an enabled boat update path.
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
This does not implement IPC peer authentication, actual artifact verification,
installer/spawn hooks, selected-directory policy or interrupted-update recovery.
Those integrations must reject missing/abandoned custody once armed, including
ordinary startup; the current product does not yet call this store.

## Four transitions

| Receipt | Required real operation before recording it | Operation that may follow only through the still-unimplemented native boundary |
| --- | --- | --- |
| `PreviousRestored` | Close the old application normally; finish its exact InspectRestore/Restore transaction with reviewed settings-preservation evidence, while its generation remains selected. Verify restored trees/profile and removal of the active marker. | Run the exact authenticated installer for the confirmed successor. |
| `CandidateCommissioned` | With the actual successor selected, inventory the resulting loader trees, complete fresh source review/Inventory/Prepare/Apply, and obtain a new independent target audit and approval. The new baseline must equal the just-restored profile. | Create the exact successor application process once, with the audited working directory and PATH. |
| `CandidateRestored` | After unsuccessful startup closes normally, inspect and restore that candidate's own commissioning transaction, preserving reviewed current user state, while the candidate is still selected. | Change the pointer back through the exact guarded rollback transaction. |
| `FallbackCommissioned` | With the previous generation selected again, perform a new inventory/commissioning/audit/approval over its current profile and complete actual loader trees. Its baseline must equal the just-restored candidate profile. | Create the exact fallback application process once. |

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

`pwsh -NoProfile -File tools/boat/test-signed-update-session.ps1` runs only inert
in-memory contracts, including complete candidate/fallback order, identity and
source mismatch, restored-baseline continuity, missing proof, reused approval,
replay, skipped steps, unknown fields, expiry and unchanged-on-refusal behavior.
Native broker/orchestration, durable-journal crash and replay tests, stale/racing
loader/profile proof tests, exact successor/fallback marker-process fixtures and
live signed-offer boat qualification remain unimplemented/unqualified. No claim
of safe live Update Now or automatic fallback follows from this model passing.

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
