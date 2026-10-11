# Boat boundary for signed updates

SCRUM-311 separates an authenticated update package from permission to start it
on the boat. The existing bootstrap qualification does not qualify live signed
offers, Update Now, or automatic rollback under boat commissioning.

`tools/boat/StartupLauncher.ps1` intentionally rejects configured update trust
and pending updates. It retains the selected `state.json` against replacement
and verifies one exact commissioned generation. Those guards must remain.

The installed Go launcher runs the authenticated installer after Update Now,
then immediately invokes `UpdateSupervisor.ps1 -Action LaunchPending`. The
supervisor starts the successor with `--xnav` before collecting its health
receipt. It has no boat commissioning check or pause before process creation.
After rollback, the launcher can also start the previous generation. A health
receipt establishes startup identity and readiness, not output safety.

The existing commissioning proof pins the installed generation, current profile,
plugin roots, complete reviewed trees and child launch environment. Changing the
generation invalidates that proof. Reviewing only the signed payload is also
insufficient: `Lifecycle.ps1` copies unbundled stock plugins and preserves
previous-generation additions. Every resulting DLL, helper and dependency in the
actual loader closure needs review, including disabled plugin candidates.

Therefore, removing the bootstrap trust check or releasing its state handle
does not make the live Update Now path safe. Observing the successor and auditing
it after startup is too late. No stage-only product mode or relaxed boat guard
is introduced by this finding.

Before live signed-flow boat qualification, an independently reviewed pre-launch
boundary must bind the exact authenticated release and installed successor,
preserve input-only profile state, and verify a fresh target-generation source,
plugin and loader-environment audit before application process creation. The
same protection must cover fallback startup; a previously healthy generation
does not waive current profile and audit checks. Existing commissioning cannot
silently transfer its active transaction to the new generation.

Until that boundary is implemented and qualified, use only the separately
authorized installation and fresh commissioning sequence that already has
evidence. Completed settings preservation restores configuration; it grants no
launch permission. Bootstrap/package evidence must not be reported as live
signed-offer acceptance on the boat.

## Source audit and negative fixtures (2026-10-11)

This audit is based on monorepo `bf68b355cd319a0ef837f4119b41d4824f95022c`
(product directory `opennav-x`). No installed launch behavior changes in this
increment. SCRUM-311 remains open; none of the following authorizes a boat run.

| Actual boundary | Existing protection | Missing signed boat protection |
| --- | --- | --- |
| Go `run` before the supervised installer | Confirmed signed release, verified artifact custody, selected-generation/stock recheck | Restore the old active commissioning while its pointer is selected; persist and authenticate that result before installation changes it |
| PowerShell `Invoke-UpdateSupervision` / `Start-SupervisedGeneration` | Exact pending candidate, complete declared file hash check, transaction lock, consumed startup attempt, startup-health channel | Fresh target commissioning plus retained profile, actual loader closure and launch environment through process creation |
| Go `launchInstalled` after successful supervisor rollback | Pending record gone, selected previous generation, ordinary startup lock and pointer recheck | Fresh fallback commissioning; known-good health cannot authorize current profile or added modules |
| Go `recoverStartup` after interrupted-update recovery | Exact candidate/previous binding, trusted previous recovery engine, no network, selected previous recheck | Candidate restoration before rollback; same fresh fallback audit and one-use spawn authority |

`Start-SupervisedGeneration` is also reached by `QualifyCurrent`; `ordinary`
also serves Later/offline and Legacy/Safe startup. A future armed boundary must
cover these entries and direct Lifecycle mutation, not only `LaunchPending`.
The existing unarmed offline and signed-manifest policy must remain intact.

### Implemented negative coverage

`tools/boat/test-signed-update-session.ps1` now also mutates retained candidate
commissioning and candidate-restoration receipts immediately before their next
legal transition. Changed generation, profile or complete-tree hashes reject
advancement without rewriting the journal. The pre-existing valid sequence
continues to cover both transitions. This is receipt-integrity evidence only:
it cannot detect a real profile/DLL change when a caller supplies stale hashes.
The portable PowerShell run passed **61 checks**, using the existing local
PowerShell runtime; native Windows behavior is not inferred from that result.

`tools/update-verifier/launcher/prelaunch_boundary_test.go` adds inert tests of
actual `launchInstalled` dispatch with the existing fake platform:

- Nine changed installation/pending identities and consumed-attempt/session
  cases must create no supervisor or application process and retain pending
  recovery evidence.
- Lost supervisor response, timeout and nonzero exit must each return an error
  after exactly one supervisor invocation, even when its fake side effects look
  like a completed rollback. No fallback application or retry is allowed.
- A fallback pointer changed as the startup lock is acquired must refuse process
  creation under that lock and release the lock afterward.

The Go fixtures were **not executed** in this audit because no local Go
compiler was available. Run `go test ./launcher -run 'TestInstalled' -count=1`
from `tools/update-verifier` and then its launcher suite before integration.
`git diff --check` passed. No application build, native CI, private origin,
installed artifact, boat state or physical output was exercised.

### Custody acceptance still required

For both successor and fallback, a native marker-only fixture must replace the
real application and assert **no marker** when selected generation, profile,
active prepared/applied records, module additions/removals/bytes, working
directory or PATH change between audit and process creation. It must also prove
one-use consumption before spawn, no replay after broker restart, and retained
armed denial after a lost or uncertain spawn response. A process-start exception
must not be treated as proof that no child existed. Denial after partial Apply
requires restoration under the still-selected candidate before rollback; it
cannot fall through the current candidate-failure catch into pointer mutation.

The source audited at `bf68b355` includes the four-receipt reducer. The later
durable store/operation commits were absent from that audit base. The subsequent
helper-only integration takes the product helper/contract deltas from `43972f2b`
then `09e2d055`, excluding their workflow changes and unrelated ancestors. See
[the integrated session contract](signed-update-session-contract.md#focused-verification-and-remaining-gate)
for the combined portable checks and still-pending native/Go qualification.
There is no safe source-grounded assumption that their existence establishes
authenticated IPC, live artifact validation, shared mutation custody or coverage
of every process-creation entry. The bootstrap trust/pending refusals remain.
