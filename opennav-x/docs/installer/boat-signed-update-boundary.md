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

## Final ordinary-start pending check (2026-10-11)

`Lifecycle.ps1` publishes the pending update before `state.json`. If installation
is interrupted between those writes, the old generation stays selected while
recovery is required. A launcher already past its initial pending check could
previously start that generation because `ordinary` rechecked only the pointer
and stock identity under `transaction.lock`.

`ordinary` now also requires the pending record to be provably absent while
holding that same lock through process creation. Presence or a read failure
refuses startup without modifying recovery evidence. This covers XNav, Later,
offline, Legacy, Safe and ordinary fallback dispatch; existing supervisor
qualification/recovery checks remain separate.

Nine deterministic cases failed on the original behavior and pass with the
guard: all three startup modes with a pending, empty or oversized record
published at lock acquisition, without changing the selected generation.
The complete Go launcher and updater-verifier module suites passed on Linux
using the official pinned Go 1.27.1 toolchain and cached dependencies with
`GOPROXY=off GOSUMDB=off`. [Evidence and toolchain provenance](../evidence/scrum311-prelaunch-pending-20261011/results.json)
retain the red run and both successful suites.

This narrow installed-launcher fix does not authenticate session-helper records,
add broker or commissioning authority, or qualify the missing native custody
boundary. SCRUM-311 and native Windows/signed boat acceptance remain open.


## Native file acquisition rejects hardlinks (2026-10-11)

At base `e1d6f5a6d42cc20605368fe307b34a607d75acbc`, the Go launcher queried
`GetFileInformationByHandle` before granting a native file owner, but checked
only directory/reparse attributes. It accepted two names for the same file,
even when those bytes matched the declared hash. `openNativeRead` and
`lockStartup` now require `NumberOfLinks == 1` on that actual acquired handle
and close it on refusal. This covers record reads, executable/helper holds,
stock-image measurement and the installation lock. Existing sharing, hash,
size and reparse checks remain. The session store independently already refused
multiple links; this change closes the corresponding launcher acquisition gap.
[Microsoft documents the handle metadata link count](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/ns-fileapi-by_handle_file_information).

`TestNativeOwnedFileAcquisitionRejectsHardlinks` creates only inert PE-shaped
bytes and a hardlink in a disposable local directory. All four native consumers
must refuse both names, return no owner and release refused handles so writes
remain possible. Removing the alias must restore ordinary single-link
acquisition, and releasing any successful owner must permit cleanup. The
fixture does not launch an application, shell, installer or physical output.

The Linux launcher suite passed with pinned Go 1.27.1 and offline cached
dependencies. Windows x86 and x64 launcher test helpers cross-compiled; they
were **not executed**. These checks establish portable regressions and native
compilation only. [Focused results](../evidence/scrum311-native-hardlink-20261011/results.json)
record the exact commands and source hashes. Native Windows must run
`go test ./launcher -run '^TestNativeOwnedFileAcquisitionRejectsHardlinks$' -count=1`
and the existing launcher suite before accepting the native invariant.

This is an acquisition-time link-count refusal, not continuous namespace,
loader, profile or generation custody, nor signed-update launch authorization.
Commissioning and bootstrap trust/pending refusals remain unchanged. SCRUM-311
and live signed-update/boat acceptance remain open.

### Executable/path custody integration dependency

A larger supervisor image/path lease was investigated and withdrawn without
changing production supervisor code. At the same base, `Get-UpdateFileHash`
closes its stream before `Start-SupervisedGeneration` creates the process
(`UpdateSupervisor.ps1:22-26,160-172`). Retaining directory and file handles
alone does not bind a DOS drive alias subsequently resolved by `Process.Start`.
A same-logon drive mapping change can leave the old handles valid while the
launch path names replacement bytes; rechecking that mapping immediately
before creation still leaves a race.

A possible stable launch spelling is the volume GUID path derived from each
retained handle, with root-to-child consistency checked during acquisition.
[GetFinalPathNameByHandle documents that path form](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-getfinalpathnamebyhandlew).
It cannot be substituted safely as a local textual fix: startup-health
`ExactProcess` compares `fresh.MainModule.FileName` with the original executable
path (`UpdateTransaction.ps1:137-144`), and normal/interrupted process cleanup
uses the same path identity (`UpdateSupervisor.ps1:174-190`). Record-path
validation also rejects device paths (`UpdateTransaction.ps1:32-44`). Module
paths can preserve the spelling used at load, including an extended prefix,
[as documented for GetModuleFileName](https://learn.microsoft.com/en-us/windows/win32/api/libloaderapi/nf-libloaderapi-getmodulefilenamew).

The necessary next boundary is a coherent retained native identity used by
creation, authenticated health and exact-process normal/interrupted cleanup,
with uncertain creation kept denied rather than entering automatic rollback.
A harmless native fixture must remap a disposable drive alias between
acquisition and creation and exercise the full supervisor/health/cleanup path:
replacement bytes must never run, and a legitimate child must remain positively
identifiable. No path-string relaxation, approval flag or class-only owner can
substitute for that integration and native evidence.
