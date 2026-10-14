# SCRUM-311 live installed-journey composition

`tools/installed_updater_coordinator.py::run_live` composes the maintained
approval HTTP/transport/binding modules with one `run_joined` invocation. It is
an executable Python API for the pending qualification job, not a publisher,
approval producer, host preparer or customer product entrypoint. The caller must
already authenticate the original candidate/operator-kit producer artifacts,
prepare both maintained declarations and supply the independently authorized
approval producer `ApprovalSource` and exact pending context. It must retain the
same original Windows job, account and files; a newly started job is not a
continuation of approval for an earlier job.

The coordinator retains the existing native account/profile observer and original
consumer executable deny-write owner through approval wait and actual journey.
The separately approved consumer hash must identify the harness-only native Go
test executable, not a shipped provisioner. The exact private invocation is:

```
<approved harness executable> --consume-installed-b0-challenge=<protected qualification parent>
```

Before approval, that same original helper authenticates and retains protected
deny-delete ancestry, then flushes one READY JSON line with exactly schema=1,
state=`ready` and parent (the full native identity below). It remains alive with
stdin open throughout approval. The final parent identity must equal READY. EOF
before a complete record drains that child with no consumption effect.

After approval, stdin is at most 4096 canonical JSON bytes, with exactly schema=1,
purpose=`installed-updater-b0-consumption`, repository, headSha, runId,
runAttempt, jobId, jobName, challenge, requestSha256 and approvalSha256. Native
code authenticates protected ancestry and exclusively creates the fixed child
`.installed-b0-consumed-<challenge>.json`; it flushes, reopens, checks full native
identity and exact bytes, and emits success only after checked cleanup. Existing,
partial, foreign or uncertain records permanently refuse; no deletion/reset or
retry is available. Product code-upgrade reservations are not reused.

The second and final bounded stdout line has exactly schema=1, state=`consumed`,
recordSha256, parent and record. Each identity has volumeSerial and creationTime
as nonzero canonical uint64 decimal strings, fileID as nonzero lowercase 32-hex,
and descriptorSha256 as lowercase 64-hex. Parent and record have the same volume
and distinct file IDs. The receipt is retained in a separately selected absent
evidence file outside the protected parent and outside the joined journey's
fresh result directory. Python retains its original output handle and obtains an
independent read cursor through Windows ReOpenFile on that handle, with
GENERIC_READ and FILE_SHARE_READ|FILE_SHARE_WRITE, never a pathname reopen. The
reader closes only after the original child drains. Each frame is at most 4096
bytes, total 8192; a third frame refuses. READY and completion each have a
sixty-second observation/execution deadline. Native code provides the directory-security authority;
Python path/identity checks do not replace it.

Approval expires and original input/account/profile checks repeat immediately
before and after consumption. A failure after consumption cannot re-enter the
journey. A sixty-second consumer deadline refuses late success; if the original
child is still live, the coordinator waits for its actual exit while retaining
executable custody, without kill/relaunch or later installation. This drain may
outlive the execution deadline. Cleanup failure suppresses success.

`run_joined` retains its existing exact online/offline publication handoff and
acceptance boundaries. The coordinator does not supply signed artifacts,
postproduction approval, private-network authorization, stock/profile fixtures,
real interactive UAC capability, Repair or automatic-fallback qualification.
Portable tests use inert API/ZIP and process fixtures; native consumption and the
actual installed customer journey require separate execution evidence.

Windows qualification must execute the real same-object ReOpenFile streaming
adapter against the native helper, including READY, EOF-without-effect, exact
consumption and permanent replay refusal. Linux fd-based inert fixtures prove
composition only and are not that native adapter evidence.

## Deferred external source resolution

The existing `source=ApprovalSource(...)` API is unchanged. Alternatively pass an
explicit trusted locator; resolution occurs only after the same original native
consumer emits READY, while the original account/profile observer, helper image
and protected parent remain retained. It never begins a different job/process or
reopens another parent to continue approval. One overall hour includes native
READY, locator wait, approval transport and final locator recheck, additionally
bounded by original request expiry. Approval expiry is checked again after the
status recheck and immediately before consumption.

Example for the future callable CLI (no CLI/workflow is added by this increment):

```python
from installed_updater_approval_rendezvous import TrustedStatusLocator

run_live(
    source=TrustedStatusLocator(
        status_creator_id=authorized_operator_id,
        producer_commit=independently_reviewed_producer_commit,
        producer_actor_id=authorized_producer_actor_id,
        producer_triggering_actor_id=authorized_triggering_actor_id,
    ),
    bind_arguments=original_pending_binding_arguments,
    initial=prepared_initial_declaration,
    phases=prepared_phase_declaration,
    output=fresh_journey_result_path,
    handoff=existing_authorized_phase_handoff,
    read_only_token=explicit_read_only_token,
    consumer_executable=original_native_qualification_executable,
    consumer_sha256=independently_authenticated_consumer_sha256,
    qualification_parent=original_protected_qualification_parent,
    consumption_receipt=separate_absent_receipt_path,
)
```

The operator independently computes `status_context(original_request)` and
`source_target_url(independently_approved_source)` using the same module. These
functions format observations/data, never approve or post them. No producer or
artifact-name discovery, approval authoring, publication, connectivity mutation,
profile preparation or automatic execution retry is introduced.

## Callable hosted Windows entrypoint

`tools/run-installed-updater-journey-windows.py` invokes the existing live
coordinator once. It accepts only three explicit options:

```powershell
$env:SKAGER_APPROVAL_READ_TOKEN = '<separately supplied read-only token>'
python opennav-x/tools/run-installed-updater-journey-windows.py `
  --config=C:\Qualification\journey.json `
  --config-sha256=<independently approved original configuration SHA-256> `
  --disposable-opt-in=SCRUM-311-disposable-initial-protected-b0
```

The token never enters command-line options or diagnostic output. The CLI refuses
non-Windows, non-GitHub-hosted, wrong-repository, non-manual-dispatch or missing
explicit disposable opt-in before reading files or invoking native effects. This
is an operational guard against accidental boat/non-disposable execution, not
cryptographic host attestation. Actual same-account/token/desktop capability and
an independently prepared disconnected stock/profile fixture remain required.

The independently hash-pinned configuration is bounded to 64 KiB, strict JSON,
with exactly these fields; none supplies authorization merely by existing:

| Field | Required original input |
| --- | --- |
| `schema`, `purpose` | `1`, `installed-updater-journey-cli` |
| `request` | `{path, sha256}` of the original pending approval request envelope |
| `runner` | Exact existing request runner tuple: repository/headSha/runId/runAttempt/jobId/jobName, independently supplied and compared with actual environment and GitHub run/job |
| `challenge` | Independently retained original pending challenge |
| `pendingWorkflow` | Exact reviewed pending workflow path |
| `admission`, `phases` | Original independently approved `{path, sha256}` records for the existing two declaration parsers |
| `artifacts` | Exactly `candidate`, `operatorKit`, `successor`; each contains `{artifact, workflow, jobName}`. `artifact` is the existing exact producer artifact tuple including original ID/digest/size/head/run/attempt/job. Candidate and kit must equal the pending request; successor commit must equal prepared B1 |
| `source` | All four explicit `TrustedStatusLocator` fields: status_creator_id/producer_commit/producer_actor_id/producer_triggering_actor_id |
| `consumer` | `{path, sha256}` of the independently authenticated native qualification executable |
| `qualificationParent` | Existing independently prepared protected parent; no creation/ACL repair |
| `consumptionReceipt` | Separate absent receipt file outside protected parent and joined evidence tree |
| `output` | Fresh joined evidence `result.json` path |
| `phasePlan` | Independently approved `{path, sha256}` for the exact phase timestamp plan |

`producer()` corroborates GitHub metadata/provenance using the existing exact
run/job/artifact validators and bounded read-only API. It does not download or
authenticate archive payload bytes. Independently verified original candidate,
operator-kit and successor archive digests, inventories, manifest/qualification
records and linkage to the exact extracted local bytes are required preparation
prerequisites **before authorizing the CLI configuration/request**. Workflow
success alone does not establish application qualification. Qualified B0/B1,
original local bytes and independent producer provenance must already be approved;
existing declaration hashes and the external postproduction approval bind those
exact prepared inputs. The CLI constructs no package,
request, approval or profile, and builds no helper. Existing declaration parsers
and coordinator checks retain their original authority and refusal boundaries.
A generic failure preserves existing state/evidence; no retry or cleanup occurs.

The strict phase-plan JSON has exactly `no-update`, `later`, `now`, `offline` and
`manual-rollback`. The first three contain independently approved lowercase
64-hex timestamp hashes, with `no-update` equal to the original request pin;
`offline` and `manual-rollback` are JSON null. The plan does not publish anything.

At each phase, the CLI prints one bounded JSON readiness request containing the
phase, actual expected selected head, fixed status context and exact target URL.
The same independently authorized status creator must post exactly one success
status on the original pending commit **after that phase wait begins**, using the
printed context and target URL. These bind the original runner/challenge, phase,
actual head and approved phase-plan digest. The signal releases waiting only; it
cannot change the approved timestamp, authorize publication or claim an offline
condition. Root/operator performs the separately authorized origin/publication
or connectivity action externally, then signals readiness. The existing journey
independently observes the actual signed endpoint/offline condition afterward.

Each phase wait is at most fifteen minutes, with bounded status-only reads and
complete pagination; foreign creators, stale/edited/malformed/conflicting
signals, incomplete pagination, API failures and timeouts refuse. A signal URL is
never followed. There are no publication retries, status writes, signing keys,
network changes or automatic later-phase continuation. Actual interactive UAC,
customer Repair/automatic fallback and boat acceptance remain outside this CLI.
