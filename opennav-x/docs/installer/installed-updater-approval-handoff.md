# Postproduction B0 operator approval binding

SCRUM-311's `tools/installed_updater_approval.py` is an inert byte-contract module
for the [initial B0 admission driver](../../tools/test-installed-protected-b0-startup-windows.md).
It binds an independently reviewed approval to the **original request and
declaration produced for one live disposable runner job**. It performs no file,
network, key, GitHub, profile, service or process operations. It has no approval
writer, polling operation, installer invocation or automatic grant.

[`tools/installed_updater_approval_transport.py`](../../tools/installed_updater_approval_transport.py)
now provides bounded polling and artifact-verification composition through
caller-supplied read-only API/download callbacks. It reuses the existing GitHub
provenance verifiers and passes authenticated originals to the byte binder.
[`tools/installed_updater_approval_http.py`](../../tools/installed_updater_approval_http.py)
provides the bounded HTTPS callbacks as `GitHubReads(explicit_read_only_token)`.
The manual [approval producer workflow](../../../.github/workflows/skager-installed-approval.yml)
and [record-retention helper](../../tools/produce-installed-updater-approval.py)
retain independently authored original records. They neither author approvals
nor enroll a runner. Live orchestration and independent authorization remain
pending. Portable checks do not establish hosted-runner, native installed-updater or four-phase
acceptance. The [existing phase driver](../../tools/test-installed-protected-updater-windows.md)
retains every original gate.

## Manual approval producer

An authorized operator supplies canonical base64 for the three already reviewed
original records as `request_b64`, `approval_b64` and `pins_b64`. Dispatch only the
reviewed software branch containing the helper. The workflow is manual-only,
uses a read-only repository token, and accepts no signing key or credential.
Inputs enter the helper through environment variables, never shell interpolation.

Before writing, the helper checks canonical encoding, exact schemas, original
request/approval hashes, runner/challenge binding, six separate pins and the
request/approval time windows. Each record is limited to 16 KiB. The producer
must be a different run from the pending enrollment request. It exclusively
creates a fresh directory and preserves exactly `request.json`, `approval.json`
and `approval-pins.json`; existing output refuses without overwriting or cleanup.
Only those originals are uploaded, under the recipient's existing artifact-name
contract. Failure prevents upload and emits no submitted record or exception body.

Eight focused checks cover the real `main()` with mocked environment/clock,
byte-preserving retention, refusal cases and the existing binder roundtrip.
These are in-process checks, not a subprocess or live Actions acceptance run.
The receiver must still independently authorize the operator and exact producer,
authenticate the uploaded artifact and pending native context, and recheck expiry.
Do not dispatch self-authored approvals merely to turn this evidence green.

## Authority stays outside the records

A SHA-256 identifies bytes; it does not identify an operator or establish
independent approval. The coordinating operator must authenticate the original
native producer, transfer, publisher and loader before first execution of the
baseline handoff or final provisioner. A kit checksum manifest, UAC, a signed
TUF target or this module cannot supply that approval. Tailnet authorization,
HTTPS access, key custody and publication remain separate prerequisites.

The existing baseline handoff produces the original client-registration request
using the **actual intended runner account SID**. Its printed hash cannot approve
its own output. After that request and the B0 admission declaration exist, the
runner creates a fresh 32-byte random challenge and the proposed envelope below.
The operator reviews those originals, their provenance and that pending job,
then authors an approval for the exact complete envelope. Approval must occur
after production; an environment gate granted before the runner creates the
request cannot approve its later bytes. Record timestamps alone cannot prove
this ordering or operator independence.

Before calling `bind_approval`, the coordinator must separately authenticate:

- The original approval byte pin and complete request-envelope byte pin from
  the independent operator's approval channel.
- Six separate original pins: client request, B0 admission declaration,
  bootstrap policy, final provisioner, native broker and B0 publication timestamp.
  These are supplied as `approved_pins`, never copied from the proposed envelope.
- The current runner's repository, checked-out harness commit, run ID, attempt,
  actual numeric GitHub job ID and exact job name. The job name alone is
  insufficient. Never take expected context from the submitted envelope.
- The candidate and operator-kit artifact identities against their original
  producer/attempt/job and unexpired GitHub artifact metadata, digest and length.
  Authenticate original candidate qualification/release records and exact inner
  bytes with the existing retained-artifact/package verifiers. The module checks
  equality with supplied observations; it does not query GitHub or authenticate
  a locally declared artifact or qualification record.
- The original B0 application release-policy bytes against the independently
  approved `b0.policySha256`, and the actual original Setup file's local path,
  SHA-256 and size. Neither a declared path nor an adjacent checksum supplies
  those observations. Setup's identity must match the original policy installer.
- The challenge held by this live pending job, retained independently of the
  submitted approval, and a trusted current UTC clock.

The implemented artifact transport accepts only an independently authorized
`ApprovalSource` from the operator's authenticated channel. Its expected producer
commit, actors and artifact pins must never come from the pending runner's
proposed envelope, artifact-name discovery or its own calculated hashes. The
transport verifies that expected identity against GitHub; it does not establish
which operator is authorized or authorize a workflow merely because it exists.
Reviewer independence, trusted producer source and write permissions remain
external prerequisites. A repository path, commit author name, adjacent checksum,
approval-looking JSON or local file creation is insufficient.

## Bounded canonical envelopes

The request and approval envelopes are UTF-8 JSON objects, serialized with sorted keys,
no spaces, ASCII escapes and exactly one final LF:

```python
raw = (json.dumps(record, sort_keys=True, separators=(',', ':'),
                  ensure_ascii=True, allow_nan=False) + '\n').encode('ascii')
```

This serialization is only a byte format, never an approval operation. Each
envelope is at most 16 KiB. Unknown/missing fields, duplicate keys at any depth,
noncanonical serialization, BOMs, trailing bytes, floats, nonfinite numbers,
invalid UTF-8, noncanonical IDs/digests, oversized or invalid records refuse.
SHA-256 values are 64 lowercase hex characters; commits are 40. IDs are positive
decimal strings of at most twenty digits, without leading zeroes. Artifact
digests have the exact `sha256:<64 lowercase hex>` form. Artifact names and job
names are bounded ASCII labels, never paths or URLs.

The request has exactly these fields:

| Field | Original binding |
| --- | --- |
| `schema`, `purpose` | `1`, `installed-updater-b0-approval-request` |
| `runner` | Exact `repository`, `headSha`, `runId`, `runAttempt`, `jobId`, `jobName`; repository is `ThereptileII/Work`. `headSha` is the harness commit, distinct from B0's product commit. |
| `challenge` | Fresh pending-job 32-byte random challenge as 64 lowercase hex characters. Generate outside this parser, for example with `secrets.token_hex(32)`. |
| `createdAt`, `expiresAt` | Exact UTC second timestamps, `YYYY-MM-DDTHH:MM:SSZ`; positive lifetime at most two hours. |
| `candidate` | Exactly `artifact`, `releaseManifestSha256`, `qualificationSha256`. These are original retained bytes, never locally rewritten qualification. |
| `operatorKit` | Exactly `artifact`, `sha256`; the latter is the original inner operator-kit ZIP hash from the B0 declaration, distinct from GitHub's outer artifact digest. |
| `b0` | Exactly `version`, `commit`, `recoverySha256`, `sourceSha256`, `policySha256`, `trustSha256`. Version uses the existing supported Beta 2 product-version parser. |
| `pins` | Exactly `clientRequestSha256`, `declarationSha256`, `bootstrapPolicySha256`, `provisionerSha256`, `brokerSha256`, `timestampSha256`. Proposed values require separate operator approval. |

Each `artifact` object has exactly `repository`, `headSha`, `runId`,
`runAttempt`, `jobId`, `artifactId`, `artifactName`, `artifactDigest`,
`artifactBytes`. Candidate and kit must have different artifact IDs and both
must come from B0's product commit. They need not have the same producer run or
job; authenticate each exact original producer independently. Bounds are 16 GiB
for the candidate outer artifact and 768 MiB for the kit outer artifact. These
are declaration bounds, not permission to download or unpack an artifact.

The approval has exactly these fields:

| Field | Contract |
| --- | --- |
| `schema`, `purpose` | `1`, `installed-updater-b0-postproduction-approval` |
| `requestSha256` | Exact original complete request-envelope SHA-256, separately approved by the operator. This binds all product/artifact/native pins as well as the context. |
| `runner`, `challenge` | Exact original pending job and challenge, repeated for explicit review. |
| `issuedAt`, `expiresAt` | Exact UTC second timestamps. Issued strictly after request creation and no later than the current clock. Positive lifetime at most thirty minutes, with expiry no later than request expiry. |

There is no `approved: true`, permissive default or self-authorizing signature.
The separately supplied original approval pin and authenticated channel supply
the caller's authority; the submitted fields cannot authenticate themselves.

## Module interface and original driver inputs

`prepare_request` requires the original client request, Setup-first admission
declaration, approved B0 application release policy and independently verified
Setup metadata:

```python
request = prepare_request(
    record,
    client_request=original_client_request_bytes,
    declaration=original_b0_declaration_bytes,
    release_policy=original_b0_application_policy_bytes,
    expected_setup=verified_original_setup,  # exactly {path, sha256, bytes}
)
```

It owns and validates the proposed envelope and links to those originals, then
returns a frozen `Request`, including canonical `original_bytes` and an
informational `request_sha256`. It writes nothing and produces no approval.
Keep original files and actual creation identities retained separately on the
runner. The declaration must contain exactly `setup: {path, sha256}` in addition
to the current admission driver's other fields; the former declaration without
`setup` refuses rather than falling back to an older form.

`bind_approval` requires keyword arguments:

```python
bound = bind_approval(
    request_bytes=original_request_envelope_bytes,
    approval_bytes=original_operator_approval_bytes,
    client_request=original_client_request_bytes,
    declaration=original_b0_declaration_bytes,
    release_policy=original_b0_application_policy_bytes,
    expected_setup=verified_original_setup,  # exactly {path, sha256, bytes}
    approved_request_sha256=independently_approved_envelope_pin,
    approved_approval_sha256=independently_authenticated_approval_pin,
    approved_pins=independently_approved_six_pins,
    expected_runner=authenticated_live_runner_identity,
    expected_challenge=retained_pending_job_challenge,
    expected_candidate_artifact=authenticated_original_candidate_artifact,
    expected_operator_kit_artifact=authenticated_original_kit_artifact,
    now=trusted_current_utc_datetime,
)
```

The client request is bounded at 1 MiB, the declaration at 64 KiB and the original
application release policy at 32 KiB. Their exact bytes are pinned, strictly
parsed for duplicate keys and linked to the proposed B0 version/commit, trust,
recovery, source, application policy, kit, bootstrap policy and baseline. Their
original serialization is preserved; they are never canonicalized or regenerated.

The original policy must select the exact installer target
`artifacts/<B0 commit>/SKAGER-Beta2-Setup.exe`. Its SHA-256 and positive integer
size, at most 1 GiB, must match the independently verified original Setup metadata;
that metadata's local path and SHA-256 must match the declared `setup`. Existing
policy and declaration pins bind these values; no additional self-approving
Setup pin is introduced. The binder compares these supplied observations without
opening the file or granting local-path authority. The admission driver still
checks the actual Setup file against that original policy before execution.
The existing driver and native provisioner retain full stock, trust,
protected-file, package inventory, profile, SID and actual-token checks.

Successful binding returns only a deeply immutable owned `BoundApproval`: frozen
nested context/artifact records, tuple pins, original request/approval bytes and
the original client/declaration/release-policy bytes, and a frozen
`Setup(path, sha256, size)` value. No `passed`, installed acceptance or execution
permission flag is emitted. The six pins map directly to the initial
driver's declaration/request, bootstrap-policy, provisioner, broker and timestamp
arguments. Recheck the original files/identities, live context and both expiries
immediately before the driver's single invocation; a stored parsed result does
not remain valid forever.

Wrong runs, attempts, jobs, artifact identities and old challenges refuse even
if the proposed records have freshly calculated matching self-hashes. The parser
does not durably consume a challenge or prevent parsing identical bytes twice
within the same live context. The coordinator must reserve/consume its pending
challenge once and must never retry enrollment. Native permanent fresh-namespace
fences and the existing one-enrollment driver retain their independent protection.
Timeout, expiry, missing independent approval or uncertainty stops execution and
retains original state for review; it never resets trust or deletes namespaces.

## Bounded GitHub artifact transport

`receive_approval` accepts one exact externally authorized approval producer and
returns the existing immutable `BoundApproval`:

```python
bound = receive_approval(
    source=trusted_approval_source,  # ApprovalSource from the operator channel
    read_api=bounded_read_only_github_json,
    download_archive=bounded_original_github_zip,
    bind_arguments={
        "request_bytes": original_request_envelope_bytes,
        "client_request": original_client_request_bytes,
        "declaration": original_b0_declaration_bytes,
        "release_policy": original_b0_application_policy_bytes,
        "expected_setup": verified_original_setup,
        "expected_runner": authenticated_live_runner_identity,
        "expected_challenge": retained_pending_job_challenge,
        "expected_candidate_artifact": authenticated_original_candidate_artifact,
        "expected_operator_kit_artifact": authenticated_original_kit_artifact,
    },
    timeout_seconds=3600,
    poll_seconds=5,
    cancelled=is_cancelled,
)
```

`bind_arguments` contains exactly those nine entries. The caller supplies original
bytes and independently authenticated expected context/artifacts. The module
copies that context before polling. Approval pins come from the separately
operator-authorized artifact, never from the runner's proposed `pins` record.

`ApprovalSource` is a frozen value containing exactly `repository`, `commit`,
`run_id`, `run_attempt`, `actor_id`, `triggering_actor_id`, `job_id`, `artifact_id`,
`artifact_name`, `artifact_digest` and `artifact_bytes`. Numeric IDs use canonical
positive decimal strings. `commit` identifies the independently reviewed approval
producer source; it need not equal the B0 product or pending harness commit.
The transport requires:

- Repository `ThereptileII/Work`, a separate approval run from the pending runner,
  and the exact workflow path `.github/workflows/skager-installed-approval.yml`
  with event `workflow_dispatch`. This is the transport's fixed producer contract,
  not a claim that the workflow has been implemented or dispatched.
- Exact approved commit, run, attempt, numeric actor and triggering-actor IDs;
  GitHub's run creation time must follow the pending request's creation time.
- A successful run with exactly one successful job named
  `Approve installed B0 originals`, matching the approved numeric job ID and
  attempt. Artifact metadata does not directly identify its producing job;
  single-job provenance, the exact artifact pin/name and creation time within
  that attempt's job interval provide the required corroboration.
- The exact unexpired artifact ID, digest and length, at most 64 KiB. Its name is
  `installed-b0-approval-<run ID>-attempt<attempt>-<full request SHA-256>`, produced
  by `artifact_name(run_id, run_attempt, request_sha256)`. A matching name alone
  is never authority. The downloaded original ZIP must match the pinned length
  and SHA-256 before its members are used.

The ZIP contains exactly three nonempty regular files, each at most 16 KiB:

| Member | Exact original content |
| --- | --- |
| `request.json` | Byte-for-byte equal to the pending job's retained original request envelope. |
| `approval.json` | The independently authored canonical approval envelope described above. |
| `approval-pins.json` | Canonical JSON with exactly `schema: 1`, `purpose: "installed-updater-b0-transport-pins"`, `requestSha256`, `approvalSha256` and `pins` containing the six separately approved original pins. |

The transport rejects extra, duplicate, aliased, linked, directory, encrypted or
oversized members, and consumes every member to EOF to check original size and
CRC. It passes the original bytes and authenticated pins to `bind_approval`,
rechecks the producer after downloading, then checks cancellation and expiry
before returning. It creates no approval, writes no files, consumes no challenge
and invokes no installer, enrollment or native process.

The `read_api(endpoint, timeout_seconds=..., max_bytes=...)` callback returns
strictly decoded GitHub JSON; the corresponding `download_archive` callback
returns original ZIP bytes. Both must enforce the supplied deadline and byte cap
**during I/O**, with no API writes, arbitrary URL following or automatic retry of
failed operations. Checking size or truncating only after an unbounded read does
not satisfy this contract. The module bounds API responses at 256 KiB and each
I/O timeout at thirty seconds or the shorter remaining deadline. Total waiting
is at most one hour, further limited by request expiry; polling intervals are at
most thirty seconds. It polls only the exact expected pending run and refuses
failed, cancelled or uncertain outcomes. The existing generic GitHub CLI wrapper
alone does not supply these bounded HTTP callbacks.

## Remaining live orchestration and native gates

The [live coordinator API](installed-updater-live-coordinator.md) now composes the
bounded HTTP callbacks, independent approval binding, original input checks,
native one-use consumption and one joined disposable Windows journey. Its
native streaming gate and actual independently approved installed run remain
separate acceptance requirements. The
pending Windows job must retain its actual account/SID, original files and
challenge throughout the postproduction approval wait; starting a new Windows
job after approval changes the native context. Artifact discovery or an
environment approval granted before request production cannot replace the
independent postproduction approval channel.

Before the single B0 invocation, the coordinator must recheck original native
file identities, approved bytes, current job/challenge and both expiry windows,
then reserve/consume the challenge exactly once. The transport modules alone
provide no durable one-use enforcement: the qualification-only native owner now
retains the same protected parent through READY/approval and exclusively writes
the challenge record with flush/readback before the journey. A timeout or
ambiguous effect must retain state and evidence
for review rather than trigger another enrollment or cleanup attempt.

The hosted Windows job also needs the approved tailnet connection and access to
the selected private HTTPS origin, independently authenticated candidate/kit and
stock installation, protected cache/acquisition inputs, and an explicitly
prepared disconnected disposable profile with nonempty navigation and approved
stable settings. Enabling the origin does not establish the runner's route.
The driver does not seed or repair the profile. No origin/allowlist, UAC policy
or authority change follows from either helper.

The existing B0 driver performs genuine default Setup, protected enrollment,
original B0 startup/normal close and installed customer Maintain uninstall. Live
native evidence must establish the actual linked limited token, desktop access,
original process ownership, relocation/exit observations and preservation
postconditions. Hosted silent elevation cannot establish user-consent acceptance.
This journey ends with retirement/uninstall; a later B1 four-phase journey cannot
continue against that retired installation. Preserve original failed evidence;
portable helper checks or component passes do not qualify the complete journey,
release, boat or public access.

## Focused checks

The focused inert checks are `python tools/test-installed-updater-approval.py -v`
and `python tools/test-installed-updater-approval-transport.py -v`, with actual
HTTP framing exercised by `python tools/test-installed-updater-approval-http.py -v`.
They exercise
original-byte retention, immutable ownership, Setup/policy binding, separate
pins, changed runner/producer/artifact provenance, stale challenges,
postproduction ordering, expiry, cancellation, deadlines, strict schemas,
ZIP membership/digest/CRC, tampering and resource bounds. Their API callbacks and
archives are fixtures. No test produces genuine approval or contacts GitHub,
Tailscale, a signing service, a Windows installation or the boat.

The HTTP adapter permits only the fixed repository's expected run-attempt,
jobs-page, artifact metadata and archive endpoints. Its single storage redirect
is restricted to GitHub's reviewed `productionresultssa<digits>.blob.core.windows.net`
host family under `/actions-results/`, without forwarding the API credential.
Unknown storage backends refuse. DNS, connect, TLS, headers, body and redirect
share a deadline; one unresolved DNS worker is allowed process-wide. Header and
body sizes are bounded during reads. Ambiguous framing, trailers, chunk
extensions, malformed JSON and nonfinite values refuse. Public errors suppress
credential and signed-URL details. The adapter neither acquires a token nor
publishes, retries, creates approval or consumes a challenge.

Socketpair fixtures test actual HTTP framing, resource limits and composition
with the original approval transport. They replace the connection function, so
they do not qualify a live TLS handshake, GitHub transfer or native enrollment.

## Fixed operator-authored status rendezvous

The independently authenticated external channel may now be one explicitly
preauthorized GitHub commit-status locator. This is an extension of the channel
contract, not permission to discover expected pins from artifact names. The
operator must independently review the original pending request and records,
authorize the existing producer, then post the exact complete `ApprovalSource`
descriptor. Neither this reader nor its serialization helper grants approval.

`installed_updater_approval_rendezvous.TrustedStatusLocator` requires four
independently supplied values: numeric status creator ID, reviewed producer
commit, numeric producer actor ID and numeric triggering actor ID. There are no
current-actor/repository-owner defaults. Repository and producer workflow remain
fixed to `ThereptileII/Work` and the existing approval workflow. Repository write
access alone does not establish that the user is an authorized operator.

The exact pending harness commit is the status endpoint. `status_context(request)`
derives a lowercase context from the original runner tuple, challenge and complete
request digest. The authorized operator posts exactly one success status after
request production, with `target_url=source_target_url(approved_source)`. That
helper only serializes an already approved descriptor. Its fixed GitHub actions
run URL fragment contains canonical base64url descriptor bytes; no URL is followed.
The independently supplied descriptor carries the exact artifact ID/digest/size;
no artifact listing or matching-name inference is permitted. Existing producer,
job, actor, original-record and archive authentication remains unchanged.

Only the exact commit-status listing endpoint is allowed, with 100 entries per
page and at most five pages. Each HTTP response remains bounded to 256 KiB and
thirty seconds or the remaining deadline. A complete terminal page is required
before accepting a match. Foreign creators, case variants, changed timestamps,
malformed descriptors, multiple matching attestations, missing recheck records,
partial pagination and bounds/expiry/cancellation failures refuse. The original
attestation ID/context/creator/state/timestamps/URL is rechecked before native
consumption; a replacement is never accepted. Mutable avatar/profile fields are
not authorization fields. Unrelated status contexts confer no authority.

The receiver needs only actions-read and commit-statuses-read permissions, never
status-write, workflow-dispatch, signing keys or a new signing secret. Actual
status creation stays with the independently authorized operator. GitHub's
[status API](https://docs.github.com/en/rest/commits/statuses) documents URL/context
fields, creator identities and separate read/write permissions. The implementation
bounds contexts below 100 characters and target URLs to 2048 ASCII characters.
The documented API has no stated URL length/fragment restriction; live acceptance
of the chosen descriptor URL has not been exercised by these read-only/portable
checks and must not be inferred from them.
