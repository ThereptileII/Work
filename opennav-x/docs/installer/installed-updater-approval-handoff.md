# Postproduction B0 operator approval binding

SCRUM-311's `tools/installed_updater_approval.py` is an inert byte-contract module
for the [initial B0 admission driver](../../tools/test-installed-protected-b0-startup-windows.md).
It binds an independently reviewed approval to the **original request and
declaration produced for one live disposable runner job**. It performs no file,
network, key, GitHub, profile, service or process operations. It has no approval
writer, polling operation, installer invocation or automatic grant.

The module is not yet connected to a workflow. Its passing portable checks are
not hosted-runner, native installed-updater or four-phase acceptance. The
[existing phase driver](../../tools/test-installed-protected-updater-windows.md)
retains every original gate.

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
- The challenge held by this live pending job, retained independently of the
  submitted approval, and a trusted current UTC clock.

A separately reviewed operator-owned GitHub artifact or immutable commit could
carry the original approval, but authenticating its uploader/author, write access,
exact producer/commit, byte identity and reviewer independence remains a required
coordinator decision. A repository path, artifact name, adjacent checksum,
approval-looking JSON or local file creation is insufficient. This module does
not choose that transport, poll it or provision its permissions.

## Bounded canonical envelopes

Both new envelope types are UTF-8 JSON objects, serialized with sorted keys,
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

`prepare_request(record, client_request=<original bytes>,
declaration=<original bytes>)` owns and validates a proposed envelope and its
links to both originals. It returns a frozen `Request`, including canonical
`original_bytes` and an informational `request_sha256`. It writes nothing and
does not produce approval. Keep original files and actual creation identities
retained separately on the runner.

`bind_approval` requires keyword arguments:

```python
bound = bind_approval(
    request_bytes=original_request_envelope_bytes,
    approval_bytes=original_operator_approval_bytes,
    client_request=original_client_request_bytes,
    declaration=original_b0_declaration_bytes,
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

The two existing originals are bounded at 1 MiB for the client request and
64 KiB for the declaration. Their exact bytes are pinned, strictly parsed for
duplicate keys and linked to the proposed B0 version/commit, trust, recovery,
source, application policy, kit, bootstrap policy and baseline. Their original
serialization is preserved; they are never canonicalized or regenerated.
The existing admission driver and native provisioner still perform full stock,
trust, protected-file, package inventory, profile, SID and actual-token checks.
This byte module neither replaces those validators nor validates local paths.

Successful binding returns only a deeply immutable owned `BoundApproval`: frozen
nested context/artifact records, tuple pins, original request/approval bytes and
the original client/declaration bytes. No `passed`, installed acceptance or
execution permission flag is emitted. The six pins map directly to the initial
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

## Focused checks

Run `python tools/test-installed-updater-approval.py -v`. The inert tests exercise
original-byte retention, immutable ownership, separate pins, changed runtime and
artifact provenance, stale challenges, postproduction ordering, both expiry
windows, strict schemas/serialization, duplicate keys, tampering and resource
bounds. No test produces a genuine approval or contacts GitHub, Tailscale, a
signing service, a Windows installation or the boat.
