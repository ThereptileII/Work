# Purpose-bound broker bootstrap declaration

SCRUM-311 separates an application update from enrollment of the machine update
broker. This is an inactive source capability. A declaration, its adjacent hash,
an application qualification record or a successful signature check does not
authenticate the first privileged verifier or authorize executing a provisioner.
The installed launcher does not invoke this enrollment path yet.

## Signed target and independent inputs

The existing TUF targets role may publish `bootstrap/broker-beta.json` alongside
`releases/beta.json`. The ordinary release reader cannot select the bootstrap
path. Both retain the existing pinned root, signature thresholds, expiry,
consistent snapshots and durable metadata rollback checks. The signed bootstrap
custom identity has exactly `purpose`, `channel`, `version`, `commit` and
`trustConfigSha256`. Purpose is `machine-update-broker-enrollment`.

The strict declaration has exactly these fields:

| Field | Contract |
| --- | --- |
| `schema`, `product`, `architecture` | `1`, `skager`, `x86` |
| `purpose` | `machine-update-broker-enrollment` |
| `channel`, `version`, `commit` | Selected channel, bounded SemVer, exact lowercase 40-hex source revision |
| `serviceName` | `SkagerUpdateBroker` |
| `serviceAccount` | `NT SERVICE\SkagerUpdateBroker` |
| `codeLocation` | `ProgramFilesX86/SKAGER/UpdateBroker` |
| `dataLocation` | `ProgramData/SKAGER/UpdateBroker` |
| `trustConfigSha256` | Exact independently selected public configuration bytes |
| `broker`, `provisioner`, `correspondingSource`, `notices` | Path, HTTPS URL, lowercase SHA-256 and bounded positive byte count |

Locations are fixed KnownFolder policy identifiers, not arbitrary paths to be
resolved or created by a parser. All four artifacts use one exact HTTPS origin
and `artifacts/<commit>/<single filename>`. Executable filenames are exactly
`skager-update-broker.exe` and `skager-update-provisioner.exe`. Names are bounded
and case-insensitively distinct. Missing, extra, duplicate, case-aliased, null,
malformed, oversized or trailing input fails closed. No machine action follows
from parsing.

`VerifyBrokerBootstrap` requires an independently trusted root, protected durable
cache, explicit metadata URL/artifact origin, and three separate caller pins:
approved policy SHA-256, pretrusted provisioner SHA-256 and public configuration
SHA-256. Missing or malformed pins fail before networking. Real go-tuf verifies
the purpose-specific target; exact policy bytes, signed custom identity, parsed
body, provisioner pin and all artifact origins must agree. Every metadata/target
redirect is refused, including same-origin redirects. Returned typed data and
bytes are owned copies. Declared executables are not downloaded or run.

## Explicit operator publication

The Linux operator's existing `publish` command accepts a paired, optional input:

```text
--broker-bootstrap-policy /independently/reviewed/broker-policy.json
--broker-bootstrap-policy-sha256 <independently approved exact SHA-256>
```

The existing application selection, five qualified release artifacts and online
role keys remain required. The new policy pin is a distinct operator review
boundary. Application qualification does not qualify the broker, provisioner or
their corresponding source. Supplying only one bootstrap flag is refused before
key handling. No keys are generated, fetched or written by this operation.

A newly supplied bootstrap declaration must match the selected app's version
and commit and the retained public trust configuration/origin. All declared
local bytes must match. Present immutable destinations and historical Windows
case aliases are rejected before state reservation. Shared app/bootstrap source
or notice artifacts are allowed only with identical complete identities.

Private operator state retains the exact original policy bytes and approval
hash, providing a separate bootstrap version floor. Later ordinary app
publication preserves that bootstrap even when the app advances. A bootstrap
downgrade or different bytes at the same version is refused; exact renewal is
allowed. Existing state without this optional field remains accepted. Older
readers reject the new field instead of silently losing it.

State/floors are durable before publication, and the atomic timestamp replacement
remains the sole publication switch. Interruption retains pending policy bytes
and consumed metadata version. Missing pending artifacts require explicit repair
with the same approved declaration; omission cannot erase it. Existing metadata
and artifacts remain immutable. The operator repository stays private; this
change does not publish a release or open public access.

## Fresh native store initialization

`brokerstore.InitializeFresh` now implements fixed native store creation, separate
from `Open`. It accepts no root, SID, trust bytes or registration arguments. Only
the actual, non-impersonated `NT SERVICE\SkagerUpdateBroker` process may call it;
Administrator and SYSTEM are not substitutes. The thread remains locked through
token checks, creation and handle cleanup.

The actual KnownFolder `ProgramData/SKAGER` parent must already have been
independently protected **at original creation**. Current ACL checks cannot prove
that history or exclude previously acquired writable handles. This API neither
creates nor adopts/repairs that parent. It holds and checks the complete physical
drive/NTFS/ProgramData/SKAGER ancestry, corroborates the fixed DOS path, rejects
unsafe owner/ACL/reparse/volume identities, and verifies parent add-file and
add-directory access before consuming initialization.

Before creating any broker directory it exclusively creates
`ProgramData/SKAGER/UpdateBroker.initialized` with the final exact broker/SYSTEM
descriptor and synchronizes a bounded **started** record. It then creates only
`UpdateBroker`, `candidates`, `registrations` and `users`, each with its final descriptor
from first exposure. Exact held-tree validation precedes the synchronized
**completed** record. Existing roots or fences are never adopted, truncated,
deleted or reset. Every failed attempt retains its fence and partial namespace;
deleting the broker root does not permit retry. This permanent fence is distinct
from the existing reopenable lifetime `store.lock`.

`ValidateInitialization` is read-only. It requires the same actual broker and
independently provisioned parent, an exact protected completed fence, and all
four exact existing directories. Missing/started/corrupt evidence refuses;
validation never repairs missing children. Non-Windows APIs refuse with
`ErrNativeRequired`.

The native service accepts exactly these **SCM start arguments**:

| Arguments | Result |
| --- | --- |
| `SkagerUpdateBroker` | Validate local completion, then normal protected `Open` and serving |
| `SkagerUpdateBroker initialize-store` | Fresh creation only, then stop before `Running` or opening the request pipe |
| `SkagerUpdateBroker verify-store` | Read-only durable completion validation as the broker, then stop before `Running` or opening the request pipe |
| `SkagerUpdateBroker register-client <SID> <approved SHA-256>` | Read only the fixed protected original request, validate independent approval, initialize one fresh client, then stop before `Running` or opening the request pipe |

Executable command-line arguments remain forbidden. Missing, extra, aliased or
unknown SCM arguments refuse. Context cancellation is checked before and after
the bounded synchronous operation; it does not interrupt a creation step. The
native provisioner verifies terminal stopped service state/exit result for both
initialization and the separate durable completion check. `StartService` success
alone is never readiness.

Completion is durable before handle cleanup. A subsequent close error or
cancellation may report failure while retaining completed local state. That
record establishes local creation only: **not** authenticated enrollment, a
client registration, installation/loader custody, healthy application startup or
authority to execute an update. Normal validation releases its owners before
`Open` reacquires them; continuous installation custody is not claimed here.

## Explicit native machine provisioner

`cmd/skager-update-provisioner` now calls the concrete `brokerenroll.Run` path:

1. Copy the bounded public trust configuration and compare its independent
   SHA-256 before network or machine mutation; reuse the operator repository's
   strict private-beta trust/root/OpenCPN-baseline validation.
2. Require an elevated non-impersonated administrator and retain current
   protected physical NTFS ancestry for the existing operator bootstrap cache
   and acquisition directories. Their history and the initial executable's
   authentication remain independent prerequisites.
3. Call the existing real `VerifyBrokerBootstrap` with all independent pins.
   Parse only its exact approved target bytes, then acquire the signed exact
   broker artifact with `DownloadArtifact`. The enrollment broker cap is 64 MiB.
4. Exclusively create actual KnownFolder `ProgramData/SKAGER` and
   `ProgramFilesX86/SKAGER/UpdateBroker`, with final protective descriptors at
   creation. Ordinary add-only access on the preexisting ProgramData parent does
   not permit adoption: a pre-created SKAGER child always refuses. Reparse,
   alias, untrusted deletion/ACL/metadata mutation, link and volume substitutions
   refuse. Every new fixed namespace remains on failure.
5. Synchronize a permanent `ProgramData/SKAGER/UpdateBroker.enrollment` started
   file before creating code. Copy and re-hash the exact bounded image into an
   exclusively created protected file, then retain its read-only native owner.
   File completion records only local code/parent creation, not service health.
6. Exclusively create the fixed demand-start own-process SCM service under
   `NT SERVICE\SkagerUpdateBroker`, with the exact quoted fixed image path.
   `CreateServiceW` has no security-descriptor argument: its documented default
   protects configuration/start/stop/delete/security mutation while stopped.
   Validate actual initial protection, then apply and verify the exact tightened
   DACL **before any start**. Do not claim the final DACL existed at creation.
   Authenticated users retain query/read rights needed by `brokerpipe.Call`;
   ordinary callers cannot control, reconfigure or delete the service.
7. Invoke `initialize-store` and then `verify-store`, each with a bounded
   60-second wait for stopped state, zero exit codes and no accepted controls.
   Hold original code/input/SCM/registry owners throughout. Neither invocation
   announces Running or opens a request pipe. If an independently approved client
   request was supplied, invoke the separate stopped `register-client` operation
   through the same retained service handle and validate its terminal result.
   Leave the service stopped.

The outer operation is capped at 30 minutes. The existing signed-bootstrap reader
has its own 30-second network deadline; cancellation is checked before subsequent
acquisition/mutation, rather than abandoning verification in a background thread.
Native input/image handle cleanup failures preserve `ErrCleanup` through fixed
redacted messages. Close releases owners only; it does not delete any partial
machine objects. Service STOPPED/PID fields are not process-exit proof. If a second
start races process drainage, refuse and retain state; there is no blind retry.

The CLI requires all six explicit, nonduplicated inputs: `--trust-config`,
`--bootstrap-cache`, `--artifact-directory`, `--policy-sha256`,
`--provisioner-sha256` and `--trust-config-sha256`. No environment-derived pins,
adjacent self-described manifest or checkbox authenticates first execution.
This is an operator-only component, **not** a currently qualified execution guide
or an automatic installer/application-startup action. Do not run it on the boat
until independent final executable/publisher/loader authentication and native
disposable-machine qualification have passed. Signing delivery remains disabled
as documented in [code signing](code-signing.md).

`package-update-provisioner.py` produces a fixed Go 1.27.1 Win32 console PE beside
same-commit verified launcher and broker copies. Its strict verifier checks the
actual embedded metadata, console subsystem, fixed recipe, exact broker binary
and build-record prerequisite hashes, and complete corresponding-source closure.
The producer order is launcher → broker → provisioner, from one clean commit.
These local checks bind bytes/source, not original producer or publisher trust;
the scripts never execute/elevate the output or activate a service. No OpenCPN
binary is bundled into this prerequisite component.

## Independently approved fresh client registration

Source revisions `129b0a64`, `910fd664`, `95cbe96e`, `55c743fd`, `24e137e9`,
`e7fabf9b`, `988b90d9`, `d0b8931f` and `b8913374` add a concrete, inactive
source path. These revisions do not establish native installation acceptance.

The provisioner optionally accepts a paired `--client-registration-request` and
`--client-registration-request-sha256`. The latter is a separately authenticated
operator approval of the original bounded request bytes, not an adjacent
self-hash. Approval contains the actual canonical user SID, exact installed
identity declarations, stock OpenCPN tuple/path and complete original public
trust configuration. Its purpose is exactly `client-registration`; duplicate,
case-aliased, extra, missing, null, invalid or unbounded fields refuse. The
request's trust bytes/hash/channel must match the enrolled bootstrap exactly.
Before network or machine mutation the native path resolves the SID as a real
user. Declared installed hashes are not physical installation or loader proof.
LocalAppData state, client self-reports and a request cannot authenticate their
own approval or establish the initial provisioner's publisher identity.

Only the retained **fresh** machine enrollment may create
`ProgramData/SKAGER/ClientRegistrationRequests/<SID>.json`. Directory and file use
final protected Administrator/SYSTEM write and broker read/traverse descriptors
from creation; ordinary clients have no ACE. `CREATE_NEW`, synchronized original
bytes and readback precede reopening with retained read-only, no-write/no-delete
sharing custody. Partial namespace is preserved and refused, never adopted or
reset. This is not an API to add clients to an existing machine enrollment.

The retained SCM owner sends only `register-client`, SID and separately approved
hash. No request pathname or alternate service is accepted. The actual native
broker reader verifies its process/effective principal, absence of impersonation,
fixed KnownFolder ancestry, exact protection, physical file identity, bounds,
original hash and SID. It returns owned original bytes. The stopped service
entry reparses approval and calls `RegisterApprovedClientNative` before any
`Running` state or request pipe. Fresh approval still requires an unexpired
bootstrap root. Administrator/SYSTEM console execution is not a broker fallback.

The broker reserves a permanent SID-specific `.once` fence before creating a
fresh protected user/trust/artifact tree. Final channel state remains absent
until the real verifier initializes it. The exact original approved request is
stored in strict schema 2 with `clientSid`, `approvedRequestSha256` and canonical
base64 `approvedRequest`; it is not replaced by mutable parallel declarations.
A synchronized completed fence binds approval and protected JSON hashes. Readers
require completed evidence; old schema 1 is rejected because no accepted native
producer wrote it. Existing or partial client state is never silently adopted,
repaired or retried. The writer is closed before a protected read is possible.

Cancellation is checked immediately before the irreversible bounded publication.
Once durable completion succeeds, late cancellation does not turn success into
failure. Actual publication or subsequent custody/close failure may already have
retained completed state: `ErrRegistrationUncertain` /
`ErrClientRegistrationUncertain` require explicit operator review, never blind
retry, deletion or trust-floor reset. Stock observation/probe cleanup failures
retain `ErrStockCleanup`; repeated close returns the cached outcome.

A completed protected read validates the original anchor's schema, signatures,
keys, exact bytes and identity without requiring that **historical** root to stay
unexpired after rotation. `ParseRetainedApprovedRequest` / `ValidateTrustIdentity`
are identity-only paths, not enrollment authorization. Current retained root and
metadata expiry, signature verification and durable rollback floors remain the
real verifier's responsibility at its current clock. Missing/corrupt retained
state refuses; reading registration never initializes or resets it.

## Remaining installation boundary

The first final provisioner must still be independently authenticated before
elevation. Machine parent/service provisioning now has a concrete source path;
actual native qualification and authenticated first execution remain unaccepted.
Fresh protected client registration now has the source path above; its actual
native principal/ACL/SCM qualification remains required. Independently
authenticated installed-byte/loader custody, launcher handoff, healthy-start
receipt, recovery and the full installed Windows/boat update sequence remain
required. Existing-enrollment/client migration is not implemented by the fresh
path and must not be simulated by deleting recovery fences. Machine/store creation is implemented in source but has not
passed native Windows execution or actual enrolled-service qualification. None of these
missing authorities is supplied by the declaration. No service is activated,
hardware control enabled or application/profile changed by its focused tests.

See [private repository operations](staging-update-repository.md) and
[secure updater](secure-updater.md). Native Windows remains mandatory for
installation acceptance; cross-compilation or local signed fixtures cannot
replace it.
