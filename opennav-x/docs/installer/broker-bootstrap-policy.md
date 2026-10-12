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
`UpdateBroker`, `candidates` and `registrations`, each with its final descriptor
from first exposure. Exact held-tree validation precedes the synchronized
**completed** record. Existing roots or fences are never adopted, truncated,
deleted or reset. Every failed attempt retains its fence and partial namespace;
deleting the broker root does not permit retry. This permanent fence is distinct
from the existing reopenable lifetime `store.lock`.

`ValidateInitialization` is read-only. It requires the same actual broker and
independently provisioned parent, an exact protected completed fence, and all
three exact existing directories. Missing/started/corrupt evidence refuses;
validation never repairs missing children. Non-Windows APIs refuse with
`ErrNativeRequired`.

The native service accepts exactly these **SCM start arguments**:

| Arguments | Result |
| --- | --- |
| `SkagerUpdateBroker` | Validate local completion, then normal protected `Open` and serving |
| `SkagerUpdateBroker initialize-store` | Fresh creation only, then stop before `Running` or opening the request pipe |

Executable command-line arguments remain forbidden. Missing, extra, aliased or
unknown SCM arguments refuse. Context cancellation is checked before and after
the bounded synchronous creator; it does not interrupt a creation step. The
future authenticated provisioner must verify terminal service state/exit result
and local completion, not interpret `StartService` success as readiness.

Completion is durable before handle cleanup. A subsequent close error or
cancellation may report failure while retaining completed local state. That
record establishes local creation only: **not** authenticated enrollment, a
client registration, installation/loader custody, healthy application startup or
authority to execute an update. Normal validation releases its owners before
`Open` reacquires them; continuous installation custody is not claimed here.

## Remaining installation boundary

An independently authenticated initial verifier/provisioner must still exist
before elevation. Authenticated native service enrollment, independent parent
provisioning, protected client registration, installation/loader custody, launcher handoff,
healthy-start receipt, recovery and full installed Windows/boat update sequence
remain required. Fresh store creation is implemented in source but has not passed
native Windows execution or actual enrolled-service qualification. None of these
missing authorities is supplied by the declaration. No service is activated,
hardware control enabled or application/profile changed by its focused tests.

See [private repository operations](staging-update-repository.md) and
[secure updater](secure-updater.md). Native Windows remains mandatory for
installation acceptance; cross-compilation or local signed fixtures cannot
replace it.
