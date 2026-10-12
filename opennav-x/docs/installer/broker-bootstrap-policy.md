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

## Remaining installation boundary

An independently authenticated initial verifier/provisioner must still exist
before elevation. The actual native service enrollment, protected root creation,
service-principal initialization, installation/loader custody, launcher handoff,
healthy-start receipt, recovery and full installed Windows/boat update sequence
remain required. None is supplied by this declaration. No service is activated,
hardware control enabled or application/profile changed by its focused tests.

See [private repository operations](staging-update-repository.md) and
[secure updater](secure-updater.md). Native Windows remains mandatory for
installation acceptance; cross-compilation or local signed fixtures cannot
replace it.
