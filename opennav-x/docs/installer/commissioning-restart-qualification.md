# Native commissioning restart tooling qualification

The isolated tooling at exact commit
`faced2585c3eccafb8240fc80e75149a83895788` passed all three jobs in
[run 36283660475](https://github.com/ThereptileII/Work/actions/runs/36283660475).
This qualifies the tested marker-process and PowerShell boundaries. It does not
qualify an installed OpenCPN generation, real plugin shutdown, the product's
in-app mode controls, or operation on the boat.

The complete downloaded artifacts, their CRCs, and every SHA-256 were checked
against both GitHub's artifact metadata and the corresponding upload log.
Sanitized machine-readable results and full hashes are in
[the evidence record](../evidence/beta2-commissioning-restart-faced258.json).
Raw temporary paths, account identities and process identifiers remain outside
this public record.

## Results

| Native Windows gate | Passed |
| --- | ---: |
| Actual linked restart protocol | 886 checks |
| Actual helper, parent handles, named pipes and marker-only children | 359 checks across 24 cases |
| Existing maintenance, source, preparation and launch policies, plus restart policies | 690 checks across nine suites |
| Actual broker and full temporary commissioning audit | 49 checks |
| Actual Prepare and scheduled Arm/Collect entrypoints | 46 checks |

The maintenance total includes 273 restart-policy checks and 77 AUI/Dashboard
persistence checks. The compiler was MSVC 19.44.35229.0, with Win32/x86 test
processes on Windows Server 2022 x64 and native Windows PowerShell 5.1.

The full broker successfully authorized one verified marker child. Changed
output connections, changed plugin bytes, an expired session and a previously
consumed transition each produced no child. A deliberately unavailable durable
receipt record preserved an uncertain consumed permit after one child started;
it did not retry or claim accepted completion.

The separate preparation test began without a constructed restart session.
The actual Prepare command copied and verified the baseline and shutdown
review, created its private ACL, and pinned every dependency without changing
the profile or independent audit. An owned build record without protocol
capability was refused before session creation.

The actual Arm command started the fixed, limited, interactive PowerShell task.
Wrong parent identity, duplicate arming and collection while the task was still
running were refused. Actual Collect removed only the owned completed task and
published its durable record, both after success and after output-change
refusal. Windows account-name-to-SID normalization and null-trigger handling
passed against actual Task Scheduler metadata.

## Tested source boundary

The copied Prepare, Arm/Collect and Broker entrypoints were byte-identical to
their native source checkout. Their recorded hashes were independently matched
to the exact source blobs with Windows CRLF checkout normalization:

| Entrypoint | SHA-256 of native checkout and executed copy |
| --- | --- |
| `RestartCommissioningPrepare.ps1` | `bbc25e3933ef8e085484c7ff3203d452258ab6609504ae285eaa0875af8f31f9` |
| `RestartCommissioningArm.ps1` | `7bf28fac37ed2eb18a78c9d26a4cdcdaf721f1de54fe88a944fd8a18693bf795` |
| `RestartCommissioningBroker.ps1` | `fa765c32ff48af981d3ab0f0fb0a259c52bdcf4dc34a0e49ed10d03156c07664` |

Only copied test identity dependencies substituted the installation and OS
known-folder locations inside marked temporary trees. Complete plugin/helper
inventory, quarantine, typed profile deltas, process ownership, creation times,
pipe peer checks and single-use permit/receipt logic remained active. Marker
programs contained no marine code, and inert plugin files were never loaded.

The marker parent and its initial cold-child journal were fixture inputs. This
does not replace a test of the real product's cold launch. No SSH, boat access,
physical actuator output, real chart, or navigation approval is part of this
qualification.

## Remaining gates

Any later explicit running-task verification, fixed UI mode request, or child
review handoff requires qualification at its own exact commit. Those changes
were not present in this run.

Before boat mode-transition testing, qualify the exact fixture-free packaged
application and helper, verify their capability metadata, inspect the final
installed-generation plugin/helper inventory, renew the full cold commissioning
review, and test actual in-app transitions with normal shutdown persistence.
The existing read-only commissioning, restoration and remote-recovery rules
remain in force.
