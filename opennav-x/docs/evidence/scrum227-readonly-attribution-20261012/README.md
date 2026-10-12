# SCRUM-227 read-only AIS attribution source evidence

Integrated source: `092c51c6` (full revision and input SHA-256 values in
[source-receipt.json](source-receipt.json)). Exporter `5851818e`, validator
`0d379cfc` plus corrections `733ad47b`, collector `7e085283` and CMake/contract
registration `092c51c6` form one source increment. The
[contract](../../ais-connection-attribution.md) defines the owned observation,
reviewed pins and explicit absence of filtering authority.

## Focused results

The existing offline Linux CMake/Ninja setup configured successfully with UI,
runtime fixtures and pilot loopback disabled. Only
`ais_connection_diagnostics_tests` was compiled. The actual registered CTest
cases passed in two bounded invocations:

- `ais_connection_owned_diagnostics`: **153 C++ checks**.
- `ais_connection_binding_contract`: **31 offline Python tests**.
- `ais_connection_native_collector_contract`: **23 inert Python tests**.

The first two ran at `733ad47b` while the CMake registration was still an
uncommitted integration change; that unchanged registration was subsequently
committed in `092c51c6`. Their source inputs did not change in the intervening
collector integration. Only the new collector case was then run. This avoids
rerunning unchanged checks and does not claim a full application build. Logs,
commands and the initial recorder-only exception are retained alongside this
record. No network or native Windows API was invoked by these tests.

## Independent review and dispositions

Three bounded GPT-6.1 Sol assignments implemented the exporter, validator and
collector in isolated worktrees. A separate read-only pass reviewed the latter
two; root reviewed their integrated boundaries. Confirmed issues corrected
before acceptance of this source increment were:

| Finding | Final disposition |
| --- | --- |
| Local IPv4 `0/8` accepted by portable validator | Reject it; matching observation/TCP regression now fails closed. |
| Full diagnostic decoder exceeded its own 1 MiB maximum | Separate 4 MiB diagnostic-only allowance; strict evidence limits retained. |
| A post-call deadline could lose an acquired native handle or skip cleanup | Register owners immediately and drain without deadline prechecks. |
| Pathname cleanup could delete a replacement output | Retain the original exclusive handle, transfer only a duplicate to Python and mark failure deletion by handle. No pathname unlink. |
| Other-PID exact tuple collisions were dropped before validation | Retain those exact competing tuples alongside selected-PID rows. |
| Single-link assumption could reject Windows component-store DLLs | Stable positive hardlink count allowed only for independently pinned retained binaries; other files remain single-link. |
| Partial IPC frame could escape the parent receive deadline | Bounded queue wait around an owned receiver; terminate only the collector child and drain IPC. |
| Native DLL search needed an explicit boundary | Load the three system DLLs from System32 only; Python 3.8+ required. |

The final independent pass found no further confirmed defect and independently
ran eight inert ownership/deadline/hardlink tests. This is source review and mock
coverage, not proof of Windows ABI or API behavior.

Claude **did not review this AIS increment**. The earlier completed
[Claude updater review](../scrum311-acquisition-claude-comparison-20261012/README.md)
identified a similar repeated-cleanup/replacement-file hazard. Root applied that
lesson here; this is not a same-snapshot Claude/Sol comparison. The expanded
private broker-source review remains pending exact-scope transfer approval and
no additional private source was sent.

## Open acceptance boundaries

Actual native Windows process/module/TCP/file-lock/disposition/watchdog behavior,
complete independently reviewed installed module pins, wx diagnostic publication
and live boat attribution remain unqualified. Disk hashes do not authenticate
mapped bytes, and two observations do not exclude transient module loads. A
successful observation never authorizes filtering. Existing inert-only WFP
guards are unchanged.

SCRUM-227 and SCRUM-16 remain In Progress. No full application build, CI dispatch,
push, service activation, boat deployment, firewall change, traffic interruption,
credential read, visual/endurance suite or physical command occurred.
