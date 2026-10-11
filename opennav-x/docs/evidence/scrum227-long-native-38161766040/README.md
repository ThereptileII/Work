# SCRUM-227 long inert WFP proof — native run 38161766040

[Run 38161766040](https://github.com/ThereptileII/Work/actions/runs/38161766040)
and [job 114535990532](https://github.com/ThereptileII/Work/actions/runs/38161766040/job/114535990532)
passed at exact source `4f423a8000378abc502406d85ba4bde0fb187ad1`.
The actual result and complete native logs establish the bounded long
packet-loss/deadman primitive for **inert IPv4/IPv6 loopback echo clients only**.
This is not real AIS provider or boat acceptance.

## Integrity and native identity

Artifact `11710626983` is 731,977 bytes; downloaded ZIP SHA-256
`f259e9dc75fe0a266f92aabf1caf1b9a7533c6c928d029c93a6a39dade8a2cd7`
matches its GitHub API digest. All 22 member CRCs pass. All three executable
SHA-256 values match the result, and direct PE-header inspection identifies
each as machine `0x014c` (x86/Win32). The compiler receipt identifies MSVC
19.44.35229 for x86, toolset 14.44.35207, on Windows Server 2022 build 20348.
The exact four source hashes match the run commit with Windows CRLF checkout
line endings; their Git-blob hashes are recorded separately, without pretending
the two byte representations have identical hashes.

[`artifact-verification.json`](artifact-verification.json) records the download
and each retained member's hash/CRC. [`log-audit.json`](log-audit.json) records
independent full-log checks, counts, identities, timing boundaries and limits.
`raw/` retains the actual result, all eight complete client event logs, both
complete helper/readback logs, both cleanup audits and three compiler logs,
plus API and compact job-launch receipts. Executables, object files and the ZIP
remain only in the private directory recorded in the verification receipt.

## Observed outcomes

Each phase began with six actual healthy echoes on all four clients, with fresh
processes/listeners for the second phase. Each marker's generation-1 established
flow failed after commit; retries continued while controls used the same
loopback addresses/port under their distinct executable path.

| Observation | Deadman expiry | Forced helper death |
| --- | --- | --- |
| Runner measured quiet window | 122.000 s | 122.000 s |
| IPv4 / IPv6 failed retries in that window | 1,112 / 1,113 | 1,112 / 1,112 |
| Successful marker echoes / new connections in that window | 0 / 0, both families | 0 / 0, both families |
| Maximum retry gap in that window | 125 ms, both families | 125 ms, both families |
| Helper outcome | Native watchdog exit 124 after 135.047 s | Retained-handle kill exit 1 after 123.047 s |
| Phase control echoes, IPv4 / IPv6 | 1,234 / 1,234 | 1,122 / 1,124 |
| Control connections / errors | One / zero, both families | One / zero, both families |
| Maximum phase control gap, IPv4 / IPv6 | 156 / 156 ms | 157 / 125 ms |
| Fresh exact-GUID audit | Both filters and sublayer absent | Both filters and sublayer absent |
| Marker recovery | Both families echo on generation 2 | Both families echo on generation 2 |

The full native logs also independently show zero marker echoes/connections
from commit plus one-second grace through the selected native boundary:
133.969 seconds for expiry and 122.016 seconds for forced death. Each family
records 1,222 and 1,112 failed retries respectively within those intervals, with
maximum gaps of 125 ms including interval edges. These native-tick intervals
are independently derived; they differ from the runner's received-monotonic
122-second window and its counts.

The helper logs contain two complete strict filter readbacks per phase. Expected
and actual keys, layers, sublayers, block actions, flags, six conditions and
weight type match. The conditions bind only the disposable marker path, TCP,
the selected ephemeral remote port, literal local/remote loopback addresses and
loopback flag. Explicit `FWPM_FILTER_FLAG_INDEXED` is 64; no unexpected flags or
condition fallback were accepted. Assigned filter IDs and effective weights
are BFE metadata, not requested-condition identity. The six owned GUIDs across
the two phases are distinct. Each fresh audit queried its exact three GUIDs,
returned exit zero and `owned_objects_absent: true`, followed by the runner's
new-connection echo check. Finalization errors are empty.

## Bounds and remaining gates

The native helper limit is fixed at 135 seconds, the independent helper-only
job timer at 150 seconds, and fresh clients at 180 seconds. The watchdog expiry
was exercised. Neither the 150-second job timer nor the 180-second client timer
expired in this run; they were configured, not qualified as exercised kill
paths. Both phases record `runner_job_deadline_fired: false` and no job errors.

The nominal native watchdog tick is a no-early-success cutoff, not a precise
real-time cleanup timestamp. Audit files contain no independent audit-completion
clock. Exact-GUID absence and post-audit recovery ordering come from the
source-bound runner's checks; boundary-to-recovery differences are not exact
process-death-to-recovery measurements.

WFP application identity remains path-based. PID/creation-time/hash guards do
not turn it into kernel PID/hash/module-specific filtering. Omitting the old
local port deliberately covers retries. No real product executable, AISStream,
DNS-address ambiguity, credential, TLS, public endpoint, remote-access path,
onboard AIS continuity, provider aging/reconnect, commissioning or boat action
was exercised. The separately accepted actual-provider report-silence result
remains distinct. This receipt qualifies the bounded inert primitive and does
not close SCRUM-227's wider product/boat gates.

This evidence task performed read-only downloads and offline inspection only:
no native rerun, build, deployment, workflow/status/older-evidence edit or Jira
mutation.
