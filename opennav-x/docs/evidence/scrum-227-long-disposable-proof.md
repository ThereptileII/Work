# SCRUM-227: longer disposable WFP deadman proof — native pending

This increment prepares the missing long packet-loss/deadman primitive selected
in SCRUM-227 comment 11320. It is test-only and cannot select a product process,
public address, AIS credential or boat network. **No native result is claimed.**

The coordinator base is `c63dee40445f4b084468e3076c7d33a90eea118a`. That tree
retains the accepted short-proof receipt but not its helper files. The five
helper/runner/validation files were recovered individually from
`97142aa1` and verified byte-identical to the corresponding product-root files
at `a6bfb805` before this extension. No old branch was merged or cherry-picked.

The accepted short primitive is recorded in
[`scrum-227-disposable-native-proof.json`](scrum-227-disposable-native-proof.json):
native run [37047212685](https://github.com/ThereptileII/Work/actions/runs/37047212685)
proved short established-flow interruption, blocked retries, independent
loopback control continuity, normal/forced helper exits, exact owned-GUID absence
and new-connection recovery. Its 15-second helper ceiling did not cover the
61/121-second provider aging gates. This longer increment does not rebuild or
repeat that accepted run.

## Fixed long mode

The already-elevated disposable GitHub-hosted Windows runner requires the same
exact environment/path/hash/administrator/BFE guards as the accepted primitive.
The coordinator must review this source before any separate native dispatch:

```
python tools/test-windows-ais-outage.py --output <fresh-directory-below-RUNNER_TEMP> --confirm-disposable --long-deadman
```

There are no target-address, product-path, policy-import or duration arguments.
The long switch adds `OUTAGE_LONG_PROOF=1` to the three small MSVC builds only.
The default short mode retains its 15-second helper and 60-second client limits.

| Long-mode bound | Fixed value and assertion |
| --- | --- |
| Native helper watchdog | 135,000 ms, independent of stdin, socket calls and BFE RPC; `TerminateProcess` exit 124 |
| Runner helper-only job | Independent 150-second timer closes the kill-on-close job; firing always fails the proof |
| Native marker/control ceiling | 180,000 ms; fresh clients and listeners for each phase |
| Queued-delivery grace | One second, recorded separately from measured blockage |
| Measured blocked-retry window | At least 122 seconds, zero marker echoes/connections, established-flow failure, live clients, failed retry gap strictly below two seconds |
| Preparation / activation waits | Four / three seconds |
| Exact-GUID cleanup audit | Fresh engine, bounded three-second retry period and three-second maximum per audit process |
| Recovery | Each family must echo on a newer connection after audit, within five seconds |
| Event evidence | At most 4,096 events per process; overflow, log-write failure or unfinished reader fails finalization |

These are fail-closed test deadlines, not a hard-real-time guarantee under an
unresponsive operating system. The runner job is assigned before `arm`; no WFP
objects are created before assignment. The native watchdog is armed before BFE
preconditions, process identity checks or engine work. Job close and owned-handle
termination also run on failure. The runner retains failed compile, native log,
behavior, cleanup and evidence-copy results. A failed finalization cannot produce
an overall pass.

## Two separately classified native outcomes

Each phase establishes six healthy echoes on both marker and both distinct-path
control processes. WFP still uses exactly the accepted six conditions per
family: marker application path, TCP, OS-selected ephemeral remote port,
literal local and remote loopback addresses, and loopback flag. It still creates
only two block filters and one unique dynamic sublayer, transactionally, with
strict readback including explicitly requested `FWPM_FILTER_FLAG_INDEXED`.

1. **Deadman expiry:** after the 122-second blocked window the runner leaves
   stdin open and sends neither stop nor kill. The helper must independently
   exit 124 within the fixed lifetime bounds. Native event ticks reject any
   marker echo or connection between the one-second commit grace and the
   nominal native watchdog deadline, including successes received later through
   a buffered pipe. Only exact owned-filter/sublayer absence and new-connection
   recovery establish successful cleanup.
2. **Forced helper death:** after the same blocked window the runner terminates
   only its retained helper process handle. Exit must be 1 and must precede the
   native deadline with at least four seconds of separation; exit 124 or normal
   exit cannot masquerade as the selected forced-death path. The same fresh
   exact-GUID audit and subsequent new-connection recovery are required.

The nominal native watchdog tick is a no-early-success cutoff, not a precise
real-time cleanup timestamp. The independent absence query and subsequent
recovery provide cleanup evidence after the observed helper exit.

Both IPv4 `127.0.0.1` and IPv6 `::1` must pass. Controls use the same loopback
addresses/port under another executable path. Through blockage, helper death,
audit and recovery they must keep one connection, no errors and maximum echo
gap below two seconds. Missing IPv6, premature helper/client exit, intermittent
marker success, stopped retries or a fired runner job deadline fails closed;
there is no broader-filter fallback.

Microsoft documents [ALE reauthorization on policy change](https://learn.microsoft.com/en-us/windows/win32/fwp/ale-re-authorization),
[dynamic-session removal on session end/process death](https://learn.microsoft.com/en-us/windows/win32/fwp/object-management)
and [kill-on-close job ownership](https://learn.microsoft.com/en-us/windows/win32/procthread/job-objects).
Those contracts motivate the assertions; they do not replace native evidence.

## Local evidence and limits

[The recorded portable receipt](scrum-227-long-disposable-portable.json) binds the
exact tested source hashes and recovered-lineage comparison. Eleven portable
test methods passed on Linux. Their negative cases reject late
marker success, stopped retries, dead clients, a short window, missing/precommit
flow failure, early/late/wrong-code watchdog outcomes, confused forced-kill
outcomes, a buffered native-clock success after 122 seconds, log-reader failure
and incomplete log finalization. A real Linux invocation retained a failed
`result.json` with `native Windows required` and launched no runtime. These checks
validate refusal/evidence decisions only. They do not simulate a passing WFP run.

Native MSVC compilation, actual >122-second IPv4/IPv6 interception, expiry/death
cleanup, controls and recovery remain pending at the exact integration commit.
The coordinator owns that separately authorized small native qualification run.

This is a path-scoped inert echo primitive. PID/creation-time/hash checks guard
the selected disposable processes but WFP application identity remains a path;
the filter deliberately omits the old local port so new retries are covered.
No process-specific kernel identity, executable/module custody, DNS ambiguity,
actual AIS provider packet-loss aging/reconnect, TLS, onboard AIS, remote-access
continuity, commissioning or boat acceptance is established. The separately
accepted real-clock report-silence fixture at `8be05c69` remains distinct from
packet loss and helper death. SCRUM-227's wider gates remain open.
