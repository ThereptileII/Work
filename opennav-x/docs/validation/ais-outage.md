# SCRUM-227: disposable Windows WFP feasibility proof

This is an unqualified, standalone loopback experiment. It cannot target
OpenCPN, AISStream, a boat address or a public endpoint. It contains no AIS
credentials, TLS changes, OpenCPN code or marine protocol messages. Native
behavior is deliberately an assertion to measure, not an assumption: if Windows
allows the established loopback flow or retry through these filters, the proof
fails and retains its evidence. No broader filter fallback is permitted.

## Invocation and build boundary

In an already-elevated, disposable GitHub-hosted Windows runner with the MSVC
x86 developer environment active:

```
python tools/test-windows-ais-outage.py --output <fresh-directory-below-RUNNER_TEMP> --confirm-disposable
```

Required exact environment values are `CI=true`, `GITHUB_ACTIONS=true`,
`RUNNER_ENVIRONMENT=github-hosted`, `XNAV_DISPOSABLE_AIS_OUTAGE=1`, plus
`RUNNER_TEMP` and the exact checked-out `GITHUB_SHA`. The runner records Windows,
compiler, administrator and BFE observations. BFE must already be running; no
service or firewall policy is enabled, reset, imported or otherwise changed.
These operational CI guards are not an attestation against a malicious local
administrator spoofing the environment.

The runner invokes `cl.exe` directly on these three small native executables:

- `xnav-ais-outage-marker.exe`: fixed-loopback echo client.
- `xnav-ais-outage-control.exe`: same protocol under a distinct executable path.
- `xnav-ais-outage-filter.exe`: guarded dynamic WFP filters and read-only audit.

They use C++17, the static MSVC runtime and Windows SDK libraries only. Python's
standard library supplies the orchestration and two loopback echo listeners.
No application, OpenCPN dependency or external network service is built or used.

## Scope and lifetime

All binaries run in a fresh `RUNNER_TEMP/xnav-ais-outage-<32 hex digits>` directory.
Native guards repeat the CI and canonical path/name checks. The helper retains
handles to both live marker processes, compares each actual image path with its
exact sibling marker executable, verifies its expected SHA-256, and holds the
file open against write/delete replacement. WFP's application ID is path-based;
this is not a claim of kernel PID- or binary-hash-specific filtering.

One dynamic WFP session creates a unique sublayer and two `FWP_ACTION_BLOCK`
filters in a single transaction. Both filters require the exact marker app ID,
TCP, the OS-selected ephemeral remote port 49152–65535, local AND remote literal
loopback address, and the loopback flag. The layers are
`ALE_AUTH_CONNECT_V4` and `ALE_AUTH_CONNECT_V6`; no allow, stream-layer, persistent
or boot-time filters are installed. IPv6 uses `::1` and an IPv6-only listener;
IPv4 uses `127.0.0.1`. Addresses and ports passed to WFP are in host byte order.
Every installed condition, action, flags, layer and sublayer is read back.

The runner assigns a helper-only kill-on-close job **before** sending `arm`.
The helper also terminates itself after 15 seconds independently of stdin/BFE
work; the marker clients terminate themselves after 60 seconds. Normal stop
closes the dynamic session. The second phase terminates the retained helper
process handle, requires the forced exit code, and exercises RPC-rundown cleanup.
A fresh engine then queries both filter GUIDs and the sublayer GUID. Only exact
`FWP_E_FILTER_NOT_FOUND` / `FWP_E_SUBLAYER_NOT_FOUND` results establish absence;
access errors, an audit timeout or remaining objects fail. No preexisting rule
is removed. Cleanup is attempted even when a behavioral assertion fails.

## Measured assertions and evidence

Before filtering, both marker families and both independent control families
must establish actual connections and receive at least six echoes. Both control
processes use the SAME remote ports/addresses as the markers, so their continued
traffic tests the application condition rather than merely a port difference.

Each normal/forced-death phase records the old connection generations, healthy
state before arming, and native filter commit time bounds. It requires the old
flows to fail after commit began, then allows a recorded one-second drain grace
and measures a 2.5-second window containing zero successful marker echoes or new
connections and at least two failed retries per family. Markers must remain
alive and attempting traffic. After filter removal, both families must recover
on new connections. Controls must keep one connection, no errors and less than
a two-second maximum echo gap through the phase. Filters must still be alive
before the selected cleanup path; deadline expiry cannot masquerade as forced
death. Missing IPv6 support is failure, never a skipped family.

`result.json` binds source commit/GitHub SHA, exact source and executable hashes,
compiler commands, OS/admin/BFE observations, phase decisions and cleanup audits.
`runtime/` retains binaries, compile logs and timestamped native event logs,
including failures. Source and binary identities do not prove the behavioral
assertions; those require a successful native run. The output is unsuccessful
if process cleanup or evidence retention fails.

Local validation at implementation: Python syntax plus 12 pure guard/CLI
checks passed, including Linux refusal with a retained failure JSON and no
runtime launch. Native MSVC compilation, loopback interception, both cleanup paths
and control continuity remain **pending**. No Windows test or CI was dispatched
from this implementation task.

Even a passing disposable proof would not qualify actual AISStream connection
loss/aging/reconnect, DNS-address changes, TLS transport, onboard AIS continuity,
SSH/Tailscale/RustDesk continuity, boat operation or real-executable scoping. Those
remain separate acceptance gates; this tool offers no argument to select them.

Microsoft references:

- [ALE reauthorization](https://learn.microsoft.com/en-us/windows/win32/fwp/ale-re-authorization): policy changes reauthorize existing flows on subsequent packets, including opposite-direction packets.
- [WFP object management](https://learn.microsoft.com/en-us/windows/win32/fwp/object-management): transactions and dynamic-session object removal.
- [Available filter conditions](https://learn.microsoft.com/en-us/windows/win32/fwp/filtering-conditions-available-at-each-filtering-layer): supported ALE application/address/protocol/port conditions.
- [Condition identifiers](https://learn.microsoft.com/en-us/windows/win32/fwp/filtering-condition-identifiers-): application-path identity and address representation.
