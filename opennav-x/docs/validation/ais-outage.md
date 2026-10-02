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

The first dedicated native attempt,
[run 37044564784](https://github.com/ThereptileII/Work/actions/runs/37044564784),
job `110962660696`, commit `a79d9184df09c30084632538dc3d30a1dfe115e9`,
failed during helper compilation before filtering. MSVC 14.44 with Windows SDK
10.0.26100 rejected `wchar_t*` as the `RPC_WSTR` (`unsigned short*`) argument to
`UuidFromStringW` (C2664). The bounded correction explicitly casts the existing
Windows wide-character buffer to the SDK's RPC pointer type; no filter behavior
or guards change. Downloaded artifact `11243223347` is retained locally at
`evidence/local/a79d-native/` (397423 bytes, SHA-256
`4251e44033803582cbf7243bd736c8076ada0fd0648a6ef2e4efed833aa6d6c7`).
Native compilation and behavioral proof remain pending after this correction.

The next native attempt,
[run 37045053523](https://github.com/ThereptileII/Work/actions/runs/37045053523),
job `110964298147`, commit `e64f448bed1aa22587d3431054f9abce0bb63e67`,
compiled successfully but refused its committed filters at strict readback,
before emitting `active`. The original diagnostic did not identify which field
differed. A fresh audit confirmed both owned filters and the sublayer absent
after helper exit. Retained artifact `11244122828` is 647838 bytes, SHA-256
`2609447b744d2dce14f93672102f1b5911ea50ab3ed111b3ff24a790c3b39306`, under
`evidence/local/e64f-native/`.

The diagnostic-only follow-up logs both families' complete expected/actual
condition GUIDs, match types, value types and values, plus action, flags, layer,
sublayer and count before the same strict decision. Byte blobs are hex encoded
and bounded at 64KiB with explicit truncation; fixed address/mask values are also
supported for diagnosis. That diagnostic revision accepted neither alternate
address representations nor added flags. The runner now surfaces helper exit/code/native error promptly instead
of waiting for the generic preparation/activation deadline. Cleanup audit remains
mandatory on this failure path.

Microsoft documents [address value alternatives](https://learn.microsoft.com/en-us/windows-hardware/drivers/network/filtering-condition-data-types)
and [BFE-assigned filter metadata](https://learn.microsoft.com/en-us/windows/win32/api/fwpmtypes/ns-fwpmtypes-fwpm_filter0).
This does not prove which, if any, canonicalization occurred in this run; the
next native readback must establish the actual discrepancy before any matching
policy changes. Local follow-up checks cover Python syntax and three bounded
runner event/early-exit cases only; native diagnostic output remains pending.

The diagnostic native attempt,
[run 37046395691](https://github.com/ThereptileII/Work/actions/runs/37046395691),
commit `ea561e4366341fe4809dcbab20a486833735a03a`, establishes the exact mismatch:
both families returned identical condition fields/types/values/match types,
action, layer, sublayer and count; the only compared difference was flags 0
requested versus 64 returned. Assigned filter IDs and effective-weight metadata
were already excluded from comparison. Artifact `11244286833` is 652913 bytes,
SHA-256 `965778d700607fe0f0bfff88309c4a6d294481cd7d46c80cc4098e5f8b995b43`,
retained under `evidence/local/ea561-native/`.

Microsoft defines [FWPM_FILTER_FLAG_INDEXED](https://learn.microsoft.com/en-us/windows/win32/api/fwpmtypes/ns-fwpmtypes-fwpm_filter0)
as a lookup optimization available from Windows 8/Server 2012; the
[Microsoft SDK header](https://github.com/microsoft/win32metadata/blob/main/generation/WinSDK/RecompiledIdlHeaders/shared/fwpmtypes.h)
defines its value as `0x00000040`. The correction explicitly requests that named
flag on both filters and requires `actual.flags == expected.flags`. Unknown,
missing or extra flags still fail; no blanket mask is introduced. The standalone
proof's SDK target becomes Windows 8 (`_WIN32_WINNT=0x0602`), consistent with its
disposable Windows 2022 runner. Product build targets do not change. Conditions,
actions, dynamic-session lifetime and complete diagnostic logs are unchanged.
Local validation inspected both retained readback records, checked Python syntax
and reviewed the diff. Native behavior after this metadata correction remains
pending; the observed metadata alone does not prove interruption or recovery.

Even a passing disposable proof would not qualify actual AISStream connection
loss/aging/reconnect, DNS-address changes, TLS transport, onboard AIS continuity,
SSH/Tailscale/RustDesk continuity, boat operation or real-executable scoping. Those
remain separate acceptance gates; this tool offers no argument to select them.

Microsoft references:

- [ALE reauthorization](https://learn.microsoft.com/en-us/windows/win32/fwp/ale-re-authorization): policy changes reauthorize existing flows on subsequent packets, including opposite-direction packets.
- [WFP object management](https://learn.microsoft.com/en-us/windows/win32/fwp/object-management): transactions and dynamic-session object removal.
- [Available filter conditions](https://learn.microsoft.com/en-us/windows/win32/fwp/filtering-conditions-available-at-each-filtering-layer): supported ALE application/address/protocol/port conditions.
- [Condition identifiers](https://learn.microsoft.com/en-us/windows/win32/fwp/filtering-condition-identifiers-): application-path identity and address representation.
