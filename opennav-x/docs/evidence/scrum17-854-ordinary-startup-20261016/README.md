# SCRUM-17 / 208 / 209: actual 854 startup, with builtin plugin refusal

The same installed `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`
process progressed after the user made the ordinary Windows desktop available.
No second launch, coordinator welcome acknowledgement, rebuild or physical
command occurred. This observation **does not accept the candidate**: four
rebuilt builtin plugins failed the production dependency guard.

## Observed working boundaries

The process was PID 11052, created at `2026-10-16T11:47:20.3603526Z`, with
executable SHA-256
`8ad205a452683316330549bb93a5d0c039b2a2acb4d59eaa6aa4c5d3b85bc511`.
Fresh, stable diagnostics at 13:16 UTC identify the fixture-free installed
product and initialized, enabled o-charts. The verified SKAGER private chart
presentation is available. The real chart database contains 583 entries and
the current quilt has one encrypted vector-chart member, with OpenGL enabled.
The native 13:18 capture shows actual land, shoreline, water, chart objects and
ownship, not an all-water or blank canvas. Its application rectangle is
1920×1008 physical pixels at 144 DPI; the chart is 1521×665. This is functional
content observation, not prototype, touch or all-resolution acceptance.

Selected navigation position and SOG, STW, apparent wind, depth and water
temperature have fresh measured data. Heading is correctly **estimated** from
magnetic heading and variation. Missing COG, propulsion/battery and other
unavailable inputs remain unavailable. The tank records include distinct live
and aging observations. No numerical vessel positions or private chart material
are committed with this receipt.

Online AIS is connected with subscription confirmed, 766 targets/chart marks
and 3,591 accepted reports at this observation. This is not reconnect, target
interaction, source precedence or local-AIS acceptance. The physical pilot
status is fresh Standby; `enabled`, `serial_session_enabled` and
`control_capability` are false, and command ID is `0`. The separate
`configured_permission=true` flag is not reported as false or as proof of
enabled output. The commissioning connections remain input-only. No physical
command was sent.

An explicit x86 Windows PowerShell 5.1 observer enumerated 121 modules, including
exactly the five selected owned libraries: libcurl, libssl-3, libcrypto-3, zlib1
and skager-ocharts-adapter. Their full paths and on-disk hashes match the pinned
installed generation inventory. Neither legacy OpenSSL basename was observed.
This is a bounded point-in-time module/path and disk-byte observation, not
loaded-memory hashing or proof covering every dependency/child process. The
earlier x64 observer returned an empty module collection without an error; that
result **cannot establish absence** and is superseded only for this module
observation by the explicit x86 result.

## Actual builtin failure

The bounded original log records the same refusal for `chartdldr_pi.dll`,
`dashboard_pi.dll`, `grib_pi.dll` and `wmm_pi.dll` at local 15:07:09–11:

```text
Plugin import closure node budget exceeded
```

Each is followed by `PluginLoader: Cannot load library`. The immutable source
inventory comparison subsequently finds exactly these four rebuilt owned files
absent. These are distinct from the four original plugins intentionally held by
the commissioning quarantine. The unchanged tree guard correctly refuses the
new welcome inspection before UI input.

Exact packaged upstream `gui/src/load_errors_dlg.cpp:133–150` can remove failed
libraries after an OK/Yes dialog result. Its SHA-256 is
`b463140f92b11a11e42b30d67deae963ac981b8ba7c1ac4a440a48d612d4531f`.
No actor or actual modal result was observed, so the removal is not attributed
to the user, Defender or a particular interaction. Ordinary duplicate handling
does not delete these DLLs.

Inert graph analysis of authenticated 854 package bytes reproduces the limit:
two distinct roots require 178 candidate probes; adding an existing empty third
root requires 267. With the two exact retained managed CRT files, the walk
requires 270 probes (62 found candidates, 208 misses). The original guard stops
before probe 257. Cycles already deduplicate. This analysis assumes the recorded
separate plain directories have distinct physical identities; it is not native
execution of the guard. It explains why the smaller CI topology passed while
the real installation refused. A bounded correction and new exact-revision
qualification remain required; no unsafe-TLS refusal is waived.

## Shutdown and remaining acceptance

The exact original process accepted normal close at 13:22:40 UTC. A retained
native process handle establishes exit code **0**. The 13:23:06 read-only check
finds no application/chart helper or running SKAGER task, with SSH, Tailscale
and RustDesk services still running. No force termination was used.

At 13:37:57 UTC, separate cold restoration recreated exactly those four missing
owned DLLs from the authenticated original 854 recovery ZIP. Every published
file matches its original size/hash; profile bytes, the original inventory and
commissioning quarantine remained unchanged. The completion SHA-256 is
`316bbdbde1cea2b38eb1494391f081f8765fb883e6f8f85373ca3a1e2df08a22`.
New files inherit their unchanged parent directory ACL; the deleted files'
individual ACLs are not claimed restored. An initial private recovery wrapper
refused before its mutation journal because PowerShell 5.1 nested a JSON array;
the corrected per-tree clone passed a native inert shape check before use.
Both attempts are preserved.

The original commissioning `InspectRestore` then passed against its unchanged
tree inventory. The post-session INI is archived and remains exactly
`494fc03d3f94d7fc15781446220df8ab208368380ac1cf8f4f9fe9330b0bfad8`.
Source-reviewed changes comprise nine display/cache/last-view fields in addition
to the intentional input-only connection change; no installed resource-location
adoption is involved. Ordinary baseline restoration is in progress and must
preserve every current non-INI profile file. File recreation is not a successful
plugin load. The original failed session and all private evidence remain kept.
No automatic relaunch, Production promotion or public release is authorized by
this record. Legacy/Safe transitions and complete ordinary builtin loading
remain unaccepted for this boat session.
