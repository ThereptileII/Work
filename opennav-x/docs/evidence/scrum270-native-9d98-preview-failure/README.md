# Native 9d98 failure audit — SCRUM-270 / 224 / 259

The [native job](https://github.com/ThereptileII/Work/actions/runs/37120549213/job/111196628593) failed its separate full preview suite at `tools/smoke-preview.py:683`: the route-progress comparison accessed `later['route']['remaining_nm']` during a legitimate unavailable state. This is a failed candidate, not Windows or boat acceptance.

Exact application source: local `1835d1b84df89aff42220ac8bb535e4262034a54`, published `9d98a500916e8a7f59dac9735427dde6d3c7d2e5`, remote tree `bc9e631e57c71992c573f5b5110b497096b23867`, run attempt 1. The independent download of artifact **11275626437** is 53,118,284 bytes, SHA256 `eb11a4ab014832e3304a387ada3b3f8afe8195f2d3850ee24c1ccaed1f064adc`; all 13,361 ZIP entries passed CRC checks. The full archive stays private/local rather than duplicating it in Git.

The retained failure diagnostic reports `ActivePointChanged`, `UNAVAILABLE`, waypoint index 2, and omits both remaining distance and arrival SOC. Its observation is 115,094 ms after the recorded window discovery. The screenshot visibly says “Route unavailable”. This matches the deliberate one-second waypoint-transition interval in `DemoSource.cpp`; the harness waited for changed battery SOC without requiring a valid route/arrival sample. The failed in-memory `first`/`later` objects themselves were not retained; the concurrent saved diagnostic, native window inventory and exact traceback are retained here. No product behavior or assertions were changed for this audit.

Passed scope before/after that failure:

- Integrated configure/build/exercise step 8 passed (11:50:43–13:14:36 UTC), including actual private adapter compilation/link and host resource verification. Its CTest report contains 139 tests, zero failures, zero skipped.
- The frozen source's strict adapter package verifier independently passed against downloaded host resources: exact source input closure, source archive, I386 DLL and all **five** exports. Private generation, private prepared resources and host generation contain the same seven files byte-for-byte; hashes are in `audit.json`.
- The raw font probe proves the hosted machine lacked Segoe UI Variable Display; UI sizes 11/23/48 and ordinary chart text selected the approved Segoe UI fallback in the actual GDI DC. Exact source-log lines are retained in `font-proof.txt`.
- Packaged source, staged loader, native pointer gestures and repeat crash recovery passed. Subsequent native 100/125/150% DPI and public ENC gates also passed despite the earlier fixture failure.

The public ENC gate covered **software and permitted software fallback**, not
working native OpenGL. Both recorded runtime observations have
`opengl_enabled: false`; the requested OpenGL phase explicitly reports that the
host rejected it. [The bounded renderer receipt](renderer-proof.json) identifies
the original report's hash and exact observations. Its screenshot filenames do
not override runtime evidence. The replacement candidate needs its own renderer
observation, and the physical boat GPU remains a separate acceptance gate.

The fixture-free product build, actual real-host private-module check, portable recovery/setup, Windows elapsed endurance and early development package were skipped. The archive contains **no `opencpn.exe` or installer**: its only four EXEs are CMake compiler-identification probes. Therefore it cannot support a native preview replay without rebuilding, nor an application/setup PE icon audit. The fixture EXE hash in the preview report is reported identity only, not an independently rehashed executable. Dependency producer logs/receipts do not constitute a qualified reusable application package.

The separate obsolete b8/d5 Linux job [111177754810](https://github.com/ThereptileII/Work/actions/runs/37114216075/job/111177754810) completed successfully with all 29 recorded steps passing, including its three-hour elapsed-time gate and fixture-free build/loader. `obsolete-b8-linux-terminal.json` preserves only that API result; it is not evidence for 1835/9d98.

Retained files include only controlled CI diagnostics, reports and the failure screenshot; no copied profile or vendor payload is included. `audit.json` hashes each retained input. The source repair is tracked separately under SCRUM-270.
