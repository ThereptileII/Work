# Current stock launch: renderer evidence

`tools/boat/observe-stock-renderer.ps1` reads only existing process, receipt and
log metadata. It does not start/stop an application, dispatch interactive work,
change a setting, capture unrelated windows, or send input/equipment commands.
It is separate from permission to launch or acknowledge a startup notice.

Use the exact successful official `LaunchStock` receipt and independently
verified cold-recovery log copy:

```powershell
.\observe-stock-renderer.ps1 -Workspace <workspace> `
  -LaunchResult <owned-launch-result.json> -ExpectedLaunchSha256 <sha256> `
  -ExpectedLaunchRequestSha256 <independently-recorded-request-sha256> `
  -BaselineLog <cold-backup-opencpn.log> -ExpectedBaselineSha256 <backup-manifest-sha256>
```

The request hash parameter is optional for a read-only observation; supply it
when available. The result explicitly records whether it was independently
pinned. In either case the request must agree with the pinned successful launch
receipt, target hash, empty arguments, exact stock executable, workspace and
owned result location. Installed/portable/restarted-child launch identities are
not accepted by this stock observer. The real shared profile path comes from
Windows' CommonApplicationData and must agree with the target configuration.
The exact running PID, creation ticks, executable and interactive session are
checked before and after the read; only one OpenCPN process may be present. The
operator may run this metadata observer through SSH under the same user SID;
it does not require or manipulate the foreground desktop.

The baseline must be a separately preserved file, not the current log or rotated
log. Its expected hash must come from the verified backup/cold-copy record.
The tool cannot establish when an arbitrary caller-supplied copy was made.
Only fixed `opencpn.log` and `opencpn.log.log` names in the real profile are read.
The existing shared `Read-StartupLogBytes` limit is 4 MiB per file. SHA-256 values
are calculated from those same byte snapshots. No raw log lines, chart paths,
coordinates, AIS identities, user SID or unrelated process metadata is exported.

Two histories can qualify:

- **EXACT_APPENDED_PREFIX:** all cold-log bytes are an exact prefix of the current
  log; only the appended bytes are considered.
- **EXACT_ROTATED_BASELINE:** the old log exceeded 1,000,000 bytes, the rotated
  file exactly matches its independently pinned hash, and the current log holds
  the new startup. This follows pinned `BasePlatform::InitializeLogFile`, which
  otherwise opens the existing log in append mode.

Unexplained replacement, stale content, a changed backup/rotation, multiple new
startup banners or malformed evidence reports **unknown**. The complete new
banner must identify official `5.12.4-0+37fd0cd`. Its date plus logger clock are
converted using the boat PC's actual timezone and must be within one second
before to 120 seconds after the process creation time. The receipt lasts at
most four hours. The logger's `UNow()` value still uses local-time getters;
the `HH:mm:ss.fff` prefix is not a UTC stamp. Missing/ambiguous DST hours are
unknown; crossing ordinary local midnight is handled explicitly. Marker times
outside the observed session refuse. Incomplete final writes are ignored and
can be observed again without altering the application.

Only these source-bound markers are exported:

- `glChartCanvas.cpp`: renderer, OpenGL version, GLSL version, late minimum-symbol
  line-width marker, or explicit initialization failure. Values are bounded and
  restricted to printable renderer/version syntax.
- `ocpn_frame.cpp`: `OnInitTimer...Finalize Canvases`.
- `OCPNPlatform.cpp`: capability-probe success/failure, kept separate from the
  canvas evidence.

A complete renderer/version/GLSL/late-setup sequence reports
**CANVAS_CONTEXT_INITIALIZED_DURING_LAUNCH**. It does not assert that OpenGL is the
current backend after later setting changes, that the driver actually uses GPU
hardware, or that chart content rendered correctly. These remain explicit false
verification fields. Multiple bounded canvas contexts are retained separately.
A capability probe or an `OpenGL=1` preference cannot supply canvas evidence.
Absent markers mean unknown, not software rendering. The pinned implementation
also overwrites its `Software OpenGL` message before logging it, so absence of
that message is not a hardware-acceleration test.

Pair this report with the same running process's qualified native chart capture
and observed chart interaction before reporting boat rendering results. The
renderer string is useful evidence for identifying the driver; it is not
navigation certification or a physical GPU execution measurement.

Source boundaries: pinned OpenCPN `model/src/base_platform.cpp:627`,
`model/src/logger.cpp:58`, `gui/src/ocpn_app.cpp:1177`,
`gui/src/OCPNPlatform.cpp:913`, and `gui/src/glChartCanvas.cpp:1067` / `OnPaint`.
The timezone interpretation also follows
[wxWidgets 3.2.8 local datetime accessors](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/include/wx/datetime.h).

`test-renderer-log.ps1` passes 66 portable groups: append/rotation, cold hash,
multiple or wrong startups, explicit timezone/DST/midnight, stale/future times,
partial/unsafe/out-of-order markers, capability-vs-context separation, bounded
input, exact receipt/request/process identity and PS5-string/PS7-datetime UTC
handling. The existing 16 startup-readiness checks remain unchanged. These are
synthetic byte tests, not native or boat renderer acceptance; native PowerShell
qualification and actual boat observation remain separate.
