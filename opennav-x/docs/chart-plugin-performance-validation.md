# Chart, plugin and performance gate

The Alpha harness `tools/smoke-charts.py` runs on Linux and native Windows with
new disposable profiles. It downloads the NOAA US5SEAFL exchange set directly
from its origin, verifies the inspected SHA-256, and retains its base cell,
update, catalog and notices together. A changed download fails until explicitly
inspected and repinned. The fixture is not included in the portable or installer
payload. [NOAA agreement](https://charts.noaa.gov/ENCs/ENC_Agreement.shtml),
[Coast Survey licensing](https://nauticalcharts.noaa.gov/data/data-licensing.html).
It is a rendering test, not an approved or current chart service for navigation.

The adjacent US5SEAFK exchange set is independently pinned and keeps its own
catalog/notices. Input-only synthetic RMC jumps between inspected Seattle
positions while following; both software and requested OpenGL must switch the
actual upstream quilt reference to US5SEAFK and back to US5SEAFL, with nautical
detail in each capture. This jump is a rendering fixture, not a simulated
continuous passage.

At the Seattle fixture viewpoint the test requires US5SEAFL in OpenCPN's actual
quilt membership and substantial interior symbol/contour detail in native
pixels. It then exercises zoom, pan, own-ship follow using explicit input-only
loopback RMC, XNav overlays and XNav→Legacy→XNav. Software rendering and requested
OpenGL use the same fixture and model. The real renderer log is retained; an
upstream rejection of OpenGL is reported as software fallback, never passed off
as hardware OpenGL. Linux llvmpipe and cloud Windows graphics are not the target
navigation-PC GPU gate.

`RuntimeDiagnostics` copies GUI-thread viewport/follow/quilt membership and
initialized plugin records. It uses the pinned `GetVP`, `GetQuiltIndexArray`,
`ChartDatabase::GetChartTableEntry` and `PluginLoader::GetPlugInArray` read
boundaries. No chart is opened or recomposed and no plugin method is invoked to
obtain diagnostics. Only chart basenames are recorded. No model pointers escape.
This diagnostic observation is not a chart-corridor hazard-query service.

Dashboard, WMM and GRIB are enabled in the disposable profile, and their actual
loaded/initialized records are asserted. Native Windows also opens the upstream
plugin manager from XNav and verifies its Dashboard/WMM entries. The integration
selects upstream's Plugins page through the pinned `options_lastPage` mechanism;
the six built-in page order is inspected in `options::CreateControls`.
Third-party marketplace plugins and radar plugins remain individually untested.

Performance evidence records startup-to-ENC time, process CPU (one-core basis),
resident memory and XNav's 250 ms update callback last/mean/maximum duration.
The latter includes snapshot acquisition/advisories/diagnostic writing but
excludes asynchronous chart paint. Idle sampling runs with real loopback input
and three initialized plugins; it is explicitly a CI workload, not representative
boat-PC acceptance or a GPU frame-time benchmark. Sensor age/quality evaluation
continues on every update even when a displayed number has not changed.

Native PNG review remains mandatory in addition to model/pixel assertions. The
returned screenshots must show nautical features, not just a nonempty window.
Hardware GL, extended chart catalogs, real plugin combinations and underway
responsiveness remain in the physical test procedure.

The native geometry gate uses the pinned desktop press-drag-release gesture.
The initial harness inserted a selection click only 300 ms before a second
press, triggering upstream double-click properties instead of dragging. No
route persistence assertion was removed; a failed edit now retains a capture
and visible window inventory. Touch route editing remains a separate manual
validation item from the injected navigation-button touch gate.

Native `1460c05` passes the two-cell route/render gate and real route-point
drag persistence. Its OpenGL request is rejected by the hosted driver; actual
upstream software fallback is recorded, not relabelled hardware GL. Review
rejected one plugin-manager capture taken before painting despite populated
child controls. The replacement waits for both named plugins and nonblank
interior screen pixels with a strict deadline.

Review of `cdff41e` finds the plugin list painted but footer buttons still blank
in a partial first frame. The native gate now also requires text contrast inside
the actual OK/Cancel/Apply button interiors, then uses an actual Cancel hit and
requires modal dismissal. [Partial-paint evidence](evidence/windows-plugin-cdff41e-partial-paint.json).
