# Chart, plugin and performance gate

The Alpha harness `tools/smoke-charts.py` runs on Linux and native Windows with
new disposable profiles. It downloads the NOAA US5SEAFL exchange set directly
from its origin, verifies the inspected SHA-256, and retains its base cell,
update, catalog and notices together. A changed download fails until explicitly
inspected and repinned. The fixture is not included in the portable or installer
payload. [NOAA agreement](https://charts.noaa.gov/ENCs/ENC_Agreement.shtml),
[Coast Survey licensing](https://nauticalcharts.noaa.gov/data/data-licensing.html).
It is a rendering test, not an approved or current chart service for navigation.

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
