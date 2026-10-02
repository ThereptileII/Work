# SCRUM-246 — native chart-selector palette

The prototype has chart-format/library controls, but no equivalent of OpenCPN's
chart/quilt selector. This bounded palette extension preserves that useful
upstream control and its navigation meaning. It does not claim an exact
prototype counterpart or replace the selector with a decorative format button.

The selected vector key now uses exact prototype `--route`; the unselected
vector key uses `--float-muted`:

| Theme | Selected | Unselected |
| --- | --- | --- |
| Day | #267c76 | #6b8380 |
| Dusk | #b0dfc8 | #a6bcb7 |
| Night | #91bca2 | #869d91 |

`ChartVectorSelectorInk` succeeds only for the verified, active SKAGER
presentation. `Piano::SetColorScheme` resolves every stock brush first and then
replaces only these two local brushes, before its existing GL-atlas invalidation.
The same brushes feed software `Paint` and `BuildGLTexture`/`DrawGLSL`.
Global GREEN1/GREEN2, raster/CM93/MBTiles colors, busy/unavailable state,
background, eclipsed outlines and projection/hidden icons remain unchanged.
Standard, Legacy, Safe and missing/corrupt-resource fallback remain stock.

The patch does not change key geometry, height, chart grouping/index arrays,
active/eclipsed state, hit testing, click/rollover/context menus, chart/quilt
selection, visibility preferences, chart objects or user configuration.
The existing OpenGL busy-state behavior is likewise not rewritten by this
palette change. A broader selector redesign needs separate evidence.

`tools/test-chart-selector.py` compiles the actual production resolver and
pinned patched `SetColorScheme` with real wx colors/brushes. Its oracle reads
the immutable prototype tokens independently. All **135 checks pass** across
three themes and all mode/resource-verification combinations; all ten local
brushes and GL-atlas invalidation are checked. Removing just the optional hook
must reproduce the pinned method byte-for-byte. All nine integrated patches
apply to the pinned source. The initial local compile used the unrelocated
wx-config prefix and could not find headers; using the actual sysroot prefix
resolved that environment error without changing production code.

Reproduce using `--wx-config`, `--piano-source` (the patched pinned file) and
`--output`. This is a focused color/method check, not actual chart interaction,
GL-driver, native Windows/DPI or boat acceptance. The combined integrated build
and screenshots must show the corrected key while retaining chart content and
selection behavior before SCRUM-15 can be accepted.

Independent review found a late-fallback edge: resource verification may pass
during configuration, then deferred library creation may fail. Existing keys
could retain styled brushes until the next theme change. Both software Paint
and DrawGLSL now call one shared `SyncChartPresentation` before drawing or
checking the GL atlas. It resolves the two vector brushes from current verified
state, restores stock after fallback, and invalidates the atlas only when a
color actually changes. It does not rebuild textures on unchanged frames.

The focused fixture executes this actual method through active → failed →
active → Legacy state transitions in each theme. All **159 checks pass**,
including unchanged-frame caching and preservation of every other brush.
An initial source-inspection helper incorrectly counted inactive upstream
preprocessor braces; restricting that structural check to the two entry
preambles corrected the harness without changing any assertion or product code.
The final integrated/native rendering gates remain pending.
