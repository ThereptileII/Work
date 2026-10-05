# SCRUM-250 — chart framebuffer size after theme layout

The exact e14de49 real-ENC Mesa GL captures show eight wrapped chart rows after
Day → Dusk/Night in both SKAGER and Standard chart styles. This is a cache/layout
fault, not new typography: the same pixels are present with Standard, and the
initial Day and software controls do not have the repeated strip.

A copied, unchanged e14de49 executable and its verified resources were rerun on
an isolated Xvfb display, with public NOAA ENC and disclosed test-only loopback
RMC. A read-only GL API tracer records actual framebuffer allocations and
viewports. After each theme change, it records **1012×558 RGBA allocation**, then
**1014×566 viewport**, without an intervening allocation. All six original
color/content/identity assertions still pass. In each Dusk/Night image, all
3,232 pixels at x690–1093/y68–75 equal x690–1093/y626–633 exactly. SKAGER also
repeats its left two columns at a period of 1012 pixels.

The cause is supported by both runtime sizes and the pinned source: upstream
`SetAndApplyColorScheme` temporarily installs a 1px AUI border and 6px sash;
`Shell::SetLight` subsequently restores zero metrics. `ChartCanvas::OnSize`
forwards a child size event before `ReloadVP` reconciles its actual size.
`glChartCanvas::buildFBOSize` allocates using the child size, whereas `Render`
uses the parent viewport. Texture coordinates exceed one when the cached FBO
is smaller, and these textures retain the default repeat mode.

The bounded repair executes only in the SKAGER shell (including its Standard
chart style). Before selecting the FBO cache, `PrepareChartFramebuffer`:

- synchronizes the GL child with the final parent client size;
- rebuilds an existing FBO only if either physical dimension is too small;
- permits cache use only if the resulting allocation covers the viewport.

If allocation fails or remains undersized, the existing direct chart renderer
is used. Disabled/unavailable FBOs stay disabled. Larger valid caches and normal
unchanged repaints avoid allocations. There is no clamp-only masking, cropping,
viewport/chart-scale change, new chart model or change to quilt/selection logic.
Legacy/Safe do not call the repair. Software is untouched.

The focused fixture extracts the **actual production method verbatim** and
exercises the measured failure with both resize-event orderings, scale1/2,
subsequent theme resize cycles, cached repaints, larger valid textures,
allocation failure, insufficient allocation and zero-size inputs. **231 checks
pass**. Removing the rebuild fails the negative control. The actual affected
`glChartCanvas.cpp` GL/GLSL object compiles with the recorded production flags;
all nine patches verify. Source hashes match fixture and compiled object.

The fixture models resize/allocation outcomes; it is not a GL driver render.
The runtime trace and screenshots prove the original defect. The fixed combined
real-ENC Mesa capture is intentionally left to root integration, followed by
native Windows/DPI and boat acceptance. No CI or boat run occurred here.

The separate light-arc difference is retained: all `RenderCARC`,
`RenderCARC_GLSL` and `RenderCARC_VBO` methods are byte-identical to pinned
OpenCPN. Their physical sizing and unset-SCAMIN limits differ (GL1000m/min0.5,
software200m/min0.4). The Standard controls show the difference too. This change
does not alter arc geometry, light sectors or infer a new font regression.

Evidence: `docs/evidence/scrum250-framebuffer/`. Reproduce the focused lifecycle
fixture using `tools/verify-chart-framebuffer.py --source <prepared-upstream>
--output <private-output> --wx-config <wx-config> --wx-prefix <prefix>`; the
`--negative-no-rebuild` option deliberately fails. The Linux-only tracer in
`tools/diagnostics/chart-gl-size-trace.cpp` can be compiled as a shared library
and used with `LD_PRELOAD` plus `SKAGER_GL_TRACE=<private-log>` in an isolated
profile. It logs only large allocation/viewport dimensions and never changes GL
arguments. The original private reproduction initially rejected a relocated
resource path, then stopped for an unavailable optional window-tree utility;
those attempts were discarded before the complete verified six-image run.
