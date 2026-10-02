# SCRUM-239 — healthy default COG predictor paint

This bounded increment applies the immutable ownship predictor's route ink,
1.2 logical-pixel width, 5/5 dash and .65 opacity to an existing valid COG line.
Its 79px illustrative length and 43° orientation are not production geometry.
No predictor, range ring, sensor value or navigation capability is invented.

## Presentation ownership and preservation

Base: `2271af3f259e791a08876dfeec77914cf38b8d53`. Inspected pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`: `MyConfig::LoadMyConfigRaw` and
configuration save, `ChartCanvas::ShipIndicatorsDraw`, software/GL ownship callers,
`PredColor`, `ocpnDC` and the solid-color shader API. Both rendering paths already
call the same `ShipIndicatorsDraw`; the patch adds one bounded paint hook there.

Verified SKAGER chart presentation owns **factory-equivalent COG line paint**:
configured width 3, style 105 and red `rgb(255,0,0)`, including defaults that
OpenCPN saved automatically. This is an explicit presentation policy, not a claim
that user intent can be inferred. A user deliberately choosing identical values
cannot be distinguished from automatic default persistence. Standard, Legacy and
Safe continue to show the stored upstream appearance. No preference is written.

Configuration is captured once before drawing. Each draw checks appearance
before upstream raises `g_cog_predictor_width` for physical display density. The
owner records that expected upstream adjustment, so subsequent frames do not
mistake it for a preference edit. Any observed nonfactory style/color or
unexpected width revokes ownership until the next startup. Saved nonfactory or
invalid appearance never qualifies. Existing upstream persistence may save a
density-raised width above 3; such a later startup conservatively retains stock
paint because it cannot be distinguished from a custom width. This limitation
is preserved rather than silently rewriting OpenCPN configuration.

The new line applies only to `SHIP_NORMAL`, default icon type and no custom icon.
All upstream validity, viewport, predictor-length and ownship-size guards remain.
COG/SOG, configured prediction minutes, geographic calculation, projected start
and endpoint, GPS offset and visibility remain upstream. The stock COG endpoint
marker and its preference, HDT divergence/paint/endpoint, low-accuracy/lost states,
custom appearance and configured range rings remain unchanged. The stock black
inner COG line is bypassed only when the new COG line was actually handled.

## Painter and focused evidence

`ChartCogPredictorMesh` creates disjoint butt-ended dash rectangles at fractional
coordinates. It clips to the expanded viewport before generating dashes, retaining
the original projected origin's dash phase. A visible span falling entirely in a
gap is handled successfully with no draw, rather than falling back to a solid
stock segment. Nonfinite, zero-length or unusable geometry falls back. CPU work
is bounded by visible screen span, not an arbitrarily long offscreen predictor.

Software fills the mesh as one native graphics path. Its 8-bit color alpha rounds
.65 to 166/255. GL uses the existing solid-color shader with .65 alpha and pinned
RGB/256 normalization; blend enable, factors, equations and prior shader program
are restored. There is no bitmap, texture upload, new cache or generic renderer
change. Pen/brush state and dirty extents are preserved.

`tools/test-cog-predictor.py` extracts the actual capture/gate/painter and reads
width, dash, gap and opacity independently from the immutable HTML. Linux real-wx
painting plus a recording GL interface passed **72 checks**: fresh/saved factory
configuration, invalid/custom values, repeated density adjustment, runtime
revocation, mode/presentation fallback, no config mutation, 100/125/150/200% widths,
dash phase and gaps, reversal/diagonal/clipping, zero/nonfinite input, long
screen-clipped geometry, renderer alpha and GL state restoration. The retained
PNG has Day/Dusk/Night rows at 100/125/150% on a neutral fixture background.
It is not a live chart capture or native GL evidence.

One optimized geometry-only run generated 10,000 predictors spanning 200 million
screen pixels, each clipped to a 1920px viewport, in **47.5501ms**. This is CPU
mesh cost only, not a chart-frame or boat responsiveness qualification.

The existing route foreground fixture also passed **78 checks**. All nine
production patches applied to a fresh private pinned checkout. Changed
`ChartPresentation.cpp` and patched `gui/src/chcanv.cpp` objects compiled with
Linux GL/GLSL flags and `-Werror`; no full build, CI run, active source/cache or
boat execution was performed.

## Open acceptance

Actual native GL antialiasing/compositing, integrated software behavior and all
validity/custom/endpoint/HDT scenarios require fresh execution. Native Windows
1280×800, 1920×1080, DPI, Night ancestor-filter fidelity and boat-display/pan
acceptance remain open. The prototype's ownship glow/circles, full degraded-state
hierarchy and route underlay/context are separate unresolved visual work. This
increment is not overall ownship/chart conformance or release acceptance.
