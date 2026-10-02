# SCRUM-239 — preserve and theme the COG endpoint fill

Base `d02dd03`; pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The unchanged prototype's ownship predictor (HTML line 202) has a dashed line
without an endpoint icon. OpenCPN's existing marker derives from VECGND02 and
identifies the real projected position at the configured predictor time. It is
separately enabled by `OwnshipCOGPredictorEndmarker` (default 1, saved/restored by
`MyConfig`). Removing it would remove configured navigation content.

The shared software/GL `ChartCanvas::ShipIndicatorsDraw` changes only that
marker's brush fill to the prototype route ink after `xnav_cog_painted` is true
and verified chart ink is available. The existing prediction, projected quad,
rotation, GPS offset, ship-scale factors, black border and width, draw order,
visibility and endmarker guard remain upstream. No alpha or glyph redesign is
inferred from the prototype, which has no equivalent endpoint. HDT is unchanged.

The existing SCRUM-239 ownership/healthy/default-icon guard is reused directly;
there is no second call to density/preference ownership tracking. Custom COG
appearance, custom/scaled icons, degraded ownship states, failed line painting,
Standard/Legacy/Safe and unverified resources retain the stock endpoint fill.
No preference or navigation-state write is added. This is an explicit semantic
extension using prototype ink, not an assertion that the marker exists in HTML.

## Focused evidence

`tools/test-cog-endpoint.py` extracts the actual production endpoint block,
actual COG style/health/icon guard, and actual `ocpnDC::StrokePolygon` body from
a patched pinned source tree. It also extracts the unmodified pinned endpoint
block to compare geometry, border and enable behavior directly. The recording
adapter verifies polygon submissions; real wxGraphicsContext executes the
software polygon body. GL driver pixels are not qualified by that adapter.

**5,477 checks pass**, covering three theme inks independently read from HTML,
enabled/disabled/nonzero enable settings, unchanged border and all four projected
points, GPS offset, bearings, width/ship-scale factors, eligibility/custom icon/
health/failure branches and verified-ink fallback. The existing actual COG
capture/ownership/painter fixture still passes **72 checks**, including stored
custom appearance, restart defaults and density handling.

The retained [fixture](../../evidence/scrum239-cog-endpoint/cog-endpoint-day-dusk-night.png)
shows the same stock square and border with stock fill, selected prototype fill,
and disabled marker, in Day/Dusk/Night rows. It is an isolated software drawing
fixture, not an integrated chart screenshot.

```sh
python tools/test-cog-endpoint.py --wx-config /path/to/wx-config \
  --opencpn-source /path/to/patched-OpenCPN --output /tmp/cog-endpoint-test
```

The actual changed Linux `chcanv.cpp` object compiled with the existing main
application's GL/GLSL flags and `-Werror`. Compiler invocation was read using
`ninja -t commands` without modifying its build cache, then redirected to the
private `/tmp/scrum239-endpoint-source` source and
`/tmp/scrum239-endpoint-test/canvas.o`; expanded command remains in
`/tmp/scrum239-endpoint-test/canvas.command`. All nine patches apply to the pinned
revision in an isolated index. Original prototype files are untouched.

Actual integrated GL, native Windows/MSVC/DPI, complete chart scenes, route/
predictor lifecycle and boat validation remain open. No full CI or boat work
was run. The separate local chart-selector palette is tracked in SCRUM-246 and
is not implemented by this endpoint change.
