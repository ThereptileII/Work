# SCRUM-232 — bounded ownship artwork

Local implementation based on `cf0719749482e62d48daed02515f2b6c6d242b67`.
This is a focused development result, not native Windows, whole-chart or boat
acceptance. Prior frozen candidate and evidence are unchanged.

## Shape and boundary

Immutable `docs/design/prototype/index.html:202` specifies the path
`M0-19 11 16 0 10-11 16Z`, route fill and floating outline of width 3.
The new `DrawChartOwnship` paints that shared native polygon only when the
verified XNav chart style is active. Both pinned `ShipDraw` callers additionally
require the default fixed icon, no user image and `SHIP_NORMAL`. No new sensor,
heading, position, predictor, ring, course vector, label or alarm is created.

The existing projection, heading/COG fallback, viewport rotation and visibility
checks remain upstream. Both paths pass their final projected midpoint and
`icon_rad - PI/2`. The shared shape uses the existing user factor
`g_ShipScaleFactorExp > 1 ? log(g_ShipScaleFactorExp) + 1 : 1`, then exactly one
`FromDIP(100)/100` conversion. It deliberately does not reuse the GL bitmap's
additional 1.1/content-scale factors. The existing bitmap size calculations and
`img_height` remain untouched for predictor visibility. Scaled bitmap/vector
vessels, antenna offsets, custom images, low-accuracy/invalid states and the
upstream small-scale symbol above chart scale 300000 retain stock rendering.
Legacy, Safe, Standard and failed resource verification retain stock artwork.

The ownship dot at the same fixed marker origin is replaced with the chevron;
scaled-vessel antenna markers are preserved. Both upstream branches still
perform their original bounding-box/GL cleanup and `ShipIndicatorsDraw` work.
The helper adds conservative outline bounds and restores the pen/brush. It
rejects invalid or unrepresentable geometry before integer conversion. It reads
the palette at every paint, bypassing stock texture tint/cache for this shape.

Day fill/outline are `#267C76` / `#F7F8F0`; Dusk `#B0DFC8` / `#243A40`.
Night applies the prototype chart ancestor's `brightness(.78)` to this artwork
only, giving `#71937E` / `#101A20` after channel rounding. Existing route palette
and real chart content are unchanged.

The pinned GL four-point renderer uses a triangle strip in order 0,1,3,2.
Starting the closed polygon at right stern (right/notch/left/bow) puts the
shared triangle diagonal inside the shape. Starting at the SVG bow would fill
the stern notch incorrectly. No GL tessellation or texture cache is added.

## Focused verification

- All nine existing patches apply from pinned OpenCPN
  `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`; a temporary index comparison
  confirms the prepared source exactly matches those patches.
- Actual changed `ChartPresentation.cpp`, `chcanv.cpp` and `glChartCanvas.cpp`
  compiled independently with the existing integrated Linux include/define
  inputs. Only optimization/output locations changed; no full app rebuild.
- `tools/test-ownship-chevron.py` executes the extracted production painter
  with a recording adapter backed by real wx raster drawing: **488 checks**.
  The extracted painter also compiles with a Windows-style `max` macro defined.
  Geometry is read independently from the immutable HTML. Cases cover
  Day/Dusk/Night, 0/41/90/180 degrees, simulated 100/125/150% DIP conversion,
  user scale, inactive/fallback gates, invalid geometry, restored DC state,
  repaint bounds and the actual pinned GL strip topology's open stern notch.
- Actual local raster sheet inspected:
  [retained painter sheet](../../evidence/scrum-232-ownship-painter.png) (640×390).
  SHA-256: `d09cb751f25f6926eeae55d5956d1edff9b25710c30450dce4b8a78dfa45cc45`.
  Four columns are 0°, 41°, 90° at 100%, and 180° at simulated 150%.
  The notch stays open, rotated shape remains coherent and all three palettes
  change immediately. This sheet is a synthetic painter fixture, not navigation
  data or a whole-chart screenshot.

Reproduction, with a running test display and matching wx runtime:

```sh
python tools/test-ownship-chevron.py --wx-config /path/to/wx-config \
  --output /path/to/local-ownship-evidence
```

## Remaining qualification

The HTML chart is a 1000×630 SVG with `xMidYMid slice` (`index.html:186`). Its
22×35 path and 3-unit outline therefore inherit viewport scaling: the 1014×566
primary chart area yields a 1.014 scale before device scaling. This fixed native
marker uses logical pixels plus explicit user/DPI scaling; it does not grow
with window width. That viewport-specific difference is recorded, not accepted
as exact prototype size at all resolutions. Integer vertex/pen rounding also
needs actual 125/150% Windows review.

The test's DPI adapter is not a native monitor; its GL topology check is not a
GL framebuffer. Actual software and GL captures, native MSVC/Windows, palette
transitions, all preserved stock states and boat-display review remain pending.
Pinned ocpnDC GL shaders have their own antialiasing/line joins and RGB/256
conversion. Those may produce visible edge/color differences; this patch does
not alter shared GL rendering. No full suite, CI, hardware or boat run occurred.
