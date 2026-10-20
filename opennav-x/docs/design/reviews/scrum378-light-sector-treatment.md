# SCRUM-378 light-sector treatment

The expanded chart light sectors now use the prototype's translucent wash,
0.65 px boundaries with 3 px / 4 px dashes, and 1 px outer arc. The compact
sector and all-round arc geometry and existing stroke/fill opacity are unchanged.
The immutable prototype is `docs/design/prototype/src/chart-symbols.css`.

The native sector renderer still computes the centre, projected range endpoints,
display radius, angles, rotation and duplicate boundary ownership. The existing
leading-light cone branch keeps its triangle and opacity. No illustrative
prototype bearings, ranges, labels or geometry enter chart rendering.
`s57_ProcessExtendedLightSectors` is unchanged, including obscured-sector
rejection, range selection, native/PI object processing and leading-light
classification. Private chart sector records use this same core renderer.

Expanded red/green wash opacity follows the prototype's 0.07 (Day/Dusk) and 0.10
(Night); white wash uses 0.24. Boundary and arc requested opacities are 0.65 and
0.70. Each layer and the final combined pixel are capped by the native sector's
current-theme opacity. This retains the existing paint-time theme resolution
for retained hover selections. Red/green/white classification is preserved;
unrecognized stock RGB classes and schemes retain the original renderer.

Red and green compact/expanded ink uses the same HLS saturation ×1.20 treatment
as the richer owned aid SVGs. It runs after theme dimming, preserves hue and
lightness before rounding, and scales Night RGB down if the weighted luminance
would increase. White fill and stroke are unchanged. The six actual theme RGB
results are checked against independent SVG-generator fixtures, including a
half-integer rounding case.

The integration change is limited to the existing `gui/src/s57chart.cpp`
section of `opencpn-5.12.4-chart-presentation.patch`. Shared style functions live
in the already packaged `ChartCaFan.h`, `ChartLightHover.h` and
`ChartLightSymbol.h`. No new private-source inventory or private patch is needed.
Private compact painters already consume the shared helpers.

Software and desktop GL paint the same antialiased RGBA tile. The existing
state-restoring `DrawCaFanGL` upload is unchanged. Native `ocpnDC` GLSL line
drawing forces alpha to 1 and cannot supply the requested translucent strokes.
Tile construction therefore completes before either renderer paints anything.
Unsupported contexts, malformed geometry or refused allocations use the full
original drawing branch. The raster region is clipped to the viewport without
changing projected endpoints or dash phase. Work is capped at 4,194,304 pixels
and an 8192 px viewport dimension; distant rays use a clipped parameter interval
so their off-screen distance cannot cause an unbounded dash loop. The compact
painter's existing smaller allocation cap is unchanged. No tile is cached across
frames or stored in a chart rule.

Focused Linux evidence (2026-10-20):

- `test-extended-light-sectors.py`: 158 checks passed on wxGTK and Mesa llvmpipe,
  executing both actual patched sector methods. Day/Dusk/Night, all three lens
  classes, rotated/wraparound sectors and software/GL pixels agree within one
  RGB level. Native bearing/range inputs, source-record immutability, leading
  cones, shared-ray ownership, unknown/size fallback, wash/dash coverage, Night
  opacity and bounded distant-ray work are checked. The native processor body
  matches the pinned source byte-for-byte.
- `chart_light_hover_tests.cpp`: passed, including the independent richness
  fixtures and Night luminance ceiling.
- Existing `test-ca-fan.py` core branch: 576 checks passed with the current
  actual compact methods, stock method comparison, original CA instructions,
  native endpoints and GL-state checks. Its fixture gained the missing
  `ChartCaAllRound.h` include required by the current methods; no assertions were
  removed. The retained private subset lacks its shader translation unit, so
  this run does not claim private DLL execution.
- The complete patched `s57chart.cpp` passed syntax-only compilation with retained
  integrated Linux flags. No application build or delivery was performed.

Evidence is retained under `build/light-sector-review/` in the isolated worktree:
`final2/receipt.json`, `compact-core-final/receipt.json`, and
`s57-syntax-command.json` / `s57-syntax.log`. The expanded fixture uses a controlled
projection adapter; it does not qualify real chart projection, charts, native
Windows/DPI rendering, private DLL loading, or boat-display behavior. Those
product gates remain pending.

Reproduce expanded-method checks with:

```sh
python3 tools/test-extended-light-sectors.py \
  --source /path/to/pinned/unpatched/OpenCPN \
  --wx-prefix /path/to/wx/prefix \
  --output build/light-sector-review/expanded
```
