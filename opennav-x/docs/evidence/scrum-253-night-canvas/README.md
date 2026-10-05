# Effective Night chart canvas — SCRUM-253

Private implementation based on frozen `1356fd1603aacbea04d7081d16331e9a181180bb`.
The immutable final CSS applies `brightness(.78)` to `.chart-canvas`, after
compositing. Canonical Windows Night pixels independently confirm land `(29,41,37)`
and water `(14,23,28)`. Previously both native renderers used the raw tokens.

Normalization now occurs once at explicit owned paint inputs; there is no screen
filter, chart-wide tint, global palette rewrite, or raster-chart recoloring.

| Night role | Previous/raw RGB | Effective RGB |
|---|---|---|
| LANDA | 37,52,47 | 29,41,37 |
| DEPDW | 18,30,36 | 14,23,28 |
| CSTLN / XNBUA | 70,87,74 | 55,68,58 |
| DEPMD | 28,45,53 | 22,35,41 |
| DEPMS / DEPCN | 42,65,73 | 33,51,57 |
| DEPVS | 61,85,96 | 48,66,75 |
| DEPIT | 51,68,59 | 40,53,46 |
| XNGEO geographic names | 117,133,121 | 91,104,94 |
| Eligible active route / COG / waypoint outline and ordinal | 145,188,162 | 113,147,126 |
| Eligible route underlay, alpha .6 unchanged | 21,33,41 | 16,26,32 |
| Verified SKAGER Online AIS body outline | 145,100,119 | 113,78,93 |
| Verified SKAGER Online AIS body fill | 21,33,41 | 16,26,32 |
| Verified SKAGER Online AIS selected fill/ring | 203,156,177 | 158,122,138 |

The generator derives XML and the GSHHS background header from the same effective
surface values, and records raw/effective roles in the manifest. All five depth
fills remain distinct and the intertidal role keeps its green hue. The unchanged
semantic-tree guard still rejects changes outside authorized paint roles.

Navigation correctness requires explicit brighter exceptions: CHBLK/CHGRD and
their neutral atlas pixels, DEPSC safety contour, SNDG1 and SNDG2 are retained.
Their previous values are `(117,133,121)` and SNDG2 `(182,195,175)`. Dimming neutral
ink would reduce water/shallow/land contrast to 3.097/1.787/2.569, violating existing
4/2/3 gates. Retaining it against corrected surfaces gives 4.652/2.685/3.859.
Factory LIGHTS descriptions use retained CHBLK only in Night, while their halo
uses effective water. Day/Dusk still use XNGEO, preserving their previous pixels.
Geographic names use normalized XNGEO separately; font/scale/opacity and custom
LIGHTS fallback remain unchanged. This documented exception favors readable
charted hazards and depths over literal dimming of every prototype primitive.

Already-effective ownship, waypoint fill, onboard AIS body, ACHARE51 anchorage,
CBLSUB06 cable and Online AIS labels are untouched. Online normalization has an
explicit Night + verified-SKAGER guard; Standard supplemental body colors retain
their original values. Online stale/lost outline ink stays brighter. Existing
route custom/MOB/selection/edit eligibility guards and all geometry/alpha remain.
Floating UI, chart selector, scale and depth-unit disclaimer are outside the
prototype canvas filter and stay unchanged. No compatibility keys are renamed.

## Focused verification

- All nine patches apply to a private pinned upstream checkout; the preparation
  tool independently reconstructed and compared the exact patched tree.
- 8,065 resource checks cover literal Night RGBs, unchanged Day/Dusk table hashes,
  unchanged raster bytes/alpha, five distinct depth roles, unchanged safety
  gates, semantic equality and corrupt-input rejection. Separate negative
  controls reject raw land, twice-dimmed water and dimmed hazard ink.
- 772 C++ helper checks cover every channel value with mixed RGB, unchanged Day/Dusk
  and unchanged raw UI palette roles. The actual production route/Online input
  blocks and LIGHTS expression are compiled into a focused guard fixture, testing
  every theme/style/verification combination and already-effective label ink
  (87 checks, including Night-only LIGHTS ink selection).
- Four real production translation units compile individually in SW and GL:
  ChartPresentation, ChartRouteUnderlay, OnlineAisOverlay and patched s52plib.
  Their production include/definition sets and existing generated platform
  headers were reused read-only, with `-O0` for this isolated compile check.
  Outputs and hashes are retained in `compile.json`. This does not link an app.
- Exact XNav/Standard surface checker controls pass; Night expectations now use
  effective CSS values, not raw tokens. `identity.json` also records independent
  byte equality for Day/Dusk palettes, all three raster sheets, RLE, and XML
  outside the ten enumerated Night paint roles.

Run focused checks with `tests/chart_presentation_resources_tests.py`,
`tests/chart_layout_tests.py`, the `chart_canvas_ink_tests` CMake target, and
`tools/test-chart-night-inputs.py --wx-config ... --wx-prefix ... --output ...`.

No full suite, CI dispatch, boat connection, frozen-root edit or cache build was
performed. Real ENC/software/GL visual review, native Windows and boat acceptance
remain open. Existing owned GL shaders use `/256.f`; their raster result can be
one byte below software for some channels. This patch deliberately preserves
those renderer conventions and all Day/Dusk behavior; it does not claim new
exact cross-renderer pixel equality or screen completion.
