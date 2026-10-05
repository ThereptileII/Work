# SCRUM-254 — pilot boarding and radar beacon artwork

This bounded increment adds the exact immutable prototype `PILBOP02` and
`RTPBCN02` paths to the verified SKAGER chart resource set. It starts from
`797afebbb21c49e37b2f3ff613c887cb8916168f`, including SCRUM-253 Night treatment.
No chart selector, conditional procedure, global semantic color, navigation
value, radar functionality or renderer changes. Standard/Legacy/Safe continue
to use stock resources through the existing verified-resource gate.

## Effective definition and placement

| Symbol | Effective RCID | Stock rectangle; pivot | New rectangle; pivot |
|---|---|---|---|
| PILBOP02 | 1, overriding earlier 1423 | (736,778,17,17); (8,8) | (52,1160,24,24); (12,12) |
| RTPBCN02 | 2259 | (816,239,20,19); (10,9) | (84,1160,24,24); (12,12) |

Both effective definitions select raster rendering. Only each final bitmap's
six width/height, pivot and graphics-location numbers change. Existing origin,
distance, HPGL, color reference, all lookup tables and display classifications
remain identical. PILBOP point labels and area boundary instructions remain;
RTPBCN remains a portrayal of the charted radar-transponder-beacon object.
Exactly six effective direct lookups select these glyphs: PILBOP02 has four
(Plain 32166, Symbolized 32507, Simplified 31253, Paper 30476), RTPBCN02 two
(Simplified 31289, Paper 30512). Each complete lookup node is unchanged;
`selected-rules.json` retains its instruction.

The exact prototype scale is `27/32`, with 1.3 source-unit round stroke and
joins. New numeric pivot `(12,12)` is the same glyph-local `(0,0)` geographic
anchor: neither art is translated or shrunk relative to its chart point.
Existing software `r-pivot`, GL viewport counter-rotation, user scale, SCAMIN,
DPI and content-scale behavior remain upstream. Existing fractional-scale
integer rounding remains, not a new placement policy.

All original declared bitmap rectangles, including overridden definitions and
patterns, the relocated SCRUM-244 anchorage tile, and the other new service tile
are checked for collision with a two-pixel moat. Both new regions and moats are
transparent in all source sheets. Atlas dimensions remain **1500×1200**. Old
PILBOP02/RTPBCN02 pixels, anchorage artwork, adjacent resources and even invisible
RGB outside the 304 new nontransparent pixels remain unchanged.

## Source and theme identity

`resources/chart-style/v1/services/` contains exact-path SVGs, precomputed alpha
coverage and provenance. The generator requires hashes of the four immutable
prototype source files, both SVGs and both coverage masks; CRLF normalization is
the only permitted checkout transformation. Runtime resource generation remains
Python-standard-library-only on Windows. CMake regeneration tracks every new
input; Windows preflight includes the helper and tracked resource assets.

PILBOP02 has 148 nontransparent pixels; RTPBCN02 has 156. Day service ink is
`#7c858a`, Dusk `#a8bbb7`, and Night `#62736c`: the last is prototype service
`#7e948a` with the chart-only `brightness(.78)` applied once. These are local
glyph pixels, never a global CHMGD change.

## Focused proof and limits

Evidence is under `docs/evidence/scrum254-service-glyphs/`; `review.json` records
input identities and results. The **10,709-check resource suite passes**, including complete semantic-tree
reverse equality, deterministic and Windows-CRLF generation, theme colors,
independent exact prototype paths/scale, full-sheet pixel isolation and anchor
identities across scales and rotations. Existing anchorage and neutral-ink
checks remain: the separately proved service rectangles are removed only for
those older, independent comparisons.

Nine dedicated negative controls reject declared-resource/moat overlap,
collision with ACHARE51 or the other service tile, occupied pixels and changed
SVG/mask provenance. Additional resource negatives reject wrong pivots, bounds,
locations, origin, earlier duplicate modifications, changed pilot lookup and
forcing RTPBCN02 to vector rendering.

The native fixture compiles unchanged pinned `ChartSymbols` loader methods and
executes final-definition selection, PNG loading, software crop and GL texture
rectangle lookup. **6,234 checks pass**; wrong final PILBOP02 and RTPBCN02 pivots
both fail the real loader fixture. There is no GL context or chart draw here.
Six freshly rasterized prototype references match loaded alpha exactly. Cairo
premultiplied color rounding produces at most 1.01 composited-channel difference;
the loaded pixels retain exact service RGB. An initial overly strict straight-RGB
comparison exposed that low-alpha rasterizer rounding and was replaced by the
explicit composited comparison, without changing the assets or product code.

The viewed `stock-prototype-loaded.png` shows both glyphs across all three themes;
individual exact-size stock/reference/current crops are retained. They are
resource fixtures, not ENC screenshots. Native full software/GL chart drawing,
real ENC identification/placement, native Windows/DPI and boat readability remain
open. No full CI, application scenario or physical boat work ran for this task.
