# SCRUM-264 — exact supplied generic beacon, 2026-10-03

The immutable prototype contains actual generic `point:BCNGEN01` artwork.
It is now derived into `XNBCNG01`, rather than left as an unimplemented supplied
asset. This is a resource-only increment based on application `d5d7135`;
it is **not** part of the already-running `b8cfbf8` Windows candidate.

The pinned OpenCPN table has two original BCNGEN01 definitions. Both remain
byte-for-byte intact. Only the two existing Simplified generic lookups
`1696/31748 _bcngn` and `1708/31760 _slgto` redirect their first symbol token.
Text, priority, selection order and all other instructions remain unchanged.
The earlier coloured/shaped `_slgto` branches and all 19 Paper consumers remain
stock. No classified marine beacon is recast as a generic beacon; no feature
class, topmark association, chart query or renderer is changed.

The owned alias uses RCID60014, atlas `(724,1160,24,28)` and pivot `(12,14)`.
Paths, round stroke 1.3, `--mark-service` and scale 27/32 come directly from the
supplied JavaScript/CSS. Night brightness `.78` is applied once. Exact source
hashes and all derived SVG/RGBA hashes are in
`resources/chart-style/v1/generic-beacon/provenance.json`. Source definitions,
selectors and available tile/moat are checked before generation.

## Evidence

- [Focused proof](focused-proof.json): 2,118 checks; whole XML reversal permits
  only two symbol-token redirects and one owned alias. All three complete RGBA
  sheets reverse exactly after validating the new tile. Each theme adds only
  102 previously transparent pixels. Seven altered-XML cases and six pixel
  mutations are rejected. Original RLE stays identical.
- [Comparison](comparison.png): supplied-prototype raster, generated SKAGER
  atlas and prior generic sprite, all at the same geographic anchor in 48×48
  crops; contact sheet enlarged 2× without interpolation. Prototype and current
  composites are byte-identical in all themes. The roof, base and crossbar are
  visible; no tile-edge clipping is present. This was visually inspected.
- [Comparison identities](comparison.json) record the exact crops and hashes.
- Generator validation and the existing broad resource inverse oracle include
  the new alias/tiles; their assertions have not been weakened. The full shared
  resource suite is intentionally batched with the parallel special-buoy change.

Reproduce with `tools/derive-generic-beacon-art.py --evidence <temporary-dir>`
(Node, librsvg and Pillow needed only to rederive immutable assets), then
`tools/generate-xnav-chart-style.py` against the pinned resource source.
Run `tests/chart_generic_beacon_resources_tests.py --source <pinned-s57data>
--generated <new-resources> --baseline <d5d7135-resources> --report <json>`.
The normal build consumes the committed coverage and needs none of those art
authoring dependencies.

## Remaining acceptance

These are resource comparisons, not screenshots of the application. Native
Windows core/private loading, actual selected ENC feature coverage and physical
boat recognition remain open. The narrow generic mappings do not claim coverage
of every classified fixed beacon or physical lighthouse tower. Standard and
Legacy resources remain untouched.
