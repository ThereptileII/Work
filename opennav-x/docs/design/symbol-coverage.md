# Chart symbol coverage

SCRUM-378 source audit, 2026-10-20. The pinned OpenCPN catalogue is commit
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. Its complete machine-readable
inventory is
[`symbol-coverage.json`](../../resources/chart-style/v1/symbol-coverage.json).
Every effective point, line and pattern has a status and an explicit reason.
The inventory includes direct lookup references, qualified aliases, duplicate
definitions and the separate prototype catalogue identity.

| Kind | Definitions | Effective names |
| --- | ---: | ---: |
| Point | 1,093 | 1,018 |
| Line style | 57 | 57 |
| Pattern | 30 | 30 |

The upstream last definition for a name/type wins. The 75 earlier point
definitions are recorded separately; they are not 75 additional usable glyphs.
The prototype catalogue was imported from a different upstream revision, so
the audit compares names against the pinned product XML rather than assuming
the two catalogues are identical. A generic labelled reference pin in the
prototype's library is not custom navigation artwork.

The 1,105 effective entries have these statuses:

| Status | Count | Meaning |
| --- | ---: | --- |
| Prototype point artwork | 13 | Effective native bitmap replaced by reviewed prototype art |
| Qualified prototype alias | 16 | Some selected uses receive an alias; other uses remain native |
| Native geometry redraw | 15 | Verified native silhouette rendered from scalable geometry |
| Prototype line artwork | 1 | Submarine cable wave |
| Prototype pattern artwork | 1 | Fishing facility diamond |
| Native retained | 1,059 | Explicit reason recorded for every entry |

This is intentionally not a claim that all chart icons have been replaced.
The aliases are counted against their native source names in this table.
The generated resource's separate alias inventory records 18 additional names,
including retained tower-resource aliases whose current qualified runtime path
instead uses the shared light glyph.

## Scalable point resources

The existing generated `XNavChartResources.h` now contains `SymbolSvg` records:
exact eight-character symbol name, DAY_BRIGHT/DUSK/NIGHT scheme, SVG, logical
width and height. There are **43 names and 129 scheme entries**: 28 existing
prototype point glyphs and 15 reviewed native redraws. The generator verifies
the original SVG provenance and the effective source node before embedding.
It does not infer SVG artwork for arbitrary native symbols.

The 28 existing glyphs include the anchor, pilot/radar/marina, both rock types,
dangerous wreck, four cardinal buoys, classified buoy aliases, three light
colors, qualified special buoys and generic beacon. All retain their existing
bitmap dimensions and pivots. Light and qualified light-support tower
portrayals may share the light artwork; their object classes, light color and
lookup conditions remain separate.

The new native redraws are:

- Fixed beacons: `BCNCAR01`–`BCNCAR04`, `BCNISD21`, `BCNLAT15`,
  `BCNLAT16`, `BCNSAW13`, `BCNSPP13`.
- Hazards: `OBSTRN01`, `OBSTRN02`, `OBSTRN03`, `OBSTRN18`,
  `WRECKS01`, `WRECKS04`.

The new source geometry is recorded in
[`native-points.json`](../../resources/chart-style/v1/native-points.json),
derived by the unchanged prototype HPGL importer from the pinned XML. Native
major beacon shapes, cardinal orientation, isolated-danger circles,
obstruction boundary dots, drying/depth/land fill roles and wreck categories
remain distinct. Hazard categorical fill/outline roles `DEPVS`, `DEPIT`,
`LANDA` and `CSTLN` keep their pinned scheme colors, as the stock hazard atlas
did; the general Very Light area palette must not wash out those distinctions.
Ordinary neutral ink uses the derived palette. The transform maps the native HPGL pivot exactly to
the existing bitmap pivot and uniformly fits the padded geometry inside the
existing logical bitmap rectangle. Both anchor coordinates and the unclipped
extent are checked. Readable pen
widths and boundary dots replace low-resolution raster sampling; there is no
additional asset size gain.

`BCNDEF13` remains the unknown/default beacon. Inland `BCNLAT23`/`BCNLAT24`
retain their separate native composition. Minor `BCNLAT21`/`BCNLAT22`,
`BCNSAW21` and `BCNSPP21` remain stock: their bitmap pivots are off-center
relative to their native vector geometry, and fitting a redraw around those
exact anchors would severely shrink the shape. Bitmap-only `OBSTRN11` has no
verified vector equivalent. Physical topmarks, sounding digits, isolated
danger indications, survey-quality marks, specialized lights, inland signs
and other unmapped variants keep their native selection and geometry.
Ordinary stock geometry can still receive already reviewed palette treatment;
that is not counted as new artwork.

## Color and native SVG compatibility

The owner's requested derived color delta increases saturation of the existing
owned red, green and yellow buoy/light inks and the new native beacon
`CHRED`/`CHGRN`/`CHYLW` roles by 1.20 in HSL space, holding hue
and lightness constant. Neutral, white, hazard, drying and land fills are
excluded. For prototype art, existing Night `.78` dimming happens first;
native beacons use the already selected Night navigation-aid palette. Night additionally
caps the weighted RGB brightness (`2126 R + 7152 G + 722 B`) at its previous
value through uniform downward scaling. This shares the light-sector color
formula; it does not change light-color classification.

The original prototype and original owned SVG files remain unchanged.
The richer colors are derived only when generating scalable resources. The
source atlas remains the existing fallback when native SVG loading is
unavailable. Standard/Legacy uses its original resources.

The SVG loader does not implement general CSS. The generator therefore
resolves only the controlled simple-class stylesheet subset into explicit
presentation attributes on each shape, preserves transforms and group opacity,
and rejects unsupported selectors, nodes and paint syntax. Embedded colors
are `#RRGGBB` or `none`; no external references are allowed. Rasterizing this
normalized SVG at the final requested dimensions avoids enlarging the existing
small atlas crop. Native DC/GL hookup and the single modest aid-size gain are
owned by the renderer integration; resources do not multiply that gain.

## Focused verification

Run the normal resource generator against pinned `data/s57data`, then:

```sh
python tools/audit-chart-symbol-coverage.py \
  --source /path/to/pinned/data/s57data/chartsymbols.xml \
  --styled /path/to/generated/chartsymbols.xml
python tools/test-chart-symbol-svg.py \
  --source /path/to/pinned/data/s57data/chartsymbols.xml \
  --styled /path/to/generated/chartsymbols.xml \
  --output build/symbol-svg-check --render
```

The focused check regenerates all 15 source geometries with the real importer,
requires exact complete-inventory equality, checks dimensions and categorical
roles, rejects unsupported CSS, pins shared color fixtures and syntax-checks
the generated C++ table. Optional librsvg checks compare original and flattened
SVG raster bytes before the deliberate color delta at 100%, 125% and 150%.
Contact sheets show atlas and final-size SVG rasters at those sizes.

The source check passed for 129 scheme entries, all 1,105 inventory entries,
all 15 exact native anchors and their actual painted extents, and the generated
C++ table. The 70 existing source SVGs produced 210 byte-identical raw/flattened
render pairs across the three sizes. All 387 point/scheme/size combinations
were rendered for inspection. Independent review caught and then verified the
fix for a fractionally clipped `OBSTRN18` outline; its final three-scheme,
three-size renders were repeated after reserving the actual pen extent.
The full resource generator preserved all five atlas/XML/RLE output hashes;
the new data is carried in the generated header and manifest.

These are source/resource checks. They do not qualify native wx/NanoSVG,
OpenGL, Windows, real chart data or the boat display, and do not prove every
remaining stock icon sharp at every scale.
