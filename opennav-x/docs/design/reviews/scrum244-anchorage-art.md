# SCRUM-244 — anchorage center artwork

The derived SKAGER chart resources now portray only `ACHARE51` with the exact
prototype anchor paths at the chart-marker scale `27/32`, a `1.3` source-unit
round stroke, and the prototype `marker-service` color. This is an anchorage
center presentation change. It does not change an anchoring-point, restriction,
boundary, depth, hazard, category, lookup or conditional-symbol rule.

## Effective resource and fit decision

Pinned OpenCPN is `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. Its final
`ACHARE51` definition, RCID 1105, overrides the earlier vector/raster definition:
`ChartSymbols::ProcessSymbols` and `BuildSymbol` choose the final bitmap and
replace both the symbol rule and atlas rectangle. Editing the earlier HPGL
would not change the shipped glyph.

The original final tile is `(373,415,25,29)`, pivot `(18,1)`. The exact prototype
bounds including the round stroke are ±8.9859375 horizontally and ±8.1421875
vertically about its geographic point. They cannot fit that pivot and tile.
After reporting this fit failure, a bounded adjustment was explicitly authorized
in Jira comment 10568: move this symbol to unused transparent atlas space and
adjust its numeric pivot while preserving its geographic point and exact scale.

The new tile is `(20,1160,20,20)`, pivot `(10,10)`. All 1,091 original bitmap
rectangles, including overwritten definitions and patterns, are checked for
intersection with a two-pixel moat. Their maximum bottom is 1149. The tile and
moat are transparent in all three pinned sheets. Atlas dimensions stay
1500×1200. The old tile and the adjacent `ACHPNT02` tile `(408,415,25,29)` remain
unchanged. Only the six width/height, pivot and graphics-location numeric
attributes in RCID 1105 change; the complete XML rule tree is otherwise checked
against the existing approved palette/name-ink changes.

## Artwork, color and alignment

`resources/chart-style/v1/anchorage/ACHARE51.svg` copies the exact circle and path
strings from immutable `chart-marker-art.js`. Its `translate(10 10)` represents
the new bitmap pivot, not a movement relative to the geographic point. The
coverage mask was generated with librsvg 2.62.3 / Cairo 1.18.4 and is checked in
so production resource generation remains Python-standard-library-only on
Windows. Provenance binds the SVG, mask and four relevant unchanged prototype
source files; only known Git CRLF conversion is normalized.

The 126 nontransparent mask pixels are painted with service ink: Day
`#7c858a`, Dusk `#a8bbb7`, Night `#62736c`. Night includes the prototype chart's
`brightness(.78)` applied to `#7e948a`. No global S-52 semantic color changes.
Every RGBA byte outside these previously transparent pixels stays identical to
the pre-existing derived sheet, including invisible RGB and all original alpha.

Pinned `LoadRasterFileForColorTable` reads actual PNG dimensions, and both
`GetImage` and `GetGLTextureRect` use the same effective atlas rectangle.
`RenderRasterSymbol` scales the image and pivot using its existing user,
SCAMIN, DPI and content factors. Software places at `r - pivot`; GL applies
its existing counter-rotation about `r` before subtracting the scaled pivot.
Thus the glyph-local origin remains the same chart anchor. Existing integer
pivot/size quantization at fractional scale is retained (less than one pixel),
not silently replaced by a new sizing or rotation policy. Area clipping and
lookup selection remain upstream behavior. Standard/Legacy/Safe keep stock
resources through the existing presentation-resource selection gate.

## Focused evidence

- `docs/evidence/scrum244-anchorage/review.json` binds inputs, exact base and gates.
- 8,017 resource checks pass: reproducibility, Windows CRLF checkout, exact source
  hashes, full semantic-tree equality outside authorized changes, all-pixel
  isolation, original alpha preservation outside the glyph, effective final
  metadata, transparent moat, geographic origin/rotation/scale identities, and
  rejection of changes to the earlier duplicate, final pivot, width and origin.
- 2,356 native wx fixture checks pass. The fixture compiles the unchanged pinned
  `ProcessSymbols`, `BuildSymbol`, PNG loader, `GetImage` and `GetGLTextureRect`
  methods with `-Werror` and GL compilation enabled. Only S-52 owner containers
  are fixtures. All actual final-definition selection and image cropping execute;
  fresh SVG coverage is compared against the wx-loaded tiles for all themes.
  Upstream deprecated-copy and unused-parameter warnings are explicitly excluded.
- A negative fixture restoring the final pivot to `(18,1)` fails check 3, proving
  the actual loader check rejects the ineffective/incorrect-hotspot result.
- `anchor-day-dusk-night.png` shows isolated stock/SVG/wx glyph comparisons. It is
  a resource fixture, not a chart screenshot or visual acceptance result.

The generator and configure dependencies are the only production code changes;
no renderer hook or upstream patch is needed. Full chart software/GL drawing,
rotated-view capture, native Windows/DPI and boat-display comparison remain open.
No full CI or boat tests ran. SCRUM-15 final visual acceptance remains open.
