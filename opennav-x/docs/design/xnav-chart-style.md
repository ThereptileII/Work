# XNav chart presentation — source inspection and palette contract

The native chart remains OpenCPN 5.12.4, pinned to
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. The prototype is fictional geography,
not chart data. Its symbol vendor snapshot is `1bf728e17feaae05fddad3aad5be677e15c1e89c`;
that newer snapshot is preserved as design evidence, **not substituted** for the
application's pinned S-52 resources.

| Visual role | Day | Dusk | Night |
|---|---|---|---|
| Water | #d5e5e5 | #344f59 | #121e24 |
| Land | #eeeee2 | #4e615d | #25342f |
| Shore | #afbfae | #748779 | #46574a |
| Chart text | #687b7a | #adbbb1 | #758579 |
| Contour / depth detail | #adcbce | #567880 | #2a4149 |
| Active route | #267c76 | #b0dfc8 | #91bca2 |

The final SVG route stroke is 2.6 CSS px with round joins. Prototype AIS paths
use #916477 stroke, 1.6px; selected fill #cb9cb1; vector line 1px dashed 4/4;
labels #835d70 9px. These literal styles and theme overrides must be assessed
together: normal, alarm, stale/lost and online provenance need semantically
distinct states even where the illustrative target set has no example.

## Pinned source boundaries inspected

- `libs/s52plib/src/chartsymbols.cpp`, `ChartSymbols::LoadConfigFile`: loads
  `chartsymbols.xml` beside the presentation-library path, but a CWD XML takes
  precedence. A dedicated XNav path must not accidentally permit that override.
- The same loader parses color tables, lookups, line styles, patterns and
  symbols. `SetColorTableIndex` / `LoadRasterFileForColorTable` manage palette
  sprites and GL texture state. Switching resources cannot ignore these caches.
- `libs/s52plib/src/s52plib.cpp`: presentation initialization and scheme changes;
  object-class CSV resources also use the shared S57 directory.
- `gui/src/ocpn_frame.cpp`: presentation-library construction. The user's
  `g_UserPresLibData` selects legacy handling; it must not be repurposed as the
  XNav style switch or silently overwrite the user's stock preference.

## Implementation boundary

Derive separately packaged XNav presentation resources from the pinned stock
resources. Preserve lookups, conditional symbology, classifications and symbol
meaning. Standard uses the stock resources unchanged. Store XNav/Standard in
OpenNav-owned preferences. Legacy and Safe retain validated stock presentation.
Choose a narrow initialization hook and a controlled restart for a style change
if live cache/lifetime safety cannot be established; never delete a library
while existing charts retain its lookup pointers.

Depth colors must retain distinct shallow, safety and deep-water roles using
OpenCPN's safety-depth/contour semantics. Do not turn all depth areas into one
prototype water swatch. Lateral/cardinal marks, hazards, lights, restrictions
and traffic schemes keep established semantics. No optimistic "corridor clear"
label unless a validated query actually supports the qualified statement.

Proper Night resources must lower luminance and preserve distinctions; do not
apply the prototype's SVG brightness filter to a Day ENC. Raster images retain
OpenCPN's supported palette/night behavior without destructive recoloring;
only overlays share XNav styling. Exact object-level visual conformance is an
ENC goal, not a claim about raster charts.

## Required evidence (pending)

Public/licensed ENC with coast, depth areas/soundings, marks/lights and hazards;
XNav vs Standard captures; Day → Dusk → Night → Day; XNav → Standard → XNav;
Legacy round trips; software and OpenGL; real boat charts without distributing
their files. Existing blank-chart regression checks remain mandatory. UI/route/
ownship/AIS overlays need separate comparisons and valid source vectors only.
No chart-presentation acceptance is claimed by this inspection document.

## Version 1 implementation (not yet visually accepted)

`resources/chart-style/v1/definition.json` maps the HTML's land, shore, water,
contour and text tokens to twelve S-52 palette entries. The HTML supplies no
complete depth-area set; explicitly documented extra shades retain all five
pinned deep/medium/shallow/very-shallow/intertidal categories. Safety contours
and safety soundings remain distinct. This is a safety-required extension to
an illustrative map, not a claim that the prototype defines those extra colors.

Generation verifies every stock input hash against `source-lock.json`, changes
only RGB attributes in DAY_BRIGHT/DUSK/NIGHT, preserves every symbol, lookup,
line style, pattern and raster symbol sheet, and emits a resource hash header.
The result installs in the separate shared-data `opennav/chart-style/v1`
directory. Original `s57data` stays untouched. The runtime verifies all five
resource hashes before constructing a library. Missing/corrupt resources use
Standard with an explicit diagnostic reason. Display settings select XNav or
Standard with an ordinary controlled XNav restart; existing chart lookup
pointers are never invalidated in a running library.

A narrow loader option prevents the current working directory from shadowing
the chosen presentation. XNav Standard also uses the explicit stock path.
Legacy/Safe retain the original loader's default behavior and ignore the XNav
style preference. The existing GSHHS and shapefile background mechanisms share
the verified XNav land/water palette; those basemaps remain coastline references,
not a substitute for a nautical ENC. No synthetic chart objects are introduced.

Resource tests verify deterministic generation, exact prototype identity,
unchanged navigation sections/sprites, depth-role separation, Night vs Dusk
luminance, original-file preservation and refusal of changed upstream input.
Real ENC appearance, hazards, OpenGL/software rendering, style/mode cycles and
boat display remain pending. Route, ownship and AIS overlay restyling is a
separate unfinished part of this workstream.

The second ink pass adds CHBLK after actual native ENC review found monochrome
text nearly invisible at Night. Day retains the pinned #070707 rather than
substituting low-contrast muted text over shallow water. Dusk uses the existing
prototype floating text token; Night uses its chart-text token. Numeric contrast
checks cover deep water, very-shallow water and land independently. These checks
do not qualify hazard visibility or recolor the unchanged raster symbol sheets.
See the [contrast investigation](reviews/chart-ink-contrast-investigation.md).
