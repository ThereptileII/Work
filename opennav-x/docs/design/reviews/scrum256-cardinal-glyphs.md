# SCRUM-256: classified Simplified cardinal artwork

Only the four explicitly classified `BOYCAR01`–`BOYCAR04` glyphs change in the
verified SKAGER pack. The immutable prototype remains untouched. This is a
bounded resource implementation and Linux native-loader proof, not native
Windows, full chart, real-ENC recognition or boat acceptance.

## Semantic boundary

Pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7` has exactly one direct
lookup for each named symbol: Simplified BOYCAR RCIDs 31074–31077, attributes
CATCAM 1/2/3/4, meanings north/east/south/west. Existing instructions, labels,
Hazards priority, Standard display category and every selector remain exact.
`TOPMAR` Simplified default RCID31314/id1262 has an empty instruction. Its crane
exceptions and all Paper topmark/body composition stay unchanged. The two cones
are a categorical cardinal portrayal, not invented physical TOPSHP observations.
Generic, lateral, special, light, hazard, conditional and Standard/Legacy symbols
are outside this change.

The effective raster definitions are RCIDs1270–1273, despite their trailing
`definition V`: the pinned loader prefers the bitmap. All original vector data,
color references and old bitmap pixels remain byte-identical. Only six bitmap
attributes (width, height, pivot x/y, atlas x/y) per symbol change. The full XML
rule tree is required to reverse to the previous pinned semantics after the
existing approved paint changes and these exact attributes are restored.

## Exact derivation

`chartMarkerArt` from immutable `chart-marker-art.js`, evaluated against the
immutable `seamarks.json`, supplies the actual paths and band order. The four
cone pairs are up/up, up/down, down/down, down/up. Black/yellow bands are B/Y,
B/Y/B, Y/B, Y/B/Y. This retains the prototype's 1.3 stroke, 1.6 band stroke,
round caps/joins, 0.13 cone fill and `27/32` artwork scale without redesign.
Theme RGB roles come from final `chart-symbols.css`; Night applies the existing
whole-chart brightness0.78 once to black, yellow and the water-filled base circle.
Alpha is unchanged. Theme-specific RGBA captures preserve compositing between
overlapping strokes; a single-color coverage mask would lose those bands.

Dedicated 24×28 tiles use x116/148/180/212,y1160 and pivot12,14, outside the
SCRUM-244 anchor and SCRUM-254 service slots. Every declared symbol/pattern,
including overwritten entries, is collision-checked with a two-pixel transparent
sampling moat; prior alpha in any tile/moat fails closed. The atlas stays1500×1200.
The translated prototype origin remains the geographic pivot, and unchanged
upstream scale/rotation/content-scale/SCAMIN handling still applies. No painter,
placement, renderer scale or global palette algorithm changes.

Source-locked SVGs, RGBA rows, raw-pixel hashes and rasterizer provenance are in
`resources/chart-style/v1/cardinals/`. Generation remains Python-stdlib only.
The retained derivation script requires Node, librsvg and Pillow and is evidence,
not a build dependency. CMake configure dependencies and native-preflight source
inventory include the helper, all new assets and the seamark guide.

## Evidence and limits

See `docs/evidence/scrum-256-cardinals/`. The actual pinned ChartSymbols constructor,
ProcessSymbols/BuildSymbol, PNG loader, GetImage and GL rectangle lookup execute
in a Linux wx fixture:11,790 checks passed across four categories and three themes.
SVG rerenders independently agree with every loaded alpha byte and premultiplied
color within1.01 channel units (wx representation rounding). Twelve native tile
PNGs are retained, with a clearly labeled 1×/3× inspection sheet. This fixture
has no GL drawing context and does not claim full software/GL chart acceptance.

19,417 focused resource checks passed. Independent tests check exact prototype path geometry, palette roles, band order,
all category/topmark/Paper lookup trees, all404 new pixels per theme, unchanged
old tiles and every other atlas byte. Existing anchor/service/neutral-ink proofs
continue after excluding only separately proved cardinal rectangles. Fourteen
negative controls reject changed selectors/table, nonblank default TOPMAR,
source corruption, atlas collisions and occupied tile/moat pixels. The separate
seven Windows preflight refusal tests pass; no native CI or full application build ran.

Observed native-size category stroke samples remain distinguishable in the
reviewed component sheet. Against prototype water, Night black contrasts3.30–3.76
and yellow3.085–3.567; Dusk yellow3.47–4.00. Exact Day yellow is weaker at2.096–2.336
(black3.905–4.604). The rasterizer blends narrow band strokes over the first-color
stem, so samples are not necessarily pure nominal RGB. The measurement selects
high-coverage pixels closer to the intended band role than the other role and
records actual RGBA/composited RGB. These are local sample measurements, not a
claim of accessibility compliance or real-chart recognition. No color/stroke
exception was silently introduced. Dense ENC background, native Windows scaling,
and the actual boat display remain required readability gates.
