# Chart-symbol atlas — design handoff

Open **Settings → Navigation → Chart symbols & seamarks**, **Chart layers → Chart symbols & seamarks**, or the corresponding link in **Help**. The complete reference is embedded in `index.html` and works offline.

## Included examples

- **1,018 point symbols**, **59 line styles** and **30 area-pattern tiles**: 1,107 unique definitions in the pinned OpenCPN resource library.
- Thirteen categories: buoys/beacons, lights/signals, rocks/wrecks, depths/survey, routes/limits, harbours/services, offshore/crossings, coast/landmarks, seabed/fishing, tides/currents/ice, vessels/navigation, inland signs and other chart objects.
- Original OpenCPN day, dusk and night palettes. All bitmap examples use the source sprite crops; vector-only examples use geometry derived from the source HPGL.
- Twelve bilingual seamark examples: port, starboard, four cardinals, two preferred-channel marks, isolated danger, safe water, special mark and emergency wreck marking. Region A/B changes the lateral colours and corresponding glyphs. Sweden uses region A.
- English descriptions, Swedish category/guide-name search, exact symbol-code search, geometry filtering, pagination, inspectable metadata and JSON exports.

The emergency-wreck example is an original **physical buoy illustration**, explicitly labelled as such. This resource snapshot contains no dedicated emergency-wreck glyph. The physical drawings are schematic examples, not construction drawings. A topmark, colour or light illustrated here is not a claim that every real mark is equipped identically.

## Examples on the chart

The main chart contains **29 selectable objects**, each with an original OpenNav vector design and a mapping to its atlas source definition: port/starboard and preferred-channel marks, all four cardinals, isolated danger, safe water, special mark, a beacon, light, racon, rocks, wreck, anchorage, marina, pilot boarding point, submarine cable and fishing-area pattern. The Swedish fictional passage uses IALA A. Its initial lateral colours do not change when exploring region B in the atlas.

Select an object to inspect its bilingual name, meaning where available, mock position and source reference. **Place on demo chart** is available for all 1,107 library definitions. Mapped definitions use their designed chart artwork; definitions without a mapped design use an explicitly labelled reference pin with the source code. The atlas continues to show the exact source glyph, rather than substituting an approximate navigation symbol. Tap to add an example; Cancel/Escape returns to its source detail. Added examples have a Remove action and last only for the page session. They are excluded from configuration backups. The emergency-wreck physical illustration has no placement action because this snapshot has no corresponding chart glyph.

Chart layers controls visibility and optional labels. Point glyphs remain upright and keep their display size during zoom and rotation; lines and patterns repeat over sample chart geometry. Day/dusk/night use a separate OpenNav chart palette: muted lateral red/green, amber lights and fine neutral or mauve detail. All chart artwork and chart-object previews are SVG, with no bitmap scaling. The atlas retains its original source palettes. Line segments, pattern spacing, centring and positions are illustrative preview choices, not validated ENC portrayal. Real source anchors and lookup information remain available in the catalogue and upstream XML.

Editable positions are in `src/chart-symbols.json`; rendering and interaction are in `src/chart-symbols.js` and `src/chart-symbols.css`. Original vector paths and explicit definition mappings are in `src/chart-marker-art.js`.

The LIGHTS13 demo instance is the fictional **Långskär sector light**. A small light point replaces the illustrative tower. Red, white and green arcs are shown at a short radius by default; hover/focus extends them, and selection pins the extension. Its compact selection card offers Details and one Collapse action. Escape, a second selection or a blank-chart click also collapses it. Sector paths are in chart coordinates, while the point symbol and labels remain upright. The preview radii are 44 and 340 chart units, chosen for readability; they are not the nominal 12 nm range. Bearings, colours, height and range are mock data.

This interaction takes inspiration from [OpenCPN’s extended light sectors on rollover](https://opencpn.org/wiki/dokuwiki/doku.php?id=opencpn%3Amanual_basic%3Achart_panel%3Achart_panel_options%3Avector_charts). The use of coloured arcs follows the [IALA sector-light concept](https://www.iala.int/wiki/dictionary/index.php/Sector_Light). These references do not validate the fictional sector geometry or imply safe passage through this illustrative chart.

## Source and provenance

Resource source: [OpenCPN data/s57data](https://github.com/OpenCPN/OpenCPN/tree/1bf728e17feaae05fddad3aad5be677e15c1e89c/data/s57data), commit `1bf728e17feaae05fddad3aad5be677e15c1e89c`, retrieved 28 September 2026. The bundled files in `vendor/opencpn/` are unmodified source downloads:

| File | Purpose |
| --- | --- |
| `chartsymbols.xml` | Definitions, colour tables, sprite locations, vector instructions and lookup data |
| `rastersymbols-day.png` | Day sprite sheet |
| `rastersymbols-dusk.png` | Dusk sprite sheet |
| `rastersymbols-dark.png` | Night sprite sheet |
| `COPYING` | Upstream GNU GPL version 2 license |
| `SOURCE.json` | Repository, pinned revision, retrieval date and file manifest |

The XML has repeated definitions. The importer retains the **last definition for each name within each geometry type**, retaining an earlier description only when the final one is empty. Codes repeated across point, line and pattern types remain distinct, e.g. `point:ACHARE51` and `line:ACHARE51`. Source descriptions, including spelling and unspecified variants, are retained. Categories are an editorial index for this prototype, not official ENC object-class mappings.

The generated JSON includes the XML SHA-256, revision and deduplication rule. The importer rejects unsupported vector commands and invalid sprite bounds. No dependency installation or runtime network access is required.

## Files for development

- `symbol-library.mjs` — reproducible resource importer, category index and embedded sprite CSS.
- `src/symbol-catalogue.json` — generated catalogue, palette tables, crop/anchor metadata and derived vector geometry.
- `src/seamarks.json` — editable bilingual guide data and region-dependent mappings.
- `src/symbols.js`, `src/symbols.css` — atlas interface, physical illustrations, glyph previews, filtering, detail views and exports.
- `vendor/opencpn/` — complete original resource files and license, including XML information not used by the preview.

Run `node build.mjs` to refresh both the standalone HTML and JSON catalogue; `node verify.mjs` checks the result. Exporting an individual definition or the catalogue from the UI produces a JSON development reference. Those JSON files reference the bundled sprite assets; they are not self-contained image packages.

## Portrayal boundary

“Complete library” means **all unique definitions in this pinned resource snapshot**. It does not claim every symbol used worldwide, every Swedish national variant, every current S-100 product, or a complete INT 1 / Kort 1 implementation. It also includes rendering aids, update marks, target graphics and inland variants which are not all physical seamarks.

Production should use the host/chart provider's portrayal engine and supported standards. A glyph alone is insufficient to draw an ENC feature correctly. Object attributes, conditional symbology, display category, chart scale, safety contour/depth, light sectors and rhythms, orientation, text, topmark placement and line/pattern spacing all affect portrayal. The atlas shows enlarged glyphs and individual line/pattern elements, not their final physical display size or a fully composed chart scene. Vector stroke widths and fit-to-tile scaling are preview choices. Pattern spacing and full lookup rules remain in the upstream XML.

Virtual/synthetic AIS aids to navigation, national extensions and other symbols absent from this snapshot need separate provider-supported examples during implementation. Vessel AIS icons must not be reused as virtual aids-to-navigation symbols. The fictional navigation chart elsewhere in this prototype remains illustrative; the atlas does not replace it with an operational ENC.

## Meaning and standards references

- [Sjöfartsverket: floating seamarks](https://sjofartsverket.se/sv/tjanster/transporter-och-farleder2/farleder-och-underhall/sjomarken/flytande-sjomarken/) — Swedish names, colours, topmarks and light patterns.
- [IALA R1001: Maritime Buoyage System](https://academy.iala.int/product/r1001/) — regions A/B and mark families.
- [Sjöfartsverket: publications / Kort 1](https://www.sjofartsverket.se/sv/tjanster/sjokortsprodukter/publikationer/) — international and Swedish chart-symbol reference.
- [IHO standards and specifications](https://iho.int/en/standards-and-specifications) — official chart and electronic-display standards. Paper-chart conventions and electronic portrayal are distinct.
- [OpenCPN vector palette documentation](https://opencpn.org/wiki/dokuwiki/doku.php?id=opencpn%3Amanual_advanced%3Acharts%3Avector_palette) — relationship between XML, sprite sheets and display palettes.

## Attribution and reuse

OpenCPN source resources and derivatives retain their upstream GPL-2.0 license. The full license is bundled in `vendor/opencpn/COPYING`; preserve it and the source/provenance when redistributing this reference. Review the production application's dependency and distribution terms when incorporating these resources. The IALA, IHO and Sjöfartsverket references are links and factual guidance; their publications and illustrations have not been copied into the atlas. The physical buoy SVG drawings and surrounding interface are original prototype work.
