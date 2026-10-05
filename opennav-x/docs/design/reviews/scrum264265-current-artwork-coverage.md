# Supplied artwork, brown paint and typography: bounded current audit

Read-only source audit at `1f349a266b5019d4567c7f1a8b98efe1be4f4c9b`.
The application under current actual-canvas review is frozen `326daf7`; this
record neither changes it nor qualifies its images. Jira readback: 263, 264,
265, 275 and 276 are Testing; 268 remains Idea. No test/build/capture/boat or Jira
mutation was performed for this audit.

The immutable `docs/design/prototype/SYMBOLS.md` distinguishes **1,107 original
catalogue definitions** from the deliberately designed chart artwork in
`src/chart-marker-art.js` and `src/seamarks.json`. Catalogue availability is not
a complete set of redesigned production glyphs. Conversely, several explicitly
designed paths really remain unimplemented; these must not be called safety
requirements merely because the stock renderer still uses them.

| Class / supplied glyph | Current source mapping | Concrete fallback or remaining gap |
|---|---|---|
| Ordinary / preferred-channel lateral buoys | `resources/chart-style/v1/seamarks/mapping.json`: eight private aliases, sixteen exact Simplified selectors; original color/class distinctions retained | Paper and unmatched classifications remain stock; no claim that every fixed lateral beacon has this buoy artwork |
| Four cardinal buoys | `tools/chart_cardinal_art.py`: BOYCAR01–04 derivative tiles selected by existing CATCAM | Classified fixed BCNCAR and other physical beacon bodies have no supplied custom equivalent; physical TOPMAR remains upstream |
| Isolated-danger / safe-water buoys | `tools/chart_seamark_art.py`: BOYISD12 / BOYSAW12 custom geometry | Other variants and separately encoded TOPMAR remain stock; separate real topmark composition is not erased for visual similarity |
| Yellow special buoy / fitted X | `ChartYellowBuoySymbol.h`, `v1/yellow-buoy`: XNSPPY01 body, XNSPPT01 head | X requires the actual TOPMAR7/COLOUR6 and unique compatible platform. Unknown shape/color, CATSPM15/52, patterns and ambiguous composition stay stock; no invented X |
| White/orange pillar | `ChartSpecialBuoySymbol.h`, `v1/special-buoy`: exact warning-buoy derivative XNSPPW01 | Day/Night only. Dusk is a **visible stock-shape difference**, retained after the alternate orange lost classification/readability; no yellow/X substitution |
| Generic fixed beacon | `tools/chart_generic_beacon_art.py`: XNBCNG01 only for Simplified generic selectors1696/1708 | Classified lattice/pile/lateral/cardinal beacon bodies remain stock. Prototype supplies no BCNSPP13/21, BCNLAT21/22 or BCNCAR01–04 custom paths. Physical tower TOWERS01/03 also has no supplied tower drawing |
| Short ordinary R/G/white-yellow LIGHTS | `ChartLightSymbol.h`, XNLIT011/012/013 | Any encoded ORIENT and special/uncertain/nonordinary cases retain original vector/conditional portrayal |
| Ordinary CA long-range/sector LIGHTS point and compact fan | `ChartCaLightPoint.h`, `ChartCaFan.h`, core/private patch boundaries | Current 275/276 source adds qualified central point/fan; actual combined canvas is still a gate. Independent tower/pile conflicts, uncertainty/direction, unsupported viewport/GLES and excessive tile work retain stock. Expanded hover/pin/readout is not implemented by this increment |
| Anchorage / pilot boarding / racon | `tools/chart_anchor_art.py`, `tools/chart_service_art.py`: ACHARE51 / PILBOP02 / RTPBCN02 | Exact owned effective nodes only; does not replace every anchorage/service variation |
| **Rocks / wreck / marina / fishing pattern** | Prototype `chart-marker-art.js` supplies UWTROC03, UWTROC04, WRECKS05, SMCFAC02 and FSHFAC03 | **These custom paths remain unimplemented.** Current neutral ink treatment does not transplant their geometry. Further class/conditional-specific mapping is needed; do not replace all hazard variants generically |
| Submarine cable | CBLSUB06 gets prototype `--mark-area` ink through `tools/chart_cable_paint.py` | Original HPGL/repeat geometry is retained; the supplied simple wave path is **not** transplanted. Ferry ink and Plain cable-area boundary are similarly scoped color changes |
| Emergency wreck physical illustration | Prototype guide has no corresponding source code/placement glyph | No ENC alias invented; an illustration is not a missing mapped production symbol |

Paths in the table under `v1` mean `resources/chart-style/v1`; `Chart*.h` are in
`src/integration`. Original prototype paths are immutable. Source guards and
reverse-equality receipts exist for the implemented mappings; they do not qualify
unseen classes, private runtime rendering, Windows OpenGL or boat recognition.

## Brown paint: removed scope versus retained meaning

`resources/chart-style/v1/definition.json`, `tools/chart_structure_paint.py` and
`tools/chart_construction_hatch.py` now cover land-colored BUAARE, fourteen
BUISGL/FLODOC/MORFAC/PONTON area fills, six BUAARE/non-conspicuous-building shore
outlines, and CROSSX01's 192 opaque **Day** hatch pixels in neutral ink. The hatch's
Dusk/Night source tiles were already transparent; no nighttime geometry was
invented. Earlier audit statements that those building outlines or that Day hatch
remain brown are historical, superseded by these changes.

Global CHBRN and global LANDF were not replaced. Concrete safety-related CHBRN
consumers in pinned `libs/s52plib/src/s52cnsy.cpp` are obscured/faint LIGHTS sectors
(:1257/:1560, LITVIS3/7/8), obstruction areas with WATLEV1/2 (:1943), and wreck areas
with WATLEV1/2 (:3502). These express different visibility/water-level states and
remain intentional stock portrayal, with their geometry/patterns/classification
intact. This is not a claim that brown is universally the only safe hue.
Other unowned LANDF uses are **unmapped scope**, not automatically safety-mandated.
The retained actual Seattle direct-rule audit
`docs/evidence/scrum265-structure-ink/remaining-landf-audit.json` found only the now
owned BUISGL/BUAARE polygon consumers in that view; it does not prove every brown
pixel everywhere is gone, or identify a new screenshot pixel without its feature.

## Fonts and smaller logo

`src/ui/SkagerWordmark.h` fixes the approved SKAGER header wordmark at **124 DIP**.
It is intentionally the approved product brand rather than the immutable
prototype's original OpenNav wording. `UiFontWeight` in `src/ui/Controls.cpp`
requests Segoe UI Variable Display → Segoe UI → Arial. Actual native154 GDI proof
selected Segoe UI because that runner lacks Variable Display; it passed the
124-DIP component drawings. This is not an obsolete-face bypass.

Current `ChartPresentation.cpp:60–88` uses Segoe UI/Arial for 12px land names and
8px normal generated LIGHTS descriptions; water names use the inherited stack,
16px italic, with prototype tracking/opacity. `ChartSoundingFont.h` follows the
**final** 10px inherited chart-depth rule, not the earlier overridden Georgia
CSS. `ChartTextFace.h` maps ordinary factory TX/TE face only, retaining upstream
sizes/weights and custom preferences. Thus ordinary nonmapped label hierarchy,
dense overlaps and native italic/halo/readability remain review boundaries, not
a claim that every chart label has the prototype's size/weight. The copied
private-library hooks need actual private-runtime font evidence.

The boat's same-machine immutable HTML font receipt and native runner font proof
are useful separate references. Neither proves the final native application on
the boat selected the same face. Current `prototype-conformance.md` correctly
keeps screen acceptance open; its earlier "long-range/sector remains open" and
old artwork paragraphs are historical source snapshots, not a description of the
new 275/276 implementation coverage.

## Eligible remaining action

Finish the already selected **263/264/265/275/276 Testing** visual/native/private/
boat gates for the exact next candidate, including the two optional source-locked
light scenes. SCRUM-268's inherited Standard GL label shift remains a separately
recorded Idea, not an authorized silent fix. The five supplied-but-unimplemented
rock/wreck/marina/fishing paths and cable waveform are concrete **parent15
follow-on candidates requiring selection and exact mapping**, rather than claims
of completed artwork or new work authorized by this audit. No blanket global
hazard recolor, chart-density change or invented lighthouse tower is justified.
