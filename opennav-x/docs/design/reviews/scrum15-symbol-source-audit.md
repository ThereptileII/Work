# SCRUM-15: mapped prototype symbol boundary audit

Read-only review, 2026-10-03; related Jira comment 10625. No product changes,
application runs, builds or CI. This inventories all **26 namespace-qualified
mapped IDs**, not every chart symbol. Different real chart geography is not a
prototype defect. This is a source-level eligibility assessment, not navigation,
Windows, or boat acceptance.

The strongest next bounded candidate is **PILBOP02**, the prototype diamond/P
pilot-boarding glyph. Its four effective lookups all mean pilot boarding, unlike
generic marina/service fallbacks. Replace only its effective raster tile in the
verified SKAGER resource pack, retaining the geographic anchor, existing text,
area boundaries and display rules. **RTPBCN02** is another narrow candidate with
only two lookups. Neither requires a global palette change or a broad atlas
rewrite. Use the exact prototype service ink (Day `#7c858a`, Dusk `#a8bbb7`, Night
`#7e948a`), subject to an actual native legibility check; do not change all CHMGD.

## Sources and effective selection

The companion [inventory](../../evidence/scrum15-symbol-source-audit/inventory.json)
records prototype hashes, every mapped definition, bitmap metadata and all direct
namespace-correct effective lookup instructions/attributes. Stock XML SHA-256 is
`84f93522576ed5872b24865cf6161872ed0b4b634f77c64ca697a04d3568e886`, matching
`resources/chart-style/v1/source-lock.json`; upstream commit is
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.

Pinned source pointers below are relative to upstream:

- `libs/s52plib/src/chartsymbols.cpp:388,617`: point symbols default to preferring
  bitmaps; the last named symbol overwrites the earlier definition. A `<definition>V`
  label alone does **not** mean the vector renders. `BuildLookup`, around line
  480, replaces an earlier RCID within the same table. Line/pattern hashes have
  different insertion behavior; each mapped line/pattern has only one definition.
- `libs/s52plib/src/s52plib.cpp:3456` (`RenderSY`): actual `SYDF` selects vector or
  raster; angle/ORIENT remains significant. `RenderRasterSymbol`, line 3015,
  uses the selected bitmap rectangle for software and GL, applying chart/SCAMIN,
  display, content-scale and pivot rules. `chartsymbols.cpp:870,879` supplies the
  same named atlas rectangle to both paths. Changing an earlier HPGL definition
  of an effective raster symbol would be ineffective.
- Immutable prototype `docs/design/prototype/src/chart-marker-art.js` supplies
  exact paths and a default `27/32` artwork scale; `seamarks.json` supplies regional
  mappings; `chart-symbols.css:10–15` supplies the final theme roles. Emergency
  wreck marking has no mapped code and is excluded. Generic reference pins are
  placeholders, not replacement symbol designs.

## Effective mappings

`R` = raster atlas, `V` = HPGL vector. S/P = Simplified/Paper point tables;
Plain/Symbolized are area tables. “Eligible” means a narrowly gated artwork
candidate retaining all existing selection and navigation rules, not approved
replacement of the surrounding chart semantics.

| Mapped ID(s) | Effective RCID / renderer | Actual lookup boundary | Artwork assessment and caveat |
|---|---|---|---|
| ACHARE51 | **1105 R**, after 1228 | ACHARE Plain 32038 / Symbolized 32377 | Already SCRUM-244: exact anchor in dedicated tile. Effective area rules still include RESTRN/RESARE and boundary instructions. Do not edit the earlier definition or same-named line symbol. |
| PILBOP02 | **1 R**, after 1423 | PILBOP S 31253 / P 30476; Plain 32166 / Symbolized 32507 | **Eligible first:** exact diamond/P preserves pilot-boarding meaning. Keep `Plt`/OBJNAM text, boundary styles and point/area placement. Stock tile has multiple colors; replacement must be local to this named artwork. |
| RTPBCN02 | **2259 R** | RTPBCN S 31289 / P 30512, default | **Eligible narrow:** center/radiating arcs match radar-transponder-beacon role. Do not apply to radar reflectors or unrelated radio equipment; preserve identification/visibility and hotspot. |
| SMCFAC02 | **2108 R** | HRBFAC CATHAF=5 S/P/areas; SMCFAC area defaults; inland hrbare/hrbfac | **Defer exact generic replacement:** roof/anchor evokes marina/building, but SMCFAC default area is not restricted to a specified marina facility. Can consider a narrower class-qualified hook; preserve land fill and boundary. |
| BCNGEN01 | **11 R**, after 1238 | Many Paper beacon defaults/shapes; Simplified `_bcngn`/`_slgto` | **No blanket exact swap:** prototype roof/cone/supports can assert physical form absent from this generic fallback. Paper topmark composition also depends on placement. |
| BOYCAR01 / 02 / 03 / 04 | **1270 / 1271 / 1272 / 1273 R** | BOYCAR Simplified CATCAM=1/2/3/4 respectively (31074–31077) | **Conditional eligible family:** N/E/S/W meaning is explicit, so categorical two-cone portrayal has a real selector. Preserve each cone orientation and black/yellow distinction. Do not infer physical TOPSHP measurements or change Paper bodies. |
| BOYLAT13 / 14 | **2050 / 2051 R** | BOYLAT Simplified shape/color and CATLAM rules; inland lower-case boylat | Green/red conical categories. Regional guide labels alone do not encode every lookup meaning; exact treatment must preserve color and each actual category. Do not infer region or a new physical topmark from ID alone. |
| BOYLAT23 / 24 | **2052 / 2053 R** | BOYLAT Simplified shape/color and CATLAM rules; inland boylat | Green/red can categories; same limitation. Existing selectors include preferred-channel contexts. A global one-color guide interpretation must not alter the lookup meaning. |
| BOYCAN72 / 73 | **26 / 26 R** | BOYLAT Paper banded cans; 72 also inland boylat/boywtw | Red-green-red / green-red-green bodies. Preserve bands and Paper physical composition, not just a guide's regional port/starboard label. RCID repetition across different names does not merge their glyphs. |
| BOYCON66 / 67 | **31 / 31 R** | BOYLAT Paper banded cones; inland boylat; 67 also boywtw | Red-green-red / green-red-green bodies. Same band/category/topmark caveat; require body/topmark alignment proof before exact replacement. |
| BOYISD12 | **2049 R** | BOYISD Simplified default 31080 | Isolated-danger class is explicit. Two-sphere/category artwork is plausible, but preserve danger recognition, black/red contrast and prominence; do not adopt decorative fading automatically. |
| BOYSAW12 | **1294 R** | BOYSAW Simplified default 31098 | Safe-water class explicit. Preserve red/white recognition; sphere must remain a class depiction rather than an asserted surveyed physical TOPSHP. |
| BOYSPP11 | **1297 R** | BOYSPP Simplified shape/CATSPM cases **and empty default 31116** | **No blanket X:** unknown/generic fallback does not establish a physical X topmark. A later qualified selector could use known TOPSHP; retain the default. |
| LIGHTS13 | **1398 V** (`prefer-bitmap=no`) | LIGHTS S/P via conditional procedure, not direct SY lookup | **Keep flare semantics:** `_selSYcol` chooses white/yellow/orange light flare; angle is appended and ORIENT can rotate it. Prototype circle/four rays is not equivalent sector geometry or directional flare. Existing light colors, sectors and labels remain. |
| UWTROC03 | **2249 R** | UWTROC S 31328 / P 30580 via OBSTRN04; direct `$CSYMB` only | **Keep safety glyph:** unknown VALSOU and WATLEV=3 selects this underwater dangerous rock; DEPVS safety background is absent from prototype X/dots. Do not apply prototype 0.8 opacity. |
| UWTROC04 | **3164 R** | Same conditional boundary; direct `$CSYMB` only | Awash/covers-and-uncovers depiction also used for missing/default WATLEV. Broken ring/plus is not enough evidence to discard existing uncertainty/water-level treatment. Keep stock pending a semantic fixture. |
| WRECKS05 | **2038 R** | WRECKS S 31350 / P 30602 via WRECKS02; direct `$CSYMB` only | **Keep dangerous unknown-depth hierarchy:** CATWRK=2/WATLEV=3 and conditional fallback choose it. Prototype ship/wave loses DEPVS safety background. Known-depth paths use other danger/depth/quality glyphs and must remain intact. |
| line:CBLSUB06 | **2012 V** | CBLSUB Lines 31786 `LC(CBLSUB06)` | Prototype wave can inspire a bounded cable artwork, but repeated line scale/phase must preserve cable recognition. Decorative `.16` opacity is not a safe blanket cable presentation. Keep actual geographic line geometry and lookup. |
| pattern:FSHFAC03 | **2001 V** | FSHFAC Plain 32101, CATFIF=1 `AP(FSHFAC03);LS(DASH,1,CHGRD)` | Fishing-stakes area: preserve tiling, density and boundary. Prototype isolated diamond/cross does not specify safe pattern spacing. **SY(FSHFAC03) is a separate point symbol**, not this mapped AP pattern. |

## Topmarks and conditional safety

Do **not** claim that every Simplified buoy receives a second physical topmark.
TOPMAR Simplified lookup **id 1262 / RCID 31314 is blank**. Its explicit crane
cases TOPSHP=16/15 use TOPMAR88/87. Paper includes physical glyph and
`CS(TOPMAR01)` rules; `s52cnsy.cpp:2934` chooses floating versus rigid composition
from coincident ATON context. Consequently a Simplified cardinal-family change
is materially different from changing a Paper body underneath physical topmarks.

`_selSYcol` at `s52cnsy.cpp:407` distinguishes flare SY from all-round circle CA;
UWTROC unknown-depth branches at 1800–1835 and WRECKS branches at 3395–3438 retain
water level, dangerous-depth, quality and fallback decisions. Prototype icon names
and sample drawing locations cannot substitute for these conditional facts.

A next PILBOP02 increment should reuse SCRUM-244's dedicated transparent-tile
approach if the exact prototype art does not fit the old pivot/bounds, preserving
the same geographic anchor. Prove all other pixels, lookup instructions and
resources unchanged, then compare software/GL Day/Dusk/Night with real pilot
boarding fixtures. Preserve Standard/Legacy resources. This review authorizes no
implementation and reports no new release qualification.
