# SCRUM-265: lighthouse-scene square, source-only audit

**The square is generic single building BUISGL RCID36**, not part of LIGHTS32 or
a hazard symbol. Its source has no CONVIS, FUNCTN, COLOUR, STATUS, CONDTN, name or
height fields. Absence does not assert that the physical building is inconspicuous;
it selects the generic/default portrayal, not the explicit CONVIS1 portrayal.

The original locked IHO cell `GB4X0000.000` (SHA256 `c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`)
has BUISGL36 at latitude-32.3772903/longitude+61.0310718. Exact recorded viewport
projection gives screen(610.94,447.35), matching the retained square near(611,448).
`source.json` retains all non-null source attributes and the nearby BUISGL35,
which projects separately at(613.58,411.31). No chart feature was injected.

## Exact stock presentation

Pinned `build/integration-source/data/s57data/chartsymbols.xml`:

- Simplified default lookup **id1091/RCID31143**, `SY(BUISGL01)`, Point Symbol,
  radar On Top, category Other, comment32220. No attribute condition.
- Explicit CONVIS1 instead selects **id1090/RCID31142**, `SY(BUISGL11)`,
  Area Symbol/Standard. Function-specific rules precede these; all are retained
  in `resource-audit.json`. Selection/visibility/priority must remain intact.
- BUISGL01 symbol **RCID1307**, description “single building”, bitmap9×9,
  pivot(4,4), rectangle(459,78)–(468,87). Its vector has the same square identity:
  CHBRN fill and LANDF outline (`WLANDFKCHBRN`). The default parser prefers the
  bitmap (`chartsymbols.cpp:397,635`), despite the available vector definition.
- The only direct XML consumers are Simplified1091/31143 and Paper2021/30292.
  No BUISGL01 literal occurs in the actual core/private S52 C++ sources.
  The tile has exactly one bitmap owner, BUISGL01; no overlapping symbol,
  pattern or line-style rectangle was found.

All three installed tile RGBA arrays equal pinned originals. The retained Day
capture's screen rectangle(607,443)–(616,452) matches all49 opaque source pixels
exactly. Alpha-compositing the full tile on observed land differs only by≤1
channel at6 edge pixels (Pillow versus GL blending), not by geometry or colors.
This source/projection/pixel identification is strong; no new runtime lookup
breakpoint was authorized or performed. Original capture source was exact5bb.

## Prototype and narrowly proposed mapping

Immutable `docs/design/prototype/index.html` SHA256 matches its original manifest.
BUISGL01 appears only in the untouched source-symbol catalog. There is **no
explicit building artwork** in `chartMarkerArtwork` (1199 onward). The fallback
is a labelled reference pin; `chartMarkerTone` assigns its service tone and
`.marker-service` uses `--mark-service`. That reference pin must not replace the
navigation building square. The proposed color adaptation below is an explicit
inference from those prototype roles, not an assertion of exact supplied building
artwork. Brown itself does not make this object a hazard.

Propose an **isolated alias selected only by Simplified1091/31143**, preserving
its full lookup signature, 9×9 geometry, pivot, every alpha byte and filled square
with a separately represented border. Use exact prototype **mark-service for
fill** and **mark-black for outline**, applying prototype Night chart brightness
0.78 once. Preserve source BUISGL01, all Paper use, global CHBRN/LANDF,
BUISGL11/CONVIS1, specialized buildings, Standard/Legacy and all other consumers.

| Theme | Proposed fill | Proposed outline | Fill:land stock→proposal | Outline:land stock→proposal |
|---|---|---|---|---|
| Day |124,133,138|83,100,95|2.574→3.220|4.471→5.353|
| Dusk |168,187,183|195,206,194|2.093→3.276|1.697→4.050|
| Night, effective |98,115,108|107,119,109|1.293→3.000|1.248→3.212|

These are opaque sRGB token ratios against actual effective LANDA, not acceptance
of antialiased native pixels. Night fill is near3:1 and needs actual visual review.
The fill/outline ratios are1.662/1.236/1.071 respectively; both roles and all alpha
must remain represented even where nighttime tones are close. BUISGL11 remains
visually distinct with its pinned **LANDF fill and CHBLK outline**, especially
its brighter Dusk/Night border. Both source building symbols are filled squares;
there is no solid-versus-hollow distinction to claim. No new conspicuity value
or function may be inferred from color. `proposed-contrast.json` records the exact
unchanged conspicuous colors and ratios alongside this proposal.

The tile contains29/19/14 RGBA colors across Day/Dusk/Night, including baked mixed
edge colors. Replacing only exact CHBRN pixels leaves brown behind. Any later
implementation must use an explicit source-locked two-role derivation for the
small alias tile, preserve all81 alpha bytes/geometry and provide matching vector
colors without changing HPGL geometry. Do not recolor the shared source tile or
use a broad hue/grayscale mask. A new isolated tile must prove exclusive unused
space before placement.

## Checks before accepting a later implementation

Focused inverse resource checks should prove one exact lookup substitution,
one owned alias and its palette/tiles only; original symbols, the Paper lookup,
CONVIS1/function rules and all unrelated bytes stay pinned. Verify alpha/pivot,
all three roles/themes, source fallback and vector/raster parity. Then use the
existing warm actual lighthouse scene for software/GL Day/Dusk/Night/Day return,
retaining strict equality and checking BUISGL36 plus the unchanged conspicuous
source distinction. Native Windows/private-renderer/boat readability stays open.

This commit contains only the audit, source extracts, tiny original/capture crops
and analytical contrast proposal. No application edit, replay, build, CI dispatch
or boat action occurred. `files.json` hashes retained evidence; decoder scripts
use the existing enc-reader and bundled Pillow runtimes, with no package install.
