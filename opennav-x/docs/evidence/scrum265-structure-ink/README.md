# SCRUM-265 — bounded structural area paint

Base `a3e84771652c920479517f0d16a1dd6133c440d2`; pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. This changes only fourteen
`AC(CHBRN)` area-fill tokens to isolated `XNSTR`, plus that role in three verified
color tables. It does not change global CHBRN, geometry, chart data, boundaries,
category selection, labels, visibility, lookup ordering, line/point symbols or
raster pixels. Standard/Legacy/Safe retain their existing resource selection.

## Actual feature and rule evidence

The retained Day image is the actual a3e software chart, not a synthesized after
capture. Read-only pyogrio/GDAL inspection of public NOAA US5SEAFL, bbox
`[-122.39,47.58,-122.33,47.62]`, found:

| Class | Applicable polygons | Other geometry retained |
|---|---:|---|
| BUISGL | 69 buildings | None in this bbox |
| FLODOC | 4 floating docks | None in this bbox |
| MORFAC | 10 dolphins: CATMOR1, WATLEV2 | 45 points, including 2 CATMOR7 mooring buoys |
| PONTON | 9 pontoons | 1 line |

CRANES (2) and LNDMRK (6) are points; SILTNK is absent. `actual-enc.json`
retains public feature IDs, selected attributes, geometry types, reader version,
input hashes and bbox, with no geometry dumps. `audit-source.py` is the read-only
inventory procedure. The screenshot alone cannot establish the class of each
individual pixel; this inventory establishes which actual structural polygons
occur in the view. `lookup-inventory.json` records every pinned lookup for these
four classes, including unchanged point/line variants.

| Class | Plain id / RCID | Symbolized id / RCID | Retained instruction after fill |
|---|---|---|---|
| BUISGL | 17/32053, 18/32054, 19/32055, 20/32056 | 357/32392, 358/32393, 359/32394, 360/32395 | Existing OBJNAM for FUNCTN33; CHBLK outline for CONVIS1, otherwise LANDF; all 1px SOLD |
| FLODOC | 61/32097 | 401/32436 | LS(SOLD,2,CSTLN) |
| MORFAC | 98/32134 | 438/32473 | LS(SOLD,1,CHBLK) |
| PONTON | 135/32171 | 477/32512 | LS(SOLD,2,CSTLN) |

MORFAC's pinned **area** rules are generic: there is no category-specific area
branch to rewrite or remove. The same selector continues to apply; no new
category assumption is introduced. CATMOR1=dolphin, 2=deviation dolphin,
3=bollard, 4=tie-up wall, 5=post/pile, 6=chain/wire/cable and 7=mooring buoy,
from pinned `s57expectedinput.csv` attribute40. Category6's dashed magenta
**line** rule768/31844 and category7's hazard **point** rules remain exact.
The actual polygon fixtures are all category1. Unexpected categories are not
reclassified, and no point/line paint is borrowed for an area.

## Prototype mapping and engineering decision

Immutable `index.html:8` declares `.chart-land` with `fill:var(--land)`,
`stroke:var(--shore)`, width1. The theme variables are at lines6,13,14 and the
chart-only Night brightness `.78` at line15. The prototype provides a land/shore
visual hierarchy, not a complete set of real floating-dock features.

The initial shore-fill proposal was rejected after inspection: FLODOC/PONTON
already have CSTLN shore borders, so filling them with shore would remove
fill-to-border contrast. Root explicitly selected **land fill** instead,
consistent with `.chart-land` and SCRUM-231. XNSTR is `(238,238,226)` Day,
`(78,97,93)` Dusk and `(29,41,37)` Night. Night brightness is applied exactly
once. Opacity remains full. All existing outlines remain.

`controlled-area-paint.png` compares the previous CHBRN fill (upper subrow) and
new land fill (lower) against land and all five depth/intertidal colors. It uses
controlled rectangles and exact resource RGBs, not the native chart renderer.
In each cell: floating structure with 2px CSTLN edge, building with 1px LANDF,
and dolphin with 1px CHBLK. `receipt.json` includes the complete contrasts.

The floating border/fill contrasts are 1.650 / 1.718 / 1.469 for Day/Dusk/Night.
Land fills match land (contrast1) and remain distinct from every depth color;
retained building/dolphin edges carry their land footprint. Some water shades
are close: new fill/medium-water contrast is 1.039 Dusk and 1.069 Night. This is
**not** a universal contrast improvement or native readability acceptance.
Original LANDF brown outlines remain intentionally unchanged, as do unrelated
brown hazards/obscured sectors. Do not claim every brown pixel is eliminated.

## Focused verification and limits

- 171 structural-area checks passed: exact IDs/RCIDs/selectors, full restored
  node equality, theme RGB, border distinction, unchanged point/line variants,
  negative mutations to labels/borders/categories/RCIDs/duplicates/role alpha,
  global CHBRN and source drift; full existing resource reverse validator runs.
- Reversing just the fourteen AC tokens and three XNSTR definitions gives
  **byte-identical** a3e XML, SHA256
  `00ac4246589f1469dec89438d3ea6a3de245dae5ece928da9ee342fbce0b525c`.
- All three raster sheets and S52RAZDS.RLE remain byte-identical to a3e.
- Added helper is included in CMake regeneration and native-preflight source
  identity dependencies. Existing broad resource-test expectations explicitly
  enumerate the new bounded exceptions; their guards were not loosened.
- Python syntax checks passed. No C++ change, full build, broad suite, app launch,
  CI or boat activity was performed. Root still needs actual Day/Dusk/Night
  software/OpenGL before/after review, native Windows and boat readability.

Focused command:

```sh
python3 tests/chart_structure_resources_tests.py \
  --source /path/to/pinned/OpenCPN/data/s57data \
  --generated /path/to/verified/generated/v1
```

## Authorized six-outline follow-up

The fill-only evidence above is retained as that stage's exact result. The next
bounded change maps only `LS(SOLD,1,LANDF)` in BUISGL18/32054,20/32056,
358/32393,360/32395 plus BUAARE16/32052,356/32391 to `LS(SOLD,1,XNSHR)`.
XNSHR is an isolated role with exact prototype shore RGB: Day `(175,191,174)`,
Dusk `(116,135,121)`, Night `(55,68,58)` after the single .78 normalization.
These now follow both parts of `.chart-land`: land fill and shore edge. All six
retain SOLD width1, class/table/selectors, labels and visibility. Conspicuous
CONVIS1 buildings retain CHBLK; floating structures retain CSTLN; global LANDF
and every other use remain exact.

`outlines/` retains the separate focused results, generated identity and controlled
1px comparison. Reversing exactly six LS tokens and three XNSHR definitions
restores the fill-only XML byte for byte; rasters/RLE remain identical. Day
outline-to-land contrast decreases **4.471 → 1.650**, Dusk changes
**1.697 → 1.718**, Night **1.248 → 1.469**. This deliberately follows the prototype
hierarchy while retaining stronger conspicuous building outlines, and requires
actual visual readability review. It is not a hazard-readability claim.

`remaining-landf-audit.json` intersects actual feature classes/geometry with every
pinned lookup instruction containing LANDF. Within this bbox, only BUISGL's 69
polygons and BUAARE's 2 polygons match such area rules. CRANES(2), LNDMRK(6) and
PRDARE(7) occur as points; their possible LANDF area/line rules do not apply to
those observed geometries. This is a direct-rule audit, not proof that no symbol
artwork or conditional procedure can use brown. No additional class was changed.
