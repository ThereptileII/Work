# SCRUM-265: neutral construction hatch without pattern loss

The user requested removal of the remaining brown. This bounded follow-up
changes **only CROSSX01 / RCID 3** to isolated palette role **XNHAT** and recolors
its **192 opaque Day pixels** from RGB `(177,145,57)` to exact prototype Day
`--mark-black` `(83,100,95)`. All 64 transparent Day pixels, every alpha byte,
16 × 16 geometry, spacing, pivot, origin, construction classifications,
conditional rules and dashed CSTLN boundaries remain exact. Global CHBRN and
all other brown consumers remain unchanged.

The earlier [feature receipt](../scrum265-pier57-ruin-hatch/README.md) establishes
that the remaining visible Pier 57 feature is a ruined, submerged pier/jetty.
Its semantic cross-hatching is retained; its brown hue is not required to retain
that distinction. This explicit follow-up supersedes the earlier color exception
only, not the chart feature's meaning or pattern.

## Exact source and dispatch audit

Pinned core is OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`; private o-charts
source is `c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8`. Both compiled SLCONS03
implementations have the sole `AP(CROSSX01)` consumer: core
`libs/s52plib/src/s52cnsy.cpp:2253`, private `:2284`. Their area branch adds the
pattern; the point and line paths do not. The complete conditional lookup
inventory contains 12 unchanged records, including Inland ENC lowercase
`slcons`: area IDs **181, 280, 522, 627**; line IDs **830, 906**; Simplified point
IDs **1253, 1564, 1565**; Paper point IDs **2371, 2797, 2798**. Exact attributes,
priorities and instructions remain unchanged. `independent-delta.json` retains
the full identities/instructions. No direct XML AP caller was found.

Both parsers choose raster when a pattern has no vector node: core
`chartsymbols.cpp:551`, private `:561`, `BuildPattern`. CROSSX01 is bitmap-only;
there is no HPGL/vector alternate to recolor or enable. `CreatePatternBufferSpec`
(core `s52plib.cpp:9036`, private `:8942`) retrieves the named image for this
raster and preserves its alpha. Software AP and GL AP both use that buffer
creator (core `:9362` and `:8641`; private `:9268` and `:8581`). Pattern/color-table
cache mechanics are unchanged. Vector dispatch remains untouched, including
its geometry and HPGL color handling.

The exact source node has a canonical hash guard. An exhaustive scan of all
symbol/pattern/line-style bitmap rectangles establishes **exclusive ownership**
of `(400,1040)-(416,1056)` by CROSSX01. Any alternate/shared owner or node
change is rejected. The whole XML validator reverses only AXNHAT→ACHBRN and the
three added palette rows alongside the already-reviewed exceptions, then
requires complete source-tree equality.

## Theme behavior and contrast

XNHAT follows Day `--mark-black`, Dusk `--float-text`, Night `--chart-text`.
Night retains the existing brighter navigation-ink level, not a second chart
dimming pass. Crucially, the pinned Dusk and Night CROSSX01 tiles are already
**entirely transparent**. Their whole PNG bytes remain unchanged: this patch
does not invent nighttime coverage or conceal an alpha change. Their existing
conditional boundary still renders. Thus only Day has a visible ink change.

Measured Day contrast uses the actual opaque stock raster RGB rather than an
assumed XML color. `contrast.json` contains all six owned land/depth roles.
Selected ratios, stock → neutral:

| Background | Stock | Neutral |
| --- | ---: | ---: |
| Land | 2.574 | 5.353 |
| Deep water | 2.317 | 4.818 |
| Very shallow water | 1.231 | 2.560 |

Contrast improves against every measured owned surface and meets the existing
3:1 land / 4:1 deep-water / 2:1 very-shallow guards. No nighttime visibility
improvement is claimed for a transparent tile. Actual native/boat readability
remains an acceptance gate.

## Shared core/private generation and qualification

There is one common resource generator. `build-pristine-windows.ps1` invokes it
before private adapter preparation; `prepare-ocharts-adapter.py` verifies and
copies the same five resource files and generated manifest/header. Host configure
regenerates them, and `verify-ocharts-adapter-package.py` requires exact resource
manifest/header/file equality. Both core and private compiled resource digests
therefore cover this same change; an old adapter package cannot silently serve
the new resources. No divergent private generator or renderer patch is added.
The new helper enters core CMake configure dependencies and the focused native
input inventory. This receipt does not claim a rebuilt private DLL or native
runtime acceptance.

Focused results only:

- **104** XML/raster/ownership/negative checks passed. Exact node drift, overlap,
  geometry, spacing, alpha preservation, neighboring pattern, global CHBRN,
  conditional replacement and whole-atlas inverse boundaries are checked.
- **15** additional contrast/shared-private-copy checks passed, including the
  actual private preparer's file verifier rejecting a changed atlas.
- **7** native input declaration/guard tests passed.
- An independent Pillow comparison against the installed frozen **9632421**
  resources confirms exactly 192 changed Day pixels and every other RGBA byte
  unchanged. Dusk/Night PNG and RLE bytes are exactly identical. Reversing only
  XNHAT and the one pattern reference restores the entire prior XML.
- Existing full-suite golden/inverse checks are updated with a narrow restoration
  that validates every hatch pixel before undoing just owned RGB; no broad atlas
  exclusion is added. The full 66,093-check suite was not rerun here.

The first focused consumer assertion deliberately failed because its initial
oracle omitted lowercase Inland ENC and Paper records. Source inspection
corrected the oracle to the complete 12 records; the failure log is retained.
No production rule was changed to make the assertion pass.

The private generated manifest and exact test logs are retained here. The frozen
9632421 builder/install, original charts/prototype/screenshots and global pattern
geometry remain untouched. No new application build/capture, full CI, boat action
or custom chart-data modification occurred. Batch native and actual ENC
before/after visual checks remain required.
