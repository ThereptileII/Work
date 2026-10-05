# SCRUM-231 — bounded built-up-area paint correction

**Current policy:** XNBUA uses prototype `--land`, including the existing Night
brightness normalization. The original shore-green policy described below is
superseded by the user's exact-prototype requirement. See the
[land-fill correction](scrum231-land-fill-correction.md) for focused proof and
the intentional changed fill-distinction assertion.

The retained FFE native chart uses verified XNav resources, not Standard. Its
large mustard regions match stock CHBRN (177,145,57), while ordinary land and
water already match the prototype. The public ENC archive is hash-pinned; a
task-local read-only GDAL inspection confirms Seattle and West Seattle are
BUAARE Area objects in the retained viewport. Object identities and cell/archive
hashes are in [the inspection record](../../evidence/scrum-231-built-area-object.json).
No chart geometry is distributed in that record.

Use dedicated XNBUA = prototype `--shore` for the two pinned BUAARE area lookup
fills only. The neutral stays distinct from ordinary land and all depth areas
in every supported theme. Preserve LANDF boundaries, labels, classification,
priorities and point symbols. Preserve CHBRN globally: pinned s52cnsy.cpp lines 1943
and 3502 use it for above-water obstruction/wreck states;1257/1560 use it for
obscured light sectors. Standard/Legacy/Safe resources are not altered.

The native XML loader accepts named five-character colors
(chartsymbols.cpp lines 85–127 and 815–819), so this needs no upstream rendering hook.
The generator's semantic comparison permits exactly two AC token replacements
and the dedicated theme colors. Focused negative cases reject CHBRN changes,
hazard/point-symbol edits, altered BUAARE labels/classification, and missing or
duplicate colors. Existing provenance, depth distinctions, sprite masks,
determinism, CRLF and original-source-preservation checks remain required.

Local before/after ENC comparison and native Windows/physical display gates
are recorded separately; this source change alone is not chart acceptance.

## Focused evidence

Commit 785ffcd passed 3,477 resource checks. The original coherent cf07197
binary and the private 28-step incremental 785ffcd build each captured the
same US5SEAFL viewport at 1280×800, Day/Dusk/Night, XNav and Standard
(12 captures total). All four app instances closed cleanly. The current private
app/upstream were verified against the exact commit and all nine upstream
patches before and after compilation. No full suite or endurance run occurred.

Across all three XNav themes, over 85,000 solid built-area pixels change from
stock CHBRN to the exact dedicated prototype neutral; over 1,500 stock CHBRN
pixels remain for other structures. The geometry, chart identity, center and
scale are unchanged. Every Standard chart-region pixel is identical before
and after in all three themes. Actual after images were visually inspected:
large mustard blocks are neutral, outlines/labels/small stock structures remain.

A subsequent independent review corrected my initial visual observation:
the retained after Day image does contain the Compass/N/North up and Layers
artwork. Its committed bytes, local capture and capture-time report all have
SHA256 `31742861559ff35cbbf69e67cba4c563b3371918e769ef3160ab19d580fb825e`.
No distinct blank-face image is retained and no file rewrite is established;
the earlier blank-face report was a visual-review error, not a confirmed defect.
This correction required no application rerun. Actual Windows, OpenGL and boat
review remain required. Broader marker artwork,
ownship and label-density mismatches remain outside this increment.

[Exact identities, pixel comparison and image hashes](../../evidence/scrum-231-built-area-local.json)
retain the source/binary distinction. The six XNav images are retained beside
that record; full disposable profiles remain local only.

## Resource guard hardening

The post-review guard now requires exact `name/r/g/b` attributes and the
intended RGB for each allowed palette role, including XNBUA. Extra alpha or
other attributes and nested color content are rejected before normalization.
The previous guard accepted five of six injected RGB/alpha/extra-attribute
mutations; the strengthened guard rejects all six, plus nested XNBUA content.
The focused resource suite passes 3,484 checks.

All seven generated files are byte-for-byte identical to the resources built
and captured from 785ffcd, including the manifest and compiled hash header.
Their hashes are retained in the local evidence record. No product rebuild or
capture rerun was needed; existing visual evidence and remaining gates are
unchanged.
