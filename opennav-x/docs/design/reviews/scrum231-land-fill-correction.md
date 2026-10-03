# SCRUM-231: use the prototype land fill for built-up areas

The user's exact-prototype requirement supersedes the earlier shore-green
BUAARE policy. In immutable `index.html`, `.chart-land` uses `--land`; shore
color is its one-pixel outline and faint land texture (lines 186–190). The
separate `#e2dec5` override belongs only to illustrative raster mode. The prior
US5SEAFL inspection identifies Seattle/West Seattle as BUAARE polygons; filling
these large land areas with shore green caused the retained visual mismatch.

Only the three `XNBUA` mappings change from `--shore` to `--land`:

| Theme | Prior effective fill | New effective fill |
| --- | --- | --- |
| Day | #afbfae | #eeeee2 |
| Dusk | #748779 | #4e615d |
| Night | #37443a | #1d2925 |

The existing Night `.78` normalization is retained once. XNBUA intentionally
matches LANDA now, and remains different from deep/medium/shallow/very-shallow/
intertidal water. This revises only the previous *fill-color distinction*;
BUAARE geometry, object identity/classification, LANDF boundaries, names and
render priorities remain intact. No new lookup replacement is introduced:
existing AC(XNBUA) and reviewed geographic-name ink rules are unchanged.
Global CHBRN and obstruction/wreck/obscured-light uses remain pinned. Standard,
Legacy, Safe, chart/display preferences, SCAMIN, decluttering and depth selection
are unchanged. This is not a chart-density adjustment.

The focused proof script at
`docs/evidence/scrum231-land-fill/verify.py` performs one resource generation,
checks literal prototype-derived colors, runs only affected existing palette/
contrast assertions, and tests three rejection cases (CHBRN mutation, incorrect
XNBUA, changed BUAARE label). It restores only the three XNBUA RGB nodes and
requires byte-for-byte equality with the retained 585fd0f/SCRUM-254 generated XML
hash. Every sprite and the RLE must match that baseline exactly. The full
production reverse-equality validator still verifies all other resource semantics.
The existing Day/Dusk whole-table golden check now restores the superseded XNBUA
shade before comparing its old hash; the separate current XNBUA==LANDA assertion
proves the new role. No other palette difference is normalized away.

The focused run passed **90 checks in 33 seconds**. The retained `result.json`
reports output hashes and contrast. Night text contrast remains 3.86:1 on
land/built-up fill, 4.65:1 on deep water and 2.68:1 on very-shallow water, passing
the existing 3/4/2 gates. These numerical checks do not qualify hazard visibility.
The six-minute complete sprite/artwork suite was not repeated for this three-RGB
increment. Root's capture collector reads built-area ink from the generated
manifest, so no stale hard-coded BUAARE capture color required a change. Source
base is `585fd0f`; no upstream source, build, CI or boat action was performed.
Integrated software/GL, native Windows and boat visual acceptance remain open.
