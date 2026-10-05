# SCRUM-268: ordinary GL text cache metric consistency

Base: `91a53301f03f99fc543de7b21c48f8c9d7931fb2`. This isolated change does not
modify the qualifying application or the retained 326 Linux executable.

The original read-only runtime trace identifies the official IHO cell's LNDELV
RCID 28 (runtime object Index 27), lookup 31187, at latitude −32.3740304,
longitude 61.0376363. ELEVAT 8 m formats as `26.2` ft. At unchanged anchor
(957,129), the first atlas miss measures M=16 and replaces the text's X=9
metric, drawing rectangle (973,108,38,22). After palette changes recreate text,
the same font/atlas is reused but the text retains X=9, drawing
(966,108,38,22). Label extent stays 38×22. `proof.json` binds the original
trace hash and first returned metrics in each phase; original screenshots and
failed whole-chart comparison remain in `scrum268-326-elevation-metrics`.

Both core and private S52 `RenderText` have the same asymmetric atlas branch.
The two patch hunks move its existing M measurement and M×DIP assignment outside
the cache-miss block. Atlas construction, eviction, font identity, text, color,
anchor, offsets, scale arithmetic, rotation, decluttering and painter calls are
otherwise byte-identical. The specialized whole-label paths and software path
are unchanged. `proof.json` verifies the complete S52 source difference is only
this move plus its explanation.

The correction intentionally applies to ordinary glyph-atlas text in all modes,
including Standard and Legacy. It preserves the original cache-miss placement;
labels which previously inherited X because another label already built the
atlas now use the same M metric. It adds one cached, single-character extent
query per atlas text call, with no new font, atlas or persistent allocation.

## Focused verification

- The existing harness executes each actual patched `RenderText` body. Each
  passes 140 checks across fresh/recreated text, persistent atlas, another font
  identity, three DIP values and presentation/stock color-policy settings.
  The retained runtime metrics are explicit deterministic glyph-provider inputs;
  cache ownership/selection and rectangle calculation execute production code.
- The same fixture using each **unaltered pre-fix actual method** exits 1 on the
  recreated-text rectangle assertion. Both expected failures and their receipts
  are retained, rather than substituting a copied positioning algorithm.
- Both existing full method fixtures also pass: 6,122,231 assertions each,
  predominantly per-pixel native raster/upload comparisons. These cover the
  unchanged geographic/LIGHTS whole-label paths, scale, overlap and registration.
- All nine core and two private patches compose from pinned source. The final
  prepared S52 bytes equal the tested source. Both changed complete S52 units
  compile at `-O3 -Werror=dangling-pointer`; exact commands and object hashes are
  retained. No application link or broad test suite was run.

Reproduce focused cache checks using `tools/verify-chart-name-boundary.py` with
the prepared `--source`, `--output`, `--wx-config`, `--wx-prefix` and
`--glyph-cache-only`; add `--private` for the private source. The negative control
adds `--original-source` pointing at the retained pre-fix S52 source. Run native
wx fixtures under an isolated X11 display. Omitting `--glyph-cache-only` retains
the existing whole-label checks.

The harness records uploads and uses a glyph-provider dependency fixture; it
does not run an actual OpenGL driver or demonstrate the fixed chart pixels.
Actual corrected canvas/Day-return, Windows, private DLL runtime and boat
qualification remain open. The original strict image comparison is unchanged.
