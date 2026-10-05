# SCRUM-284: ordinary all-round outline

Isolated implementation based on `76aec54058bec5e8bc6805a90029e8fb199a12e1`.
This extends the immutable prototype's finite-sector arc paint to a full circle;
it is **not an exact supplied all-round glyph**. No application, dependency
producer, native Windows workflow or boat action was run.

The unchanged prototype `docs/design/prototype/index.html` supplies
`.sector-arc` width 1.2, opacity .8 and `--sector-stroke`; `ChartCaFan.h` already
maps the per-theme white/red/green arc roles. The new outline reuses those
values, with no fill or rays. The source boundary is explained in
`docs/design/reviews/scrum264-all-round-light-boundary.md`.

## Narrow boundary

`ChartCaAllRound.h` accepts only the exact Simplified LIGHTS lookup 31183 and
ordinary single white/red/green objects with known positive nominal range.
Both sectors must be absent (range at least 10), exactly closed, or an exact
360-degree sweep. Special, directional, obscured, uncertain, malformed,
unknown-range, other-color and partial-sector objects retain stock paint.
Paper, Standard and Legacy remain stock. The second guard requires the exact
upstream OUTLW/4, corresponding light color/2, 0–360 CA signature and pinned
`_selSYcol` range-band radius, with no sector legs.

The range bands validate the original conditional output; they do not replace
the renderer's radius. Each renderer supplies its own already-scaled center
and radius. Original conditional sources, visibility, CA string/cache,
full-circle distinction and bounding-box convention remain unchanged. No
geographic nominal-range circle or physical tower is invented.

The two CARC methods are the only changed renderer bodies. Finite sectors,
cables, fishing patterns and the CARC dispatch wrapper remain unchanged.
`DrawCaFanGL`, including its optional matrix arguments used by cables, is
byte-identical to the base. Tile allocation checks reject null image/alpha
data before composition; refusal leaves no tile or bounds and uses stock paint.

## Focused results

- **339 assertions per renderer**, core and private, in two actual-method
  fixtures. Earlier 327-assertion outputs are retained separately in
  `proof2-run.log`; these numbers are assertions, not separate test cases.
- Actual extracted LIGHTS06, `_selSYcol`, CARC software/GL methods, matrix
  setup and shaders execute through native wx and Mesa llvmpipe. Controlled
  object construction, coordinate projection and stock-color lookup remain
  fixture boundaries. They do not constitute application canvas proof.
- Day/Dusk/Night × white/red/green × software/GL yields **36 images**.
  `comparison.png` shows their unscaled center crops on a fixed controlled
  background, not real chart/theme backgrounds. Software/GL pixels agree to
  one RGB level. Centers remain empty; opacity/width, absent rays, hostile GL
  state restoration and stock fallback are checked.
- Original radius/center/bounds are compared separately for each renderer.
  At the three retained scenarios, software radius is 51/18/51 for both;
  core GL is 51/25.5/27 and private GL is 51/20.4/51. Those existing private
  differences are preserved, not normalized. Rotation centers also match.
- Range thresholds and all three colors use actual conditional output.
  Disabled, special, uncertain, unknown and paint-refusal cases compare
  exactly with original methods. Finite-sector tile pixels/bounds match the
  base class; cable output before/after all-round painting also matches.
- The two complete affected `s52plib.cpp` translation units compile at `-O3`
  with real prepared headers and production macro selection, sequentially.
  `objects/commands.json` records exact commands; no application/DLL link.
- Nine core patches and three private patches compose exactly. Source proof
  retains the unchanged conditional/private callback headers and unaffected
  method hashes. `private-owned-inputs.json` verifies the final owned-header
  copy after the allocation guard, including the new header in LOCAL/INPUTS.

## Controls and limits

`first-range-string-assumption.log` preserves an initial fixture failure: the
expected CA text incorrectly included a closing parenthesis; actual pinned
LIGHTS06 emits the instruction terminated by a unit separator. Only that
fixture expectation was corrected. It was not a production geometry failure.

`original-negative/` retains the original core-method substitution's exit 1
on a software/GL raster mismatch. This is a rejected old-method control,
not an independent proof that its first failure specifically identifies the
new outline. No old-method result is relabeled as passing.

The launcher is `tools/test-ca-fan.py --all-round --baseline-header
.local/baseline-ChartCaFan.h`, with `--source .local/core`,
`--private-source .local/private`, the pinned stock sources, sysroot wx prefix
and `--output .local/final`. `receipt.json` contains exact compile commands,
source/method/helper/fixture/executable hashes and original runtime output.
`source-proof.py` and `compile-objects.py` retain local reproduction commands.
All temporary preparation and object outputs belong to this isolated tree.

Native MSVC/Win32, private DLL runtime, real IHO chart canvas, theme return,
recognition/contrast on chart backgrounds and boat acceptance remain open.
The supplied artwork still has no exact full-circle design. Unknown and
nonordinary full-circle variants intentionally retain stock portrayal. This
proof does not qualify the concurrently frozen Windows candidate or establish
whole-screen conformance.
