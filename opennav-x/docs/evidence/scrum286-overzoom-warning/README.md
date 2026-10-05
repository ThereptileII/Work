# SCRUM-286: compact overzoom warning

Isolated base: `fc6348a6df8374878802b3e051a3670d83ed9a56`.
The real translated **OverZoom** warning uses the existing SKAGER warning-callout
heading roles. The prototype has no literal overzoom component: this one-line
callout is an explicit necessary presentation extension, not a supplied asset.

## Source and paint boundary

`ChartCanvas::EmbossOverzoomIndicator`, `SetOverzoomFont` and
`CreateOZEmbossMapData` remain byte-identical. Both existing SW/GL callers obtain
the indicator once, offer its current non-null map coordinates to the owned
helper, and otherwise pass the same pointer to stock `DrawEmboss`. The exact
3.9 threshold, quilt/MBTiles dynamic zoom factor, single-chart presence,
primary-toolbar offset and null-map behavior remain upstream-owned. No warning
is newly dismissed, acknowledged, hidden or reclassified. Standard/Legacy/Safe
retain the stock path through the existing presentation mode guards.

`DrawChartOverzoomWarning` follows the existing chart-depth-label boundary,
using final warning-callout typography instead of its low-emphasis disclaimer
font. The immutable `docs/design/prototype/index.html` and unchanged owned
`XNavPainter::Callout` supply 12px/550 UI font, 13×15 padding, theme amber,
#ecc48c09 backing and #ecc48c30 edge over the theme panel background. Existing
rounded-8 and left-2 geometry is reused through ocpnDC. Computed text/backing
contrast is 9.15 Day, 6.55 Dusk and 6.00 Night; this is token contrast, not native
or boat readability acceptance. See `prototype-roles.json`.

The complete translation is measured with native wxClientDC: ocpnDC's own
measurement clamps width to 500 and could falsely accept a long warning.
Client fit and existing software clip bounding-box fit are checked before
painting. No elision, wrapping or chart-model mutation occurs. Invalid font,
empty/oversized text, negative origin, disabled presentation or clipped fit
refuses custom paint. Pen, brush, font and text ink are restored with RAII;
all tested refusals leave pixels and sampled GL state unchanged. Font/GPU/heap
allocation failure was not forcibly injected. Paint uses the existing native
primitives, without a new texture/theme cache or rounding subsystem.

## Focused proof

- `tools/test-overzoom-warning.py` executed **260 assertions in one fixture**:
  original threshold/quilt/MBTiles behavior, toolbar/null-map cases,
  same-pointer fallback, mode/translation/empty/fit/clip refusals, state and
  visible paint. `run.log` is the original final output.
- **Eight images**, software and Mesa Day/Dusk/Night/Day-return. Both Day
  returns are byte-exact. `comparison.png` is an unscaled crop sheet. These
  are controlled method fixtures, not real IHO application captures. Slight
  GL rounded-edge differences belong to the unchanged upstream primitive.
- The original indicator, actual UiFontWeight, all used ocpnDC methods,
  GLShaderProgram and shader sources are extracted unchanged. Their canvas,
  chart database and MBTiles hosts are controlled fixtures. An offscreen FBO
  supplies deterministic framebuffer capture. Disabled atlas/thick-line
  branches refuse if reached; those branches are not qualified by this proof.
  `dc-effective-equality.json` confirms all 14 extracted DC methods equal the
  effective fc6348a prepared source. Exact inputs/commands are in `receipt.json`.
- Three complete production units—ChartPresentation, chcanv and glChartCanvas—
  compiled sequentially with actual **-O3/-Werror** flags and read-only fc6348a
  dependency/header inputs, to isolated object outputs. No app link/build.
  `objects/commands.json` retains commands, source/object hashes and exits.
- Both modified canvas files reproduce through all **nine ordered patches**;
  only the two caller blocks differ from the prior prepared source.
  `source-proof.json` records this result. Macro-off branches retain the exact
  original call. Python syntax and changed-file whitespace checks passed.

`fixture-setup-failures/` preserves development setup failures: missing GL and
linmath include paths; omitted original desktop shader preamble; window-buffer
capture returning black (replaced by a complete checked FBO); and an ambiguous
wxString test ternary when adding empty-label coverage. None is relabeled as a
production painter failure or a passing test. The earlier successful 188-check
fixture preceded the extra empty-label/refusal-GL-state coverage.

Final launcher:

```sh
python tools/test-overzoom-warning.py --source .local/source \
  --upstream /home/standard/Projects/X-nav-worktrees/skager-product-fidelity/upstream/OpenCPN \
  --wx-prefix /home/standard/Projects/X-nav/.local/sysroot/usr --output .local/final2
```

Actual IHO application before/after and strict theme returns remain a separate
warm integrated gate. Native Windows fonts/DPI, release acceptance and boat
readability remain open. No full CI, application launch or boat action was
performed here. This evidence does not qualify the concurrently frozen package.
