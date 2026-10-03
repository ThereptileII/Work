# SCRUM-243: bounded native Windows alpha diagnosis

Full source `61a0a7838b56ad841bb458af6fc62651464bdafe` (local
`78eccb8b7f21b260ded57d3ba763f884d60c8180`) compiled and installed, then
[failed its first offline name-painter check](https://github.com/ThereptileII/Work/actions/runs/37088759582/job/111105527254):
“Water label must not become opaque or ignore alpha”. This is not evidence of a
specific backend defect: the prior fixture emitted neither failed pixels nor
DPI/renderer details and saved its image after the assertion.

The diagnostic change saves the final image before checking pixels and reports
the first failing theme, region, coordinate, RGB, channel, background, foreground,
opacity and unchanged bound. It also reports the actual renderer, PPI, font and
water-name extents. No threshold, glyph drawing, region, production header or
production call site changes in this diagnostic increment.

`skager-chart-name-painter.yml` runs only on its dedicated branch or explicit
workflow dispatch. The one ten-minute Windows job builds the existing single
fixture with the locked wxWidgets 3.2.8 x86 closure, using existing bounded
fetch/process/runtime-staging helpers. It retains input/command/runtime/executable
identities, stdout/stderr, PNG and failed summary. It does not build OpenCPN,
producers or unrelated UI, use a chart/profile, or touch physical equipment.
A native failure remains a failed job; collecting evidence does not waive it.

Local Linux Cairo replay: 5,776 existing assertions pass at 96 PPI. A private
negative control changes only the test invocation's opacity to opaque; it fails
the same bound and retains its PNG plus the exact first offending pixel.
These checks verify diagnostic retention, not Windows alpha correctness.
The source remains intentionally unchanged until the native image distinguishes
actual blending from fixture-region or DPI contamination.
