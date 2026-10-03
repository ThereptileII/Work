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

## Native diagnosis and bounded correction

The unchanged production replay [37093438232](https://github.com/ThereptileII/Work/actions/runs/37093438232)
confirmed GDI+ at 96 PPI, Arial 12pt/16px, water extent 145x19. The first
Dusk violation was (381,87), RGB (70,108,123): blue delta 34 against the
unchanged limit 33. Decoded PNG analysis found 588 over-bound channels each in
Dusk and Night, all within the actual water glyphs. Day had none. This rules out
adjacent-row or DPI contamination; native smoothing remains the repair boundary.

The correction changes the fresh translucent Windows GDI+ context to
`TextRenderingHintAntiAliasGridFit` before drawing. It leaves other backends,
opaque DC text, fonts, spacing, origins, colors and brush alpha untouched. It
returns failure if the native hint cannot be set, retaining the existing caller
fallback. The SDK call is isolated in `ChartNameAlphaWindows.cpp`; main S52 and
the standalone fixture explicitly compile/link the helper and `gdiplus`.
Any private adapter consuming the updated header must compile the same source
and link `gdiplus`. No global font smoothing setting is changed.

Native repaired replay is required before acceptance. The existing pixel oracle
and its bounds remain unchanged; no full application or boat claim is made.

## Native repair result

[The exact repaired replay passed](https://github.com/ThereptileII/Work/actions/runs/37094223771)
all 2,452 existing native assertions, at the same GDI+/96PPI/Arial geometry.
Independent archive/source/runtime verification and paired original/corrected
PNGs are retained in [the evidence receipt](../../evidence/scrum243-windows-alpha/README.md).
Dusk/Night over-bound channels each fell from 588 to zero; all 1,821 changed
pixels lie inside the existing translucent water-label region. Opaque rows are
byte-identical. The rightmost antialias coverage contracts 2px, with measured
extent/origin unchanged. This is a native painter repair proof, not a new full
application, installer or physical-boat qualification.
