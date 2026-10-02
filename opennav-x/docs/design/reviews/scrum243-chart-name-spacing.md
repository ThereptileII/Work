# SCRUM-243 — geographic name spacing and opacity

The unchanged HTML declares `.chart-label` letter-spacing 1px and
`.chart-water-label` letter-spacing 5px, opacity .36. Chromium independently
measured a 15px increase for three letters at 5px spacing, including the final
advance. The earlier font/ink correction did not implement these properties.

The existing SCRUM-238 resolver now supplies these properties only for its
bounded geographic OBJNAM classes. S-52 still owns real chart strings, object
positions, visibility, offsets, justification and collision decisions. Width
and collision rectangles include the actual added spacing. Other chart text,
soundings, Standard, Legacy and Safe retain their default paint paths.

The software renderer uses native font advances; water-name opacity uses its
native alpha-capable graphics context. The GL renderer uses OpenCPN's existing
cached whole-label texture path for these names, with the same glyph placement
and alpha. Each styled texture invalidates on scale/content-scale/ink changes
and deletes its old texture before replacement. Explicit user text colors are
retained. No new navigation model or chart visibility filtering is introduced.

Tracking is bounded to at most 512 precomposed Latin characters and 4096 added
pixels. Complex scripts and combining sequences retain native whole-string
shaping. A first painter inspection exposed an incorrect gap after a decomposed
accent; the corrected fallback preserves that name without guessed spacing.
The renderer never rewrites or truncates the chart name. This shaping fallback
is an explicit correctness limitation, not an exact-spacing claim for all text.

[Recorded checks](../../evidence/scrum243-chart-name-spacing/review.json):
46 policy checks, 5,776 native painter/pixel checks, four actual production
object compilations across software/GL, and exact nine-patch verification.
The [corrected painter image](../../evidence/scrum243-chart-name-spacing/names-day-dusk-night.png)
was inspected. These are Linux development results. The integrated real-ENC,
actual GL, native Windows fonts/DPI and boat comparisons remain mandatory.
Normal Linux/Windows integration scripts now run the short offline geographic
name and onboard-AIS painter fixtures; neither touches a profile or hardware.
