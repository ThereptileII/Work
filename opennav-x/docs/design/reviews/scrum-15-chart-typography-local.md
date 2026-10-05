# Chart supporting typography — SCRUM-15

Selected scope: Jira comment 10494. This isolated increment starts at
`632463c`; it does not include the separate System-flow change `27e2d507`.
The preceding native `29ea06a` review is recorded by documentation commit
`fb7bd52` and uses Windows artifact `11235841232` from run `37026807977`.

The active immutable prototype override at `index.html:60` specifies row
supporting text at 11px and drawer notes at 11px, line-height 1.65 and secondary
color. Native29 instead used 9px row details and 9px muted notes with 15px
leading. The earlier CSS definitions at line 11 are superseded.

This patch changes only `ChartPresentationDrawer.cpp/.h`:

- Row supporting text is now 11px. Existing row positions, heights, switches,
  selected states and data meanings remain unchanged.
- Explanatory notes and feedback use 11px secondary text. Line positions use
  accumulated 18.15px leading, rounded onto the native pixel grid. Wrapping
  measures the same font used to draw the text; the note surface reserves the
  required height instead of imposing the old fixed paragraph slots and line
  caps. The palette action follows the complete note content in the scroll area.
- Reflow occurs on changed presentation state or note-surface size. The existing
  identical-update shortcut remains intact. No shared control role or theme
  was changed.

The unchanged focused `tests/chart_presentation_drawer_test.cpp` passed all
46 checks after compiling its six UI/test translation units with the local
wxGTK runtime and linking the existing vessel library. This includes readback
semantics, rejected commands, dismissal generation, quiet identical updates,
scroll retention/reset, palette invocation and enlarged-scale containment.
No full application build, added mirrored tests, CI, native run, publication or
boat change was performed.

Four actual 1280×800 Linux component images were opened and inspected, retained
with `result.json`, source/file hashes and platform limits in
`docs/evidence/scrum15-chart-typography-linux/`:

- `chart-day-top.png`: larger supporting text remains within the unchanged rows.
- `chart-day-bottom.png`: complete format, safety and source-health explanations
  wrap without overlap; the separate palette action is visible below them.
- `chart-dusk-raster.png`: larger disabled raster explanations fit the fixture
  rows; observed preferences remain distinct from editability.
- `chart-night-unavailable.png`: Managed and Unavailable states are retained.

The actual row reason from an adapter can be longer than the fixture copy.
Rows retain their existing single-line ellipsis behavior and the editable-layer
controls retain their full reason hint; this patch does not redesign those rows.
Notes can require additional scrolling at narrower widths. The existing fixture
checks containment at increased scale but does not capture every compact/DPI
combination or every feedback string. Linux fallback glyphs do not qualify
Windows typography. Exact-source native Windows and boat-display review remain
pending; no full drawer or whole-view acceptance is claimed.
