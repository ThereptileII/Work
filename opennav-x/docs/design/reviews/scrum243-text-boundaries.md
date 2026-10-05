# SCRUM-243 independent text-boundary review

Review of `23a397e` found two defects in the existing S-52 whole-label GL path
which becomes the normal path for styled geographic names:

1. `RenderText` discarded a collision rejection by setting `bdraw=true` at the
   end of that path. `RenderT_All` then registered the invisible name and its
   rectangle could suppress a later navigation label. Styled names now retain
   the actual overlap result. The existing screen-space rectangle rotation used
   by stock glyphs also applies before styled-name overlap checks. Nonstyled
   return behavior remains unchanged.
2. The raster quad draws raw pixels, but cached width, height and character
   height were multiplied by `m_dipfactor`. At Windows 200% scaling this reports
   half the drawn width. `PrepareS52ShaderUniforms` only maps viewport pixel
   coordinates with `2/pix_width` and `-2/pix_height`; it does not compensate for
   this mismatch. Styled names now retain actual raster metrics, match the
   software baseline, and measure the native `X` advance for horizontal S-52
   offset units. Software styled offsets use that same measured-font unit.
   Nonstyled metrics and offsets remain unchanged.

The geographic role allowlist and OBJNAM/TX-only selection remain unchanged;
no strings, explicit colors, visibility rules, soundings or navigation-label
styles are changed. Fonts remain FontMgr-owned and cached text remains
S-52-owned. Per-label scale/content-scale/ink texture invalidation deletes the
previous texture before replacement; the fixture exercises scale invalidation.

`tests/chart_name_render_boundary_test.cpp` executes verbatim methods from the
actual prepared production source: `RenderText`, `CheckTextRectList`, S-52 text
construction/destruction and the `RenderT_All` registration block. Native wx
performs font measurement/rasterization and alpha painting. GL calls record
uploads, and the fixture owns the viewport and text-list containers. It does
not pretend to render through a GL driver or a full chart.

The fixture passes 72 layout combinations: four Windows-like DIP factors,
three rotations, two content scales and three user text scales (6,107,722
individual geometry/cache/color/alpha assertions). They compare software/GL bounds and offsets, reject an
invisible geographic label without suppressing a subsequent navigation label,
retain nonstyled behavior, and check explicit upload color, opacity and texture
release. Independent regressions restoring the unconditional draw result, old DPI
metric scaling, or old tracking formula each fail. Actual S-52 software and GL production objects
compile with their recorded flags; the existing unrelated `strncpy` warning is
retained. The final exact integration patches verify.

Win32 macro review used the pinned wxWidgets 3.2.8 headers archive:
`wx/msw/wrapwin.h` includes `winundef.h` after `windows.h`, and `winundef.h` removes
`DrawText` before wrapping it as an inline function. No confirmed normal include
order failure was found, and no speculative macro workaround was added. This
is source inspection, not a native Windows compile result.

A follow-up probe confirmed another inconsistency at content scale 2: GL
tracking divided by content scale while software did not, and its font request
followed a different scaling/clamping path. The styled GL path now requests the
same actual font as software, accounting for FontMgr's content-scale multiplication,
and uses the same spacing quantity. This includes software's existing behavior
at user text scale <=1. No stock font-scale behavior changes. The final fixture
models the actual FontMgr scaling contract for both content scales and checks
renderer parity, including fractional user scales. The initial failed probe is
retained as `before-content-scale.json`; it is corrected by this commit.
Evidence is under `docs/evidence/scrum243-text-boundaries/`.

Full native chart rendering, GL-driver capture, native Windows fonts/DPI and
boat-display acceptance remain open. No broad suite, CI or boat operation ran.
