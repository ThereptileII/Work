# SCRUM-263 remaining chart font boundary and primary UI audit

Read-only source audit on 2026-10-03. Product source is
`5c05eb55c15b67d4014df55896d95452e9cc9d64`; root was also inspected at
`f2a853e3356725a362ff407668b4958cda50a4d9`. There is no `src/ui` change between
those revisions. SCRUM-263 acceptance and all four material comments were read.
This record changes no font, chart preference, renderer, test or collection tool.
No build, installation, CI dispatch, runtime observer or boat action was performed.

## Existing chart boundaries

The immutable prototype root stack is Segoe UI Variable Display, Segoe UI,
Arial, sans-serif (`docs/design/prototype/index.html:6`). The chart-specific
rules remain distinct: `.chart-label` uses Segoe UI at 12px with 1px tracking;
`.chart-water-label` inherits the root stack at 16px, italic, 5px tracking and
0.36 opacity; final vector `.chart-depth` is 10px, while raster-mode depths
explicitly use Georgia italic. Final `.chart-symbol-label` is Segoe UI at 8px,
0.12px tracking and a 3.5px water-colored stroke (lines 12, 24, 104, 118).
These roles do not authorize one universal font replacement.

`src/integration/ChartPresentation.cpp:59–87` and
`ChartNameTypography.h:14–28` own geographic TX OBJNAM only for BUAARE, LNDARE,
LNDRGN and SEAARE. They select the installed prototype stack, with normal land
and italic water fonts. Generated LIGHTS descriptions use the exact three
pinned LIGHTS06 suffixes in `ChartLightLabel.h:13–25`; LIGHTS OBJNAM and ORIENT
TE remain upstream. The light mapping applies only to the factory-equivalent
font/color (`ChartLightLabel.h:27–34`) and requests Segoe UI, falling back to
Arial. `ChartSoundingFont.h:10–24` requests the prototype root stack at 7.5pt
(10 CSS px), retaining sounding/content scale. Its installation is local to a
verified SKAGER library (`ChartPresentation.cpp:153–175`), not Standard/Legacy.

Ordinary unclassified TX/TE remains configured upstream text: examples include
buoy/beacon and harbor names, hazard annotations, LIGHTS names and directional
annotations. In the actual patched core `libs/s52plib/src/s52plib.cpp:2632–2667`,
`GetOCPNScaledFont_PlugIn("ChartTexts")` supplies the template face/style. S52
still determines the weight, normalized body size and minimum 10pt size.
The actual host wrapper `gui/src/ocpn_plugin_gui.cpp:323–325` calls
`FontMgr::GetFontLegacy`, whose first matching locale entry is returned
(`FontMgr.cpp:183–200`). This is not necessarily the newer default-entry lookup.
Only when an applicable saved entry is absent does `GetFont` create a font from
explicit global defaults or `wxNORMAL_FONT` (`FontMgr.cpp:157–171,203–244`).
There is no source basis to call the Windows factory face Arial or Segoe UI.
The private o-charts adapter also preserves this API17 legacy preference boundary
(`src/plugin-adapters/ocharts/ChartPresentationAdapter.cpp:20–56`).

## What existing evidence proves

- The retained Windows Chromium reference records Segoe UI and Segoe UI Bold
  for its measured component selectors; it is not every SVG/chart glyph or the
  actual boat's font resolution. See `docs/design/reviews/scrum263-header-typeface.md`.
- `tests/ui_font_resolution_test.cpp:26–49` measures production UiFontWeight and
  selected-HDC GetTextFaceW on Windows. The existing integrated drawing group
  runs it (`tools/build-pristine-windows.ps1:337`). It does not measure S52 text.
- The chart-name and light-label painter fixtures explicitly construct Arial
  (`tests/chart_name_text_test.cpp:32,39`, `chart_light_label_test.cpp:36`), so
  even their native pass cannot establish GeographicNameFont/ChartTexts selection.
- Boat inventory found the three stack families installed. Its explicit scope
  excludes actual HDC/chart resolution (`docs/evidence/scrum263-header/boat-font-inventory.json`).
- Linux captures and Linux fontconfig substitution are not Windows qualification.
  A native changed-unit preflight establishes compilation, not actual chart face.

The narrow future measurement can reuse existing isolated Windows capture
identity/profile/screenshot/shutdown collection and one public ENC viewport.
Observe a bounded, deduplicated set of already-created fonts: geographic name,
sounding, generated LIGHTS description and ordinary TX/TE. Use the final software
DC and GL texture-rasterization boundary; do not call FontMgr getters as observers,
since they can create persisted entries. Record role, requested/resolved face,
weight, size, DPI and renderer without label strings or private geometry.
Existing collectors do not currently emit these chart-face facts. Matching-PDB
read-only inspection can establish requested font state; actual GDI substitution
requires observation of the already-selected HDC. Neither screenshots nor a
wxFont face string alone prove the latter. This is a proposal only; no observer
or test was added. Candidate Windows and physical boat acceptance remain open.

## Customer-facing primary XNav UI audit

No explicit obsolete family bypass was found in the inspected primary UI source.
All explicit font construction in `src/ui` funnels through
`Controls.cpp:153–177` (UiFont delegates to UiFontWeight). The latter selects the
exact ordered prototype root stack and numeric weight. Source search covered
wxFont/FontInfo construction, FaceName/SetFaceName, font-family, named legacy
families, native font creation and all SetFont calls in `src` and `resources`;
immutable HTML, Legacy and declared chart/log-specific roles are not replacement
candidates.

The two native-specialized paths also preserve that selection:
`UiTextWidth` obtains its DirectWrite face from UiFontWeight
(`Controls.cpp:191–199`); settings-tab Direct2D painting obtains the face from
the DC already set with UiFontWeight (`Controls.cpp:609,618–619`). Shell/header,
footer, instruments, product pages, drawers, fields, choices and sheets use
UiFont/UiFontWeight. ProductPanel's stored wxFont is a cache value, not a separate
family constructor. Thus there is no demonstrated reachable explicit family
mismatch to edit. This source result does not claim every native default dialog,
font size, weight, fallback glyph or DPI state is visually qualified.
