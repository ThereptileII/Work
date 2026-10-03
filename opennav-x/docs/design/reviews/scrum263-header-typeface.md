# SCRUM-263 prototype typeface and smaller approved wordmark

The approved SKAGER raster now draws at **124 DIP** wide instead of 148 DIP
(16.216% smaller). Its origin moves from x16 to x28, preserving the center at
x90 in the unchanged 180 × 68 DIP identity slot. Height follows the approved
680:214 aspect ratio and stays vertically centered. No navigation spacing,
header height, divider, logo source/crop/coverage/ink, icon or installer asset
changes. The immutable prototype and its hash remain unchanged.

## Typeface audit before implementation

The entire immutable HTML was inspected, including later style blocks, font
shorthands and inherited rules, rather than treating its first declaration as
sufficient evidence. The root stack is `Segoe UI Variable Display`, `Segoe UI`,
`Arial`, `sans-serif`; no later rule replaces that root stack. Specific rules
still exist for earlier `.chart-label` (Segoe UI), raster `.chart-depth`
(Georgia), chart symbol/sector/reference text (Segoe UI), and monospace logs and
symbol-code annotations. These distinct roles do not authorize a global family,
size or weight override.

The retained authoritative Windows `reference/windows/capture.json` has the
exact immutable HTML hash, 60 states and 7,464 computed component records. All
recorded component families use the inherited root stack. Recorded actual
Chromium platform faces are **Segoe UI / SegoeUI (174 records)** and **Segoe UI /
SegoeUI-Bold (60 records)**. These counts describe the recorded selectors, not
every SVG glyph or every installed font on every Windows system. The detailed
per-selector family/size/weight mapping is in `font-audit.json`. In particular,
metric values are 48px/400, metric labels 11px/400, page titles 30px/400 and vector
depths 10px/400. Older brief typography is not the final reference.

Production `Controls.cpp::UiFontWeight` already selects the first installed
face from this same ordered stack. It retains numeric weights; wxMSW uses the
DIP-scaled negative character height and wxGTK uses CSS px × 72/96 points. There
is no demonstrated UI-family discrepancy to fix, so this implementation does
not change family policy, weight or size. No Microsoft font is copied/bundled.

For charts, `ChartPresentation.cpp::GeographicNameFont` already uses the root
stack for its eligible land/water names. Factory LIGHTS uses explicit Segoe UI
for its final symbol text rule. The sounding resolver retains its own narrow
factory/custom boundary. Other OBJNAM/TE text follows pinned
`s52plib.cpp:2452` through `GetOCPNScaledFont_PlugIn("ChartTexts")`, retaining
`templateFont->GetFaceName()` at line2488. `FontMgr.cpp:203–256` honors persisted
user fonts first, then explicit `g_default_font_facename` or the actual system
font. It does not hard-code Arial. Generic ENC factory font resolution therefore
needs measurement on the candidate Windows/boat runtime before any change;
there is no justified universal replacement here. Baked Wk/mark artwork is not
a font-selection result. No integration file or chart preference is modified.

## Focused evidence and native qualification interface

`ui_font_resolution_test` calls the actual production `UiFontWeight` at
11px/400, 23px/650 and 48px/400, independently enumerates the stack and checks
family/weight. On Windows it additionally reads **GetTextFaceW from the selected
memory-DC HDC**, rejects an unexpected substituted face, and records device DPI.
This is an opt-in component executable linked to `opennav_ui`; build/run it in
the next existing native batch. It installs no fonts and does not launch a boat
application. Windows 100/125/150% integrated geometry, actual chart factory face
and physical boat rendering remain pending.

The existing `build-pristine-windows.ps1` integrated drawing group now invokes
this executable beside `skager_wordmark_test`, in both development and production
integrated builds. The unchanged `Run` helper retains availability and selected
HDC face/DPI output in `windows-native-output.log` and the variant transcript,
and fails the gate for any nonzero exit. Pristine upstream builds are unaffected.
No new workflow or independent build/dispatch is introduced.

The Linux probe confirms all three named Windows faces are absent, so production
requests Arial; local fontconfig resolves Arial to Liberation Sans. The Linux
system font is Adwaita Sans. This explains why Linux factory ENC and UI text can
look different, and is explicitly **not** Windows font qualification.

`skager_wordmark_test` preserves the old 148-DIP checks and adds the production
124-DIP size at 100/125/150% for Day/Dusk/Night: **189,932 checks and 18 actual wx
component drawings passed**. The original absolute ink checks are unchanged at
148 DIP; the new smaller geometry uses the same minimum ink density, scaled by
pixel area. Six SKAGER and three APP glyph columns, alpha/matte rejection,
unchanged geometry, cache/failure behavior and the existing 4.5:1 core-contrast
bound remain required. The smallest Night APP core measures 5.4917:1. The APP
line is visibly small at 100%, as expected from the approved artwork; it remains
separated without clipping in this focused Linux raster. The full before/after
sheet was visually inspected without modifying its pixels.

The changed Shell compiled separately. The new fixture compiled with
`-Wall -Wextra -Werror`; unmodified Controls.cpp used its ordinary warnings
because pre-existing unused trace variables/GTK parameters fail an artificially
added `-Werror`. Those warnings are retained in `font-compile-linux.log`; no
production warning or test gate was weakened. The brand verifier confirms the
approved original, crop, embedded bytes and all nine Windows icon frames remain
unchanged. No full application build, CI dispatch or boat operation occurred.

## Boat installed-family observation

At 2026-10-03 07:05:05 UTC the read-only
`tools/boat/inspect-fonts.ps1` inventory found all three named families installed
on the boat PC: Segoe UI Variable Display, Segoe UI and Arial. The exact script
hash and sanitized result are retained in
`docs/evidence/scrum263-header/boat-font-inventory.json`. No application, profile
or hardware was opened or changed. This establishes family availability only;
it does not replace the actual native HDC or ENC font-selection checks.
