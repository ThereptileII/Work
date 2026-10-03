# SCRUM-261 exact Day neutral marker ink

Day CHBLK/CHGRD now use the final immutable prototype's `--mark-black`
`#53645f`, rather than retaining `#070707`. The earlier rejected candidate was
`--chart-text` (`#687b7a`), which is a different role. Independent measured
contrast for the accepted marker token is 4.817878 on deep water, 2.559707 on
very-shallow water and 5.352773 on land; existing 4/2/3 thresholds remain intact
and now apply to all three themes. Safety contour and sounding palette roles,
Dusk/Night and Standard/Legacy behavior are unchanged.

The original Day atlas contains 42,627 visible RGB(7,7,7) pixels and 2,070
RGB(125,137,140) pixels. A blind whole-atlas replacement is unsafe: those RGBs
are shared by other palette roles. `chart_day_neutral_ink.py` therefore verifies
the exact PNG/XML hashes and derives an ownership mask from every referenced
symbol/pattern/line bitmap rectangle and its color-ref. A pixel is eligible
only if **every** referencing rectangle maps its exact source RGB solely to
CHBLK/CHGRD. Unknown, unreferenced and shared other-role pixels remain intact.
All SOUND* bitmap rectangles are additionally excluded: some themselves name
CHGRD, and changing sounding paint is outside this task.

The final mask changes **40,482** pixels: 39,511 black and 971 gray. It preserves
4,215 same-RGB pixels (3,089 other/ambiguous-role, 1,099 sounding and 27
unreferenced), all alpha bytes and all chromatic/off-palette/invisible bytes.
No antialias tolerance, shape, coordinate, size, classification, label,
conditional procedure or symbol selection changes. Existing separately owned
prototype tiles are painted after this source-only derivation.

Provenance and independent golden:

- Original XML SHA-256: `84f93522576ed5872b24865cf6161872ed0b4b634f77c64ca697a04d3568e886`.
- Original Day PNG SHA-256: `ee1020a8b94b312faba8c146974e49147d280e5ef3cc13f3cd688bbfd54dde0c`.
- Original decoded RGBA SHA-256: `9ec0625741a7d7780291232653aefd0dea929e76d670ab6ee821a406c2e63990`.
- Changed decoded RGBA SHA-256: `3d7f1b8210a701bada753367dbf9aa8a71a3dabe553b5f57a270c81446cf471f`.
- Sorted little-endian uint32 pixel-index mask SHA-256: `5e4ecc6dbc228ed8b5874ba4094922c03b4a12b68220fc4553266460b228ce73`.

The full per-bitmap ownership/count audit is retained in
`docs/evidence/scrum261-day-neutral/independent-audit.json.gz`. It was derived
with Pillow 12.3.0 and an independent XML/pixel enumeration before the production
helper, which uses the existing standard-library decoder. The immutable HTML
hash is `b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.

The focused resource suite checks this independent decoded golden/mask, full
alpha and reverse equality, every SOUND bitmap byte, unchanged complete
Dusk/Night PNG hashes and changed-input/wrong-token refusals. Existing XML
reverse-equality and semantic negative controls remain. Anchor/service/cardinal
atlas tests account for the independently verified neutral baseline before
checking their own isolated tiles. CMake and native compile-preflight inventories
include the new generator dependency.

Limits are deliberate: WRECKS04/05 bitmaps bake RGB(0,0,0), not the palette's
RGB(7,7,7); WRECKS01 bakes RGB(109,119,122), not RGB(125,137,140). Those off-palette
raster hazard pixels remain byte-identical. Vector CHBLK/CHGRD uses receive the
new Day role, but this change does not claim all raster wreck artwork matches
the prototype. Dangerous, non-dangerous, exposed and known-depth wreck
classifications remain upstream-owned; no generic WRECKS05 substitution occurs.
Native and actual public-ENC software/GL small-scale readability, private
renderer and boat review remain required. Numerical/golden checks do not grant
navigation acceptance.

Focused validation: `python3 tests/chart_presentation_resources_tests.py` passed
20,675 checks, including the Windows CRLF regeneration proof;
`python3 tests/windows_chart_units_tests.py` passed all 7 dependency/guard tests.
The two new Python modules passed syntax compilation and `git diff --check`
passed. Logs are retained beside the independent audit. No application build,
CI dispatch or boat operation was performed for this increment.
