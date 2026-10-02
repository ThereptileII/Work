# SCRUM-238 — geographic chart-name typography

The verified SKAGER chart presentation now owns the typography of geographic
`OBJNAM` labels only. Land, built-up-area and land-region names use the immutable
prototype stack (Segoe UI Variable Display, Segoe UI, Arial), normal weight,
12 CSS px / 9 points. Sea-area names use 16 CSS px / 12 points and italic.
Installed fonts are selected; proprietary font files are not bundled.

The optional `s52plib::SetTextFontResolver` callback is installed before first
render only on the verified SKAGER library. A null callback/result retains
upstream behavior. It supplies application-lifetime FontMgr cache entries and
updates average-character metrics before upstream offsets and declutter run.
No OpenCPN route, chart or waypoint pointer leaves its existing owner.

This is an explicit selected-style policy, not an inference that the user's
ChartTexts preference is unmodified. That preference is never written. Standard,
Legacy and Safe retain their original font selection; the existing chart text
scale, content/DIP scaling, visibility, importance and declutter gates remain.
Only TX with leading `OBJNAM,` and BUAARE/LNDARE/LNDRGN/SEAARE qualifies. TE,
soundings, depths, light characteristics, hazards, buoys and restricted-area
labels keep their upstream typography.

A dedicated XNGEO ink follows the exact prototype `--chart-text` value in each
palette. It changes only the OBJNAM ink token in 18 pinned lookup records
(including the upstream's two duplicate LNDARE entries). Lookup body sizes,
offsets, symbol instructions, classifications and conditional symbology remain
byte-equivalent after the enumerated paint substitutions are reversed. CHBLK
and other hazard ink are not repurposed.

## Focused evidence

- 25 boundary checks pass with AddressSanitizer and UndefinedBehaviorSanitizer,
  including fixed-width unterminated S-57 class codes and excluded hazards.
- 6,552 resource checks pass against the pinned source: deterministic generation,
  Windows newline normalization, exact unchanged tree and raster geometry,
  duplicate/missing ink rejection and attempted hazard/attribute changes.
- Both actual changed production translation units (ChartPresentation.cpp and
  s52plib.cpp) compile using existing Linux GL/GLSL flags and warnings-as-errors,
  synthetic input/control flags disabled; commands and object identities are in
  [evidence](../../evidence/scrum238-chart-names/production-objects.json).
- Independent source review found no font lifetime or rule-scope defect.

## Still required

Integrated before/after chart captures, authoritative Windows typography and
96/120/144-DPI checks, actual GL/software rendering, and boat screenshots remain.
The prototype's 1px land/5px water tracking is not yet implemented by this font
policy. S-52 anchors/offsets still follow chart data; illustrative HTML label
positions cannot be copied into real geography. No view receives conformance
PASS here. These limitations remain in SCRUM-15/238, not hidden by tolerances.
