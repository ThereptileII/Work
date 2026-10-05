# SCRUM-248 — bounded LIGHTS description typography

Base: `e14de49`, pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
This is a presentation mapping for real ENC content, not a claim that the
prototype contains a complete ENC light-characteristic vocabulary.

The immutable prototype's effective `.chart-symbol-label` rule specifies Segoe
UI regular, 8 CSS px, .12px letter spacing, `--chart-text`, water-colored 3.5px
round stroke painted before fill (index.html:104,118). The separate 600-weight
landmark-name rule is not the light-characteristic role. At 96 DPI the native
base font is 6pt. Arial/native substitution is used where Segoe UI is absent.

Only the three exact normal LIGHTS06 literal TX suffixes (`15110`, group23) are
eligible. The instruction scanner respects the pinned shared instruction buffer
semicolon/end-record terminators. ORIENT TE, OBJNAM, other fonts/colors/groups,
other navigation labels and safety symbols stay upstream. Actual `_LITDSN01`
text, units, light descriptions/sectors, chart positions, S-52 offset and
justification values, important-text filtering and visibility are untouched.
The existing overlap algorithm uses the actual new font plus halo bounds; it
is not enabled or disabled by this change.

The selected style owns factory-equivalent ChartTexts face/size, normal style
and weight, no underline/strikeout, and default black color sentinel. Factory
face/size are the same `wxNORMAL_FONT` values used by pinned FontMgr. Automatically
saved factory values remain eligible; identical-to-factory user intent cannot
be inferred. Any nonfactory appearance stays entirely stock. Eligibility is
checked before cached font selection on each render, so runtime preference
changes leave/re-enter the mapping. Preferences are never rewritten. The text
resolver is only installed on the verified SKAGER library; Standard/Legacy/Safe
retain their prior behavior.

`XNGEO` supplies exact theme chart-text ink and `DEPDW` supplies exact theme water
for this label only. Global CHBLK and all symbol/depth colors are unchanged.
A shared label-sized glyph-mask raster expands the native antialiased glyph
edge by a nominal 1.75px circular radius and composites glyph fill over this
halo. Software draws the cached bitmap; GL uploads the same straight-alpha
pixels through the existing cached whole-string texture path. Geometry remains
in the existing renderer. This raster edge estimate is not a byte-identical SVG
vector stroke; fractional positions are rounded by the native wxDC glyph path.
Linux fallback font rendering is not Windows typography acceptance.

The cache retains native font ref-data, exact text, scale and both colors.
Identical paints allocate no label raster and perform no texture upload. Font,
scale or theme changes replace the cached raster/texture. Raster work is bounded
to 256 characters, scale .5–4, measured text at most 2048x128 pixels and a padded
power-of-two image at most 262144 pixels. Unsupported cases retain the whole
upstream label with stock font/ink; no partial halo or dropped string is painted.
These bounds are per cached label, not a new chart-wide bitmap.

## Focused evidence

- `chart_light_label_test` is a version-controlled native component target:
  exact instruction-buffer classes, custom appearance, shared halo/foreground,
  cache reuse, enlargement and bounded rejection. 47 checks pass.
- `tools/verify-chart-name-boundary.py` compiles and executes actual patched
  RenderText/overlap/registration bodies. Added LIGHTS cases compare every GL
  uploaded pixel to the shared SW raster, check placement and expanded bounds,
  overlap rejection and palette cache invalidation. GL calls are recorded, not
  a real graphics-context draw. Existing geographic DPI cases remain included; 6,122,091 assertions pass.
- Production ChartPresentation.cpp and s52plib.cpp compile with GL definitions
  and with software-only definitions. Original build metadata was read only;
  all outputs are private. Full chart patch applies to the pinned source.
- [Day/Dusk/Night fixture](../../evidence/scrum248-light-label-linux/day-dusk-night.png)
  was opened and inspected. Rows show original 8px labels on deep/shallow colors
  and 2x user enlargement. All text is fixture text explicitly outside any chart
  or data feed. Base-size text is visibly very small; no boat-readability pass is
  claimed. Native 8px antialiasing can have no fully opaque foreground pixel,
  while foreground remains distinct from the opaque water halo.
- Optimized local component: first sample raster 3027us, later theme samples
  663/1375us; 10000 same-raster cache hits 323/351/366us total. These are short
  local measurements, not worst-case viewport or target-hardware performance.

Native Windows/MSVC/DPI, actual GL driver draw, complete real-ENC labels in
Day/Dusk/Night and boat-display hazard readability remain open. No CI, full
application build, live chart capture, input configuration or hardware activity
was performed. The 8px size follows the explicit prototype mapping and is a
material readability risk requiring those gates; it must not be described as
unrestricted visual or nautical acceptance.
