# SCRUM-265: remaining Pier 57 brown hatch

The remaining brown cross-hatch is distinct from the ordinary brown structural
fills corrected in SCRUM-265. The exact public NOAA feature is **SLCONS RCID 90,
LNAM `02261EC35F311923`**. With the retained `.001` update applied, its attributes
are CATSLC 4 **pier (jetty)**, CONDTN 2 **ruined**, and WATLEV 3 **always under
water/submerged**. These meanings were independently checked against the pinned
S-57 attribute dictionaries, not inferred from the screenshot.

`feature.json` records the unchanged input hashes, full polygon, attributes and
UPDN 1. Applying the pinned spherical Mercator projection to that polygon using
the actual 9632421 diagnostic viewport gives screenshot bounds
**x 346.479–517.965, y 66.147–118.910**. The chart begins at y 68, so a small part
of the feature is correctly clipped above the chart. This matches the remaining
hatch at the upper left of the Pier 57 view.

The exact pinned rendering chain is:

- Plain area lookup **181 / RCID 32216**, SLCONS → `CS(SLCONS03)`.
- `libs/s52plib/src/s52cnsy.cpp:2224`: area shoreline constructions receive
  `AP(CROSSX01)` (2252–2253). On the ordinary position-quality branch, CONDTN 1
  or 2 selects `LS(DASH,1,CSTLN)` (2261–2264).
- Pattern **CROSSX01 / RCID 3** is the general cross-hatch, 16 × 16 pixels at
  atlas `(400,1040)`, pivot `(8,8)`. Its color reference is literally `ACHBRN`
  (reference identifier A followed by palette color CHBRN).

The **hatch alone is not a unique code for submerged ruins**: the pinned
conditional adds it to area shoreline constructions generally. The ruin and
submerged meaning here comes from this exact feature's chart attributes.
Retaining its distinct pattern and dashed outline avoids presenting the object
as ordinary solid land or an intact above-water pier. Recoloring or removing
the shared global pattern would affect other classes/conditions and is outside
the bounded ordinary-fill correction.

`resources.json` independently verifies that the stock and installed SKAGER
lookup and CROSSX01 pattern XML are identical. The decoded RGBA bytes of the
16 × 16 tile are also identical for **Day, Dusk and Night**, with separate hashes.
Source, dictionary, generated XML and manifest hashes are retained. The source
rule and actual feature establish the expected rendering; this receipt does not
claim an instrumented per-object command trace.

Actual source is `9632421f701c5ec74d7a1360bc713c0034faf9af`; installed ELF is
`55b5f9f707be6188bf283c4ece5e992caa43352d77c4a7639a195f60d52e27a3` and chart manifest
is `4af8f1245a59d2280fd74b563fd24f98401dfeb4ff8cdabf927352e5df856fc6`.
Original captures and diagnostics remain unchanged in
[the combined receipt](../skager-chart-9632421-linux/README.md), including
[software Day](../skager-chart-9632421-linux/pier57/software/SKAGER-Day.png) and
[OpenGL Day](../skager-chart-9632421-linux/pier57/opengl/SKAGER-Day.png).
`capture-inputs.json` identifies those original bytes explicitly.

This was read-only provenance inspection using the existing GDAL reader and
installed pinned resources. No source/artwork/palette changes, chart mutations,
new capture, test suite, build, CI or boat action occurred. The two small Python
files reproduce the inspection in the retained local environment.
