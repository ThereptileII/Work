# SCRUM-264: classified white/orange pillar body

Implementation is based on `5c05eb55c15b67d4014df55896d95452e9cc9d64`.
Jira comments 10728 and 10733 record the narrow selection and legibility decision.
This is a derivative of the supplied prototype, not newly approved prototype art.

Only a verified SKAGER library, Simplified lookup, selected `BOYSPP11`, point
primitive, and exact typed `BOYSHP=4`, `COLOUR="1,11"`, `COLPAT="1"`,
`CATSPM="27"` can select `XNSPPW01`. Attribute lists are the pinned readers'
owned NUL-terminated `OGR_STR`; shape is `OGR_INT`. Unbounded `OGR_INT_LST`
representations are rejected rather than guessed. Missing, duplicate, malformed,
reordered color lists, ORIENT, and direct TOPSHP attributes all retain stock.
Separate TOPMAR composition is unchanged; absence of a direct TOPSHP attribute
is not evidence that no physical or associated topmark exists.

The real NOAA US5SEAFL `.001`-updated Pier 57 points RCID 23/24 have these exact
attributes. Their linked white LIGHTS 39/43 remain separate and unchanged.
The source Paper lookup 1947/RCID 30221 selects BOYPIL81 for shape 4, white/orange,
horizontal bands; its color reference explicitly supplies CHWHT and CHCOR.
The actual Simplified lookup is not rewritten. Its original Rule and metadata
remain intact, with the alias held in the existing library-owned symbol map.
The historical `m_presentationLightSymbols` flag now also guards this verified
owned symbol; the LIGHTS substitution branch itself is unchanged.

The supplied stem, horizontal color segments, crossbar, and base circle are
reused at the established 27/32 scale. No unsupported X, head, or topmark is
added. Tile `(660,1160,24,28)`, pivot `(12,14)`, has 50 painted pixels and alpha
bounds `[9,7,15,24]`, relative `(-3,-7)` to `(+3,+10)` before the real renderer's
scale. This does not claim measured physical stripe dimensions.

## Theme decision and remaining gap

Day uses source CHCOR `(235,125,54)` unchanged. Night uses the owned fixed
`#d07131` `(208,113,49)`: the minimum quantized HLS-lightness step 32982/65535
preserving source CHCOR `(52,28,12)` hue and saturation which reaches 3:1 solid
contrast across DEPDW, DEPMD, DEPMS, DEPVS, DEPIT. Ratios are respectively
5.2577, 4.6599, 3.8117, 3.0342, 3.7149. The immediately preceding RGB
`(207,112,49)` reaches only 2.9977:1 on the limiting fill. This is a documented
navigation legibility exception: the prototype supplies no orange token.
It changes neither global CHCOR nor unrelated symbols, and is not dimmed again.

Dusk's tested `#f7e9e1` lift passed 3.0125:1 but looked cream at native size,
losing the white/orange distinction. It was rejected after independent review.
**Dusk deliberately remains the original stock BOYSPP11 Rule.** Unknown scheme
names also fall back. The unused Dusk alias tile retains original CHCOR; it is
not the rejected cream. Known DAY/DAY_BRIGHT and NIGHT may use the derivative.
This is an explicit Dusk conformance gap, not acceptance of all themes.
Day's stock orange has no new 3:1 claim; the prototype neutral silhouette is
preserved. Solid-color contrast is not a substitute for actual raster review.

## Focused verification

- All nine core patches and both private patches apply to exact pinned sources
  in isolated copies. Source readers and real RenderSY primitive handling were
  inspected for both core and API-17 private paths.
- Actual core s52plib, private s52plib with its production GL flags, and private
  integration-disabled translation units compile. No full application is built.
- Focused resource checks independently reverse only the owned alias, recover
  the whole previous XML and all three complete RGBA atlases, and retain RLE.
  Alias metadata, original HPGL, lookup attributes/instructions, and duplicate
  node mutations are rejected. The new tile owns exactly 50 pixels per theme;
  transparent RGB, unrelated pixels, alpha, all lookup metadata, and global
  CHCOR remain intact. Both core and private packages consume this generator.
- The fixture executes extracted actual pinned loader and RenderSY methods,
  checks actual raster loading/cropping against independently rasterized SVG,
  original Rule identity, Day→Dusk→Night→Day selection, and rejects instance,
  orientation, Paper, and theme-bypass mutations. The unchanged LIGHTS checks
  remain included. Projection and painter recording are fixture substitutes;
  it does not exercise a chart canvas, real GL draw, or hardware cache.
- Existing private preparation checks pass, including the added header closure.

Final checks: **2,235** focused resource assertions, **38,809** actual loader/RenderSY assertions plus **five** mutation controls, **three** production object compiles, and **15** private preparation checks passed. Exact results and hashes are retained beside this file. A missing
sysroot `xvfb-run` on the first attempt and one prematurely started resource
check (generation not yet finished) were tooling failures, not passes; their
logs remain in the private `.local/pillar` directory. One initial new mutation probe used the wrong XML path for HPGL; it was corrected to the actual `vector/HPGL` node and all six mutation probes then passed. The rejected lift sheet
is retained here to explain the fallback decision.

Actual Pier 57 Day→Dusk→Night→Day software/GL captures, native Windows rendering,
and boat display acceptance remain separate gates owned by the parent task.
No application build, CI dispatch, remote change, or boat action occurred here.
