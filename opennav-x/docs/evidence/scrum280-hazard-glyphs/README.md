# SCRUM-280: supplied rock and wreck raster artwork

Implementation is isolated on audit `88bd69c` over application `48c2f8b`.
Only the owned SKAGER effective bitmap portrayals UWTROC03, UWTROC04 and
WRECKS05 change. Their original names/RCIDs remain; three previously transparent
24×24 tiles at x852/884/916, y1160 use geographic pivot (12,12), with two-pixel
moats. Marina x820 and proposed fishing x948 are outside this allocation.

The checked-in SVGs contain the immutable supplied paths and final CSS, scaled
27/32 without squeezing the old bitmap bounds. Day/Dusk chart-text colors and
0.8 group opacity are exact; the wreck wave retains its additional 0.55 opacity.
Night uses the existing single 0.78 normalization. Offline librsvg derives locked
coverage (56/88/161 nontransparent pixels); normal resource generation is stdlib
only. `hazards/provenance.json` binds authoring inputs, geometry, alpha and ink.
The raw prototype hash is b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447;
asset provenance normalizes its one CRLF for the existing cross-platform convention.
The immutable prototype itself is unchanged.

All lookup/classification/visibility/priority, conditional procedures, sounding,
uncertainty, stock HPGL/vector geometry and original raster tiles are unchanged.
Standard/Legacy still select their original resource set. These shared names
also affect SKAGER Paper and six `$CSYMB` consumers; this is not Simplified-only.
Missing WATLEV is still unknown: the artwork does not assert an awash state.
The prior audit gives exact source consumers and original definitions.

## Focused proof

- 152 resource assertions, including eight rejection controls; independent SVG,
  alpha/RGB, lookup/metadata and neighboring-symbol checks; all three prior
  SKAGER atlases restored byte-for-byte in decoded RGBA with PNG metadata intact.
- One additional complete prior-SKAGER XML inverse: precisely the three bitmap
  metadata sets restore the prior tree. No other XML fields change.
- Actual unchanged pinned loader methods compiled with warnings as errors and
  wxWidgets: **200 checks each for core and original private conditional bodies**.
  Each run exercises 26 conditional cases in both original and owned symbol
  contexts, with exact output/category equality within that source. The private
  bodies use the core ABI fixture; byte-identical shared private loader bodies
  are verified. This is not a private-DLL build or runtime proof.
- Actual-loader negative: moving UWTROC03 pivot x12→13 fails at check4, exit1.
  Other rejection controls cover HPGL, selector, display category, quality symbol,
  occupied declaration/moat and source SVG tampering.

The real loader selects raster definition R despite unchanged XML definition V,
returns the exact dimensions/pivots and crops the new alpha/color in all themes.
`loader.json`, logs and `generated-and-executed-inputs.json` bind actual sources,
compile command, binaries and generated/prior resources. The private result JSON
was retained after the successful run; subsequent runner edits only automate
that receipt emission. No application build, GL draw, Windows CI or boat action
was performed. Earlier fixture-only compile mistakes (missing cstring, snprintf
macro and XML temporary-reference argument) were corrected without changing
production source, compile warnings or assertions.

## Separate failed private safety qualification — SCRUM-283

The original private `_UDWHAZ03` leaves its associated-area list null; the chart
query is commented out under `FIXME plib`. The original core-parity attempt
failed on case18 and remains verbatim in `private-core-parity-failure.log`.
`private-parity-failure.json` and `private-udwhaz03.cpp.txt` retain inputs and the
exact private source/excerpt. This failure was not reclassified as a pass.

With a point UWTROC, missing VALSOU, WATLEV3, safety contour5m and associated
DEPARE DRVAL1=10m, core emits ISODGR51 and promotes DisplayBase. Original private
emits UWTROC03 and stays Other. The analogous CATWRK2/WATLEV3 wreck emits private
WRECKS05 instead of core ISODGR51. Final per-source preservation checks explicitly
assert both original results. **Private hazard safety acceptance remains open**;
root recorded this existing defect as launch blocker SCRUM-283.

Known-depth `Wk` text also retains its original source-specific alignment:
core `TX('Wk',3,1,2,'15110',2,0,CHBLK,21)`, private
`TX('Wk',2,1,2,'15110',1,0,CHBLK,21)`. No private behavior was rewritten under280.
LOWACC03 is emitted by the original QUAPOS2 path but has no definition in the
pinned XML; that absence is preserved, not replaced with invented artwork.

## Visual review and remaining gates

`three-theme-loader-review.png` shows actual loader crops at 3× nearest-neighbor,
old/new at their geographic anchors, with retained WRECKS01/04, ISODGR51 and
PA/PD/REP. The bottom row is an explicitly synthetic composition of loaded
rock/quality images with their source pivots, **not actual chart rendering**.
Individual original-size loader crops are retained for review.

The supplied thin/partly transparent strokes remove the previous DEPVS-filled
danger shapes. The supplied WRECKS05 hull/mast may resemble retained visible-wreck
WRECKS01. Day thin-stroke contrast and Night visibility require actual recognition
review; unchanged neighboring symbols have their own existing palette limits.
This sheet proves the requested derivation, not navigational recognition or
all-screen conformance. Actual SW/GL canvas and overlay placement, native Windows,
private DLL, display scaling and boat display acceptance remain unpassed.

Reproduce with the existing generator into a disposable directory, then
`verify-anchor-loader.py --hazards --source <pinned-core> --private-source
<pinned-private> --generated <owned-output> --output <proof-directory>
--wx-config <actual-wx-config> --wx-prefix <actual-wx-prefix>`.
The new resource verifier is included in the existing combined chart-resource
entry point and exact inverse helper. This task ran its focused verifier plus
prior-resource inverse only, not the unrelated full suite.
