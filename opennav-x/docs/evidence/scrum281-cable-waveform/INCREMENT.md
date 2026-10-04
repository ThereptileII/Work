# SCRUM-281 exact-size waveform increment

The rejected86.7px experiment in README.md remains immutable evidence. This
increment implements the separately approved24px-equivalent resource and two
native paint hooks. It is not native Windows, actual ENC or boat acceptance.

## Source and mapping

Only owned CBLSUB06 line RCID2012 changes. Lookup710/RCID31786 remains exact
`LC(CBLSUB06)`; lookup709/CATCBL6 remains the original mooring-chain dashed line.
Canonical source node and both lookup hashes are checked before transformation.
The immutable chart-marker-art.js hash and literal supplied quadratic path are
checked. Global CHMGD, ferry/cable-area geometry, other lookups and all raster
atlases remain unchanged. The strict generator inverse restores exactly the
reviewed HPGL/box/pivot/origin/color; all other tree content must equal source.

The owned box is635x168, origin(0,-84), pivot(0,0), min/max distance0.
The aspect-preserving transform is x=(u+12)*635/24,y=v*635/24. Its635 units
are6.35mm,24px at nominal96DPI. Both existing modern(width+originX-pivotX)
and legacy(width) repeat formulas now give635. Changing this visual density is
explicitly approved; actual geographic vertices, priorities/masks, traversal,
tangent, clipping, short-segment handling and straight remainders are unchanged.
Real monitor PPMM and existing SCAMIN scaling still apply; this is not a claim
of24physicalpixels at every DPI/SCAMIN setting or of prototype chart zoom parity.

The resource retains a64-segment integer approximation as a fallback. Exact
quadratic painting is selected only for a verified presentation library plus
the exact owned Rule identity, HPGL, color binding and complete physical
metadata. It uses four quadratic segments with1.3 CSS-equivalent round
caps/joins. The source supporting-line opacity.16 is never applied to them.
Day/Dusk/Night XNCBL colors remain156/134/150,184/160/177,113/99/110;
Night includes the existing.78 factor once.

## Drawing and failure boundary

Core/private `draw_lc_poly` each wrap only the existing software and OpenGL
HPGL motif call. No point selection or chart query is added. The helper uses
original r,theta,PPMM and SCAMIN, and one invocation-local CPU tile cache;
no Rule/chart pointer or GL object survives the call. Bounds include the
rotated Bézier control hull, round stroke and antialias padding. Effective
scale outside.25..8, nonfinite coordinates/angles, invalid resources and
failed image/context creation returns to original HPGL. Pixel allocation is
capped at65,536 per coverage image. Existing stock resources/modes never qualify.
Fallback uses the current owned HPGL approximation, not substituted stock
metadata; it retains the original HPGL renderer and traversal behavior.

The OpenGL draw reads the original line program's actual MVMatrix and
TransformMatrix. The existing reviewed texture uploader gains optional matrix
arguments; all CA defaults remain unchanged. It saves/restores every touched
program/uniform/attribute/buffer/texture/sampler/pixel-store/blend/cull value.
Full viewport guard remains strict; a subrect/translated viewport refuses
before mutation. Viewport, stencil/scissor and shared shader source are not
changed. No-OpenGL builds retain original HPGL paths. Integer/native fallback
stroke quality remains distinguishable from the preferred alpha path.

## Focused evidence

`exact-size/identity.json` binds final implementation/tests, generated manifest,
source artwork and retained evidence. The final source composition is recorded
in `composition.json`: all9 core patches reconstruct the prepared source;
all2 private patches independently reconstruct their changed files.

- 152 resource/analytic/inverse/negative checks passed. A later141-check subset
  adds the exact generated-HPGL-to-C++ guard binding; this is overlapping proof,
  not293 unique checks. Maximum sampled fallback error0.651964 HPGL units;
  analytical bound0.965489 units. Renderer uses exact quadratics, not that polyline.
- Actual pinned shared XML loader methods pass using actual headers. Private
  method bodies are byte-identical. Container ownership is a fixture; no
  private DLL load is claimed. The earlier unrelated pugixml-O3 warning is
  retained; this loader fixture uses-O0/Werror, while both complete changed
  renderer objects separately compile-O3.
- Core and private each pass554 actual-method/helper/wx/Mesa checks. Original
  and changed traversal execute verbatim. The private fixture defines only
  SKAGER_OCHARTS_ADAPTER (not OPENNAV_X); positive nonempty stock motif sequences
  become no-fallback owned painting, so an excluded private hook fails. Stock HPGL is a call recorder, so
  fallback argument/sequence equality is proved, not stock HPGL pixel output.
  Cases include reversed/masked/invisible/short/remainder lines, disabled and
  foreign rules, renderer refusal, theme/angle/cache changes and all three
  actual SW/GL tile pairs. Ordinary rotated loaded matrices and hostile GL
  states are checked; SW/GL blend rounding differs by at most1 channel value.
  Old/new CA default upload pixels match exactly. Full/subrect viewport guards
  and no-GL source compilation pass.
- Private owned include copier check passes with ChartCableWave.h in LOCAL/INPUTS.
  Native source inventory already includes all tracked src/ files; CMake's
  existing generator/helper and prototype dependencies cover this change.

The original400x400 PNGs are retained. comparison.png presents source24px SVG
beside actual native-sized fixture crops, then clearly labeled4x enlargements of both original1x rasters.
This is painter evidence, not ENC screenshots or a complete prototype match.

Measured on this Linux/wx/Mesa machine,200 fresh builds across angles0..pi:
scale1 maximum650pixels, mean43us, slowest118us; scale8 maximum28,598pixels,
mean575–603us, slowest1.53ms.20 uploads including final glFinish average39us
at scale1 and399–448us at scale8. Same-angle motifs reuse the CPU tile, but
GL upload still occurs per motif. The hard65,536-pixel cap is larger than the
measured maximum. Disabled short software traversal averages3.3–4.4us versus
original3.0–3.5us; these noisy micro-observations are not boat performance
acceptance. Dense visible cable geometry may multiply upload cost.

Retained failed fixtures include an incorrect initial wx method name/signature,
a missing private test config, desktop GLSL precision setup and a no-GL test
config which accidentally reenabled GL. All were corrected in fixture/setup
or the explicit no-GL hook guard; no pixel tolerance was broadened to pass.

Actual ENC software/OpenGL themes, useful chart scales/DPI and rotated views,
Windows/private DLL runtime, boat appearance/performance and final acceptance
remain pending. No application build, CI, boat action or frozen candidate change.
