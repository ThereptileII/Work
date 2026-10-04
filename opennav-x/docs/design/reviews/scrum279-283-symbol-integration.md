# Supplied artwork integration: marina, hazards, cable and fishing area

These increments follow the original prototype, not the prior Beta styling.
They are developed separately from the frozen `4ddf1f3` Windows candidate.
The original HTML and symbol package remain unchanged. This record does not
qualify a Windows binary or a boat screenshot.

## Root visual review

- **Marina / SMCFAC02:** the three-theme reference and actual-loader crops match
  the supplied roof/anchor service glyph at the retained geographic anchor.
  This depicts the service class, not a surveyed physical building. Actual
  chart/background recognition remains open.
- **Rock / UWTROC03 and UWTROC04:** the loaded supplied cross/dotted shapes match
  their source geometry. Thin, partly transparent strokes need chart and boat
  recognition review. Their original conditional selection is preserved.
- **Wreck / WRECKS05:** the supplied hull/mast and wave match the prototype, but
  similarity to the retained visible-wreck WRECKS01 is a recognition concern.
  An exact source copy alone is not navigational acceptance.
- **Fishing area / FSHFAC03:** the source diamond/cross fits the original native
  repeat cell. The reviewed diagram is explanatory; it is not a full polygon
  capture. Original cell size, stagger and polygon anchoring are retained.
  Minimum glyph inset is capped at one nominal 96-DPI pixel, scaled and rounded
  up: one device pixel at 100%, two at 125/150% in the measured cases.
- **Cable / CBLSUB06:** retaining the old HPGL width would make the supplied
  24-pixel waveform about 86.7 pixels wide at 96 DPI. That experiment was
  rejected. SCRUM-281 implements the exact physical motif size and round stroke
  at the existing line-painter boundary. Root reviewed source/wx/Mesa samples;
  combined optimized core/private object and source composition checks pass.
  Actual chart and boat review remain open.

The retained images and source identities are under
`docs/evidence/scrum279-marina/`, `scrum280-hazard-glyphs/` and
`scrum282-fishing-pattern/`. Their enlarged glyph sheets are not substituted
for actual software/OpenGL, native Windows or boat captures.

## Source coverage and integration findings

The read-only [public-chart audit](../../evidence/scrum279282-public-scenes/README.md)
found no exact SMCFAC02 selector or CATFIF1 fishing polygon in the retained NOAA
and IHO charts. It does identify cable and potential rock/wreck examples.
We do not change chart classifications to manufacture coverage or treat a
fishing line as the area-pattern test. A suitable public chart or clearly
identified isolated test fixture is still needed for the absent selections.

Root review found that the initial fishing helper used the core `OPENNAV_X`
guard inside the private renderer. The actual private adapter deliberately uses
`SKAGER_OCHARTS_ADAPTER` instead. Follow-up `f90c654` corrects only that guard and
adds checked RGB/alpha allocation. The actual private translation unit compiled
with its own definitions; separate core/private fixtures pass, and the original
private guard fails the positive-path control. The private target did not gain
the host's macro to hide the mistake. This remains Linux object/fixture proof,
not a native Windows DLL or actual chart-runtime pass.

SCRUM-283 separately tracks an inherited safety difference: original private
`_UDWHAZ03` never calls its available associated-depth-area query. The original
failed core/private comparison is retained. Resource-preservation checks do not
qualify that behavior, and glyph changes cannot fix it. The narrow owned-adapter
callback repair is integrated with focused actual-source checks passing,
without changing chart geometry or the host API. Native/private-chart and boat
acceptance remain open, including the original line/area query limitation.

## Existing user-requested corrections

The frozen candidate already contains the 124-DIP SKAGER wordmark, prototype
font selection, neutral building/area paint and classified buoy/light work.
The new symbols do not replace those changes. Ordinary all-round light circles,
unmapped physical tower variants and the white/orange Dusk buoy fallback remain
explicit visual gaps. The prototype's symbol catalogue includes stock symbols;
it is not evidence of a custom replacement for every one of them.

No global brown-color substitution is introduced. Each removed brown feature
has an identified portrayal; obscured lights and above-water hazard states must
retain distinguishable navigational meaning. Exact screen conformance remains
open until the combined application and boat display have been reviewed.
