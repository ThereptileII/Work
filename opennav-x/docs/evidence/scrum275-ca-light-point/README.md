# SCRUM-275: homogeneous CA light central point, isolated implementation

Base `96c0c27`; this work does not change the frozen 154 candidate, any installed
application, shared prepared source/build, charts, profiles or boat. No full
application build, canvas launch, CI or remote operation was performed.

The immutable prototype's `lighthouseArtwork()` in
`docs/design/prototype/src/chart-marker-art.js` supplies its compact circle and
four rays. This increment reuses existing, verified, library-owned
`XNLIT011/012/013` Rules. No new artwork/resource bytes or chart values are added.
The original CA arc instruction, bearing conversion, range, color, leg, text and
conditional logic remain unchanged. This is additional central point paint,
not replacement of a sector or a representation of a physical lighthouse tower.

## Exact boundary and lifetime

The four active functions are core `s57chart::DoRenderOnGL` and `DCRenderLPB`, and
private `eSENCChart::DoRender2RectOnGL` and `DCRenderLPB`. The private two-rectangle
GL function has one scope per rectangle, making five scopes total. The older
private `DoRenderRectOnGL` is inside `#if 0` and was not changed. Text-only passes
are unchanged. The software private caller's `PI_GetPLIBSymbolStyle()` resolves
to the existing patched host `GetEffectiveSymbolStyle()` in pluginmanager.cpp;
there is no new symbol-table policy here.

Each scope examines all selected point heads before per-object visibility,
SCAMIN or reduced-bbox filtering. It never evaluates conditional rules or calls
visibility as a getter. It copies coordinate/classification facts and retains
only the chosen existing object's identity in the final inventory. Exact x/y
and chart-context identity follow the pinned TOPMAR co-location relation.
Multipoint sounding children are not LIGHTS/platform objects. The original
render traversal, object processing and lookup order are unchanged.

A bounded two-pass hash grouping detects cycles/duplicate objects and then
marks separately charted co-located point objects. Only LIGHTS keys allocate
groups. The last member of a homogeneous eligible group owns the added point,
so earlier sector legs cannot overpaint it. If that member fails the original
visibility check, no other member is promoted. A missing/invalid owned symbol,
malformed inventory, allocation failure or exceeded bound leaves the added
point unavailable; original painting proceeds. Standard/unverified/Paper calls
construct an **empty scope**, not an absent scope; they allocate no entries.
RAII restores the previous inventory on normal/early/exceptional return. No
object pointer persists across a pass/frame. No `Rules` or raster cache is
created on the stack and passed to a retaining painter.

The exact original CA lookup must be Simplified LIGHTS/31183, and actual
attributes must establish an ordinary CA case. The first slice accepts single
white/yellow/red/green color; same-location records must have the same actual
color (white and yellow are not combined merely because they share artwork).
ORIENT presence, any CATLIT/LITVIS/STATUS/QUAPOS/QUASOU, unknown/multicolor color,
malformed/duplicate relevant fields, partial/nonfinite sectors or a short-range
non-sector light decline the added point. A separate co-located structure or
topmark also declines it. This is deliberately conservative presentation scope,
not a declaration that valid mixed-color sectors or those chart features are
invalid. Such light groups remain an explicit visual gap.

`RenderCARC` retains its exact original result. The legacy no-DC/non-GLSL VBO
branch returns 1 despite having an empty GL drawing block; it gets no new point.
On software/GLSL paths, the original arc call precedes the added raster point.
Return values are not treated as pixel proof. The unchanged
`RenderRasterSymbol` performs projection-dependent drawing, scaling, its own
stable Rule caches and normal BBObj expansion. Nothing enlarges bounds ahead
of or bypasses the original visibility checks.

## Focused evidence

- `source-proof.json`: all nine core and both private patches compose; exactly
  five active scopes; core/private `RenderCARC_GLSL`, `RenderCARC_VBO` and
  `RenderRasterSymbol` method bodies remain byte-identical to their pinned source.
- `method-receipt.json`: 74 core and 74 private checks, plus 72 core no-GL checks.
  These execute the actual production classifier, scope methods, CA wrapper,
  point method and original LIGHTS06/accessors. They verify ordering and exact
  original result, unchanged source CA strings, one-owner grouping, distinct
  chart identities, cycles, scope restoration, stock guards, invalid aliases
  and whole-inventory refusal. Terminal painters/projection are recorded stubs;
  text-description generation is an empty fixture callback. They are not canvas
  or GL-context qualification.
- `objects.json` and `compile-commands.json`: actual changed core s52plib.cpp and
  s57chart.cpp, private s52plib.cpp and eSENCChart.cpp compile; private S52 also
  compiles with adapter integration disabled. Dependencies/build flags came
  from retained local builds, used read-only. The private caller's initial ad
  hoc compile needed its existing TinyXML include and public TIXML_USE_STL
  definition; final commands include both actual production requirements.
- Existing qualification closures: seven Windows chart-unit input tests and
  seventeen private-adapter preparation tests passed. Core s57chart is now in
  the explicit native changed-unit list (24 total); the new shared header is
  in the private source-copy/hash closure. No checks were removed.

Reproduce the method test with `tools/test-ca-light-point.py --source <patched
core> --private-source <patched private> --output <owned directory> --wx-config
<wx-config> --wx-prefix <prefix>`. `verify-source.py` and `compile-changed.py`
retain the exact local source/compile recipes; their local dependency paths
must exist before reusing them.

## Workload and actual source coverage

The cap is 32,768 selected objects and 262,144 LIGHTS attribute fields. Hash
operations give expected linear work with bounded allocation, no pairwise
co-location scan. A focused optimized desktop observation (not a benchmark
framework or boat claim) measured 512 all-light records at 0.62–0.85 ms, 512
non-light points at 0.044–0.072 ms, and the maximum 32,768 all-light records at
51.8–52.9 ms, including construction/destruction. The maximum is material;
boat pan responsiveness remains unqualified. A superseded local 4,096-cap
probe was not retained as product scope; the final cap remains 32,768.

`source-point-counts.json` reads the retained exact NOAA US5SEAFL cell/update
and official IHO GB4X0000 test cell: respectively 506 and 757 PRIM=1 features
before native lookup selection, including Unknown and MultiPoint layers.
These are source inventory counts, not runtime razRules observations or a
claim that every supported ENC is that small. All input bytes were rehashed
unchanged. No additional chart was downloaded. The IHO cell is presentation
test geography, not an operational nautical chart.

## Open acceptance

Actual new software/GL canvas views, native Windows, private DLL canvas and
boat review remain open. The isolated objects/method fixtures do not produce a
runnable new application without linking/staging a candidate; no full build or
shared-cache alteration was authorized. Source-backed follow-up cases are
retained in `source-cases.json`: LIGHTS32's 20 nm circle near
(-32.3760351, 61.0307025) has an independently charted FOGSIG33 at its center
and therefore remains stock under this conservative guard. The co-located
LIGHTS992/993/994 and physical LNDMRK1003 group near
(-32.5420331, 60.9234263) also remains stock. The standalone source candidate
is LIGHTS1719 at (-32.3501784, 60.9288803); this is not yet native lookup,
visibility or pixel proof. No full lighthouse-family or interaction acceptance
is claimed. Hover/focus/pin/fan/card work remains outside SCRUM-275.
