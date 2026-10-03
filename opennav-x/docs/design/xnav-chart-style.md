# XNav chart presentation — source inspection and palette contract

The native chart remains OpenCPN 5.12.4, pinned to
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. The prototype is fictional geography,
not chart data. Its symbol vendor snapshot is `1bf728e17feaae05fddad3aad5be677e15c1e89c`;
that newer snapshot is preserved as design evidence, **not substituted** for the
application's pinned S-52 resources.

| Visual role | Day | Dusk | Night |
|---|---|---|---|
| Water | #d5e5e5 | #344f59 | #121e24 |
| Land | #eeeee2 | #4e615d | #25342f |
| Shore | #afbfae | #748779 | #46574a |
| Chart text | #687b7a | #adbbb1 | #758579 |
| Contour / depth detail | #adcbce | #567880 | #2a4149 |
| Active route | #267c76 | #b0dfc8 | #91bca2 |

These are raw CSS tokens. The final prototype additionally applies
`brightness(.78)` to its Night chart canvas. SCRUM-253 now normalizes owned
Night surfaces/ordinary contours, geographic names and eligible overlay paint
once at their inputs; water becomes #0e171c and land #1d2925. Already-normalized
roles are unchanged. Safety ink, safety contour, soundings and light descriptions
remain brighter under explicit navigation-readability exceptions. See the
[exact role mapping and retained contrast checks](../evidence/scrum-253-night-canvas/README.md).
Standard, floating controls and raster chart content are not recolored by this
rule. Actual combined renderer and boat acceptance remain open.

Chart depth units are an actual navigation label, not the prototype's fictional
location/depth metadata. The native presentation uses the final `.map-disclaimer`
8px typography, 22px right inset and floating muted ink. It states `Chart depths`
and OpenCPN's resolved Feet/Meters/Fathoms; this is separate from measured depth
at the transducer in the vessel rail. Stock chart-selector space is respected;
[SCRUM-246](reviews/scrum246-chart-selector.md) applies a bounded prototype-token
palette to its vector keys. A broader redesign remains pending. No quilt unit, sounding or user preference
is converted by this label. Mixed/unknown units fall through to stock behavior.
The large emboss stays in Standard, Legacy and Safe Mode. This is an in-progress
presentation correction, not accepted chart conformance.

The chart scale retains OpenCPN's geographic projection, selected distance units
and nice-distance rounding. XNav places its content 25px after Follow Boat,
37px above the chart bottom, with 8px type, 5px arms/gap and 1px floating-muted
lines. A 130px reference span feeds upstream's existing halving calculation;
the painted bar length is the resulting real distance, never a hard-coded
65px with an invented label. Standard keeps its original scale presentation.
The first real-ENC pass exposed a sounding directly behind the small scale,
which could be mistaken for its distance. A 4px neutral floating-surface backing
(3px radius) is a documented navigation-legibility exception to the fictional
HTML map. It changes no sounding or scale value. Corrective software/GL/native
and boat reviews are required; the exception does not imply visual acceptance.

Final HTML marker variables (from the appended stylesheet, not the earlier
`src/style.css` fragment):

| Marker role | Day | Dusk | Night |
|---|---|---|---|
| Red | #b66e6c | #d3948c | #ae7870 |
| Green | #508d78 | #8fbaa2 | #789d84 |
| Yellow | #ac8d4c | #cbb17a | #a89061 |
| Black | #53645f | #c3cec2 | #89988c |
| White | #f6f7ef | #d7decf | #a4afa0 |
| Blue | #648b9d | #8eaebe | #6d8f9e |
| Service | #7c858a | #a8bbb7 | #7e948a |
| Area | #9c8696 | #b8a0b1 | #917f8d |

These are design targets, not an accepted recoloring of navigation marks.
The current bounded ink pass retains pinned chromatic pixels. SCRUM-261 uses
the exact Day marker-black token for CHBLK/CHGRD, with unchanged 4/2/3 contrast
guards and an independently verified 40,482-pixel bitmap ownership mask.
All sounding rectangles and shared/unknown/off-palette raster roles are excluded.
[Exact mask and remaining limits](reviews/scrum261-day-neutral-ink.md).
SCRUM-260 additionally uses the exact area hue for only the FERYRT01 pen and
Plain CBLARE boundary; all geometry, restrictions and other CHMGD uses remain
unchanged. [Exact mapping](../evidence/scrum260-area-ink/README.md).

The final SVG route stroke is 2.6 CSS px with round joins. Prototype AIS paths
use #916477 stroke, 1.6px; selected fill #cb9cb1; vector line 1px dashed 4/4;
labels #835d70 9px. These literal styles and theme overrides must be assessed
together: normal, alarm, stale/lost and online provenance need semantically
distinct states even where the illustrative target set has no example.

### Active-route ink increment (qualification pending)

The native active route now has a bounded prototype-palette hook for its
software, incremental-segment and OpenGL drawing paths. It uses the exact
`--route` values above only with verified XNav presentation. Standard uses the
upstream active pen. This changes local paint state, never a stored route color
or the route model. Upstream selection remains visible using its existing
appearance; selected/inactive routes, waypoints, tracks, MOB, anchor radius and
the complete route-state visual hierarchy remain unfinished. The bounded
default-ownship increment below has separate evidence and open release gates.

SCRUM-237 adds a bounded default foreground: 2.6 logical pixels, round interior
joins and butt route ends through a shared software/GL vector mesh. It applies
only to untouched default active-route presentation; explicit/global custom
width, explicit style/color, selection, highlight, editing and MOB keep upstream
behavior. It changes no stored preference, route geometry or navigation state.
[SCRUM-241](reviews/scrum241-route-underlay.md) adds the bounded 6px/.6-opacity
whole-union understroke, with explicit tiny-leg/workload fallbacks. The 32px
illustrative context and integrated software, actual GL, native Windows and boat
acceptance remain pending; see also
[the foreground review](reviews/scrum237-route-foreground.md).
Do not claim full route conformance from this increment.

The test-only route driver returns upstream-projected screen positions. Tests
sample multiple interior points of both actual route legs, require exact ink
pixels, retain images and also run Standard. Day/Dusk/Night and actual renderer
are checked explicitly. The existing arrival, skip, reversal, editing,
deletion, freshness and immutable-retention scenario remains in the same run.
Physical chart/symbol review and native replacement evidence remain required.

The first Night GL pixel check failed by exactly one RGB level. Inspection of
the pinned `ocpnDC` shader path shows its existing RGB/256 normalization before
8-bit framebuffer quantization. Tests now require exactly
`round(prototype_channel * 255 / 256)` for that path and retain the requested
prototype color separately. Software requires the original exact RGB. No image
similarity tolerance or renderer-wide recoloring is introduced. The negative
capture is retained; waypoint text/icons remain visibly unrefined at Night.

## Pinned source boundaries inspected

- `libs/s52plib/src/chartsymbols.cpp`, `ChartSymbols::LoadConfigFile`: loads
  `chartsymbols.xml` beside the presentation-library path, but a CWD XML takes
  precedence. A dedicated XNav path must not accidentally permit that override.
- The same loader parses color tables, lookups, line styles, patterns and
  symbols. `SetColorTableIndex` / `LoadRasterFileForColorTable` manage palette
  sprites and GL texture state. Switching resources cannot ignore these caches.
- `libs/s52plib/src/s52plib.cpp`: presentation initialization and scheme changes;
  object-class CSV resources also use the shared S57 directory.
- `gui/src/ocpn_frame.cpp`: presentation-library construction. The user's
  `g_UserPresLibData` selects legacy handling; it must not be repurposed as the
  XNav style switch or silently overwrite the user's stock preference.

## Implementation boundary

Derive separately packaged XNav presentation resources from the pinned stock
resources. Preserve lookups, conditional symbology, classifications and symbol
meaning. Standard uses the stock resources unchanged. Store XNav/Standard in
OpenNav-owned preferences. Legacy and Safe retain validated stock presentation.
Choose a narrow initialization hook and a controlled restart for a style change
if live cache/lifetime safety cannot be established; never delete a library
while existing charts retain its lookup pointers.

Depth colors must retain distinct shallow, safety and deep-water roles using
OpenCPN's safety-depth/contour semantics. Do not turn all depth areas into one
prototype water swatch. Lateral/cardinal marks, hazards, lights, restrictions
and traffic schemes keep established semantics. No optimistic "corridor clear"
label unless a validated query actually supports the qualified statement.

Proper Night resources must lower luminance and preserve distinctions; do not
apply the prototype's SVG brightness filter to a Day ENC. Raster images retain
OpenCPN's supported palette/night behavior without destructive recoloring;
only overlays share XNav styling. Exact object-level visual conformance is an
ENC goal, not a claim about raster charts.

## Required evidence (pending)

Public/licensed ENC with coast, depth areas/soundings, marks/lights and hazards;
XNav vs Standard captures; Day → Dusk → Night → Day; XNav → Standard → XNav;
Legacy round trips; software and OpenGL; real boat charts without distributing
their files. Existing blank-chart regression checks remain mandatory. UI/route/
ownship/AIS overlays need separate comparisons and valid source vectors only.
No chart-presentation acceptance is claimed by this inspection document.

## Version 1 implementation (not yet visually accepted)

`resources/chart-style/v1/definition.json` maps the HTML's land, shore, water,
contour and text tokens to thirteen S-52 palette entries. The HTML supplies no
complete depth-area set; explicitly documented extra shades retain all five
pinned deep/medium/shallow/very-shallow/intertidal categories. Safety contours
and safety soundings remain distinct. This is a safety-required extension to
an illustrative map, not a claim that the prototype defines those extra colors.

Generation verifies every stock input hash against `source-lock.json`, changes
only the allowed RGB attributes in DAY_BRIGHT/DUSK/NIGHT, preserves every symbol,
lookup, line style and pattern definition, and emits a resource hash header.
The RLE resource remains byte-identical. The initial Day atlas was unchanged;
the separately reviewed owned tiles and SCRUM-261 mask now alter only their
documented regions. Dusk/Night sprites
derive only neutral pixels matching the Day neutral RGB, the pinned theme's
neutral RGB and identical nonzero alpha: 42,100 pixels per sheet. Every other
pixel, all alpha values and PNG metadata remain unchanged. This preserves
symbol shape and chromatic navigation distinctions; it does not establish
hazard visibility without native ENC review.
The result installs in the separate shared-data `opennav/chart-style/v1`
directory. Original `s57data` stays untouched. The runtime verifies all five
resource hashes before constructing a library. Missing/corrupt resources use
Standard with an explicit diagnostic reason. Display settings select XNav or
Standard with an ordinary controlled XNav restart; existing chart lookup
pointers are never invalidated in a running library.

A narrow loader option prevents the current working directory from shadowing
the chosen presentation. XNav Standard also uses the explicit stock path.
Legacy/Safe retain the original loader's default behavior and ignore the XNav
style preference. The existing GSHHS and shapefile background mechanisms share
the verified XNav land/water palette; those basemaps remain coastline references,
not a substitute for a nautical ENC. No synthetic chart objects are introduced.

Resource tests verify deterministic generation, exact prototype identity,
unchanged navigation sections, exact neutral-pixel masks, depth-role separation, Night vs Dusk
luminance, original-file preservation and refusal of changed upstream input.
Real ENC appearance, hazards, OpenGL/software rendering, style/mode cycles and
boat display remain pending. Route, ownship and AIS overlay restyling is a
separate unfinished part of this workstream.

The historical second ink pass added CHBLK after actual native ENC review found
monochrome text nearly invisible at Night. At that stage Day retained #070707 rather than
substituting low-contrast muted text over shallow water. Dusk uses the existing
prototype floating text token; Night uses its chart-text token. Numeric contrast
checks cover deep water, very-shallow water and land independently. These checks
do not qualify hazard visibility. The third pass keeps CHGRD aligned with CHBLK
and changes only the bounded neutral raster pixels described above. Independent
Pillow-decoded golden pixel hashes test the stdlib PNG implementation. The 388
resource checks pass. Local software and llvmpipe GL each retain real ENC through
four theme captures, but GL general text still exposes a pinned renderer-color
difference. This is a known defect, not accepted parity. The large embossed
"Feet" overlay is OpenCPN's real chart depth unit, not a place label; it must
remain semantically visible if its presentation is changed.
See the [contrast investigation](reviews/chart-ink-contrast-investigation.md).
SCRUM-261 supersedes that historical Day decision with the actual marker-black
token, rather than the previously rejected chart-text token; its independent
contrast measurements retain the same thresholds. This is still subject to
actual Windows/ENC and boat readability review.

## SCRUM-231 built-up areas (qualification pending)

The hash-pinned US5SEAFL chart identifies Seattle and West Seattle as BUAARE
polygons. Their stock CHBRN fill produced the large mustard regions in the
retained native XNav capture even though LANDA was correctly themed.
The separate XNBUA color now uses the exact prototype `--land` fill
(Day #eeeee2, Dusk #4e615d, effective Night #1d2925). This supersedes the
earlier shore-green mapping: the user's exact prototype requirement takes
precedence over the former separate built-up fill shade. BUAARE remains
classified and bounded, while its fill intentionally matches ordinary land. Only the fill token in pinned
BUAARE Area lookups 16/32052 (Plain) and 356/32391 (Symbolized) changes.
Their boundaries, text, classification, priorities and geometry remain intact.
This narrowly enumerated exception supersedes the earlier blanket statement
that every lookup is byte-identical. Point BUAARE symbols stay unchanged.

CHBRN remains unchanged because the same stock role paints structures,
above-water obstructions/wrecks and obscured light sectors. No conditional
symbology, depth role, chromatic symbol, source chart or Standard resource is
modified. The generator rejects any other rule change, including label changes
inside the two allowed lookups. Runtime verification and controlled style
restart retain their existing behavior. This increment does not qualify the
remaining ownship, symbol, label-density or chart-presentation work.

## Default ownship and online AIS names (qualification pending)

SCRUM-232 adds the immutable prototype chevron for the default, accurate,
fixed-size ownship only. Shared drawing uses upstream position/rotation and
retains user-size scaling; custom images, scaled hulls and inaccurate states
remain stock. The [ownship review](reviews/scrum232-ownship-chevron.md) records
exact path, theme colors, GL topology, DPI scope and safety boundaries.
Day/Dusk/Night now have a short integrated Linux real-ENC capture, with fresh
controlled loopback navigation and clean exit. It is not native Windows,
actual OpenGL, physical touch or boat acceptance.

SCRUM-233 adds available online AIS names at the prototype baseline and theme
ink through owned snapshots. Freshness, selected-target priority, collisions,
viewport bounds and bounded workload remain explicit. Stale/lost marks have no
current name label; target details still carry their source state. Saturated
500-pixel upstream text metrics cause omission, never falsely accepted bounds.
The [AIS-label review](reviews/scrum233-online-ais-labels.md) scopes its fixture
and production-object evidence. Standard receives no new labels.

The [combined comparison](reviews/scrum15-chart-comparison-20261002.md) records
remaining visual differences, including text density, stock chart symbols and
course-predictor artwork. These increments do not accept the complete chart.
See [bounded review](reviews/scrum231-built-area-style.md).


## SKAGER identity and geographic text (2026-10-02, pending visual gates)

The selected style is presented to the user as SKAGER; the saved `XNav` value is
retained for configuration compatibility. The immutable HTML is not renamed.
The [geographic-name policy](reviews/scrum238-geographic-names.md) applies the
prototype's 12px regular land /16px italic water hierarchy and chart-text ink
to geographic names only. It preserves stored OpenCPN font settings for Standard
and all navigation labels/soundings. The full strict resource guard now covers
18 geographic ink substitutions in addition to two built-area fills.

The [active-route foreground](reviews/scrum237-route-foreground.md) now uses
shared 2.6px fractional geometry and round joins for factory-equivalent active
route appearance. [SCRUM-241](reviews/scrum241-route-underlay.md) adds the
translucent 6px whole-union underlay for supported geometry, with an atomic
underlay-only fallback for delicate short joins and bounded workload. A 32px decorative halo cannot stand in for the prototype's meaningful
route-corridor setting. These bounded changes and the current source/branding
batch require fresh integrated/native/boat comparisons; no conformance PASS is
added by this record.

### Healthy factory-equivalent COG predictor increment (SCRUM-239)

The shared software/GL ownship-indicator path can paint the existing COG line
with route ink, 1.2 logical pixels, 5/5 dash and .65 opacity. Its real projected
geometry, time horizon, validity/visibility and endpoint remain upstream.
SKAGER presentation explicitly owns factory-equivalent width/style/color;
startup capture precedes upstream density mutation and observed runtime custom
changes revoke ownership. Nonfactory/custom/degraded states, HDT and endpoint
markers keep stock appearance. No configuration is rewritten. Saved defaults
cannot reveal identical-value user intent; density-raised persisted widths are
conservatively stock on a later startup. See
[the bounded predictor review](reviews/scrum239-cog-predictor.md). Actual GL,
native Windows, boat and full ownship conformance remain pending.

## Geographic label tracking and opacity (SCRUM-243, pending qualification)

Geographic land names now use the prototype's 1px tracking; water names use
5px tracking and .36 opacity. The existing S-52 placement and declutter boxes
account for their measured widths. Precomposed Latin names use native glyph
advances; combining sequences and complex scripts retain native whole-string
shaping to prevent detached accents or broken text. No chart name is rewritten.
The GL path reuses cached native whole-label textures with per-label scale/ink
invalidation; software uses native drawing and alpha. Other label classes,
soundings, user preferences and Standard remain intact. The
[focused review](reviews/scrum243-chart-name-spacing.md) records a corrected
Latin/Unicode Day/Dusk/Night fixture. Native Windows/GL/boat acceptance is open.

## SCRUM-248 light descriptions (qualification pending)

Proven normal generated LIGHTS descriptions now map factory-equivalent appearance
to the prototype's 8px regular symbol-label font, .12px tracking, chart-text ink
and 3.5px round water halo. Both native software and GL texture upload consume
the same bounded cached label raster. Actual text, chart preferences, visibility,
light sectors and symbols remain upstream; custom appearance and out-of-bound
raster requests retain stock presentation. The native raster approximates SVG
stroke edges and rounds glyph placement. Small default-size readability,
Windows/DPI, actual GL driver and boat review remain open. See
[scope, guards and evidence](reviews/scrum248-light-description-typography.md).
## SCRUM-251 submarine cable paint (qualification pending)

Only pinned line-style RCID2012/CBLSUB06 selects new XNCBL ink instead of CHMGD:
prototype area hue Day#9c8696, Dusk#b8a0b1, Night#71636e after its chart brightness.
Its HPGL, widths, geographic geometry and lookups remain unchanged. Global
magenta, cable-area restrictions, ferry lines, dumping-ground boundaries and
information symbols retain their meanings and rendering. Reverse equality and
negative resource tests constrain this one node. See
[real-chart audit and scope](reviews/scrum251-submarine-cable-paint.md).

SCRUM-252 adds the bounded default active-route name treatment documented in
[the route label review](reviews/scrum252-route-labels.md). It preserves actual
names and name visibility, every SCRUM-242 eligibility fallback, custom Marks
appearance and offsets, and stock navigation geometry. Its Night-only label
palette includes the immutable chart ancestor brightness. Native/real-route/
boat acceptance remains open.

### SCRUM-264: classified prototype seamark artwork

The owned resource derivative now maps the sixteen exact Simplified marine
BOYLAT selectors 1029–1044 to eight private ordinary/preferred-channel glyphs,
and gives BOYISD12/BOYSAW12 the supplied prototype Simplified artwork. Verified
core/private presentation instances select XNLIT011/012/013 compact light aliases
only when the actual LIGHTS object has no ORIENT attribute. The original
LIGHTS11/12/13 vectors, pivots and bounds remain stock, including the correction
to the earlier unconditional LIGHTS13 bitmap preference. Any ORIENT presence
(including malformed/non-finite values) retains upstream vector/angle handling;
this guard does not validate or sanitize the attribute. Missing/invalid aliases,
Standard and disabled integrations retain the original Rule.

The supplied package maps LIGHTS13 only. Red/green are explicit derivatives of
its exact circle/rays geometry using prototype --mark-red/--mark-green for the
rays; neutral point fill/ring and 25/32 scale remain identical. No global
LITRD/LITGN recoloring occurs. Color/sector/range conditional procedures and
co-located buoy/TOPMAR composition remain unchanged. Source lookup
order, physical TOPMAR, unknown/inland/Paper Chart users and original glyphs are
preserved. All owned colors apply the prototype Night brightness exactly once.

BOYSPP11 remains stock because its existing selectors do not prove yellow;
actual white/orange buoys must not acquire a yellow/X mark. Generic beacon
composition, remaining Paper Chart art and actual native/private-chart display
qualification remain open. See `docs/evidence/scrum264-seamark-art/README.md`
and the complete preceding family audit for exact boundaries and evidence.

The orientation correction and focused core/private source evidence are in
`docs/evidence/scrum264-oriented-light-aliases/README.md`. Actual revised
red/green ENC captures and native/private-chart/boat qualification remain open.
