# SCRUM-251 — one submarine-cable paint role

Base `195ad43`; pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`.
The audit compares `capture-e14de49-sw-short/SKAGER-Day.png` with immutable
prototype `index.html`, without altering the capture, chart or profile.

## Actual chart findings

The small read-only audit program in the evidence directory uses pinned
OpenCPN's `OGRS57DataSource`/`S57Reader`, with the default update application,
to read the actual `US5SEAFL.000` and `.001`. Both input hashes match the capture
manifest. The recorded selection is a bounding-box intersection near
47.6,-122.36, not a claim that every selected object is visible at this scale.
The camera profile is 47.6000,-122.3600, VPScale0.15, rotation0.

- **Submarine cable:** feature RCID1571, CATCBL1, SCAMIN29999,129 real vertices.
  Bounds: longitude -122.349421..-122.340269; latitude47.598522..47.609258.
  The default Lines lookup id710/RCID31786 is `LC(CBLSUB06)`. It follows the
  eastern shoreline; it is not the large cross-bay magenta boundary.
- **Cross-bay cable area:** CBLARE RCID1566, RESTRN2,6,24. The Plain lookup
  id25/RCID32061 uses `SY(CBLARE51);LS(DASH,2,CHMGD);CS(RESTRN01)`;
  Symbolized id365/RCID32400 uses `LC(CBLARE51)` plus the same symbol and
  restriction procedure. The captured boundary preference is78/Plain. These
  bounds and restriction symbols remain unchanged.
- **Ferry lines with repeated rectangles:** FERYRT RCID3032, CATFRY1; Lines
  lookup id745/RCID31821 is `LC(FERYRT01)`. The actual line crosses the bay,
  turns near the terminal and returns northwest, explaining two visible legs.
  The chart's ferry-deviation information text is retained verbatim in the
  audit. It is not cable geometry and receives no change.
- **Small magenta circular boundary near ownship:** DMPGRD RCID2968, CATDPG5,
  has an actual polygonal boundary and chart information about a2020 survey.
  Plain lookup id41/RCID32077 is `LS(DASH,1,CHMGD);CS(RESTRN01)`.
  Its actual shape and all restriction handling remain unchanged.
- **Magenta i marks on the east shore:** HRBFAC RCID2781 (Terminal No46,
 47.596993,-122.338923) and2784 (Terminal No37,47.592888,-122.340149),
  CATHAF10, use the default point `SY(CHINFO07)` lookup (Simplified1128/
  RCID31180; Paper2195/RCID30375). Their locations agree with the two screenshot
  marks; these information symbols are not decorative clutter to delete.
- Nearby PIPSOL, other cable areas, anchorages and restricted areas are recorded
  in the audit with their exact attributes and lookup candidates. Their style,
  visibility and navigation semantics are outside this change.

[Feature records and lookup evidence](../../evidence/scrum251-cable-paint/real-enc-audit.json)
retain exact cable/ferry/harbour WKT plus compact bounds/attributes for the other
classes. Positional correspondence with screenshot marks is an audit inference,
not instrumentation of the running renderer's draw list.

## Exact mapping and boundary

The prototype explicitly places `line:CBLSUB06` (index.html:1143).
`chartMarkerTone` classifies line symbols as `area` (1230), and
`.chart-marker-art.marker-area` uses `--mark-area` (117). Its effective colors
are Day `#9c8696`, Dusk `#b8a0b1`, and Night `#917f8d`; the Night chart's
`brightness(.78)` (15) makes the isolated effective ink `#71636e`.
The separate low-opacity connector line in the illustrative prototype is not
an instruction to fade the real cable or change its chart geometry.

Only the single line-style RCID2012/nameCBLSUB06 `<color-ref>` changes:
`ACHMGD` → `AXNCBL`. Three dedicated XNCBL palette entries supply these effective
colors. Global CHMGD, all lookups, HPGL bytes and its SW1 widths, vector size,
origin/pivot/distance, all other line styles and every symbol remain identical.
This maps every actual feature using that exact node, not just the example
chart. CATCBL6's separate id709 `LS(DASH,1,CHMGD)` remains unchanged.
Standard/Legacy continue loading their original resources.

The prototype's illustrative smooth curve and1.3px line are not substituted for
the pinned S-52 geometry. This is a bounded color-only improvement; it does not
claim complete symbol or chart-canvas visual conformance.

## Verification and limits

The generator first verifies pinned source and immutable prototype identity.
Its reverse-equality validator restores exactly this color-ref and the three
new palette nodes, then checks the complete resource tree against the pinned
original alongside previously authorized exceptions. Independent tests restore
the node themselves and check the whole line-style section. Negative cases
reject HPGL/width/pivot/vector-size edits, wrong or duplicate nodes, missing or
duplicate roles, global magenta changes, ferry recoloring, and altered CATCBL6
lookup. Palette expectations are extracted independently from the immutable CSS,
including Night brightness.8041 focused resource checks pass, including repeat
and Windows-newline deterministic generation and refusal of unknown inputs.

[Color fixture](../../evidence/scrum251-cable-paint/cable-colors.png) uses actual
pinned HPGL pen-up/down paths with the same comparison stroke on each side;
it was opened and inspected. It is a scaled symbol/color comparison, not an
OpenCPN render or chart capture. The Day cable becomes muted, while Dusk/Night
remain visibly distinct in this fixture. This does not qualify real-chart
hazard readability or pixel parity.

No renderer implementation, profile, chart file, navigation data, live source,
CI, full build or boat was modified. Integrated real-ENC SW/GL, native Windows,
DPI/theme and boat readability remain acceptance gates. Remaining ferry,
restriction-boundary, dumping-ground and information-symbol differences are
concrete findings for parent SCRUM-15 acceptance; no further symbol expansion
is included in this batch.
