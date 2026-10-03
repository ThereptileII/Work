# SCRUM-275: ordinary mixed-sector location point and exact offset fog exception

Increment on `ef266211e214c9ad57b14c7283b3ecd2fe71c67a`, selected by Jira
comment 10812. The earlier `scrum275-ca-light-point` evidence is unchanged and
records the narrower first slice. This increment changes only the shared
inventory helper and its focused fixture. No caller/painter patch, conditional
rule, artwork, lookup, resource or installed application changes here. No full
application build, canvas launch, CI or boat operation was performed.

## New boundary

Individually eligible ordinary single-color CA records may now have different
colors within their co-located group. Such a group uses one existing
`XNLIT013` generic location point, owned by the last LIGHTS member in the
unchanged traversal. It does not take the last record's color or combine ranges,
sectors, bearings or all-round coverage. Each original LIGHTS06 CA instruction,
including the upstream seaward-to-display 180-degree conversion, still paints
independently. The immutable prototype's mixed R/W/G Långskär sector example
uses this same neutral-circle/yellow-ray point (`chart-symbols.json`,
`chart-marker-art.js`, `chart-symbols.css`). This identifies a light location;
it is not a claim about one physical lamp or lighthouse tower.

Only the exact Simplified FOGSIG lookup 31164 is exempted from the independent
co-located-point refusal: Point, Area Symbol priority, radar On Top, Standard,
comment 27080, no selectors, exact `SY(FOGSIG01)`. ORIENT, STATUS, QUAPOS or
QUASOU presence still refuses. A loaded rule must be the one non-private SY
node with the exact instruction and FOGSIG01/1338 raster geometry (12×13,
pivot 15,-3, origin 0,0). Missing/mismatched definitions and altered chains
refuse. Fog attributes and stock fog painting are otherwise untouched.

A null lazy `ruleList` is allowed only after that full lookup check, inside the
already verified resource/library scope. The inventory neither parses it nor
calls conditional or visibility processing. Thus first paint does not depend
on whether the normal renderer has already parsed the lookup. Core initial
and regenerated chart lookup construction calls `_LUP2rules`; private initial
construction does too, but the guard does not depend on those timing details.

The real pinned core and private `ProcessSymbols`/`BuildSymbol` bodies are
byte-identical. Although FOGSIG01 declares V and retains its vector, it also
has a bitmap and no explicit prefer-bitmap override. Both parsers default
`preferBitmap=true`, so both builders select definition R and **bitmap** size
and pivot. `source-resource-proof.json` binds this conclusion to both full
source hashes, the private git-blob lock and extracted method hashes. This is
source proof, not a new native loader execution.

All other existing guards remain: malformed/uncertain/directional/special
records, multiple colors inside one record, short-range non-CA lights, stock
and disabled modes, separate structures/topmarks, invalid inventories and
missing aliases. A filtered last owner is never promoted to another record.
The five existing synchronous scopes, 32,768-object cap, one-shot Take and
Rule lifetime remain unchanged. LIGHTS **and FOGSIG** fields now share the
existing 262,144 attribute budget.

## Focused proof

- `method-receipt.json`: 121 core, 121 private and 119 core-without-GL checks
  pass. The affected tests reverse mixed-group order, compare actual original
  LIGHTS06 CA strings before/after inventory, check lazy/loaded fog cases and
  refuse 18 wrong lookup/rule variants plus orientation/uncertainty. Existing
  lifetime, visibility-boundary, stock, alias and cap checks remain. Actual
  production wrapper/point methods execute, but terminal painters/projection
  are recorded stubs and the description callback is empty: this is not a
  software canvas, GL context or private DLL execution.
- `compile-commands.json`, `objects.json` and four logs: actual core s52plib.cpp
  and s57chart.cpp, private s52plib.cpp and eSENCChart.cpp compile successfully
  against the new header. Dependencies and earlier prepared source were read
  without modification; outputs are private. No unchanged full suite rerun.
- `source-resource-proof.json` and `verify-fog-art.py`: all locked stock files
  and all files in the existing generated resource manifest are rehashed;
  FOGSIG lookup/node and its Day/Dusk/Night tiles remain identical. The existing
  XNLIT013 tile matches the exact committed nontransparent-pixel paint operation
  while preserving stock transparent RGBA. At native source 1×, occupied fog
  pixels overlap the point by zero pixels in all three themes. Tower and pile
  controls overlap by 56 and 21 Day pixels respectively, so remain refused.

The initial private fixture compilation rejected a const string passed to
API17's mutable selector-vector type. `initial-private-fixture.log` retains it;
the test now owns a mutable selector buffer. The production helper was not the
failure. The resource proof initially compared transparent RGB against the
committed RGBA JSON rather than the actual alpha-positive replacement contract;
it now validates that exact operation, including unchanged transparent bytes.
No resource bytes or visual tolerance were changed to pass the proof.

The focused desktop observation records 512 all-light objects at 0.61–0.64 ms,
512 non-light points at 0.044–0.047 ms and the allowed maximum of 32,768
all-light objects at 53.1–54.9 ms. This includes inventory lifetime in the
existing fixture. The maximum is material; it is not boat pan-performance
acceptance. Fog-heavy allocation/attribute workloads have no new timing claim.

## Source cases and remaining limits

The unchanged official IHO GB4X0000.000 source hash is
`c70d9e0f53e149270f85900f8576082db86781d64fbb71aa7d5ceb4af4aa22e3`.
The retained prior `source-cases.json` records LIGHTS32 (white, 20 nm) and
FOGSIG33 at (-32.3760351, 61.0307025): the exact fog exception makes this an
intended source-positive case, pending actual lookup/visibility/canvas proof.
This is official presentation-test geography, not an operational nautical ENC.

LIGHTS992/993/994 with FOGSIG991, LNDMRK1003 and PILPNT1004 near
(-32.5420331, 60.9234263) still refuse the added point: TOWERS01 and PILPNT02
would overlap its center. Mixed ordinary colors alone no longer cause refusal;
independent physical structures still do. No generic tower/pile composition is
implemented or authorized by this increment.

Source-alpha separation does not establish separation after scaling, filtering,
DPI, rotation, clipping, SCAMIN or fractional-pivot rounding. Actual software
and GL captures, native Windows, private DLL canvas, boat visibility and pan
responsiveness remain open gates. Original CA arcs and fog remain authoritative.
No complete lighthouse-family or interaction acceptance is claimed.
