# Beta chart look-ahead and radar integration boundaries

Inspected baseline: OpenCPN 5.12.4,
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. These are capability limits, not
successful hardware or chart-hazard acceptance.

## Future chart corridor

- `gui/src/s57chart.cpp`, `s57chart::GetObjRuleListAtLatLon` (around 4725):
  calls `PrepareForRender`, traverses render-rule lists and applies
  `ObjectRenderCheck`. Selected render style/category and the viewport affect
  returned objects. Multipoint children come from rendering preparation. It is
  neither a side-effect-free getter nor complete ENC corridor coverage.
- `s57chart::GetAssociatedObjects` (around 3487): upstream explicitly uses a
  line/area reference point, not boundary intersection. Associated depth/dredged
  areas cannot establish all shallow areas/isolated dangers along a future path.
- `GetNearestSafeContour` (around 3460) chooses among chart contour values;
  it does not spatially inspect an active route.
- The visible-light-sector query (around 6860) obtains the current chart or
  quilt chart at a viewport pixel. It cannot discover all charts along a future
  corridor or establish complete coverage/datum consistency.
- Plugin API declarations in `include/ocpn_plugin.h` provide chart object picking,
  not a complete route-corridor discovery/query contract.

Calling these on a grid would still inherit display filtering, miss narrow
objects and claim unjustified coverage. Beta therefore retains an explicit
unavailable live provider. No unavailable result becomes a green/safe route.

The tested `IChartCorridor` boundary uses copied future route geometry, exact
revision and observation provenance, configured corridor width, draft and margin.
Demo/replay cannot query current charts. Position source freshness must pass in
addition to route freshness. Provider failures are unavailable; oversized results
are bounded and marked partial. Charted depth is compared to draft plus margin
only as an advisory; it is not measured under-keel clearance or tide-corrected
water depth. Missing datum degrades coverage. Unknown obstruction depth remains
potentially relevant.

Remaining upstream integration: an application-thread chart-database provider
which discovers/intersects the entire requested corridor independent of the
viewport, returns ENC objects with geometry, coverage, chart identity/edition,
datum and uncertainty, and is validated against public test ENCs including
antimeridian, boundaries, overlapping coverage and isolated dangers. Distance
and ETA to returned objects cannot be invented before that provider exists.

## Radar

The pinned plugin API has no standardized radar image/control interface. The
bundled supported plugin set contains Dashboard, GRIB, WMM and Chart Downloader;
there is no radar provider to discover in that installation. Existing radar
plugins may render their own overlay through normal OpenCPN plugin callbacks,
but the presence of a generic plugin does not establish a radar-specific
capability, scanner connection or command acknowledgement.

`IRadar` retains Off/Overlay/Focus capability negotiation and timestamped status.
The live adapter is unavailable and offers no invented echoes or scanner
commands. Legacy remains the plugin UI escape hatch. A status-only fake tests
capability/loss behavior and is unmistakably DEMO.

No Pathfinder PC receive/control implementation or verified radar plugin is
present in the provided workspace. Required next pieces are the actual scanner
interface/transport, plugin and version, receive-image ownership/timing contract,
capability/status bridge and hardware validation. Beta cannot safely enable
Overlay/Focus without these. Radar/AIS fusion and automatic collision avoidance
remain excluded.
