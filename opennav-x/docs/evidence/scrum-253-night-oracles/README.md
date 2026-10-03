# Effective Night paint oracles — SCRUM-253 follow-up

Follow-up to `1be97180e1d7b2ce546572acf280fb0835172a7a`; no production paint changes.
Five dependent fixtures still supplied the raw Night route color through their
paint-input doubles. Their Night values and independent pixel expectations now
use `71937e`; endpoint/underlay water uses `0e171c` and underlay fill `101a20`.
The underlay excerpt fixture includes the new production color helper. The
endpoint reference separately resolves the final immutable CSS ancestor filter.

The real-navigation smoke oracle now applies Night canvas brightness before its
existing exact GL `/256` framebuffer conversion, only in the SKAGER-style branch.
Standard continues using the reported upstream color. A direct execution of this
actual oracle block checked all style/theme/renderer combinations: 24 equality
checks plus a negative control rejecting the prior raw Night result. No integrated
navigation smoke, application launch, live input, boat or CI was run here.

Focused retained results:

| Fixture | Passed checks |
|---|---:|
| COG endpoint/guard/actual ocpnDC painter | 5,477 |
| Route underlay geometry/actual wx paint/workload | 308,141 |
| Actual patched route collection | 1,426 |
| Waypoint painter/ordinal/icon provenance | 62 |
| COG predictor ownership/paint | 72 |
| Route foreground eligibility/paint | 78 |
| Actual smoke-oracle expressions | 25 |

Existing geometry, MOB/custom/selected/invalid-state, alpha composition, GL
state restoration and retained failure assertions remain unchanged. The three
additional underlay assertions check its illustrative stand-in point's ink.
The COG predictor and foreground fixtures deliberately retain a uniform synthetic
background to isolate alpha/geometry; that background is not a Night sea-color
claim. No raw Night route-ink doubles remain in the five audited fixtures.

`route-underlay-turns-crossings.png` is the actual wx software fixture, visually
inspected. GL is a recording interface in these fixtures, not driver-rendered
acceptance. Native Windows, actual integrated chart screenshots and boat gates
remain separate. The fixture scripts are `tools/test-{cog-endpoint,route-underlay,
route-waypoint,cog-predictor,route-foreground}.py`; all ran against the private
patched upstream, with the bundled wx runtime.
