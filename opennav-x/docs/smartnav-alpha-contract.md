# SmartNav Alpha advisory contract

SmartNav consumes owned Vessel Data, route and AIS copies. It has no dependency
on the hardware-adapter library, UI actions, OpenCPN globals or command output.
Its output is an advisory event stream for a human to review.

## Route steps and timeline

The existing before/after normal-progress observer now copies waypoint names,
positions, active-point range/bearing and later stored route-leg distances and
courses. `RoutePoint::GetCourse()` reads the value set by upstream
`Route::UpdateSegmentDistance`; OpenNav does not call that update function.
Names and courses participate in revision/coherence comparison. Remaining
distance retains its accepted semantics and all invalidation rules. Invalid
course suppresses turn prediction without inventing a course or distance.

`RouteProgressSnapshot::remaining_steps` owns these values. The first step's
distance is the current active-point range; later steps use their stored legs.
Consumers validate identity, step count and agreement with remaining distance.
The existing upstream-linked antimeridian test compares copied courses and
distances directly with pinned OpenCPN values. No additional upstream hook is
needed. Invalidated publications do not supply a usable timeline.

With fresh SOG at least 0.5 kn, timing is cumulative remaining distance / present
SOG. Next-turn course is the next stored leg; angle is relative to present COG.
Later turn angles compare adjacent planned leg courses. All timing is estimated
at constant present speed, not a promise about arrival. Missing/stale motion
withholds dependent timing/turn information. Event output is bounded.

## Energy and AIS

Energy events include destination SOC, reserve arrival/crossing and shortfall.
They require a prediction calculated for the current epoch and exact immutable
route publication. A cached prediction or another route's prediction cannot be
reused accidentally. Existing energy input validity rules remain authoritative.

AIS copies retain upstream CPA, TCPA and alarm status. SmartNav only presents
future encounters already alarmed by OpenCPN with usable CPA/TCPA; it neither
recalculates collision geometry nor adds COLREG interpretations. Lost, doubtful,
stale or past-CPA targets do not become future events. UI AIS details may still
show these explicit states.

## Hazard look-ahead foundation

`IChartCorridor` accepts a future path with route revision, observation/position
times and configured half-width. Its provider must perform spatial intersection
using the correct chart/ENC semantics. Results carry matching query provenance,
coverage, chart object identity, datum and uncertainty. Classification compares
charted depths with configured draft plus safety margin; unknown obstructions
remain potential hazards. It does not confuse measured depth with charted depth,
or assume tidal corrections. No-results messages explicitly deny proof of safety.

The live provider is unavailable. Pinned OpenCPN's rendered object picker is not
a complete corridor-query boundary. The tested fake provider proves the query,
revision, coverage, uncertainty and threshold contract, not live chart coverage.
Synthetic Demo geometry is not queried against live charts. This is the explicit
allowed Alpha abstraction gate; upstream chart-query integration remains open.

## Tests

`smartnav_route_timeline`, `smartnav_energy_ais_events` and
`smartnav_chart_corridor` exercise planned steps, turns/north wrap, stable owned
copies, stale/invalid/missing data, conflicting revisions and totals, energy
provenance, AIS alarms/past/lost state, corridor geometry/draft, unknown datum,
unavailable coverage and the no-proof-of-safety invariant. UI, live AIS bridge
and chart-provider acceptance are separate integration gates.
