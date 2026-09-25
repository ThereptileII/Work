# Beta AIS workflow

OpenCPN remains the source of targets, navigation status, ranges, bearings,
CPA/TCPA and alarms. XNav lists targets and presents copied, timestamped values;
SmartNav presents only existing upstream encounter context. No collision rule
or steering command is introduced.

A live target card offers **Select target on chart**. The integration rechecks
current target validity, enables existing AIS chart display if needed, and uses
OpenCPN's existing chart centering. One owned target copy identifies the selection.
The narrow renderer hook reuses OpenCPN's own `TargetFrame`; it does not mutate
AIS targets. Losing/removing/invalidating the target removes selection. Merely
reading the selection does not refresh its age. Repeated MMSI or backwards
position observation is rejected. Demo/replay never select synthetic traffic on
the real chart.

The target list's Refresh action rebuilds rows from the current owned catalog;
an open card updates its current values/states and reports a removed target.
The target list is bounded by the 2,000-target upstream-copy boundary. Target
alarm acknowledgement remains OpenCPN-owned: XNav alert acknowledgement affects
only the presentation of a condition, and an empty AIS list is not proof that
an AIS receiver is healthy or failed.

`ais_selection_lifetime` covers retained copies, removal/loss/age, duplicate
identity, invalid coordinates, antimeridian coordinates, future/incoherent and
out-of-order observations, and Demo isolation. `smoke-navigation.py --objects`
exercises the actual target card and chart selection in both integrated builds.
Native rendering and physical AIS receiver reception remain distinct gates.

Beta inspection also found that the shell sampled its tick clock before calling
an AIS copy callback which sampled a later clock. The advice consumer correctly
rejected that apparently future publication. The callback now receives the
shell's observation epoch explicitly; original AIS report age is still preserved.
Pilot polling uses the same wall-clock tick epoch as its displayed readings,
while replay retains its separate historical clock. This avoids briefly marking
new simulator feedback as future data. Neither change renews a sensor timestamp
on read or alters real feedback acknowledgement rules.

Run `36117089450` exposed another clock-boundary defect: converting an unchanged
wall-clock `PositionReportTicks` on every copy introduced tiny monotonic-time
regressions, clearing retained chart selection. The integration now keeps one
owned converted epoch per MMSI/report timestamp, bounded by the copied target
set; deleted targets lose that cache. Only a new upstream report changes the
observation, preserving OpenCPN's one-second report resolution. Sixty-four
repeated real bridge reads must return the exact same position epoch. The
isolated AIS fixture now supplies continuing synthetic report state and waits
for the actual shell advisory observation, avoiding a race with OpenCPN's
separate CPA/alarm timer. No production AIS calculation or timer is altered.
