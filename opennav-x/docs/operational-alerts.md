# Beta operational alerts

`application::AlertCenter` consumes owned Vessel Data, route-energy, copied AIS,
anchor-watch and pilot views. It has no adapter/command reference. The shell
observes the same coherent tick used for the displayed advice; retained sensor
values are assessed using their original timestamps and source freshness limits.

INFO, ADVISORY, WARNING and CRITICAL are presentation levels. GPS loss, upstream
AIS/anchor alarms, unconfirmed manual pilot commands and loss of previously
engaged pilot feedback receive critical presentation. Energy reserve is advisory;
insufficient route energy is warning. The prediction must reference the exact
current route publication and current calculation tick. Invalid/stale dependent
inputs suppress the prediction, not manufacture an energy result.

Known sensor loss is reported independently for position, heading, measured depth,
apparent wind, rudder, RPM, motor temperature, battery SOC/voltage/current. A
never-installed sensor does not generate an alert. An explicit invalid input is
reportable; valid zero remains valid. Aging is visible in source diagnostics and
is not itself treated as sensor loss. No measured-depth clearance threshold is
invented without a sounder offset contract.

The global 56-DIP strip remains above all center pages while a condition exists.
Menu → Alerts (Ctrl+Shift+F9) opens the actionable list. Acknowledge marks only the
current XNav episode. It neither clears an active condition nor acknowledges
OpenCPN/device alarms. Recovery removes a condition; recurrence creates a new,
unacknowledged episode. Demo, replay and live histories are isolated. Replay
rewind begins a new historical health interval. No automatic steering, retries,
command transmission, audible alarm override or OpenCPN alarm suppression occurs.

AIS alarm state comes from OpenCPN. An empty target list or no received targets
cannot establish receiver failure; source diagnostics retain that uncertainty.
Pilot feedback loss does not imply physical STANDBY. The instruction remains to
check the physical pilot, including physical STANDBY access. XNav control output
may already be disabled while the physical device remains engaged.

Runtime diagnostics expose IDs/severity/episode/acknowledgement. The bounded field
journal records typed transitions; the field bundle omits target/route/device
identities. The current set is bounded by the fixed implemented conditions.

Validation: three portable groups cover sensor loss/zeros/future observations,
upstream flags/physical-feedback/energy route coherence, and mode/replay isolation.
The actual preview smoke requires a persistent strip, acknowledges GPS loss,
changes pages, restores input and proves a new unacknowledged dropout episode.
Native capture and DPI review remain mandatory for acceptance. Acoustic alarms
and target navigation-PC/physical touch operation require separate field review;
existing OpenCPN alarm behavior remains intact.
