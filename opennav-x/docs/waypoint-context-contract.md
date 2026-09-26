# Chart context and waypoint range — Beta 2

OpenCPN remains the owner of waypoint identities, geometry and storage. The
integration copies a single uniquely identified waypoint on the application
thread. `WaypointContext` contains only owned values; consumers may retain it
after the native waypoint changes or disappears. A missing or repeated GUID
produces an unavailable context, never a best-effort choice.

Direct range and true bearing use pinned OpenCPN's `DistanceBearingMercator`,
the same rhumb-line calculation used by the native waypoint manager and chart
cursor. This value is **direct waypoint range**, not remaining active-route
distance. No route-progress or autopilot-output processing is invoked.

The calculation requires current, coherent, measured selected latitude and
longitude matching OpenCPN's valid selected position. Missing, stale, uncertain,
nonfinite or mismatched position suppresses range and bearing. Invalid waypoint
coordinates do the same. The result is estimated, uses NM and true degrees, and
retains the source position observation time and freshness policy. Reading the
context does not renew either measurement. The context's own observation time
records when the object was copied; it is not a replacement sensor timestamp.

Waypoint, AIS and chart-position cards share a modeless, chart-bounded XNav
component. They have no pointer grab and do not disable global alerts or manual
controls. Escape, Close or an outside click dismisses the card. The original
outside click is delivered once, not synthesized or replayed. Opening a full
Details page or another screen closes the compact card. Deferred actions use a
shell lifetime token; destruction cannot leave a callback using a deleted shell.

Waypoint Go To needs current range and bearing; stale retained numeric values
do not enable it. Editing and removing use the same focused sheets as full
Details. The integration rechecks identity/revision and native protection rules
at mutation time. Invalid geometry renders as `Position unavailable`. AIS cards
copy upstream values and preserve the separate AIS freshness/selection contract.
Chart-position Go To and waypoint creation are disabled in synthetic/historical
views; a second dispatch check prevents stale UI state from enabling mutation.

The isolated native object scenario compares range/bearing directly with pinned
OpenCPN, including an antimeridian crossing. It covers repeated reads, missing
and stale position, nonfinite waypoint geometry, missing/duplicate identity,
worker-thread rejection and retained context after deletion. The interaction
harness checks compact chart-bounded cards, actual stale-GPS Go To disablement,
Details transitions and native AIS selection. Linux development, native Windows
UI/DPI gates and boat-display review remain separate acceptance evidence.

The Linux interaction harness runs in bare Xvfb without a window manager.
OpenCPN's canvas resize schedules a main-frame raise one second later; a real
window manager normally keeps owned dialogs above that frame. The Linux
harness explicitly raises/focuses a visible context before real pointer input,
and records this adjustment. It does not invoke actions programmatically or
apply any stacking workaround to native Windows. Windows interaction and
real-desktop review are still required for acceptance.
