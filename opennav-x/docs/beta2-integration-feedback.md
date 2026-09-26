# Beta 2 integration feedback — inspected boundaries

Implementation notes, not release acceptance. Final Linux, native Windows and
boat-PC evidence is required before closing feedback items.

## Options and chart lifetime

Pinned `DoOptionsDialog()` is **modeless**: a hook immediately after `DoSettings`
returns cannot repair a later Apply/OK. Options ultimately invokes
`MyFrame::ScheduleReconfigAndSettingsReload`, detaching/re-registering native
AUI canvas panes. XNav may have a different center pane visible at that time.
The new `AfterSettingsReconfigured()` hook runs after normal upstream processing,
restores Navigation against current pane identities, invalidates chart rendering
and reloads viewports. It does not own/reparent a canvas, rebuild the chart
database or process navigation. Legacy/Safe skip the XNav work. The object
scenario exercises this exact path while an XNav page has hidden the chart and
requires an immediately visible, nonzero canvas.

## Late GPS and AIS

`NavigationBridge` observes process-wide selected `BasicNavData`, without cached
connection/driver pointers. The pinned connections editor calls
`UpdateDatastreams()` on add/edit, and `CommBridge` resets selection on driver
changes. No speculative priority reset or second GPS selection has been added.
The reported restart requirement needs a supported-binary connection-add
reproduction; baud, filters and received sentence health must be observed.

AIS copies the actual OpenCPN target model. Fixture-enabled builds have explicitly
synthetic AIS; product builds omit Demo. The object gate retains upstream CPA/TCPA, stale/lost/sentinel,
target-removal and chart-selection checks. Actual receive/decoder evidence is
required before claiming live boat AIS acceptance; absent targets stay absent.

The Beta 2 `--objects` harness now starts with no marine connection and no GPS.
After deferred startup and two normal timer passes, its marked isolated scenario
adds an input-only loopback `ConnectionParams` and calls the same
`UpdateDatastreams()` API used by the native editor. It requires real TCP GGA/RMC
to reach the selected-navigation snapshot without restarting the process. This
tests the actual connection/subscription path, not the Windows connection editor
widgets or the boat's serial baud setting.

The same socket carries `!AIVDM` type-1 reports into the actual pinned OpenCPN
decoder. Received coordinates, SOG/COG/heading and the owned current AIS snapshot
are checked; an invalid-checksum target must never appear. These acquisition
checks precede and preserve the existing isolated model-injection tests for
precise CPA/TCPA, alarms and target lifetime. Initial chart land/water colors
provide a deterministic reference for the additional settings-return screenshot;
an all-water or covered chart fails. Final gate results remain to be recorded.

The actual receive test exposed a concrete Beta 1 AIS defect on a UTC+2 host:
newly received targets decoded correctly but the bridge assigned them an age
of 7,201 seconds. Pinned `AisDecoder::Parse_VDXBitstring`, N2K and Signal K
position acquisition store `PositionReportTicks` after `MakeGMT`/`MakeUTC`, and
`OnTimerAIS` compares the same shifted clock domain. Treating those ticks as
Unix `system_clock` timestamps was incorrect. The bridge now calculates age in
the exact upstream clock domain, bounds it, and pairs it once with OpenNav's
monotonic clock. Retained reads preserve their observation epoch. Four native
contract tests cover wx clock pairing, equivalent UTC/UTC+2/UTC-5 elapsed time,
stale observations, and future/missing/extreme values. Model fixtures now use
upstream ticks too; they no longer mask this difference by injecting Unix time.

## Anchor watch

OpenCPN stores radius in the waypoint name. New XNav watches use whole metres
without trailing decimals and the existing upstream `anchor` icon. Fractional
values are rejected rather than silently rounded. Normal OpenCPN watch
processing remains authoritative.

Clear removes only a safely owned isolated watch mark through upstream waypoint
deletion/undo. The exact Beta 1 ownership description is recognized for upgrade
continuity. Shared, protected and user-repurposed marks remain. Failed deletion
restores the watch and reports failure. The old assertion that every watch mark
must remain is intentionally replaced by owned-mark-removal and user-mark-
preservation assertions, as requested by the user.

## Chart context and routes

`ChartCanvas::InvokeCanvasMenu` keeps upstream hit-testing and geographic
conversion. XNav receives only copied IDs/coordinates through deferred cards.
Other advanced, measurement and route-creation contexts retain native handling;
Legacy menus remain intact.

Go To matches pinned `canvasMenu.cpp` temporary two-point route semantics:
OpenCPN owns activation, progress and delete-on-arrival. An existing destination
is shared and survives temporary-route deletion. Fresh selected GPS is required;
active navigation is not silently replaced. Explicit Finish/Cancel controls the
touch route workflow instead of focus loss. Undo accepts only the current
draft's native `Undo_AppendWaypoint`; Cancel resets native cursor/creation/undo
state and removes only that unprotected draft. Replay guards all new mutations.

Finish now requests a route name/description before saving. The integration
validates the current registered draft and requires at least two points. It
checks `NavObj_dB::UpdateRoute` before invoking normal `ChartCanvas::FinishRoute`,
whose upstream save call otherwise discards the result. A failed checked save
restores the previous name/description and leaves the draft available to retry;
cancelling the naming sheet leaves the draft unchanged.

## Native DPI regression expansion

The native Windows 100/125/150% gate compares actual four-card rail geometry
before and after an explicit fixture critical AIS alert. All cards must remain
fully inside the viewport with identical geometry; no rail scrolling is accepted.
The global critical-alert action is checked using the native window hit-test
while System is open. System is now a scrollable product page, so the old popup
bounds assertion is replaced by all eight current actions being reachable,
inside the page, at least 48 DIP high, and clear of the status/navigation bars.
Day/dusk/night, fullscreen recovery, all existing mode/chart checks, native
touch-injected scrolling and the expanded grouped instruments remain covered.
These are CI checks; they do not substitute for physical-display or finger-touch
acceptance on the boat PC.

## Current development evidence

The Linux integrated object scenario passed under both `TZ=Europe/Stockholm`
(UTC+2) and `TZ=UTC`: 15 grouped contract checks and six captures per run. The
actual settings-return capture retained approximately 60.76% water and 37.27%
land in the fixed chart sampling region. Chart content, all four rail values,
route/waypoint cards, live AIS card and selected target chart were inspected.
Four new AIS clock tests passed in each timezone. Local evidence is retained
under `evidence/local/objects-timezone-stockholm` and `objects-timezone-utc`.
These are development results from the working tree, not exact-commit release
or boat acceptance. The serial GPS editor widgets and real boat reception are
not covered by the loopback test.

## Autopilot protocol clarification

The **local prior setup**, translator commit
`a94b816b2d5d8238366c2e7c0fe8ef15f9dde4ee`, matches the Beta 1 parser oracle and
documents AutoTrackRaymarine Evolution mode with Actisense USB. This is prior
setup evidence, not a fresh inventory of the boat PC.

PGN **126208** is the standard NMEA 2000 Group Function envelope. Target PGNs
**65379/65360**, manufacturer/industry selectors and command meaning here are
Raymarine-specific. There is no supported generic standard-N2K AUTO/button
substitution in the inspected path. XNav UI sends high-level intent; the isolated
adapter encodes the group function; the ESP32 translator alone produces SeaTalk1.
SeaTalk wire encoding does not belong in UI or desktop Vessel Data.

The plugin's USB compatibility profile differs from XNav's qualified
bidirectional TCP Actisense control transport. Serial/USB remains display-only
for this adapter because of the inspected upstream constraints in
[the pilot contract](st4000-beta-contract.md). No guessed command substitution,
new serial-control claim, firmware flashing or physical command is part of this
Beta 2 work. Boat output needs separate explicit user authorization.
