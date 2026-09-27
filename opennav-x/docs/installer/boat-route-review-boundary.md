# Boundary for an existing-route display trial

This source audit describes a possible later, bounded Activate → inspect → Stop
trial against an existing saved route. It does not authorize or implement a
boat action. The current cold-launch `noActiveRouteOutput`, input-only connection
checks, full plugin-tree/quarantine proof and fixed display-action allowlist are
unchanged. [Current commissioning](read-only-commissioning.md) still excludes
route activation. Real GPS and the installed chart must first be observed; a
missing prerequisite is not replaced with synthetic data or a newly created
route merely to obtain a screenshot.

## Source result

The inspected source provides a path that need not emit route or actuator data
when **all actual output drivers are absent and every retained plugin's route
callbacks are accounted for**. XNav pilot control OFF is only one condition; it
does not govern OpenCPN's native route output or third-party plugins. This is a
software boundary, not a claim of physical bus silence or navigation acceptance.

`src/integration/NavigationObjects.cpp::ActivateRoute` resolves the copied route
identity/revision, checks editability and fresh selected position, asks OpenCPN
for its normal activation point, then calls native activation and persists the
route. It does not call the XNav pilot adapter. The UI captures its rendered
route before confirmation. `StopRoute` revalidates the exact active identity and
revision so a stale confirmation for A cannot stop a different route B.

Pinned `model/src/routeman.cpp::ActivateRoute` sends its plugin activation
notification **before** collecting output drivers. It resets the N0183/N2K output
flags and selects only drivers whose actual `ioDirection` is `OUT` or `IN/OUT`.
`UpdateAutopilot` invokes the two native output paths only when their flags are
set. N2K serial and the reviewed network/0183 drivers derive this attribute from
their configured connection direction. Deactivation sends plugin notifications
but contains no direct bus transmission call.

Plugin notifications remain significant with an empty output-driver list:
activation, waypoint activation, arrival and deactivation notifications are
delivered through the plugin bus; active-leg data is also delivered by
`model/src/plugin_comm.cpp::SendActiveLegInfoToAllPlugIns`. The retained plugin
source therefore needs a route-specific boundary in addition to the existing
startup/idle attestation:

| Retained plugin | Inspected route-notification behavior |
| --- | --- |
| Chart downloader | No plugin-messaging or NMEA-event capability |
| Dashboard | Message callback handles WMM variation and Signal K; no route-message handler or active-leg override |
| GRIB | Message callback handles its GRIB request/configuration IDs; no route-message handler or active-leg override |
| WMM | Message callback handles WMM requests; no route-message handler or active-leg override |
| o-charts | No NMEA-event capability or active-leg override; route/waypoint IDs are not handled by its message callback |

The bundled plugin evidence is pinned to OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`; the default active-leg callback in
`model/src/ocpn_plugin.cpp` is empty. The external chart plugin was checked at
[its recorded c98bf5f source](https://github.com/bdbcat/o-charts_pi/blob/c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8/src/o-charts_pi.cpp).
Its existing closed chart-decode/license helper remains the separately documented
vendor trust boundary. This audit does not extend that trust to equipment control.

The quarantined autopilot-route, AutoTrackRaymarine, AIS-voyage-data and unqualified
receiver-loader DLLs must remain absent from all recursive loader roots and PATH.
In particular, AutoTrackRaymarine route deactivation can itself send standby;
disabling its checkbox or XNav's pilot controller is not equivalent to quarantine.
No previously loaded control-capable plugin/helper may survive into the session.

## Additional gates before any trial

1. Pin the exact installed candidate, current full plugin/dependency trees and
   fresh process. Extend retained-plugin evidence specifically to the callbacks
   above, without fabricating a new startup/idle or output attestation. Confirm
   the live driver registry contains no route-output-capable connection and that
   no plugin has taken over native route handling. Do not enter connection,
   plugin, Send-to-GPS, pilot discovery/control or upload flows during the trial.
2. Observe coherent fresh actual GPS. Recheck selected route identity/revision,
   current inactive state and existing geometry. Prefer an already-visible saved
   route to avoid an unnecessary visibility mutation. Keep a private baseline
   of the route database/profile and reject duplicate, protected or edited routes.
3. Establish that this is not a temporary delete-on-arrival route. Normal arrival
   can advance/end a route; `gui/src/routeman_gui.cpp::DoAdvance` deletes routes
   marked `m_bDeleteOnArrival`. That flag is not currently exposed in the copied
   XNav Route contract, so a future guarded tool needs an independently supported
   check rather than guessing from its name or screenshot. Do not alter existing
   waypoints, arrival radii or position to force a particular view.
4. Qualify the exact bounded UI path with native disposable fixtures: one selected
   route, explicit confirmation, actual active identity, visible progress, and
   identity/revision-guarded Stop. Verify zero native route transmissions with
   input-only drivers, including ordinary advance/completion and plugin message
   delivery. Refuse changed selection, missing GPS, changed output direction,
   returned plugin, unknown modal and stale Stop intent. Existing generic view
   actions must not gain arbitrary activation or confirmation capability.
5. During later supervised review, record one activation and its actual state;
   inspect without changing settings. Stop only that still-active route. If it
   completes naturally, explicitly record completion; do not stop another route.
6. Review resulting navigation/profile writes. Native activation persists route
   state, and clean exit writes `Settings/ActiveRoute`. Keep geometry/identities,
   chart paths and connection/plugin settings unchanged. The current restart
   typed-delta policy must not be expanded to all route/configuration changes or
   blindly renewed. No restart with an unexpectedly active/persisted route.

Existing integration tests establish the copied-object/stale-intent behavior;
they do not establish real boat output isolation. These extra gates and actual
boat acceptance remain open. No route action, equipment command, profile change
or guard relaxation was performed for this audit.
