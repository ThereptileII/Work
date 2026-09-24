# Alpha 1 source boundaries and implementation decisions

Inspection baseline: OpenCPN 5.12.4,
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`. These are implementation boundaries,
not claims that the proposed Alpha features are accepted. See status/evidence
for execution gates.

## Marine data

`model/comm_appmsg.h`, `comm_bridge.cpp` and the existing NavigationBridge keep
selected position/SOG/COG authoritative. Do not replace their precedence with
OpenNav's sensor selection. `NavMsgBus::Notify` publishes received messages through
Observable; `ObsListener` marshals callbacks to the application thread. OpenNav
can observe additional quantities without opening transports or sending messages.

`comm_navmsg.h::NavMsg::created_at` is a **system_clock** receipt timestamp,
unlike selected BasicNavData's monotonic timestamp. Preserve elapsed queue age
when translating to the owned steady-clock contract; reject future/invalid time.
It is not the sensor's original measurement time. Signal K timestamps require
their own validation and must not be replaced with UI read time.

`libs/N2KParser` already decodes standard PGNs for heading, rudder, attitude,
water speed, depth, wind, sea temperature, engine RPM, DC state/battery
status and fluid levels. Reuse these parsers. Its vector wrapper assumes an
Actisense envelope and accesses its header without bounds checks; validate
envelope type, PGN and declared length before calling it. NA sentinels are not
measurements. `plugins/dashboard_pi/src/dashboard_pi.cpp` demonstrates supported
subscriptions and source identity, but its depth offset presentation is **not**
the OpenNav below-transducer field: retain physical meaning rather than copying
Dashboard's display conversion blindly.

Implementation inspection found PGN 127489's coolant parser and supporting
types are compiled out in this baseline (`#if 0`). Alpha does not enable or
rewrite them; coolant remains available through a supported Signal K path.
PGN 127508 uses signed 0.01 V and its encoder saturates at 327.66 V. Alpha
rejects that ambiguous endpoint; a higher-voltage pack requires Signal K or a
separately specified marine extension. Do not reinterpret unknown signed wire
data as unsigned. Both limits are covered in the marine decoder tests.

Per-quantity OpenNav precedence applies only to the newly observed instruments.
Keep source identity including transport/PGN/device instance, source receipt
watermark, explicit configured selection and fallback diagnostics. Never join
battery voltage/current/SOC across different battery instances to predict energy.
Do not infer motor winding temperature from generic engine coolant temperature,
or whole-pack discharge from motor-only power. Such mapping needs explicit
configuration/provenance. No desktop Leaf EV-CAN decoder is planned.

## Route and waypoint operations

Use `Routeman::FindRouteByGUID`, `WayPointman::FindWaypointByGuid` only inside the
application-thread integration boundary. Publish owned GUIDs, names, coordinates,
leg values and revision information; resolve identities again before a command.
Reject disappeared, ambiguous, changed, layer-owned or actively edited objects.

`gui/src/routemanagerdialog.cpp::OnRteActivateClick` uses FindBestActivatePoint,
ActivateRoute/DeactivateRoute, NavObj_dB and normal canvas refresh. Reuse this
behavior for explicit human route activation. `Route::Reverse` plus selectable
segment rebuilding and NavObj_dB::UpdateRoute is the upstream reverse path.
`canvasMenu.cpp::ID_DEF_MENU_DROP_WP` shows waypoint creation, selectable
registration and NavObj_dB::InsertRoutePoint. Deletion must include the upstream
WayPointman cleanup and navigation database update, with a deliberate confirmation.

Basic interactive route creation can reuse ChartCanvas::StartRoute, preserving
upstream construction/storage and chart interaction. Basic edits must account
for route selectable geometry, undo invalidation, persistence and active-progress
invalidation. Advanced editing remains reachable in Legacy.

`Route::UpdateSegmentDistance` stores the pinned Mercator leg distance and course
on its destination RoutePoint. Read GetCourse and m_seg_len; never call this
updater as a getter. The accepted normal-progress observer remains the only
remaining-distance producer. Extend its copied evidence for future waypoint
events rather than recomputing navigation geometry in SmartNav.

## AIS and chart interaction

`AisDecoder::GetTargetList` and AisTargetData expose received identity, position
report time, lost/doubtful flags, range/bearing, bCPA_Valid, CPA, TCPA and upstream
alarm state. Copy on the application thread; no target pointers leave integration.
AIS freshness must reflect its upstream report cadence and lost state, not the
five-second high-rate GPS rule. Own-ship validity also gates relative/CPA values.
Do not call UpdateAllCPA/UpdateOneCPA just to obtain a card or an advisory.

ChartCanvas has zoom, follow, orientation, StartMeasureRoute, StartRoute, AIS
visibility and selected route/point getters. Existing AIS query entry points in
chcanv/canvasMenu can use a narrow XNav-only presentation hook. Legacy keeps its
normal query dialogs. Object queries keep their upstream chart semantics.

## Hazard look-ahead

`s57chart::GetObjRuleListAtLatLon` is a view-dependent rendered-object selection
boundary, not a documented complete corridor safety query. Sampling it cannot
prove the corridor clear or resolve all chart coverage/datum/quality questions.
Alpha will expose a tested path/corridor query interface with chart provenance,
depth datum, coverage and uncertainty. The live provider must report unavailable
until a complete, validated upstream integration exists. Fake chart providers
exercise hazard/advisory logic in tests; measured depth is a different input.

## Anchor watch

`MyFrame::ProcessAnchorWatch` uses pAnchorWatchPoint1/2, the waypoint radius,
AnchorDistFix and DistanceBearingMercator; upstream already owns alarm behavior.
Observe these values and selected position. Do not invoke ProcessAnchorWatch
from OpenNav. Configuration/actions should reuse the existing anchor-watch
objects and preserve upstream alarms. No intelligent-drag claim is planned.

## Boat autopilot and radar

The separately inspected boat checkout's README and NmeaBus/BridgeCore define
addressed NMEA2000 PGN126208 commands targeting proprietary 65379/65360, with
SeaTalk 0x84 feedback, status PGNs127237/65379/65360 and a three-second feedback
limit. Group-function acceptance is distinct from physical pilot confirmation.
Native TRACK requires fresh navigation/variation and a separate human turn
acceptance; no automatic second press or retry is appropriate. The bridge keeps
rudder publication disabled without a verified sensor.

Alpha's high-level adapter interface will model permissions, capabilities,
fresh measured feedback, one pending command, explicit rejection/timeouts and
STANDBY preemption. A deterministic simulator is the only command-capable Alpha
adapter until a physical Windows/boat gate is completed. No SmartNav-to-command
dependency is permitted. Radar has capability/status interfaces and honest
unavailable state; no synthetic radar in live mode.

## Windows distribution

The official pinned release asset is
`opencpn_5.12.4-0+3720.37fd0cd_setup.exe`, downloaded from the upstream GitHub
release and independently SHA-256 checked as
`e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa`.
This is the **installer** hash, not the installed executable allowlist hash.
Native disposable installation must establish executable/resource hashes and
validate integration before any supported entry is published.
Extraction of that verified installer yields an 11,624,448-byte PE32 executable,
SHA-256 `7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.
The [release inventory](evidence/upstream-5.12.4-release-inventory.json) is
inspection evidence only; it does not populate the compatibility allowlist.

`NSIS.template.in.in` records installed paths under the OpenCPN uninstall key;
registry discovery is a hint, never compatibility evidence. Installer operations
must preflight version, PE architecture and exact manifest hashes, reject unsafe
paths/running processes, journal verified backups before replacing files and
preserve user data. No portable marker may be installed into normal OpenCPN.
Update/repair/rollback operate on OpenNav's file inventory, never recursive
deletion of a user's OpenCPN directory or profile.
