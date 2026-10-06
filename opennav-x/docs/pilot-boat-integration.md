# Pilot connection and measured feedback

Jira: SCRUM-295 (detection), SCRUM-307 (shared AutoTrack setup), SCRUM-313
(actual serial manual-control qualification), SCRUM-19 (safety boundary).
This is a contract/evidence explanation, not a separate backlog.

## What was observed on the boat

Read-only inspection on 2026-10-05 found the existing OpenCPN Actisense serial
receive connection at 115200 baud, with current marine data and approximately
one heading report per second from address 204. The running product was
`c0d8d85fb602e86d40e2f3f1be32307919702408`, executable SHA-256
`8ed5cc1fad45bfc9cda13f98ec0b55cf18270b45a348764673aa7702c12c192c`.
Its pilot state was unavailable, feedback sequence zero, control disabled and
all command capabilities unavailable. This observation proves neither a current
translator identity nor receipt of physical pilot mode. The app was left running;
no connection direction, plugin, profile, firmware or hardware output changed.

## Same connection, stronger confirmation

The inspected AutoTrackRaymarine 2.3.0.0 source uses an existing OpenCPN NMEA 2000
driver. It observes compatibility messages 126720 and 65359 without a separate
manual interface/NAME setup. Its hard-coded destination and outgoing-message
state inference must not be copied as physical acknowledgement.

SKAGER also uses OpenCPN's existing receive drivers. Its accepted pilot state
requires a compatible observed NAME/address followed by PGN 65379 physical mode.
PGN 65360 supplies locked magnetic heading outside Standby; PGN 127250 supplies
actual magnetic heading. No raw OpenCPN pointer escapes this boundary. The
supported bridge parser and controller are hash-pinned by
`tools/fetch-st4000-oracle.py`, at source revision
`9baf01bca09522794a9678dfb3f3c0720d5c9943`.

The actual inspected bridge advertises and publishes 65379 once per second,
using unavailable mode after physical feedback expires. Its NMEA 2000 library
sends NAME/address claims at startup, on ISO requests and during arbitration;
they are not periodic. A PC which joins later can receive heading and mode but
miss the identity claim. No passive algorithm can recover a NAME never received.
Gateway filtering has not been established as the cause on the actual boat.

## Passive commissioning diagnostics

New `PilotStatusDiscovery` diagnostics distinguish fresh unidentified mode
traffic, verified identity awaiting feedback, stale traffic and latched address
conflicts. These explanations do not grant capabilities or infer identity.

`PilotTrafficDiagnostics` observes 60928, 65379, 65360, 127250 and AutoTrack's
65359/126720 compatibility envelopes. At most 64 interface/address/PGN buckets
retain counts and last observation times; invalid envelope/type/length/PGN,
source, time, ordering and capacity have explicit saturating rejection counters.
The runtime diagnostic JSON contains copied counters and age, never raw message
payloads. Counting a received envelope is explicitly not verified pilot state.
The compatibility PGNs are diagnostic-only; they do not bypass accepted identity
or physical-feedback semantics. Reading diagnostics never refreshes observations.
Do not publish raw local diagnostics containing vessel/device identifiers.

## Buttons and remaining physical boundary

The six manual requests are STANDBY, AUTO, -1, +1, -10 and +10. Tests feed their
actual encoded commands into the pinned bridge parser/controller and verify
SeaTalk keys, rejection, echo failure and matching synthetic physical feedback.
Native component checks exercise every button callback, duplicate queued events,
pending commands and disabled controls. TRACK/WIND remain unavailable; no
unverified mode is advertised as supported. SmartNav never invokes commands.

## Manual serial commissioning contract (2026-10-06)

The new product policy is `manual-commissioning`, contract version1. It is not
labelled `status-only`. Normal installation, configuration and every process
start with control OFF. Fixture builds cannot reach a physical serial endpoint;
the existing isolated TCP loopback fixture remains test-only. Native Windows
and actual boat acceptance of this new software are pending until separately
recorded. The prior 2026-10-05 observations above remain historical evidence.

Use **Autopilot setup → Advanced connection setup** to select the same existing
OpenCPN Actisense serial connection used by AutoTrack. Saving a selected interface
with an empty NAME permits only an explicit **Refresh device identity** request.
This sends PGN59904 requesting60928, once at most per five seconds, on that
selected enabled bidirectional connection. It cannot enable steering. There is
no automatic request on startup, reconnect, polling or diagnostics reads. Copy
only a compatible NAME actually observed on that connection; address204 or a
heading report is insufficient. Saving identity always clears saved permission.

After identity and physical feedback are observed, an operator can save manual
commissioning permission and then separately enable control for the session.
The final sink independently checks the concrete serial driver, bidirectional
connection, enabled/current epoch, session, configured exact identity/address,
physical mode younger than three seconds, and exact six-command encoding. AUTO
also requires fresh measured magnetic heading; course changes require confirmed
AUTO and fresh measured locked heading. TRACK/WIND remain unavailable, and
SmartNav has no command path. Saved permission alone never opens the sink.

The controller permits one unresolved command (STANDBY may preempt it), suppresses
repeated requests for250ms, and times out at three seconds without retry. The
worker accepts at most one queued pilot/discovery frame, discards it after500ms,
and purges on disable, disconnect, read/write failure or close. STANDBY atomically
replaces an unsent pilot command so a queued AUTO cannot outlive its cancellation. Reconnect never
restores the session or identity. A write already started cannot be recalled.
Close requests worker stop and joins it before freeing the connection. It does
not rely on the worker having entered its main loop; failed thread creation or
startup is cleaned up before the driver can be used.
Session revocation also occurs on stale/unavailable feedback, identity conflict,
source-settings changes, replay requests, DEMO changes, mode restart and close.
These events cancel unsent pilot commands rather than just greying the buttons.

Receive provenance originates in the serial worker, before the first read of
each frame, and includes its connection epoch. A delayed application event cannot
refresh an old physical sample. A command can be confirmed only by a subsequent
matching physical mode/heading frame captured after the actual complete serial
write of that exact command's monotonic queue ticket. A preceding command's
completed write cannot satisfy this gate. Neither queue acceptance nor a
transmit notification confirms it.

Runtime diagnostics expose `pilot.enabled`, `pilot.serial_session_enabled`,
`pilot.configured_permission`, `pilot.control_capability`, feedback sequence and
connection epoch. Default/new-profile checks require both enabled fields and
configured permission false, with TRACK/WIND false. Product identity reports
`xnav_hardware_output_policy=manual-commissioning` and
`xnav_manual_control_contract=1`. Native qualification, transport failure and
actual physical response must be recorded against exact candidate bytes.

The user subsequently authorized actual manual commissioning. Only the root
coordinator operates the boat after the software/native gates; this software
work performed no port access, connection/profile change or physical command.
Physical feedback, command, timeout/reconnect and observed-response acceptance
remains open in Jira until recorded. Keep a person present, steering gear clear
and physical STANDBY available during that separately controlled operation.
