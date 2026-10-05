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

The product's final output sink remains closed. The existing isolated TCP
loopback tests qualify only a test transport, not the boat's serial path.
The first serial correction removes a reproduced memory overread and serializer
bounds error; it does not yet resolve reconnect queues, short-write handling,
fresh receive provenance or an enabled manual-control session. A transmitted
packet or accepted queue entry must never display CONFIRMED.

Physical testing requires separate explicit authorization with a person present,
steering gear clear and physical STANDBY available. Before that, do not restore
output-capable plugins or a bidirectional connection as a shortcut. A bounded
ISO60928 discovery request is a distinct future commissioning operation, with
steering still disabled and the transport checked first. Physical command,
timeout/reconnect and observed-response acceptance remains open in Jira.
