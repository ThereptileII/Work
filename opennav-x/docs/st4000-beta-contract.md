# ST4000 manual adapter — Beta development contract

The portable protocol/controller is connected to the application through
OpenCPN's existing N2K driver registry and application-thread subscriptions.
Desktop loopback and native acceptance are separate from physical commissioning.
No physical commands or firmware flashing occurred during this work.

## Inspected implementation

The actual local translator is `a94b816b2d5d8238366c2e7c0fe8ef15f9dde4ee`.
Its parser/header/config are byte-identical to the public
[`autopilot-controller` at `9baf01b`](https://github.com/ThereptileII/Work/tree/9baf01bca09522794a9678dfb3f3c0720d5c9943/autopilot-controller).
`tools/fetch-st4000-oracle.py` fetches only these three hash-pinned files. Both
contract CI platforms compile the actual `BridgeCore.cpp` into a host test and
pass all six generated commands through `bridge::parseCommand`. This tests
protocol interpretation, not physical delivery or SeaTalk electrical behavior.

The producer emits PGN 65379 only from fresh physical SeaTalk pilot status,
PGN 65360 for locked magnetic heading, and 127250 for actual magnetic heading.
Its three-second feedback expiry must remain in the commissioned firmware.
Standard rudder is absent with `publishRudder=false`; no rudder is invented.
65379 exposes observed TRACK/WIND modes, but Beta command capabilities exclude
both until the complete physical path is verified.

## Ownership, identity and permission

`St4000Pilot` consumes copied PGN values and an `IN2kPilotTransport` interface.
The integration owns application-thread driver lookup/subscriptions. No driver,
route or waypoint pointer enters this adapter, Vessel Data or SmartNav.
SmartNav does not link the adapter library.

Configuration specifies the exact OpenCPN interface and a 16-digit hexadecimal
NMEA 2000 NAME. The supported translator's manufacturer 1851, industry 4,
function 135 and class 40 are checked; its boat-specific unique number is never
guessed. A matching observed PGN 60928 address claim is required before accepting
pilot state or targeting commands. Preferred address 204 is not hard-coded as
identity. NAME is bus identity, not cryptographic authentication.

Claims and feedback from other interfaces/devices are ignored. Address reuse,
duplicate NAME at another address, disconnect or transport generation change
invalidates retained state. Conflicting claims require the human to remove and
reverify the binding; they do not automatically restore control. A reconnect
requires new identity observation and deliberate session enablement.

Configuration permission defaults to display-only. Even with stored permission,
`ManualAutopilot` starts disabled. Both explicit permission/capability and a
human-enabled session are required. Replay, Demo/live transitions, Safe, Legacy
and shutdown are independently enforced integration isolation boundaries. The
application saves permission and exact binding in its validated shared settings;
session enablement is never persisted. Editing identity resets permission.

Human-requested identity refresh is ISO request 59904 for address claim 60928,
on only the configured writable connection, at most once per five seconds.
It cannot enable control and sends no pilot-mode command. There is no automatic
background discovery/transmit loop.

## Commands and acknowledgement

STANDBY, AUTO and ±1/±10 are addressed PGN 126208 targeting 65379. Manufacturer,
industry and action are explicit structural parameters matching the actual
firmware parser. No vendor fields are encoded in the UI. Heading steps require
confirmed AUTO and fresh measured locked magnetic heading; AUTO requires fresh
actual magnetic heading. STANDBY may be attempted with stale mode, provided the
transport/device identity remains valid.

Lifecycle records include requested, sent/pending, confirmed, rejected, timeout
and disabled. Only one non-standby request can be pending. STANDBY can supersede
an uncertain earlier action. A 250-ms minimum interval also bounds repeated
STANDBY and rejected-send retries; there is no automatic retransmission.

Success requires subsequent physical mode feedback from the same source and
connection generation. A course change additionally needs a **new** locked
heading observation after the request, within 0.5° of the requested wrapped
heading. A newer mode packet cannot refresh an old locked-heading value. Timeout
is three seconds, with outcome unknown; late feedback remains visible but cannot
retroactively confirm. Disabling does not pretend to cancel transmitted output.
Physical STANDBY remains necessary when an outcome is uncertain.

## Qualified OpenCPN transport boundary

Pinned `model/src/plugin_api.cpp::WriteCommDriverN2K` ignores the boolean result
of `SendMessage`; the network driver's `SendN2KNetwork` also returns true after
calling `SendSentenceNetwork` regardless of its result. Neither is pilot success.
The portable adapter deliberately treats transport acceptance as pending only.

The received-message `PayloadToName` in network/serial drivers copies bytes from
the normalized envelope; it is not an observed PGN 60928 NAME. The pilot adapter
must use the actual address-claim payload and envelope source address. Management
handling consumes network product-info 126996, but permits 60928 to reach the
normal bus. Do not assume product-info subscriptions can identify the translator.

Network output varies by transport and detected format: TCP Actisense ASCII has
an inspected output path; SeaSmart cannot transmit; UDP output is compiled out;
other paths may start gateway discovery. `OpenCPNPilot` currently enables output
only for an enabled bidirectional TCP connection with detected Actisense
complete-PGN ASCII format. Other paths remain status-only. Serial output is not
qualified: the pinned serial `SendMessage` attempts an eight-byte NAME extraction
even for the three-byte ISO request payload. No serial control claim is made.

Three narrow read-only network getters expose detected format, connection
generation and its monotonic change time. Socket replacement, connect, loss,
watchdog close, write-error close and explicit close advance the generation.
Queued observations predating it are rejected. This prevents a reconnect of the
same driver object retaining a previous device claim. Registry changes also
invalidate state. No getter transmits/discovers/registers output PGNs.

The application resolves a driver only while executing on the application
thread. `SendMessage` is used solely for the explicit command/identity request;
no driver pointer is retained by the portable adapter. Transport acceptance is
always pending, never physical acknowledgement. The loopback fixture binds only
127.0.0.1 and validates the actual OpenCPN-serialized bytes and UI behavior.

## Automated and physical gates

Six new suites cover binding/claim conflicts, exact feedback meaning, malformed
and out-of-order data, command encoding, feedback acknowledgement, disconnect/
re-enable isolation, discovery permission and the actual pinned firmware parser.
Existing manual-controller failure tests retain exact timeout/stale/disable
coverage. Their send-attempt timestamps now respect the new 250-ms anti-repeat
rule; an explicit immediate-retry assertion was added, not removed.

Local contracts currently pass 44 suites with the firmware oracle enabled.
The integrated Linux build and 97 regressions also pass; native replacement gates remain required. Final boat acceptance must
verify configured device identity, read-only physical status, each secured-vessel
command separately, rejected/lost feedback, communication loss and physical
STANDBY. Prior firmware dockside evidence does not qualify this new PC adapter.
