# Native Track and Wind — source-only work

Jira: SCRUM-313. User request: 2026-10-16. This change has **not** been built,
tested, installed or commissioned. The user explicitly deferred builds and
tests. The installed boat application and controller firmware remain unchanged.

SKAGER implementation and unexecuted regression source are integrated through
`25327428` on `codex/scrum298-source-only-20261016`. This is a local source
revision, not a replacement for the installed/qualified package.

## Source boundaries

The local controller project is `Autopilot controller`, originally at
`a94b816`, now locally updated through
`d4af012f7fc86b419ee68545d5ceecc2e3f4034d` (Wind handling in
`2ae1297247109759abdac4494339345f3fd3d9c2`, late-echo follow-up in `d4af012f`).
These commits have not been pushed or flashed. Its `BridgeCore.cpp` already decodes the explicit field-6 mode keys
in addressed PGN126208: Track `0x80`, Wind `0x01`. It emits one SeaTalk key per
explicit mode request and requires subsequent physical `0x84` feedback. It does
not infer successful mode changes from serial transmission or a wire echo.

SKAGER's `EncodeSt4000Command` already represented those two modes, but both
`OpenCPNPilot::Send` and the patched serial driver's final allowlist rejected
them. The new exact allowlists admit the two existing encodings without opening
a general message or heading-command output path. Startup/session OFF, selected
bidirectional interface, observed pilot identity, connection generation, current
physical feedback, rate limits, one pending command and exact-write confirmation
remain required.

## Track is the physical pilot's mode

The inspected AutoTrack plugin's Tracking feature runs a software steering loop
and sends target headings while the physical pilot remains in AUTO. That is not
evidence that the controller has accepted native TRACK. SKAGER uses the physical
pilot's Track mode, not a copied AutoTrack loop. SmartNav remains advisory and
cannot invoke the control path.

The operator must explicitly select AUTO and wait for measured AUTO feedback
before requesting TRACK. A confirmed TRACK state permits another explicit TRACK
press when the pilot requires waypoint-turn confirmation; the software does not
automatically repeat or acknowledge a turn. An accepted transport request stays
pending until later physical mode feedback meets the existing confirmation rules.
If the pilot was already in TRACK when the key was requested, another unchanged
TRACK report cannot prove that its waypoint-turn prompt was accepted. That request
remains unconfirmed and expires with an explicit physical-helm instruction; the
software neither invents a turn acknowledgement nor repeats the key.

Native Track also needs fresh navigation at the controller: PGN129283 cross-track
error, PGN129284 bearing/range and a changing destination identity. True bearings
require the controller's separately observed fresh magnetic variation; a retained
desktop `gVar` value without observation age is not a substitute. Missing or
expired route/position/variation information must withhold eligibility, not extend
freshness limits. SKAGER can establish only its local route-output eligibility;
the bridge independently checks navigation and variation before emitting a key.

### Guarded route feed

The integration observes normal OpenCPN route processing and reuses its existing
PGN129285/129284/129283 serializers, in that order, through a bounded in-memory
collector. It does not trigger route processing or calculate its own guidance.
The accepted route copy includes the virtual activation origin as well as route
revision, active point and original selected-position observation. Edits,
reactivation, reversal, skips, position changes and expiry invalidate local
eligibility until another coherent normal pass. Rapid position updates may
conservatively disable TRACK between such passes; transport timing remains
unqualified.

The pinned serializers otherwise hard-code waypoint identities `0`/`1`. The
new overloads retain the old entry points and accept matching bounded identities
for the two route/navigation messages. `/OpenNav/Pilot/NextNavigationWaypointToken`
reserves and flushes each pair before use, without wrapping or reuse on restart.
A corrupt/exhausted counter or failed flush withholds output. The 16-bit range
is limited by PGN129285; exhaustion requires operator review. The pinned
PGN129284 setter accepts 8-bit identifiers despite 32-bit wire fields, so the
overload writes only those two defined fields after serialization. The guarded
path sends unavailable ETA/closing velocity rather than evaluating unaged
SOG/COG globals. True-bearing and cross-track semantics remain OpenCPN's.

The selected pilot interface is reserved from stock automatic N2K route output
in XNav. This does not redirect stock output to a different interface. A separate
latest-only serial slot validates the exact three-message shape, limits the
encoded batch to 512 bytes, and binds it to the enabled session, connection epoch,
original position age and a 500 ms queue residence. Disable, disconnect, changed
route or failed/partial write clears the slot and pending route-bound TRACK.
STANDBY remains independent. A queued TRACK key retains the original navigation
receipt's token and expiry; later same-leg output cannot renew that request.

A complete local write is a distinct receipt, **not** proof of bridge delivery,
usable magnetic variation or physical TRACK acceptance. The command still needs
later measured pilot feedback. No automatic second key or waypoint-turn
confirmation is generated. Legacy/Safe retain stock serializer entry points and
selection behavior. Third-party plugin output is not intercepted by this guard;
do not run AutoTrack steering and SKAGER manual control concurrently.

## Wind uses measured apparent wind

The controller only selects its initial NMEA2000 wind source from a complete,
valid, SeaTalk-representable APPARENT PGN130306 observation. An unavailable first
sender cannot reserve the source. Once selected, invalid values clear both wind
components without silently switching source.

Before a NMEA-backed WIND request, both forwarded SeaTalk wind angle and speed
must have successful recent echoes tied to the original measurement age and
source. Initial zero timestamps do not count as successful transmission. Invalid
data, failed/aborted forwarding, CAN loss and listen-only transitions invalidate
receipts. Late echoes cannot re-authorize invalidated data. Fresh independently
received SeaTalk wind remains a separate source path; local echo must never
manufacture a fresh instrument observation.

The receiver retains one failed/aborted own wind-angle packet until its queued
captures drain and the existing echo deadline/idle boundary and decoder reset
are satisfied. Identical real instrument packets cannot be distinguished from
that old echo and are withheld during the guard; a persistent receive backlog
can prolong the hold. This deliberately favors unavailable wind over invented
freshness. Actual RMT capture scheduling still needs later hardware validation.

No true-wind estimate, weather forecast or simulation substitutes for apparent
wind. The physical pilot still decides whether to accept WIND. `publishRudder`
remains false; the patch adds no invented rudder feedback or automatic retry.

## Operator feedback and pending qualification

The drawer distinguishes a rejected request from a timeout and from measured
confirmation. Local permission, data or transport refusal is not described as
"refused by the pilot". Failure messages must not imply that a previously sent
command was cancelled or that physical STANDBY is unnecessary.

Source changes and revised regression cases are unexecuted. Added source cases
cover mode prerequisites, exact payloads, route tokens, source/session/queue
expiry, local-write receipts and repeated TRACK without a turn acknowledgement.
Remaining acceptance includes the new controller revision's parser/echo guards,
serial queue checks, Linux and native Windows integration, route transitions and
invalid data, and separately authorized physical Track/Wind commissioning. The
old six-command evidence and pinned controller oracle remain historical
evidence; they do not qualify these additions.
