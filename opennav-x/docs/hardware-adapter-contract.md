# Hardware-adapter contract

The portable adapter library depends only on Vessel Data. SmartNav does not
link it. UI manual actions are the only intended caller of `ManualAutopilot`.
Safe and Legacy do not instantiate OpenNav control components.

## Autopilot

`IAutopilot` separates capabilities, observed state, explicit polling and send.
`ManualAutopilot` defaults globally disabled. The accepted Alpha permits commands only to an
explicitly enabled simulator; there is no live hardware transmitter. The live
placeholder reports unavailable. Capability bits gate STANDBY/AUTO/TRACK/WIND
and ±1/±10 course changes. Course changes require confirmed AUTO and fresh
locked magnetic heading. AUTO requires fresh measured magnetic heading.

Beta adds the separately tested [ST4000 protocol boundary](st4000-beta-contract.md).
The live application still uses the unavailable placeholder until transport and
native interaction qualification. The common controller now records requested
state, bounds repeated send attempts to 250 ms and requires a new locked-heading
observation for course confirmation. Changing device/connection cannot acknowledge
an old command. No SmartNav output path is added.

Only one command may be pending. STANDBY can supersede it and can be attempted
with stale feedback. Transport acceptance means pending, never successful mode
change. Success needs a subsequent fresh feedback sequence, later than the
request, matching the requested mode/heading. Timeout at three seconds reports
unknown outcome, with no automatic retry. Late feedback remains observed truth
but cannot retroactively acknowledge an expired request. Disabling control does
not claim to cancel an already transmitted command; physical STANDBY remains
necessary if its outcome is uncertain. The last 128 command transitions are
retained for diagnostics. UI confirmation is a separate manual interaction gate.

The deterministic simulator publishes feedback after 500 ms and periodic state
at one second. Reading state never refreshes it. Rejection and communication-loss
scenarios are controllable. Simulated TRACK/WIND exercise capabilities only;
they are not evidence of physical navigation control.

The inspected ESP32/ST4000 path and required boat tests are recorded in
[source inspection](alpha1-source-inspection.md) and
[physical validation](physical-validation.md). Real commands require a separately
validated transport and fresh physical pilot feedback; transmitted command echo
or NMEA group acknowledgement is not sufficient. No Leaf/SeaTalk wire details
are embedded in XNav UI.

## Radar

`IRadar` exposes availability, capabilities and presentation (Off/Overlay/Focus).
The live placeholder reports unavailable and refuses active presentation. The
status simulator exercises capability/disconnection transitions, clearly marks
DEMO, and generates no radar echoes. Presentation changes do not renew source
time. Existing plugin integration remains a separate gate; no custom Pathfinder
control or radar/AIS fusion is claimed.

## Tests

`autopilot_manual_feedback`, `autopilot_failures`, `autopilot_ack_evidence` and
`radar_capability_contract` cover defaults, capabilities, measured feedback,
heading wrap, one pending request, rejected transmission, exact timeout, late
feedback, STANDBY preemption, global disable and no automatic retransmission.
They require no physical hardware. Native UI qualification passed at `7bc36e4`;
physical tests remain open. Final packaged acceptance is recorded in [status](status.md).

## Alpha UI integration

The integration session owns separate unavailable-live and simulated manual
controllers. XNav receives copied feedback, capabilities and command log entries.
Leaving Demo disables that simulator controller; normal restart reconstructs
both with control disabled. The pilot sheet requires explicit simulator enable,
confirms mode requests, shows pending/confirmed/rejected/timeout state and keeps
STANDBY directly accessible. No UI request reaches live hardware in this Alpha
foundation. Windows automation exercises enable, AUTO, +1, STANDBY and disable,
checking new-feedback confirmation rather than the button press alone.

[Native qualification](evidence/alpha-installer-7bc36e4-qualification.json)
includes the actual simulator enable, feedback confirmation and disable flow.
This establishes the desktop foundation only; no physical pilot/radar command
path or boat acceptance is implied.
