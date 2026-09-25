# OpenNav X Beta 1 boat commissioning

Physical acceptance remains open. Match the package build to its published
software qualification record before beginning; this checklist is not evidence
that any boat test has passed.
Record the exact application/bridge versions and hashes used for each test.
Desktop simulation does not certify navigation or steering. Keep normal vessel
instruments and physical STANDBY available. SmartNav is advice only.

## Phase A — dockside, read-only

1. Back up the existing OpenCPN profile. Begin with the isolated portable build
   and **autopilot control OFF**. Confirm LIVE rather than DEMO/REPLAY mode.
2. Configure the existing OpenCPN marine connection in Advanced settings. Check
   its interface and actual device identities in OpenNav source diagnostics.
3. Compare GPS, true/magnetic heading reference, STW, wind reference, measured
   transducer depth, water temperature and rudder with independent instruments.
   Confirm units, instances and age. No rudder sensor means unavailable.
4. Check fresh/waste/other tank instances. The inspected propulsion firmware's
   virtual fuel tank is SOC, not physical fuel; configure its explicit mapping.
5. Verify AIS target identity, bearing/range, CPA/TCPA and loss of input. OpenNav
   uses OpenCPN calculations; absence of a target or alarm does not mean safety.
6. Stop one sensor at a time, then the gateway. Verify LIVE → AGING → STALE or
   explicit INVALID/UNAVAILABLE. Dependent advice/estimates must disappear.
   Restore input and confirm real new timestamps, not a UI-triggered refresh.
7. Inspect **producer sensor expiry**: the bridge must stop presenting retained
   EV data as fresh N2K values when its sensor input disappears. A network-only
   unplug test is insufficient. See [inspection](beta-boat-source-inspection.md).
   The [reviewed producer expiry patch](boat-propulsion-contract.md) is available
   as source and has a full C6 compile gate. Firmware flashing remains deliberate
   and separate from installing OpenNav. Configure the exact observed boat bridge
   NAME in Data Sources. v1 data stays uncertain; verify v2 per-group loss and
   recovery before trusting dependent energy estimates.

## Phase B — propulsion and calibration

With no autopilot commands, compare SOC, V, A, kW, RPM and temperature against the
pack/controller. Confirm pack identity, charging/discharging current sign and
whether the shunt includes auxiliary loads. Cross-check 127751 HV readings;
127508 cannot carry the full high-voltage range in the pinned codec. SOH stays
unavailable unless genuinely provided. Explicitly verify the configured motor
versus coolant temperature meaning and regeneration extension.

Record bounded normalized sessions with time, speed reference and conditions.
Collect steady speeds only in suitable supervised conditions; no prescribed
speed is safe for all locations. Review data before importing a speed/power
curve. Set measured usable capacity, reserve and auxiliary/efficiency assumptions.
Compare estimated consumption and arrival SOC against actual results over several
trips. The estimate is advisory, not a guaranteed range.

## Phase C — autopilot status only

Leave control OFF. Identify the translator by interface and device identity.
In **Menu → Manual autopilot → Translator configuration**, bind the exact
observed interface and hexadecimal NAME. Saving identity always selects
display-only. The currently qualified PC output path is a bidirectional OpenCPN
TCP connection using Actisense complete-PGN ASCII. Serial, UDP and SeaSmart
remain status-only in this Beta integration; do not assume physical delivery
from a transport description. Confirm the complete gateway path before commands.
Compare reported STANDBY/AUTO and actual/commanded magnetic heading with the
physical ST4000. Change state **at the physical pilot** and verify fresh feedback
at the PC. Disconnect communications: PC state must become unavailable/stale.
Repeated outgoing messages are not proof of physical feedback. TRACK/WIND remain
unavailable until that complete path is specifically validated.

## Phase D — deliberate manual commands

Only after status tests pass, in a secured vessel/safe environment with a person
at physical STANDBY and the drive safe for testing:

1. In translator configuration, deliberately save manual-control permission for
   that verified identity. Return to the panel and enable this session. Saved
   permission alone never starts an enabled session.
2. Test STANDBY alone and compare physical feedback.
3. Test AUTO alone; verify target/actual heading, physical mode and confirmation.
4. Test +1, −1, +10 and −10 individually. Wait for physical feedback and the
   PC's CONFIRMED state after each; stop on rejection, mismatch or timeout.
5. Return to STANDBY between groups. Test communication loss and command rejection
   without creating an unsafe vessel state. There must be no automatic retries
   or command storm from repeated touches.
6. Disable control and confirm further PC commands are refused. Safe/Legacy and
   replay must not retain an active XNav control session.

Do not test TRACK/WIND until the supported path, navigation freshness, magnetic
variation and acknowledgement behavior are understood and independently accepted.
Do not combine initial status, command and underway tests into one first run.

## Phase E — supervised underway validation

Use an appropriate safe area and a human navigator. Compare active route progress,
waypoint changes, turn timing, energy prediction, AIS encounters and alarms with
OpenCPN/independent instruments. Separate measured depth below the transducer
from charted depth and datum uncertainty. No detected chart hazard does not mean
a safe corridor. Test safe loss/recovery scenarios one at a time. SmartNav never
initiates steering; a human must choose and manually request any command.

## Radar and reporting

If the radar adapter is unavailable, record its exact missing plugin/hardware
path; no synthetic radar in live mode. With a supported source, verify detection,
receive/display and only explicitly supported controls separately.

Save versions, source-health records, recent events, command lifecycle and
screenshots. Use **Menu → Field diagnostic bundle → Export Diagnostic Bundle**.
The separate recording action requires explicit selection and consent; the
default export withholds positions and device identities.
[Bundle contents and privacy](field-diagnostic-bundle.md).
Export only the diagnostic bundle and explicitly selected recording
needed for the report; review navigation/location contents before sharing.
Record PASS/FAIL/NOT TESTED for every step, failures and exact recovery procedure.
No step is accepted merely because its procedure exists.
