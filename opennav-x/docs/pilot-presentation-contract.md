# Manual pilot presentation

`PilotPresentation` copies display values and button availability from the
existing `ManualAutopilot::GetState` result. It owns no OpenCPN pointers,
transport, device or SmartNav object. Painting, opening, updating or closing the
drawer cannot poll a driver, request a mode or transmit a command.

The existing integration tick still owns polling and timeout processing.
`OpenCPNPilot` receives the identity-bound 65379 mode, 65360 locked magnetic
heading and 127250 measured magnetic heading through OpenCPN observers. The
source/transport and `St4000Pilot` contracts are unchanged. No upstream hook or
new protocol path is introduced.

The large dial shows measured locked magnetic heading in confirmed AUTO and
measured actual magnetic heading otherwise. The label explicitly says M. It
never substitutes COG, true heading, requested heading or an optimistic increment.
Missing, invalid, estimated, future or stale headings suppress the dial value
and arrow. Mode needs the original fresh state, nonempty source, sequence and
observation younger than three seconds. The consumer does not refresh age.

Manual requests retain the established adapter gates, explicit session enable,
physical-helm warning, confirmation for AUTO/TRACK/WIND, repeat suppression,
single pending request and feedback acknowledgement. Unsupported modes remain
disabled. Pending requests suppress further course/mode requests; STANDBY stays
available when configured control permits it, including after feedback loss.
No UI success is inferred from transport acceptance. Timeouts explicitly direct
the user to the physical helm without automatically retrying a command.

The drawer uses the same guarded callbacks as the former page. Replay cannot
enable or command equipment. A read-only commissioning session is independently
blocked by both Shell and integration. Only the fixture build includes the
simulator-specific confirmation; the normal product has no simulator UI.

Four automated presentation cases cover measured/zero/missing headings,
confirmed modes, pending/timeout, seven invalid mode observations, eight invalid
heading states, permission revocation, disable access and replay. The dedicated
non-installed drawer executable uses callbacks with no equipment access. It
tests cancelled enable and mode confirmation, duplicate input, pending state,
measured acknowledgement, unsupported modes, stale feedback, replay and Close.
The existing adapter/transport tests remain unchanged and mandatory.
