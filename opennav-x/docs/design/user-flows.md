# Beta 2 interaction flows

These are implemented target flows, pending exact-build native and boat-display
validation. Counts below include a tap/long press or confirmation as one action;
typing a name is noted separately. Hardware-control actions are never part of
remote boat smoke tests.

| Flow | Starting state and actions | Interactions | Back/cancellation | Error behavior |
| --- | --- | --- | --- | --- |
| Go To | Chart position long press/right click → Go To → Start | 3 | Outside/Escape closes position card; Cancel closes confirmation | Missing position, protected/shared changes or invalid destination are rejected by the integration; no partial independent route |
| Create waypoint | Chart position long press/right click → Waypoint → name → Save | 3 + text | Cancel writes nothing | Storage/position errors remain visible; no invented coordinates |
| Edit waypoint | Select saved mark → Edit waypoint → Save | 3 + text | Cancel retains original | Active/protected/shared cases are read-only where the integration cannot safely edit |
| Waypoint Go To | Selected waypoint → GO TO → START | 2 after selection | Cancel retains navigation | Integration revalidates waypoint and selected position |
| Remove waypoint | Selected mark → Delete waypoint → confirm | 2 after selection | Cancel preserves mark | Shared/protected marks remain disabled; no cascade deletion |
| Create route | Menu → Routes → Create route on chart → tap points → Done → name/save | 5 + points/name | Undo removes last draft point; Cancel in naming retains the draft; chart Cancel discards the draft after confirmation | OpenCPN owns the draft and validation; details remain reviewable before activation |
| Activate route | Routes → select route → Activate route → confirm | 3 after list | Back returns to Routes | Revision/state revalidated; activation is not an autopilot mode request |
| Stop route | Active route detail → Stop navigation → confirm | 2 | Cancel leaves passage active | No steering command; normal OpenCPN route output semantics remain unchanged |
| Select AIS | Tap received target → compact card | 1 | Close/outside returns to chart | Expired/lost selection is cleared; no synthetic target substitution |
| Inspect AIS | Selected card → Details; or Menu → AIS targets → vessel | 1 or 3 | Back returns to targets | Lost target shows unavailable; CPA/TCPA remain upstream results |
| Chart orientation | Tap North/Course control beside Center | 1 | Chart remains visible | Label reads the actual OpenCPN selection; uses the existing North/Course action |
| Display mode | Tap current Day/Dusk/Night label | 1 per step | No modal | Label and chart/XNav palette change together |
| Instruments | Menu → Vessel instruments | 2 | Back → Menu; Navigation → chart | Stale/missing readings show their state; groups retain configured selections |
| Propulsion/energy | Energy action, or Menu → Propulsion & energy | 1 or 2 | Navigation returns to chart | Dependent predictions are withheld and their blocking reason shown |
| Acknowledge alert | Alerts → condition → Acknowledge | 2 after alert access | Back retains active alert | Acknowledgement affects XNav presentation only; unresolved critical conditions remain visible |
| Open autopilot | Pilot action, or Menu → Manual autopilot | 1 or 2 | Back → Menu | Display-only unless explicitly permitted and enabled for this session |
| STANDBY | Enabled live pilot → STBY in persistent bar or STANDBY in panel | 1 | No automatic retry or automatic follow-on command | Command remains pending until physical feedback; failure/timeout visible; physical STANDBY remains essential |
| XNav → Legacy | System → Open Legacy OpenCPN | 2 | OpenCPN may veto close during protected work | Shared profile saved through normal shutdown; no forced kill |
| Legacy → XNav | OpenNav interface entry → XNav | 2 | Existing confirmation/close rules | Restart uses the same profile/chart configuration |
| Safe Mode | System → Safe Mode | 2 | Controlled restart | OpenNav advisory/control modules remain disabled |
| Diagnostics | System → Diagnostics | 2 | Navigation returns to chart | Correct edition, build purpose and source ages distinguish product/test/replay |
| Sensors | Settings → SENSORS → quantity | 3 | Back returns to Sensors, then Settings | No-data/aging/stale states visible before opening technical details |
| Reorder rail | Settings → DISPLAY → Configure data rail → Move up/down | 4+ | Each change is saved; Back returns to Display | Up to four primary readings; older extra selections are preserved in Instruments when changing the rail |

## Shared interaction rules

- Back follows the logical parent. Home Back returns to navigation.
- Escape performs Back on a product page when focus is not editing text. A
  modal edit/confirmation sheet owns its own Escape/Cancel behavior.
- Outside/Escape dismisses a transient chart card. An edit sheet does not
  silently save or discard a route; use its explicit Save/Cancel controls.
- While creating a route, full-width Cancel/Undo/Done actions replace the three
  page-navigation buttons in the bottom row. The chart and rail retain their
  normal size, and Pilot/STBY/System remain accessible.
- Alerts use the reserved status area and do not reduce the rail viewport.
  Acknowledgement never means the underlying condition is resolved.
- Unsupported/missing callbacks produce disabled controls. Autopilot mode and
  course controls are disabled while control is OFF; STANDBY remains available
  with stale feedback only when an enabled adapter still supports the request.
- Source IDs, protocol fields, model assumptions and command logs belong in
  Advanced/Diagnostics. Normal Sensors begins with connection health.
- No normal installed page, launcher or shortcut exposes Demo. CI-only builds
  retain labelled deterministic scenarios for repeatable regression testing.

## Remaining acceptance work

`tools/smoke-user-flows.py` exercises actual mouse presses and name entry using
copied native control geometry. Its disposable profile has only a local,
input-only GPS connection; it asserts that the connection emits no output.
The Linux run covers orientation, card dismissal, chart-position waypoint
creation, selected-waypoint Go To and stop, route point entry/Undo, both levels
of cancellation, and named save. A read-only database audit verifies retained
waypoint identity and the saved route's point count. This is automated desktop
interaction evidence, not a physical touch-panel acceptance claim.

Capture before/after native screens and repeat these flows on the boat display
after compatibility preflight succeeds. The presence of a flow in this document
does not claim physical-device acceptance, touch acceptance or navigation approval.
