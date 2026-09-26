# Beta 2 — Product workflow and screen review

Status: implementation ready for iterative native review; no boat-PC acceptance
claimed until a compatible installation has passed preflight and deployment.

| Screen | Reference intent / Beta 1 issue | Implemented refinement | Required evidence |
| --- | --- | --- | --- |
| Instruments | Related values, rather than identical gauge boxes | Shared Navigation/Wind/Conditions groups, quieter labels, large values; configured energy/tanks remain optional groups | Four selected nav values, unavailable/zero/stale, Night and 150% |
| Pilot | Large commanded heading, clear small number of manual controls | Heading/actual/rudder group; ±1/±10, large STANDBY/AUTO; disabled unsupported and control-OFF actions; logs in Advanced | Isolated feedback/timeout test, no physical output, all DPI |
| AIS | Vessel motion and approach in one coherent card | SOG/COG/CPA/TCPA group, relative range/bearing/heading, chart selection and expiring state | Real upstream target fixture and live received target when available |
| Anchor | Useful watch state and conditions without a coordinate log dominating | Distance/radius relationship, upstream alarm state, depth/wind/battery group, optional recorded positions | Own watch mark cleanup/icon/name regression and stale GPS |
| Alerts | Persistent actionable conditions | Semantic accent, title/action/acknowledgement card; inspect condition; unresolved state retained | Warning/critical overlap and recurrence checks |
| Settings | Seven understandable categories | Vessel, Navigation, Sensors, Autopilot, Radar, Display, System | Category/back paths and no dead controls |
| Sensors | Connection health before protocol details | GPS and sensor status buttons; source/PGN detail only after selection; separate advanced boat mapping | Live connection reconfiguration and chart preservation |
| Data rail selection | Primary values always visible | Four-item presets, reorder controls; old extras preserved under Instruments during edits | Alert and 100/125/150% rail geometry |

Every product page has the same Back behavior. Escape returns to the logical
parent without swallowing text editing or the active modal sheet. OpenCPN
continues to validate real route/mark mutations. The normal UI removes terms
such as snapshot/provider/source revision; they remain appropriate in technical
diagnostics. Signal K custom-mapping import is removed from normal Sensors,
while existing stored mappings and upstream Legacy capabilities are preserved.

Painted regions and controls expose actual native screen geometry for tests.
The old identical-value-box test is retained as a meaningful minimum numeric
region check and supplemented with visible grouped-region bounds. Autopilot
OFF-state tests now assert disabled controls and no request/output, replacing
the former behavior that opened a confirmation only to reject the command.
