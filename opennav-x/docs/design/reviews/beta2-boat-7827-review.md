# Boat display review — 7827acb

Work in progress; not Beta 2 acceptance. Exact fixture-free installed commit:
`7827acb7c8b0d708285bd26a4c48d545dd64d139`, executable SHA-256
`56f435d08904ec5a0b8a840fe13625b9476353e7c97a8d2368f16c100b5672e5`.
The 09:14 UTC launch uses the actual shared profile/licensed charts under the
reviewed input-only commissioning transaction. No physical command is sent.

Native screenshots measure 1280×800 at DPI 144, on a 1920×1080 Windows desktop.
This is actual boat-PC window validation, not physical 1280×800-panel or touch
acceptance. Screenshot originals and exact before/after receipts stay private
because they contain licensed chart content, positions and saved object names.

Reference intent: approved design board, chart dominance, four clear rail values,
restrained surfaces and meaningful color. Observations so far:

| Screen | Observed result | Limit / next review |
| --- | --- | --- |
| Navigation Day | Actual channel/shoreline/features/ownship; three zoom-out steps preserve detail. Center icon and label clear. SOG/depth/wind/estimated heading all remain within the rail. Bundled floating Dashboard is absent. | Mode round-trip, pan and follow checks pending. |
| Navigation Dusk | Correct Dusk caption; XNav and chart dim coherently, four values remain readable. | First immediate action image preceded the palette repaint; separate settled capture reviewed. |
| Navigation Night | Correct Night caption; dark chart and low-light XNav/Windows caption, no bright sheet. | Chart palette is the existing chart renderer's behavior. No claim of underway legibility. |
| Menu Night | Direct, large action grid and Back; no cascading menu or Demo controls. | Physical touch pending. |
| Instruments Night | Navigation, wind and condition groups; complete values/units/quality reached with explicit scrolling. Real STW zero remains distinct from unavailable COG/true wind/pressure/rudder. | At 150% DPI expanded groups intentionally scroll; permanent rail does not. |
| Energy Night | Battery/propulsion/destination hierarchy clear; missing motor/pack values and arrival/range withheld. Lower range/drive/regeneration state reached by scrolling. | No real pack/motor data or configured pack identity observed. |
| Autopilot Night | Revised feedback-requirement wording, status unavailable, no invented target/actual heading/rudder. | Display-only inspection; no command, enablement or discovery actions. Lower panel still to review. |
| Settings / Sensors Night | Seven plain-language categories; connection health distinguishes available GPS/heading/depth/wind/STW/water/tanks from missing rudder/motor/battery/AIS. No Signal K option or PGN jargon in the normal view. | No connection edits or reconnection acceptance inferred. |
| System / Diagnostics Night | System is a full page, with no overlapping popup. Diagnostics states Beta 2 / 0.4.0-beta2, exact build, actual shared profile, DPI 144 and INSTALLED PRODUCT. Route/energy reasons and per-item validity/age remain technical and explicit. | Naturally occurring active-alert coexistence still to review. |
| AIS Night | Clear no-targets-received message; no simulated target or implied receiver health. | Actual AIS reception remains unverified. |
| Anchor Night | WATCH OFF and unavailable distance/radius; dark surfaces and readable labels. | No watch created/removed; active-watch feedback cases remain conditional. |
| Passage Night | No active route/waypoint/distance/timing; no valid-zero arrival. | Actual active passage is not exercised during this read-only dockside review. |

Technical observations are recorded separately from design judgments. The
09:38 diagnostic copy identifies the actual installed product, fixture flag
false, six received PGN families (127250, 127505, 128259, 128267, 130306, 130310),
estimated magnetic-plus-variation heading, and control disabled/command ID zero.
No real motor, battery or pilot feedback was observed. Source identities, exact
positions and chart filenames are excluded from public evidence.

Remaining screen rows, mode lifecycle and maintenance must be recorded before
this review can contribute to a final handoff.
