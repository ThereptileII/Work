# OpenNav X design handoff

## Direction: a clearer course

The chart is the spatial anchor. Four large values answer the immediate questions: speed, depth, wind and energy. A persistent navigation timeline answers what comes next. Context sheets reveal detail without removing the surrounding chart.

Warm chart land and pale blue-green water sit within a dark graphite helm. Sea-glass green signals the vessel and route; amber flags a situation to review; muted magenta separates AIS traffic. Thin contours, restrained boundaries and large, lightly weighted numerals keep the view calm.

The work preserves the original chart-first structure, four-value rail, route/AIS views, propulsion, instruments, autopilot, anchor, settings and three display modes. It replaces the elliptical islands with original coastline artwork, redesigns the chrome and typography, moves upcoming events into the persistent horizon, and expands the original click-through into editable flows. Originals are retained in `archive/`.

## Research and interpretation

Official sources reviewed on 28 September 2026. Orca instrument imagery was also inspected visually in a browser. These inform principles; no product artwork or code is copied into the deliverable.

| Reference | Observed pattern | OpenNav X interpretation |
| --- | --- | --- |
| [Orca instrument panels](https://getorca.com/blog/orca_instrument_panels/) | Flexible layouts, split chart/instrument views and readable representations. | Four configurable essentials with focused instrument and energy views. |
| [Savvy Navvy interface guide](https://help.savvy-navvy.com/en/article/getting-started-navigating-the-savvy-navvy-interface-9rjxtf/) | Primary chart, direct manipulation, tap-for-detail, layers and measurement. | Contextual sheets and a compact floating chart-tool group. |
| [B&G Zeus S](https://www.bandg.com/en-gb/zeus-s/) | Contextual sailing modes, day/night presentation and setup assistance. | Anchor/instrument contexts, lower-luminance night styling and a vessel-first welcome flow. |

The **Your horizon** timeline is this concept’s defining element. A turn, an AIS encounter and projected arrival energy share a sequence, while remaining individually inspectable. This is a design proposal, not a claim that the production integrations already support those forecasts.

## Composition

At 1280 × 800: 68 px top bar, 80 px navigation, 186 px instrument rail, 132 px horizon and 34 px status footer. The chart receives the remaining area. The original viewport cap is replaced by an adaptive layout.

Context panels are 398 px wide, expanding to 432 px for configuration flows. Root panels place one Close beside the heading; nested panels use one Back on the left. Accessible labels/tooltips name the parent without an extra breadcrumb. The body scrolls independently; wizards keep their sole return control in the action footer. On narrow screens panels become nearly full-width sheets. Instruments move above the chart; More keeps additional controls reachable. Installation and vessel setup use a dedicated step rail and spacious main canvas. Radar focus uses the full workspace height to keep its complete scope visible.

The embedded vector chart uses forty deterministic fictional landforms, depth text, contours, lights and ship symbols. Place names borrow a Swedish setting; geometry and navigational content are not geographically accurate.

## Interaction and implementation coverage

| Area | Demonstrated | Production work |
| --- | --- | --- |
| Charts | Pan, zoom, follow, orientations, raster-style concept, layers, object sheets. | OpenCPN raster/vector rendering, licensed charts, projection and object queries. |
| Waypoints | Create, rename, reposition by chart coordinates, confirm deletion. | Geographic coordinate formats, drag editing, persistence and shared waypoint model. |
| Routes | Graphical plotting, undo, save, activate, reverse, end, append/edit/delete points and restore reference route. | OpenCPN route/leg calculations, XTE, ETA, turn anticipation and chart validation. |
| Go To / measure | Select a destination; example distance/bearing between chart points. | Geographic calculations using chart projection. |
| Horizon | Turn, traffic and arrival events; route-edit and data-loss states. | Real event scheduling, quality gating and deduplication. |
| SmartNav | Corridor and depth advice, source quality and unavailable state. | Actual look-ahead analysis and coverage validation; advice never steers the boat. |
| AIS | Four mock targets, sorting, highlighting, details and CPA/TCPA. | AIS reception, lifecycle and established OpenCPN alerts. |
| Instruments | Configurable four-value rail; wind, heading, depth, heel and water values. | N2K, 0183 and Signal K sources and freshness per value. |
| Energy | SOC, capacity, voltage/current, motor, reserve and speed/power exploration. | Calibrated vessel model, real signals and operational forecast validation. |
| Autopilot | Disabled by default; explicit enable, pending, acknowledgement, timeout and critical alert. | Validated adapter protocol, interlocks and status correlation. Track/Wind remain disabled until supported. |
| Radar | Synthetic returns ray-cast from the fictional coastline; vessel clusters, sea/rain clutter, 24 rpm sweep, focus/overlay, range/gain/suppression/opacity, pause and guard sector. | Real radar acquisition and OpenCPN integration; Pathfinder and radar/AIS fusion remain future work. |
| Anchor | Position, radius, watch, swing graphic, track and vessel context. | OpenCPN anchor alarm; intelligent drag detection remains future work. |
| Alerts / health | Priority model, persistent critical state, acknowledgement, per-source health and dropouts. | Complete alarm inventory, reconnect behaviour, real cadence and source precedence. |
| Setup / maintenance | Six-step installation with guide, compatible/unknown/missing host, options, review/progress/cancel/self-test; six-step vessel setup; repair/rollback/uninstall flows, Legacy/Safe modes, updates, plugins and help. | Actual installer, compatibility/hashes, backup, signatures, transactional updater and crash-loop recovery. |
| Diagnostics / backup | Downloadable mock JSON diagnostics/session, validated mock restore and labelled replay concept. | Real log collection, recording/replay, persistence, recovery and compatibility testing. Charts remain excluded from backup. |

## State contract

- Source quality is per measurement. A bus message alone does not establish that every sensor works.
- GPS loss removes turn/arrival predictions and dims the vessel as a last-known position. Battery staleness removes arrival-charge predictions. Critical alerts remain visible when another screen opens.
- An autopilot command is confirmed only after the mock acknowledgement. Timeout leaves the last confirmed heading unchanged and raises a persistent alert.
- Replay uses a technical-test label. The entire normal interface belongs to a marked design preview.
- Night mode changes surfaces, labels, instruments and chart tokens, with additional chart dimming. Onboard luminance still needs physical validation.
- Controls have accessible names. Each active context has one local return: Close for a root, Back for a child, and Cancel/Back/Done for the relevant wizard step. Dialogs use either a header return or an explicit Cancel/Keep action, never both. Escape and Alt+Left follow that context's return behavior. Nested pages/dialogs restore fields, parent tabs and scroll; root returns restore the opener's focus when available. Completed/cancelled flows discard obsolete history. See `NAVIGATION-AUDIT.md`. Reduced motion disables animation and holds the radar sweep.

## Scope and assumptions

This is an offline visual reference with no external runtime dependencies, licensed charts, real sensors, geolocation, autopilot connection or installer. The expanded screen inventory is mapped in `UI-INVENTORY.md`. Adapter-dependent and future requirements are represented by design states or descriptions where appropriate; production integrations are not implemented.

The reference scenario uses 18.2 nm remaining, 6.3 kn, 68% battery, 24.8 kWh usable capacity and 25% reserve. The illustrative power curve starts at 2.15 kW at 6.3 kn, yielding about 43% at arrival. The energy slider changes its forecast, not the vessel’s actual speed. Edited routes use example distances from fictional chart coordinates.

Preferences last for the page session unless exported. Version 2 mock backups cover selected vessel, navigation/display/alarm, sensor/adapter and route/waypoint sections. Charts/licenses remain excluded. Import validates the schema before presenting a review, and restore retains a recovery point. Some preferences, such as units, are stored while reference labels remain nautical. Physical touch/pinch, Windows DPI, accessibility compliance and hardware safety need separate validation.


## v5 — Chart-symbol atlas

The chart reference uses authentic source glyphs inside a quiet catalogue surface. Physical buoy illustrations are separated from chart symbols; Swedish/English guide names and region A/B mappings explain the common mark families. The complete pinned resource library, three original palettes and developer exports make the visual design useful for implementation. Navigation, coverage and licensing are documented in `SYMBOLS.md`. The main chart, heading-relative radar guard sector and Energy layout remain as in v4.
