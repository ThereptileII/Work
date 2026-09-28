# Screen and state inventory — design suite v8

All paths below are available in `index.html`. Shared sheets, forms, confirmations and return controls are reused throughout. This maps visual design coverage, not implemented hardware capabilities.

| Area | UI coverage |
| --- | --- |
| Main helm | Day/dusk/night; chart; vessel/vector; configurable rail; navigation horizon; source status; alerts. |
| Chart | Pan, zoom, follow, orientations; vector/raster concept; soundings, contours, corridor, wind, AIS, radar; object/position sheets, Go To and measurement. |
| Chart symbols | 29 initial point/line/pattern examples with original theme-aware vector artwork; labelled reference pins for unmapped atlas definitions; selectable with pointer or keyboard; meaning/position/source sheet; nested atlas reference; placement from any library definition; Cancel, return to origin and removal of added examples; layer and label switches; day/dusk/night and upright point glyphs during zoom/rotation. |
| Symbol atlas | Settings → Navigation, Chart layers and Help entry points; 12 bilingual seamark examples; IALA A/B; 1,107 source definitions across 13 categories; day/dusk/night; search and empty state; type filter; 36-item pagination; detail and implementation metadata; full/individual JSON export; source/coverage dialog. |
| Lighthouse sectors | Compact R/W/G arcs; hover/focus extension; click/tap/Enter to pin; single Collapse control, second selection, blank-chart click and Escape to clear; Details opens the shared object sheet; world-space sector bearings follow chart rotation and zoom. |
| Waypoints / passages | Create/edit/delete with confirmation; graphical plotting, undo, naming, save/activate, append/edit/remove, reverse, end and reference-route restore. |
| Passage library | Saved route/places, empty state, sample GPX import review and mock GPX export. |
| SmartNav / horizon | Next turn/course/time, ETA, corridor/hazard information, traffic, arrival energy and quality-dependent unavailable states. Advisory only. |
| AIS | Target list, risk/range sort, details, MMSI, SOG/COG, range/bearing, CPA/TCPA, chart highlight and return to card/list. |
| Instruments | Wind/heading dial, SOG/COG/STW, depth, apparent/true wind, heel, water temperature, freshness; four configurable rail slots. |
| Propulsion / energy | RPM, gear, temperature, voltage/current, power, SOC/SOH, regeneration, capacity/reserve, range/arrival SOC, speed/power exploration and calibration. |
| Autopilot | Adapter/connection/timeout, capabilities, control off/enable, Standby/Auto, heading increments, pending/acknowledged/timeout; unsupported modes disabled. |
| Radar | Standby, full scope, chart overlay, synthetic coastal/target/clutter returns, sweep, range, gain, sea/rain suppression, opacity, pause and guard sector. |
| Anchor | Set position, radius/distance, swing graphic, example track, depth/wind/battery context, armed/disarmed watch. |
| Source network | Inventory, per-signal health, message/PGN, cadence, priority, connection details, calibration, disconnect/reconnect and remove confirmation. |
| Add sensor | NMEA 2000, NMEA 0183 and Signal K; endpoint/serial settings; discovery; error/retry/manual entry; measurement/priority; verified sample and completion. |
| Alerts | Notification centre, advisory/critical states, persistent critical banner, acknowledgement, contextual action, GPS loss, battery stale, command timeout; CPA/TCPA/depth/energy thresholds. |
| Charts | Package list, coverage detail, example import/validation, update/verify, removal confirmation and license boundary. |
| Vessel / navigation | Name, draft, safety depth, capacity/reserve, calibration, units, corridor, route preference, presentation and libraries. |
| Display | Day/dusk/night, scale preference, balanced/chart/instrument layouts, rail customisation and fullscreen request. |
| Software centre | Installation/recovery, updates, backups, diagnostics, plugins, help, About and first-run setup. |
| Installation guide | Prerequisites, supported host, options, verification/backup, first launch and recovery; available inside setup. |
| Installer | Welcome, supported/unknown/missing host, validated progression, destination/shortcuts, review, progress, cancellation/rollback, self-test and completion. |
| First-run setup | Vessel, units/display, sources and nested sensor wizard, battery/reserve, control off, summary and launch. |
| Maintenance | Repair, rollback and uninstall: review, progress, completion and original-host/data preservation. |
| Workspace modes | OpenNav X, Legacy and Safe Mode, restart explanation/confirmation, shared navigation data and recovery intent. |
| Updates | Stable/Beta, checking, available release/notes, background download, verified cache, install review, completion/history and recovery. One contextual Back replaces the extra Later/return action. |
| Backup / new computer | Section selection/exclusions, JSON export, import validation/error, review/restore, pre-restore recovery point and sample restore. |
| Diagnostics | Overview, build/display/source/adapter state, logs/export, session capture/export, replay badge and source-loss reproduction. |
| Plugins | Adapter/dashboard/weather inventory, compatibility, enable/disable and generic autopilot adapter settings. |
| Help / notices | Searchable guides, no-results state, keyboard/touch references, About, version, support-bundle path and license notice. |

## Navigation contract

- Each active page/pane has one local return action. Root sheets/views use Close; nested ones use Back with an accessible parent label.
- Wizards have a single footer return: Cancel on entry, Back between editable steps, Done after completion. Running operations use their cancellation action.
- Dialogs with an explicit Cancel/Keep action omit header Back/Close. Other dialogs have one Close or nested Back. Prior dialogs and uncommitted fields are retained.
- Chart inspections return to their initiating context. Chart tools temporarily replace that return with Cancel/Done.
- Escape/Alt+Left follow the same contextual return. Finished/cancelled wizard history is removed.
- Main navigation stays available. `NAVIGATION-AUDIT.md` records the decisions for every screen family.

## Production boundaries

Pathfinder integration, radar/AIS correlation, intelligent anchor-drag analysis and unverified autopilot modes remain capability descriptions. Cryptographic verification, transactional installation, compatibility manifests, crash recovery and signed updates are represented by UI states/guide content; no executables are used. Geography, calculations and signals are mock data. Production needs its complete alarm and recovery/error matrix tested against real integrations.
