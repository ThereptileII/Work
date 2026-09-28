# OpenNav X — A clearer course

Standalone interactive design reference for the future OpenNav X / OpenCPN application. **Design suite v8 · 28 September 2026.**

## Open

Open **index.html** in a modern browser. Styles, chart artwork, radar renderer and application code are embedded. No installation, account, external assets or connection is required.

Primary reference: **1280 × 800**. Compact layouts accommodate 1024 × 640 and 854 × 533 CSS viewports. Mobile uses an instrument strip and bottom navigation; **More** opens settings.

Every reading, chart, vessel, forecast, release, installer operation and hardware interaction is simulated. This is a software design template, not an operational navigation application.

## Explore

- **Chart / Passage / Traffic:** pan, zoom, follow, orientations, layers, objects, measure, Go To, waypoints, graphical route creation/editing, horizon events, AIS details and chart highlighting.
- **Långskär lighthouse:** a minimal light point with short red/white/green sectors. Hover or keyboard focus extends them temporarily; click, tap or Enter pins the longer sectors on the chart. Select again, use the single Collapse control, click open chart space or press Escape to collapse. Details remain available from the compact selection card. Sectors keep their chart bearings through rotation and zoom; all values are fictional.
- **Symbols on the chart:** 29 selectable examples use original, theme-aware SVG artwork: fine stems, restrained topmarks and muted colours. They include lateral and cardinal marks, lights, rocks, wrecks, harbour features, a cable and a fishing-area pattern. Select a chart object for its meaning and atlas reference; the lighthouse first expands its sectors and offers a Details action. From any atlas glyph, choose **Place on demo chart** and tap a position (definitions without a designed chart variant use a labelled reference pin); added examples can be removed from their detail sheet. Chart layers controls symbol visibility and optional Swedish labels. Placements last for the page session and are excluded from configuration backups.
- **Energy / Instruments / Anchor:** focused views, configurable primary values, consumption/reserve exploration, calibration and anchor watch.
- **Radar:** use the chart radar icon for **simulated echoes**, or Radar for the full scope. Coastline returns, vessel clusters and clutter follow a 24 rpm sweep. Range, gain, sea/rain suppression, opacity, guard sector and pause are interactive. Imagery is generated from the fictional scene, not a recording or live feed.
- **Settings → Sensors:** manage sources; add a sensor through connection/discovery/assignment/verification; failed discovery, retry and manual entry; priority, calibration, disconnect/reconnect and removal.
- **Settings → Navigation:** presentation, chart-symbol atlas, charts and coverage, alarms and passage library. Includes sample GPX import and downloadable mock-GPX export.
- **Chart symbols & seamarks:** a twelve-example Swedish/English IALA guide, region A/B switch, and the complete pinned OpenCPN resource catalogue: 1,018 points, 59 lines and 30 patterns. Search, filter, inspect day/dusk/night variants and export definitions. Also available from Chart layers and Help. See `SYMBOLS.md` for source, license and portrayal scope.
- **Settings → Autopilot:** generic adapter, capabilities and acknowledgement settings. Control starts off; explicit enablement is required. Pending, success and timeout states are demonstrated.
- **Settings → System → Installation & recovery:** installation guide; six-step installer; host compatibility checks; folder/shortcut options, review, progress, cancellation and self-test; repair, rollback, uninstall, Legacy and Safe modes.
- **Run vessel setup:** vessel, display, sensors, battery/reserve, helm control and final summary. Nested sensor screens return to the correct setup step.
- **Updates:** Stable/Beta, check, download, verified package, review, installation history and recovery.
- **Backups:** section selection, JSON export, validated import, review, restore and prior-configuration recovery point. Charts/licenses are excluded. Accepts v1 and v2 design backups.
- **Diagnostics:** source health/cadence, logs, mock recording/export, explicitly labelled replay, dropouts and local diagnostic bundle.
- **Help:** searchable installation, sensor, navigation, radar and recovery guides; keyboard/touch guidance; About and license notices.

## Returning through the interface

Each context has **one return action**: **Close** for a root sheet/view, **Back** for a nested page, or **Cancel / Back / Done** in a wizard footer. Confirmations use one explicit Cancel/Keep action without an extra header dismiss button. **Escape** and **Alt + Left** follow the same return behavior, preserving parent tabs, form values and scroll position. **Chart** remains the main workspace destination. See `NAVIGATION-AUDIT.md` for the page-by-page decisions.

Preferences last for the page session. Export a backup to retain demonstrated configuration. Some preferences, such as units, are stored without converting every reference instrument label.

## Handoff

- `index.html` — self-contained preview.
- `UI-INVENTORY.md` — screen paths and state coverage.
- `NAVIGATION-AUDIT.md` — return-control decisions for every page/pane family.
- `DESIGN.md` — direction, original research and production boundaries.
- `design-tokens.json` — palettes, geometry, type and motion.
- `QA.md` — checks performed and remaining validation.
- `SYMBOLS.md` — chart-symbol scope, provenance, attribution and production handoff.
- `screenshots/v8/` — compact and expanded lighthouse sectors; `screenshots/v7/` retains the previous marker artwork; `screenshots/v6/` retains the previous chart pass; `screenshots/v5/` retains atlas layouts and `screenshots/v4/` retains the navigation audit.
- `src/` — shell, core/chart styles and logic, navigation history, software suite, health, backup validation and radar renderer.
- `vendor/opencpn/` — pinned source symbol assets, license and provenance.
- `src/symbol-catalogue.json`, `src/seamarks.json` — generated symbol data and editable bilingual guide.
- `src/chart-symbols.json`, `src/chart-marker-art.js`, `src/chart-symbols.js`, `src/chart-symbols.css`, `src/light-sectors.js` — editable fictional placements, chart rendering, selection and placement tools.
- `build.mjs`, `build-lib.mjs`, `symbol-library.mjs`, `verify.mjs` — dependency-free build/import/checks.
- `archive/` — original design files, with retired hardware naming removed from text.

```
node build.mjs
node verify.mjs
```

Node is only needed to rebuild or verify source. Production must use OpenCPN chart/navigation models, licensed data, validated adapters and a real installer/updater.
