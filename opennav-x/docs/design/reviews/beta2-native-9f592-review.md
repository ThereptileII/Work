# Retained native design review — 9f592

This is a design inspection of retained native Windows evidence, **not release
acceptance**. Candidate `9f59209914f57ff97af7e184b201752b8f422f0a` failed its
separate first DPI chart gesture. Its replacement and actual boat review remain
required. Reference: `docs/design/OpenNavX_Design_Reference.png`.

Artifact `10926137333` from run `36299835767` was downloaded and verified against
API/upload SHA-256 `ab9383eeb504db26c3f330aeebde0ab1cf0b3609a6687eaf6d5972b23583d95e`,
23,899,129 bytes and ZIP CRC. Twenty images below were individually inspected.
The `recovery-*` views use the actual fixture-free product; other application
views use explicitly isolated CI scenarios/loopback input. No image establishes
real boat data, physical touch, field GPU behavior or non-default DPI acceptance.

| Screen / images | Reference intent | Observation / disposition |
| --- | --- | --- |
| Navigation: `recovery-navigation-day.png`, `recovery-navigation-night.png` | Chart dominant, minimal chrome, four important values | Coastline remains visible; four rail values/units fit; no Demo launcher or synthetic readings in product. Day/Night caption matches palette. Night reduces emitted light. The CI recovery scene is a basemap, not the boat's licensed charts. |
| Menu/System: `recovery-menu.png`, `recovery-system.png` | Direct, touch-sized navigation and recovery access | Coherent dark action grid, readable Back and shortcuts, no cascading menu. System occupies the content area. Higher-DPI and alert coexistence still require the replacement gate. |
| Diagnostics: `recovery-diagnostics.png` | Technical details separated from everyday operation | Correct Beta 2/version/commit and INSTALLED PRODUCT; no stale Alpha caption. Route and energy unavailability remain explicit. Scroll controls are visible. |
| Instruments: `alpha-instruments.png`, `beta-night-instruments.png` | Large related navigation/wind/condition values | Three related groups with values larger than labels; unavailable pressure remains a dash with NO DATA. Night is consistently dim. The directional wind cue is small; assess readability on the boat before treating its physical legibility as accepted. |
| Energy: `preview-04-energy.png` | Battery, propulsion, destination hierarchy | Large SOC/power, subordinate V/A/RPM/temperature, distinctly estimated destination SOC/range. The image is an explicitly labelled synthetic CI scenario; it does not establish live battery integration. |
| AIS: `objects-ais-card.png` | Compact chart-native target context | Card preserves the chart, shows source-supported identity, speed/course, CPA/TCPA, range/bearing and bounded actions. This actual pixel capture includes the disposable Windows taskbar; it is not a clean full-frame visual acceptance image. |
| Charts/plugins: `chart-opengl-01-loaded.png`, `chart-opengl-05-legacy.png`, `chart-opengl-06-returned.png` | Real charts with XNav clear and Legacy intact | Detailed public ENC soundings/coastline/marks survive restart. Bundled Dashboard is absent in XNav, restored in Legacy, then hidden again on return. The old native chart bell remains an upstream chart element. This does not qualify the boat GPU or plugins. |
| Settings/Sensors: `alpha-settings.png`, `alpha-sources.png` | Friendly categories before technical diagnostics | Seven normal categories and quantity-level health; no Signal K choice or PGN jargon in the first Sensors view. AIS explicitly says Target data unavailable. Fixture data is not real vessel reception. |
| Autopilot: `pilot-01-status-only.png` | Clear mode and manual-control boundary | Status-only controls are disabled and Control OFF is explicit, but the static subtitle “Feedback confirmed” can read as a current acknowledgement. Change it to “Manual commands require feedback confirmation”; retain actual feedback and command status in their own fields. |
| Anchor: `alpha-anchor.png` | Clear watch state and current conditions | WATCH OFF and missing distance/radius are explicit; no invented watch. This inactive image does not close active-watch label/icon/removal boat checks. |
| Setup: `installer-wizard-welcome.png`, `installer-wizard-backup.png`, `installer-wizard-ready.png`, `installer-wizard-installed.png` | Conventional guided Windows installation | Welcome, preservation/recovery explanation, detected version/paths and completion are readable. Launch remains an explicit choice. Internal `OpenNavXAlpha1` storage directory is retained for migration compatibility and is not the displayed product version. |

The subtitle repair changes wording only, not adapter permissions, feedback
freshness, acknowledgement or physical output behavior. It remains separate from
the currently running 608756 harness candidate and must be included in the final
exact-commit Linux/Windows and boat validation.
