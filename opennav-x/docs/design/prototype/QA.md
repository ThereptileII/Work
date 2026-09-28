# Prototype verification — 28 September 2026

## Current v8 sector-light checks

- Simplified light point, compact R/W/G sectors and extended sectors visually inspected. Hover changes the radius from 44 to 340 chart units; pointer leave restores 44 without opening a drawer.
- Day/dusk/night colours inspected; no console errors observed. Blank-chart click collapses the extension without opening a new sheet.
- Clicking pins the 340-unit extension after pointer leave. Details opens the correct object sheet and hides the chart card; Close returns to the pinned selection.
- Course-up rotates the parent chart by -43 degrees while sector geometry retains its local origin and bearings.
- Enter pins the light; Escape returns to compact sectors. The Collapse control works at 390 × 844 with no document overflow. This mobile check used pointer input; native touch injection is unavailable in the in-app browser, so physical touch was not verified.
- Build and existing integrity checks pass. Geometry checks verify clockwise bearings from chart north, contiguous sector boundaries and an expanded radius more than five times the compact radius.

Captures are under `screenshots/v8/`. Sector geometry and extents are illustrative, not navigation data or nominal light range.

## Earlier lighthouse refinement

Långskär fyr replaces the small generic skerry-light drawing with a named vector landmark: tower, band, gallery, lantern, foundation and a soft gradient glow. Selection, the matching detail card, its Close return and day/dusk/night rendering were inspected in the live 650 × 698 browser viewport. No console errors were observed. The build and existing source-integrity checks pass. Captures are in `screenshots/v7/lighthouse-*.png`. Light, height and range are fictional design data.

## Current v7 marker-design checks

All 29 initial chart objects now use original SVG artwork inspired by the earlier thin-stem lateral marks. The chart palette is separate from the untouched source atlas. Unmapped catalogue entries use labelled reference pins; the placement action explains that distinction.

| Check | Observed result |
| --- | --- |
| Artwork | Every default object has an explicit vector design. Automated checks reject image/foreignObject/background-image content in all 3,321 chart-art/palette combinations. The live default layer contains 38 vector groups including repeated cable/pattern elements and no raster nodes. |
| Visual inspection | Day and dusk at 1280 × 800; night after zoom and Course up; mobile at 390 × 844. Fine stems, small topmarks and restrained colours inspected against the retained v2 chart reference. Mobile document width equals viewport width. |
| Details and input | Port mark pointer selection opens its matching vector preview; starboard keyboard selection still works after zoom/rotation. Close returns to the chart. No browser console errors observed. |
| Build | Standalone build, catalogue integrity, backup validation and source/build equality pass. Original source glyphs remain in the atlas. |

Captures are in `screenshots/v7/`. These are design concepts, not standards-approved chart portrayals. Physical-device visibility and navigation accuracy are not validated.

## Earlier v6 chart-symbol checks

| Check | Observed result |
| --- | --- |
| Initial chart | 29 selectable examples use valid catalogue and guide references; all positions are inside the fictional scene. Desktop chart and mobile 390 × 844 visually inspected. No horizontal document overflow on mobile. |
| Selection and return | Cardinal and lateral marks open the correct source/meaning sheet. Opening the atlas reference presents Back; returning restores the chart-object sheet. Root sheet has one Close. Enter on a chart symbol opens its details. |
| Placement | Point, line and area-pattern examples placed through the UI. Added point was selected and removed. Cancel restored its originating source detail. Successful placement focuses the new mark. Clean reload restores the 29 defaults. |
| Presentation | Visibility toggle hides the layer; labels render for all 29 examples. Zoom and Course up retain upright point glyphs and labels. Day, dusk and night palettes visually inspected. |
| Runtime and build | No console errors observed in these chart paths. Build/source equality, earlier atlas checks and backup checks pass. All 3,321 chart-glyph/palette combinations generate finite nonempty markup. |

Screenshots are in `screenshots/v6/`. Positions, sample line/pattern geometry and source-anchor centring are illustrative; these checks do not validate ENC portrayal or real navigation. Placement requires a pointer/touch position; chart-symbol selection is also keyboard accessible.

## Earlier v5 symbol-atlas checks

| Check | Observed result |
| --- | --- |
| Resource coverage | 1,107 unique definitions: 1,018 point symbols, 59 lines, 30 patterns. All sprite crops are inside all three 1,500 × 1,200 sheets. All vector colours resolve. All 3,321 glyph/palette combinations generate nonempty finite markup. |
| Guide | All 12 detail panes opened and returned through one Back control. Region B port mark displayed green, BOYLAT23; region A restored red/green lateral marks. Emergency wreck is explicitly a physical illustration. |
| Categories | All 13 categories and All symbols exercised; counts match the catalogue. No horizontal overflow in these desktop checks. |
| Search | Exact LIGHTS11 and DRGARE01, Swedish Nordmärke and fyrar, no-results/reset, keyboard clearing and the clear-search button exercised. |
| Palettes and geometry | Day/dusk/night selected; three palette previews inspected in details. Line styles produced 59 entries, patterns 30. Dredged-area vector dot tile opened correctly. |
| Pagination | Last page displayed 1081–1107 (27 cards) and disabled Next. Previous returned 1045–1080; Next restored the final page. Returning from a wreck detail retained page 31. |
| Return navigation | Navigation settings, Chart presentation and Help entry points exercised. Detail Back restored query, category, palette, page and scroll. Underlying atlas is inert and its return hidden while details are open. |
| Sources and keyboard | Sources dialog has one Close; Tab from its Close button reaches the first source link. Mobile source control has an accessible name. |
| Responsive rendering | 1280 × 800, 390 × 844 and 854 × 533 inspected. No atlas horizontal overflow in the checked states; compact header return remains visible. Screenshots are under screenshots/v5/. |
| Build and runtime | node build.mjs and node verify.mjs pass, including source/build/catalogue equality, standalone assets and earlier backup checks. No browser console errors observed in the exercised final atlas paths. |
| Export boundary | JSON catalogue is generated, parsed and compared against the source by verify.mjs, and included in the package. The UI export action ran without console errors, but the in-app browser did not expose a download event; on-disk browser download completion is not claimed. Use src/symbol-catalogue.json for the verified handoff. |

These checks validate the reference UI and catalogue integrity, not operational chart portrayal or every conditional ENC combination. Enlarged raster previews and fitted vector strokes are documented in SYMBOLS.md.

## Earlier v4 navigation checks

The return-control audit exercised **122 context/state checks** through the browser UI. Each recorded active context had exactly one local return control. This count includes revisits and responsive checks; it is not a count of unique screens. The complete source inventory and decisions are in `NAVIGATION-AUDIT.md`; raw observed labels/counts are in `screenshots/v4/navigation-audit.json`.

| Check | Observed result |
| --- | --- |
| Root and child pages | Root Settings, Energy, Instruments, Radar and direct chart sheets use Close. Nested configuration, libraries, sensor details and guides use Back. No duplicated header/footer return. |
| Settings → Navigation → Chart presentation | One Back; correct tab restored with unsaved corridor value 0.31. Closing Settings restored keyboard focus to its opener. |
| Sensor wizard | Cancel on Connection, one footer Back on subsequent steps; Assignment value retained after returning from Verify. Completion returned to the originating sensor list with no duplicate list entry in history. Cancellation from vessel setup returned to Sources. |
| Installer | Welcome/host/options/review/progress/verified/Ready exercised. Only one return per step. Cancel confirmation exited to its origin; Done skipped obsolete installation steps. Guide used Back, retained the checkbox and omitted a redundant Open setup wizard link. |
| Vessel setup | All six stages exercised; nested source inventory used Back, nested sensor cancellation restored Sources, final review Back returned to Helm, and Open your helm completed the flow. |
| Recovery | Repair/rollback/uninstall review cancellation; running repair cancellation; rollback completion and Done returned to origin. A cancelled operation did not overwrite a subsequent recovery flow. |
| Dialogs | New/edit waypoint, nested delete/keep, chart import/removal, sensor calibration/removal, control enablement, backup review, updates/history, GPX preview, search, energy model, keyboard help and workspace modes checked. One explicit Cancel/Keep or one header return. Unsaved waypoint name survived nested confirmation. |
| Chart and traffic | Traffic → target had no redundant All vessels action. Show on chart → Back restored the target, then Traffic. Plot cancellation by Escape returned to Passage. Chart position/object and passage naming/success dialogs checked. |
| Updates and guides | Idle/checking/available/downloading/verified/installed/history paths and all five guide pages retained one appropriate return. |
| Responsive layout | 1280 × 800, 390 × 844, 854 × 533 and 1280 × 640 inspected. No document horizontal overflow in checked layouts. Wizard returns stay above mobile navigation and the short-desktop status bar. |
| Runtime/build | No console errors in exercised paths. Dependency-free build and `node verify.mjs` passed. |

Current navigation captures are under `screenshots/v4/`. The main chart intentionally has no return action. Main navigation and work actions such as Save/Continue are separate from the local return count. Rare file-error/empty-state branches were reviewed in source; this is not a claim of exhaustive combinatorial testing.

## Earlier v3 checks

| Check | Observed result |
| --- | --- |
| Settings → Navigation → Chart presentation → Back | Correct Navigation tab restored; unsaved corridor value retained. |
| Add sensor | Connection, discovery, assignment, verification and completion exercised; name retained after returning from Verify. |
| Installer | Welcome validation, installation-guide return with checkbox retained, unknown-host rejection, supported detection, options, review, simulated install and self-test completed. |
| Vessel setup | All six steps completed; nested Add sensor returned to Sources; configured vessel name appeared on the helm. |
| Radar | Coastal/target/clutter echoes visually inspected in overlay and focus. Full scope fits 1280 × 800. Overlay-to-focus return preserved. |
| AIS context | List → Freja → Show on chart → Back restored Freja; next Back returned to list. |
| Nested dialogs / keyboard | Waypoint edit → Delete → Keep restored the edit form and unsaved name; Escape returned to passage, then chart. |
| Updates | Check → download → verified install review → completion exercised; release state changed to installed. |
| Backup | Sample v2 review/restore completed and pre-restore recovery point appeared. Valid/invalid schemas also checked in Node. |
| Maintenance | Repair review, progress and completion exercised. |
| Help / charts / plugins | Help search and no-results state, plugin inventory, and example chart-package creation exercised. |
| Radar controls | Keyboard gain adjustment updated 68% → 69%; range selection updated the scope to 3 NM and back. |
| Heading-relative guard zone | Sector and vessel marker share the vessel heading. Visually checked ahead of the bow in north-up, head-up and radar focus; course-up transition also exercised. No console errors. |
| Energy footer clearance | Energy fills the workspace without the chart horizon. Checked 1280 × 800, 650 × 698 and 390 × 844; the final calibration control remains above the footer/navigation. Chart and Instruments retain a correctly positioned horizon with no overlap. |
| Responsive | 1280 × 800 desktop and 390 × 844 mobile visually inspected. Sensor/installer layouts checked for horizontal overflow; compact 854 × 533 installer width also checked. |
| Runtime | No console errors in the exercised flows. |
| Static checks | Embedded script syntax, no external data/runtime dependencies, unique IDs, build/source equality, backup rejection cases, return controls and retired hardware-name removal. |

Current screenshots are under `screenshots/v3/`. Mobile installation Back/Continue remain fixed above navigation. Full-screen views hide the underlying chart from the accessibility tree. Earlier checks below describe the preceding design revision; they are retained as history rather than presented as a full fresh regression pass.

## Earlier v2 browser checks

The generated `index.html` was served over loopback and inspected in the Codex Chromium browser. Direct `file://` inspection was blocked by the browser tool's URL policy; it was not worked around. Static verification confirms that the HTML embeds its styles, chart and script and has no external runtime/data dependency.

| Check | Result |
| --- | --- |
| 1280 × 800 reference layout | Chart, controls, instrument rail, timeline and footer visible; no horizontal document overflow. |
| 1024 × 640 compact layout | No horizontal overflow; autopilot summary remains above footer. |
| 854 × 533 compact layout | No horizontal overflow; settings and autopilot remain reachable. |
| 390 × 844 mobile layout | Four-value top strip, chart, horizon and six-item bottom navigation; no horizontal overflow. More reaches settings and autopilot. |
| Day / dusk / night | Visually inspected; separate chart/surface tokens and reduced night luminance. |
| Measurement | Two chart selections produced distance and bearing; Done cleared the tool. |
| Waypoint lifecycle | Created a named waypoint, reopened it, displayed delete confirmation and verified removal. |
| Graphical route | Plotted three positions, undid the last, saved two points and activated the new passage. Timeline/destination/example estimates updated. |
| Passage editing | Renamed a waypoint; reversed route; restored the reference route; ended and reactivated navigation. |
| Instrument configuration | Changed the fourth slot to Heading, then restored the original four essentials. |
| AIS | Opened list and selected Freja; inspected target identity and CPA/TCPA details. |
| Energy | Speed exploration changed arrival estimate; high speed produced reduced energy margin. |
| Radar | Enabled mock adapter and changed range to 3 nm; radar focus and controls rendered. |
| Anchor | Adjusted radius from 40 to 45 m; armed and stopped the watch. |
| Autopilot | Auto initially disabled; explicit enable required. Pending state preceded mock confirmation. |
| Autopilot timeout | Simulated unacknowledged +10° command. Last confirmed heading remained 041°; critical alert persisted after opening Energy. |
| Sensor loss | Stale battery removed arrival SOC in rail and horizon. GPS loss hid next-turn predictions and left a critical banner visible in Energy. |
| Update flow | Opened release concept and completed simulated update path. |
| Replay labeling | Entered and exited replay from mobile settings; persistent technical-test banner was visible. |
| Final runtime | Fresh preview reported no browser console warnings or errors. |
| Standalone structure | `node verify.mjs` checks embedded script syntax, external dependency absence, unique shell IDs, three palettes and source/build equality. |

Screenshots are in `screenshots/v2/`. The original reference screenshots are retained outside that subfolder.

## Limits of this verification

- The compact desktop sizes approximate the available CSS area at increased scaling; actual Windows 125%/150% DPI operation was not tested.
- Mouse and keyboard interaction were exercised. Physical touchscreen, multitouch pinch, sunlight readability and measured night-display luminance were not tested on hardware.
- Export/restore, recording, replay, welcome and recovery are prototype implementations/concepts, not certified production functionality. Full end-to-end backup round-trip and fullscreen permission behaviour were not part of the browser checks.
- No real chart, GPS, NMEA, battery, AIS, radar or autopilot was connected. Real-world accuracy and safe vessel operation were not tested.
- No accessibility compliance audit was conducted. Some compact secondary labels and chart controls require physical-display and touch-target review.

## Rebuild and verify

```text
node build.mjs
node verify.mjs
```

Both scripts use only built-in Node modules. Opening the generated HTML does not require Node.
