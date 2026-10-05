# XNav component inventory — Beta 2

Primary views compose the following shared controls and rendering primitives.
Conceptual names below describe reusable roles; the implementation column is
the actual source boundary, not a claim that every role is a separate C++ class.

| Component/role | Implementation | Behavior and use |
| --- | --- | --- |
| XNavButton | `ui/Controls.*`, `XNavButton` | Owner-drawn Normal/Quiet/Primary/Critical roles, pressed/focus/disabled states, keyboard/touch, semantic palette |
| XNavIconButton | `ui/Controls.*`, `XNavIconButton` | Shared vector icons including Center, zoom, Back and menu; accessible text retained |
| XNavCard | `XNavPainter::Card` | Shared rounded dark surface, quiet heading and border treatment |
| XNavContextCard | `ui/ContextCard.*` | Modeless chart-bounded AIS, waypoint and chart-position cards; copied state, explicit actions, outside/Escape dismissal, no pointer grab |
| XNavRouteContextCard | `ui/RouteContextCard.*`, `application/RouteContext.*` | Chart-bounded modeless route summary; copied identity/state/endpoints, View on chart and Details. Hover does not take focus or replace an open task; Close/outside/Escape dismisses; no activation callback |
| XNavNameEditor | `ui/NameEditor.*`, `application/NavigationNaming.h` | Inline Route name / Waypoint name, explicit Save or Enter; Cancel/Escape restores the saved name without leaving details. Changed drafts survive timer/theme refreshes; invalid/read-only/stale writes cannot silently replace saved data |
| Navigation naming sheet | `ui/NameEditor.*`, `EditNavigationNameSheet` | Shared themed Name and optional Description fields for creation and fuller edits; chart-derived suggestions or coordinate fallback for new objects only; explicit Save/Cancel with draft retained after invalid input |
| XNavChartInfoDrawer | `ui/ChartInfoDrawer.*`, `application/ChartInfo.*` | Read-only chart information sections, concise summaries and Show all chart details / Hide chart details. Preserves unknown fields and attachment references as inert text; empty/truncated notices remain explicit; shares drawer scrolling and Day/Dusk/Night roles |
| Product value group | `ProductPanel::Visual` plus `XNavPainter` | Owned-state instrument, pilot, AIS detail, anchor and alert groups |
| XNavSheet | `ui/Sheet.*`, `EditSheet`/`ConfirmSheet` | Focused dark modal, explicit actions, bounded fields, scrollable content; no write before acceptance |
| XNavDataValue | `ui/Controls.*`, `XNavDataValue` | Freshness/provenance-aware number; compact rail layout and standard larger layout |
| XNavStatusIndicator | Shell status/alerts and `ProductPanel::StatusAction` | Connection/age states in plain language; detailed source metadata after selection |
| XNavToggle | `XNavButton` selected state and explicit permission actions | Shared state styling; hardware enablement additionally requires confirmation and controller interlocks |
| XNavSegmentedControl | Grouped selected XNavButtons | Day/Dusk/Night and related mutually exclusive settings; no native tabs |
| XNavListRow | Quiet `StatusAction`/`Action` composition | Sensor health, saved routes/marks and target selections; unavailable actions disabled |
| XNavNavigationSummary | Shell summary and `PreviewPanel` Passage | Existing immutable route progress and SmartNav advice; no independent route geometry |
| XNavAlert | Shell reserved status slot + alert context cards | Persistent condition, severity, inspect and acknowledge; never hides a critical condition behind page content |
| XNavBottomBar | `ui/Shell.*` | Navigation/passage/pilot/system access and persistent STBY; route creation replaces only page shortcuts with 88-DIP Cancel/Undo/Done, without resizing the chart |
| XNavDataRail | Shell + compact XNavDataValue | Four chosen primary values; no ordinary narrow-rail scrolling; alert presence does not change height |
| XNavScroll | `ui/Controls.*` | Wheel/touch pan, hidden native scrollbar, explicit Up/Down, correct content extent |
| XNavPainter | `ui/Controls.*` | Authoritative DIP text, card, separator, palette use; all painted product views share it |

## Tokens and layout

`ui/Theme.h` owns semantic colors, touch sizes, spacing and radii. `UiFont`
owns font choice/scaling. Native Windows uses Segoe UI; no additional font
installation is required. Body spacing uses the 8-DIP family; high-frequency
controls are at least 48×48 DIP and pilot course/standby actions use 56-DIP rows.

Large values carry the hierarchy. Labels and units are quieter, with extra
source/engineering detail withheld from primary screens. Cyan means navigation
or an estimate; green means confirmed/current status; amber/red carry attention
and alarm meaning. Dusk/Night use the same semantic roles with reduced light.

Painted groups receive OpenNav-owned state and refresh their own surface.
They do not redraw the chart, retain upstream pointers, or renew observations.
Native screen rectangles of controls/painted regions are exposed in local
diagnostics so automated tests click actual layouts and check clipping rather
than using fixed coordinates from an older UI.

## Deliberate native boundaries

Advanced OpenCPN settings, plugin dialogs and system file pickers remain native
and may be bright. Text-entry controls live inside themed XNav sheets and the
inline saved-object name editor. In XNav, chart object information uses the
owned read-only drawer; Legacy retains OpenCPN Object Query. Chart/plugin
selection and value formatting remain upstream responsibilities. No
placeholder slider, radar transmitter control or unsupported pilot mode is
presented as working. Popup/card changes are immediate; decorative animation
has not been added to navigation data.

## Validation status

The inventory records implementation, not final visual acceptance. Native
100/125/150% and boat-PC review records must identify clipping, palette and
interaction results for the exact candidate revision.

## Bundled Dashboard presentation boundary

`integration/DashboardPresentation` registers actual bundled plugin windows with
weak lifetime references. XNav's rail and Instruments stay the primary visual
components; the Dashboard continues acquiring data with its desktop panes hidden.
Legacy restores the saved layout. This is an integration presentation boundary,
not a replacement plugin API or independent instrument data model.

`XNavAlertDrawer` now owns the prototype notification-centre composition and
copied alert episodes, sharing XNavDrawer, XNavButton, callout, rule and text
primitives. Source inspection and acknowledgement remain separate callbacks;
there is no equipment command callback in this component.
