# Prototype → native component map

This is a migration map, not a conformance claim. The older implementations
remain functional while each is converted; acceptance is recorded separately
in [prototype conformance](prototype-conformance.md).

| HTML selector | Native owner | Contract |
|---|---|---|
| `:root`, `#app[data-theme]` | `ui/Theme.h` | Exact inherited palette, centralized roles |
| `.btn`, `.btn.primary`, `.btn.danger` | `XNavButton` | Computed font/weight, padding, border, focus, hover, disabled state |
| `.icon`, `.icon-btn` | `XNavIconButton` | Supplied vector path vocabulary; 22px, 1.65px round strokes |
| `.nav-btn` | Native navigation control | Icon over label, selected strip and subtle mint background |
| `.profile-btn` | `XNavButton::SetVesselProfile` | 32px round vessel shortcut; exact bottom-group spacing; hidden below 600px desktop height |
| `.dashboard-card`, `.floating` | `XNavPainter` / card component | Distinct panel vs chart-floating surface, correct radius/shadow |
| `.drawer`, `.drawer-head`, `.drawer-body` | `XNavDrawer` / AIS, Passage and Settings drawers | 398px AIS/Passage; 432px Preferences, 410px at <=1100 logical width; current shell workspace, bounded body scroll, one contextual return; remaining sheets pending |
| `.metric`, `.metric-value` | `XNavDataValue` / `XNavDataRail` | Four values, light 48px numbers; actual provenance/freshness |
| `.sensor-details`, `.sensor-details summary` | `XNavHealthDrawer` / `XNavButton::SetDisclosure` | Independent signal disclosures; exact collapsed rows, owned quality/provenance, separate onboard and online AIS |
| `.status-dot`, `.tag` | Status indicator | Meaningful state color and text; no inferred connectivity |
| `.toggle`, `.segment` | Native toggle / segmented control | Explicit selected state, keyboard input, unavailable semantics |
| `.list-card`, `.suite-link`, `.row` | `XNavListView` / remaining list migrations | AIS list paints visible rows only, identity-bound selection; shared alignment and separation |
| `.next-turn` | Navigation summary | Valid upstream progress and advisory turn only |
| `.timeline`, `.timeline-heading`, `.timeline-events`, `.timeline-event` | `XNavHorizon` / owned `HorizonView` | Exact fractional grid, native text/event buttons, independent advice validity and contextual action availability; existing SmartNav event order/calculations |
| `.critical-banner`, alert drawer | Alert layer | Does not displace rail or hide behind sheets |
| `.autopilot-summary` | Pilot summary / expanded panel | Fresh confirmed state; installed build status-only, with equipment enablement unavailable |
| `.map-tools`, `.compass`, `.follow-btn` | Chart overlay controls | Existing upstream actions, proper chart hit testing |
| `.statusbar` | `XNavStatusFooter` / owned `FooterView` | Three prototype groups, current position/COG and source health; XTE unavailable until a validated source exists |
| `.ais-ship`, target card | AIS presentation | Online provenance, local precedence; no fabricated CPA |
| `.stats-grid` | `XNavPainter::Stat` | 23px numeric value and separate 10px unit; 16px column gap |
| `.tag` / `.tag.neutral` / `.tag.warning` | `XNavPainter::Tag` | 26px source/status pills, actual availability |
| `.callout` / `.callout.warning` | `XNavPainter::Callout` | Shared wrapped advice surface, exact source alpha colors |
| `.wind-rose`, `.instrument-grid`, `.instrument-tile` | `XNavInstrumentPanel` | Native SVG-equivalent paths and paired tiles; assessed owned readings, true-heading/relative-wind validity |
| `.energy-grid`, `.energy-gauge`, `.battery-visual`, `.power-row` | `XNavPreviewPanel` Energy view / `XNavPainter` | Computed primary card geometry, native battery and tested model endpoint; lower Explore pace migration pending |
| `.settings-tabs` | `XNavSettingsDrawer` / `XNavButton::SetSettingsTab` | Eight native sections, exact 37px rows and five-pixel gaps; fractional DirectWrite layout and matching native paint on Windows; preserved selection |
| `.settings-intro`, `.suite-link` | `XNavPainter::Wrapped` / `XNavButton::SetSuiteLink` | Bounded native text and actual clickable sensor/settings links; no mock connection counts |
| `.anchor-graphic`, `.anchor-distance` | `XNavAnchorDrawer` / `AnchorView` | Copied upstream watch distance and projected observed history; no illustrative trail or invented heading |
| `#anchorRadius` | `XNavRange` | Native stepped pointer/keyboard input; pending radius only; explicit confirmed watch commands |
| `.heading-dial`, `.heading-controls` | `XNavPilotDrawer` / `PilotPresentation` | Measured magnetic feedback; native manual controls; no optimistic heading or mode |
| `.toggle` | `XNavButton::SetToggle` | 42×25 face inside 48×49 hit target, observed owner-supplied state; activation cannot imply success |

All geometry is measured from the final CSS cascade at the target viewport;
earlier CSS declarations are sometimes superseded. The capture manifest records
those final styles. Existing Beta `ui/Theme.h` geometry and custom drawings are
migration inputs, not a second design authority.

The user-requested AISStream key input is a functional extension not defined by
the illustrative HTML. It uses the same theme, typography and `XNavButton`
actions, with a masked native text input (minimum 48px high). The field is never
prefilled or serialized into UI diagnostics. `XNavDrawer` defers Escape/Back
while a modal is active; only that modal handles cancellation. No invented HTML
reference or prototype-conformance PASS is assigned to the credential form.

Passage uses the same stat, tag, callout, button and drawer primitives. The
read-only `application::PresentPassage` preserves the route/SmartNav/energy
observation batch. Its displayed leg distances/times are copied from SmartNav,
not recomputed from coordinates. Existing route commands remain guarded owned
value callbacks; saved-route workflows currently retain the earlier product
page and are not claimed prototype-conformant.

Instruments uses `application::PresentInstruments`, independently testable with
no GUI/OpenCPN dependencies. The supplied compass mock has inconsistent fixed
heading geometry; [the review](reviews/prototype-instruments-in-progress.md)
documents the navigation-correct rotation and required quality annotations.
The full page retains the horizon; existing configurable fields remain
available below the prototype's primary eight tiles.

Energy uses `application::PresentEnergy` to keep the view tied to current owned
observations and the existing model. The illustrative HTML forecast curve and
good-quality claim cannot override actual model assumptions or missing inputs.
See the [corrective review](reviews/prototype-energy-in-progress.md).

Settings now retains the real chart/horizon behind a native owned drawer.
Existing validated editors remain reachable; Vessel and Navigation inline forms
are still migration work, not conformant replacements. Theme segments follow
external light changes as well as their own clicks. The drawer has no actuator
methods; unavailable capabilities remain disabled or absent. See
[Preferences review](reviews/prototype-settings-in-progress.md).

The lower profile action opens the real Vessel settings section. Current
configuration has no vessel-name field, so its initial and status dot remain
unavailable instead of copying the prototype's fictional name/green state.
Adding a persisted vessel identity and the remaining inline forms is separate
migration work. The action does not alter navigation or issue equipment commands.

Notification centre: `case 'alerts'` → `XNavAlertDrawer`; `.callout` → shared
`XNavPainter::Callout` with critical semantic ink; `.action-row .btn` →
`XNavButton`; `.row` / `.note` → shared painter. `AlertCenter` owns episode
semantics. The drawer holds copies and no OpenCPN or adapter object.

Radar migration in progress: `.radar-layout` / `.radar-display` /
`.radar-scope` → `XNavRadarPanel`; `.radar-control-panel` → independently
scrolling `XNavScroll`; `.toggle` → disabled shared XNavButton toggle. Missing
range/gain has no invented numeric value or thumb. The view owns copied status,
not `IRadar`, a receive image or a scanner command callback.
