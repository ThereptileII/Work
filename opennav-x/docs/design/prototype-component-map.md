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
| `.dashboard-card`, `.floating` | `XNavPainter` / card component | Distinct panel vs chart-floating surface, correct radius/shadow |
| `.drawer`, `.drawer-head`, `.drawer-body` | `XNavDrawer` / `XNavAisDrawer` / `XNavPassageDrawer` | 398px AIS/Passage sheets, overlay, bounded body scroll, one contextual return; remaining sheets pending |
| `.metric`, `.metric-value` | `XNavDataValue` / `XNavDataRail` | Four values, light 48px numbers; actual provenance/freshness |
| `.status-dot`, `.tag` | Status indicator | Meaningful state color and text; no inferred connectivity |
| `.toggle`, `.segment` | Native toggle / segmented control | Explicit selected state, keyboard input, unavailable semantics |
| `.list-card`, `.suite-link`, `.row` | `XNavListView` / remaining list migrations | AIS list paints visible rows only, identity-bound selection; shared alignment and separation |
| `.next-turn` | Navigation summary | Valid upstream progress and advisory turn only |
| `.timeline`, `.timeline-event` | Navigation horizon | Owned SmartNav events, unavailable dependent events withheld |
| `.critical-banner`, alert drawer | Alert layer | Does not displace rail or hide behind sheets |
| `.autopilot-summary` | Pilot summary / expanded panel | Fresh confirmed state; existing enable/acknowledgement interlocks |
| `.map-tools`, `.compass`, `.follow-btn` | Chart overlay controls | Existing upstream actions, proper chart hit testing |
| `.statusbar` | Native footer | Live source/position context, missing values explicit |
| `.ais-ship`, target card | AIS presentation | Online provenance, local precedence; no fabricated CPA |
| `.stats-grid` | `XNavPainter::Stat` | 23px numeric value and separate 10px unit; 16px column gap |
| `.tag` / `.tag.neutral` / `.tag.warning` | `XNavPainter::Tag` | 26px source/status pills, actual availability |
| `.callout` / `.callout.warning` | `XNavPainter::Callout` | Shared wrapped advice surface, exact source alpha colors |

All geometry is measured from the final CSS cascade at the target viewport;
earlier CSS declarations are sometimes superseded. The capture manifest records
those final styles. Existing Beta `ui/Theme.h` geometry and custom drawings are
migration inputs, not a second design authority.

Passage uses the same stat, tag, callout, button and drawer primitives. The
read-only `application::PresentPassage` preserves the route/SmartNav/energy
observation batch. Its displayed leg distances/times are copied from SmartNav,
not recomputed from coordinates. Existing route commands remain guarded owned
value callbacks; saved-route workflows currently retain the earlier product
page and are not claimed prototype-conformant.
