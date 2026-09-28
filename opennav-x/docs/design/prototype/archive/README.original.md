# OpenNav X — HTML UI Reference Prototype

This is a self-contained design/interaction prototype for Codex and human review.

## Open

Open `index.html` in a modern browser. No web server, packages or internet connection are required.

Recommended reference viewport: **1280×800**.

## What it demonstrates

- Chart-first primary navigation layout
- 44 px status bar, narrow left tool strip, 4-value right rail and contextual bottom strip
- AIS target selection/card
- Active route + navigation timeline
- Instruments/wind presentation
- Propulsion/energy hierarchy
- Expandable autopilot panel
- Anchor mode
- Modern settings pattern
- Day → Dusk → Night theme cycling
- Startup update popup concept

## Interaction hints

- Click the magenta AIS target.
- Click `AUTOPILOT AUTO 143°` in the bottom bar.
- Click SOG/Depth/Wind to open Instruments.
- Click Battery to open Propulsion & Energy.
- Use left Route / Anchor buttons.
- Use the gear icon for Settings.
- Use the moon icon to cycle Day/Dusk/Night.
- Use the circular-arrow icon to show the proposed startup update popup.

## Codex implementation rule

This HTML is a **design and interaction reference**, not production code. XNav remains wxWidgets/OpenCPN-based. Codex should reproduce the visual hierarchy, geometry, state language and interaction flows using the reusable native XNav component library.

The authoritative written requirements remain in `OpenNavX_Codex_Project_Specification.md`.

## Priority visual rules

1. Chart stays dominant.
2. Permanent chrome is minimal.
3. Main rail has four high-value metrics, not every available sensor.
4. Contextual cards replace large desktop dialogs for normal workflows.
5. Values dominate labels typographically.
6. Color is semantic, not decorative.
7. Missing/stale/estimated data must be explicit.
8. Touch targets are at least ~48×48 px.
9. Night mode must reduce emitted light, not merely invert colors.
10. Legacy OpenCPN remains visually separate and stock-like.
