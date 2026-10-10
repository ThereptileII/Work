# Prototype parity review — 2026-10-10

Requested by the owner after boat testing: "evaluate differences between the
prototype and the real software in terms of design and information given …
create a list of UI changes to make it look the same".

Reference: `docs/design/prototype/index.html` (identical to the owner's copy),
rendered headless at 1280×800, DSF 1, day/dusk/night and with wind vectors on.
Implementation: boat display (1440×900, live data, no active route) and the
Linux route fixture at 1280×800 (active route).

## Changes (P1 = named by the owner)

| # | Area | Prototype | SKAGER today | Change |
|---|---|---|---|---|
| 1 | GRIB wind arrows | Thin open chevron strokes in route ink, 35 % opacity, sparse lattice; part of the chart, so they pan with it. | Filled light-blue arrows with dark outline and a speed number; laid out on a **screen** grid, so they stay put while the chart pans (owner-reported bug). | P1. Geographic lattice snapped to nice degree steps per zoom; prototype chevron, route ink, low opacity, no per-arrow numbers (speed stays in the forecast box). |
| 2 | Data rail tiles | Each tile carries a visual: SOG sparkline; depth meter with the safety depth marked ("3.5 m safety"); wind direction arrow + angle + PORT/STBD; battery bar + "At destination 43 %". | Label, value, unit only. | P1. Add the visuals from real data only: SOG history, safety depth from settings, measured wind angle, energy prediction. The per-source "GPS 0.2 s" line stays removed (owner request SCRUM-352). |
| 3 | Next turn card | Floating card on the chart while a route is active: turn icon tile, "NEXT · LÅNGHOLMEN", "Starboard 32°", "0.7 nm · in 7 min · new course 075°", chevron to the passage. | None. | P1. Card from OpenCPN route progress (next waypoint, distance, ETA, course change); hidden without an active route or position. |
| 4 | Routes and waypoints | Teal 3 px line with a faint corridor; white circular markers with a teal ring and bold number; names in white rounded chips. | Active route close to this; **inactive routes** grey double line with heavy grey rings and no names; standalone waypoint names as bare teal text. | P1. Same marker geometry for every route (inactive: muted ink, thinner); waypoint and route-point names in the prototype chip. |
| 5 | Chart water | Pale water with near-invisible depth bands (#d5e5e5 → #d1e2e2 → #ccdfdf → #c4dadb measured) and thin contour outlines ("very light"). | Shallow depth areas filled with darker teal-blue tones (#86acb6). | P1, Day palette: bands 6/12/20 % from water toward the old shallow tone, matching the measured prototype values. The bands stay distinct (existing shallow-water guard) and the safety contour stays emphasised. Dusk/Night unchanged: their buoy-contrast proofs depend on those fills, and the feedback concerned the light theme. |
| 6 | Chart piano bar | None. | OpenCPN's chart-stack bar (two long bars) along the bottom of the chart. | Hide in SKAGER mode (chart selection remains in Legacy). |
| 7 | Own vessel label | Chip "● Reptil · 6.3 kn" beside the vessel. | None. | Chip with vessel name and SOG; omitted when SOG is unavailable. |
| 8 | Follow control | "Following Reptil", filled/active while following. | "Follow boat" in one state. | Two states, vessel name when known. |
| 9 | Header status | "● Systems nominal" health chip; clock with "LOCAL". | "OpenCPN navigation" text. | Health chip from source health ("Systems nominal" / "n sources need attention"); LOCAL sublabel. |
| 10 | Horizon | Up to four events (NOW, NEXT, traffic, ARRIVAL), two lines each. | Passage column's third line is cut off at the bottom edge. | Fix clipping; NEXT/ARRIVAL columns from route progress. |
| 11 | Unknown chart objects | "Other definitions use a labelled reference pin, never a guessed symbol." | OpenCPN's magenta "?" glyph (seen near Linköping). | Follow-up: needs a new atlas symbol with resource-generator and atlas tests; tracked in SCRUM-363. |

## Not changed, and why

- **Chart region title** ("SWEDISH EAST COAST / St. Anna archipelago"): no
  reliable region name from charts; inventing one would break the data rule.
- **SmartNav advisory card** ("A clearer course ahead"): only once SmartNav
  produces a real corridor check; never as static text.
- **Radar overlay button**: radar remains unavailable on the boat.
- **Brand**: SKAGER identity (SCRUM-89) replaces "opennav x".
