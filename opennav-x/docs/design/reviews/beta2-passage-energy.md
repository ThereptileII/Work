# Beta 2 — Passage, Energy and Diagnostics refinement

Status: implemented for rendering review; native CI and boat-PC image gates
are still required. This is not a visual-acceptance record.

## Reference intent

- Large values and quiet labels, with cyan reserved for navigation/estimates.
- Battery/propulsion relationships visible together, rather than many gauges.
- Destination and next waypoint presented as navigation information.
- Technical provenance stays in Diagnostics and commissioning views.
- Consistent dark surfaces in Day, Dusk and low-light Night.

## Beta 1 observations

- Energy mixed large cards with technical paragraphs and model terminology.
- Live/quality captions repeated everywhere, competing with numeric values.
- Passage repeated internal route ownership and source identity information.
- Next-point range and planned turn required a separate SmartNav page.
- Diagnostics retained an Alpha warning despite the correct Beta build fields.

## Changes

- Shared `XNavPainter` supplies typography, cards and separators. Local code
  formats vessel readings and predictions but does not define a second theme.
- Energy uses Battery, Propulsion and Destination across the first row where
  the available width permits. Voltage/current and RPM/temperature are grouped
  below their principal value. Battery load remains explicitly separate from
  motor power; one cannot silently substitute for the other.
- Range, drive and regeneration occupy a secondary strip. Model configuration
  and provenance stay under Settings/Diagnostics. Arrival charge remains a
  whole-percent advisory, with reserve/shortfall and unavailable reason shown.
- A stale reading displays an em dash and its stale age, rather than a retained
  number that could be mistaken for current data. Missing data is never zero.
- Passage displays destination, remaining distance, next waypoint/leg, and
  advisory turn/course/timing from the existing SmartNav output. It performs
  no replacement geometry or ETA calculation. Unnamed points use plain language,
  never a route GUID.
- Diagnostics uses the current edition/build, separated visually from the
  everyday screens. Recording/replay remains explicitly historical diagnostics.
- Synthetic mode captions and synthetic calibration defaults compile only in
  developer executables; they are absent from the installed product.

## Required review

Capture each screen at 1280×800 in Day, Dusk and Night, then at 125%/150%.
Confirm that narrower layouts scroll without clipped controls; the permanent
navigation rail is a separate no-scroll requirement. Test unavailable/stale
voltage, SOC and motor readings, no active route, GPS loss, reserve warning and
insufficient energy using isolated CI input. On the boat PC use read-only real
data only. Compare each image to the reference before accepting the screen.
