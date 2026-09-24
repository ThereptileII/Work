# Display layout and instrument selection

The shared Alpha settings record stores ordered, bounded, unique instrument
keys for the chart data rail (1–6) and instrument page (1–23). Navigation,
sailing and energy rail presets and individual selections are available under
Display. The instrument page links to its own selection controls. Older records
without these optional keys retain the established default layout.

Stable keys resolve only to existing OpenNav Vessel Data samples. There is no
new navigation calculation or acquisition path; selecting a value preserves its
source, observation time, validity and freshness. Missing values remain missing.
The rail rebuilds only when selection changes; the 250 ms refresh copies the
current assessments without resetting sensor ages. Persistence uses the existing
validated, flushed wxFileConfig store. Diagnostics include the selected keys.

Day, Dusk and Night buttons use the existing OpenCPN chart color-scheme action
and the shared XNav tokens. Fullscreen uses the existing frame action. Hardware
screen brightness and Windows scaling are clearly identified as display/OS
controls, not simulated device controls. The scrolled layout retains touch
sizes at higher DPI; the native gate remains authoritative.

Portable contract coverage checks persisted selection order, invalid/duplicate/
oversized lists, missing data and unchanged stale observations. The native
preview scenario changes all palettes, applies an energy rail, restores the
navigation preset and toggles an instrument through the real custom controls.
