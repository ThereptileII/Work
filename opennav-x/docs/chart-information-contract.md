# Native chart information (SCRUM-309)

OpenCPN 5.12.4 `ChartCanvas::ShowObjectQueryWindow` retains its existing chart,
plugin, light-sector, overlay and AIS-area-notice selection and formatting.
After this normal query finishes, XNav copies the complete result into an owned
text presentation; Legacy/Safe keep the original query dialog and file handling.
An empty query explicitly says no information is available.

The native drawer shows human-readable names and values first, with complete
upstream text available per section. Unknown attributes, overlapping objects,
sector conventions and source units are retained. HTML is never executed;
attachment/image references are inert text. Input is bounded to 1 MiB/512
sections, with explicit truncation and a Legacy fallback message. No chart
object or waypoint pointers reach the UI. Delayed display is guarded against
shutdown, mode restart and owner replacement.

Focused parser and native wx drawer checks cover overlapping objects, lights,
unknown fields, markup/attachments, malformed/empty/large input, three palettes,
Escape and touch-sized dismissal. These pass on Linux; the bridge and pinned
chart canvas compile. Native Windows and boat checks remain pending for the
coordinated staging candidate.
