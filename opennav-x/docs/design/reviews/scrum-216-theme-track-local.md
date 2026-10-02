# SCRUM-216 Display light track — local correction

Selected in Jira 10473. The native 2060 component review found the Display
theme choices drawn as standalone buttons. Canonical Windows `display-day.png`
has a continuous surface track: pixels (850,520) and (900,540) are #1d2d31;
the old native image showed the page background #152326 there.

The unchanged HTML `.segment` rules at `index.html:11,24,464` specify a
9px-radius surface, 4px padding and gaps, and 40px-minimum buttons. The actual
Windows `reference/windows/capture.json` records the Day button at
(675,507.109375), 123.328125×40, with 10px/400 text and radius 6. The selected
surface is #26393d. These computed values, rather than a similarity score,
define this correction.

Display now owns a painted track with those insets and spacing. Its existing
buttons inherit the track background; a narrow `SetSegmentInTrack()` opt-in
uses the full button bounds and weight 400. Other segmented controls retain
their existing style. Theme refresh repaints the track, including refreshes
that preserve an unapplied draft. Existing callbacks, save semantics and
interface-scale/layout behavior are unchanged. System was not changed.

Validation: compiled the offline Settings harness from the current
SettingsDrawer, Controls, ChoiceField, Drawer and FloatingSurface sources,
using the local wx runtime and cached non-UI application/adapters/smartnav/
vessel libraries. The existing harness passed 179 checks, including theme
callbacks, failed/successful Apply, 100/125/150% sizing, draft preservation,
fullscreen/personalisation actions and Settings reopening. No OpenCPN build
or product execution was involved.

Inspected the retained Day/Dusk/Night component captures. The Day track pixels
above are now #1d2d31, with selected (735,510) #26393d. The track is present in
all three themes. Evidence and source hashes are in
`docs/evidence/scrum216-theme-track-linux/`. The full 14-image local run is in
`.local/theme-track/capture/` in the working checkout.

This is focused Linux feedback only. Linux font/tab wrapping does not qualify
Windows typography, and Dusk/Night captures use the harness's applied 125%
Chart focus state. Native Windows exact-revision visual review and boat-display
acceptance remain open; no whole-view conformance claim is made.
