# Chart presentation drawer — SCRUM-14 / SCRUM-15 local increment

Selected scope: Jira 10475. The native drawer consumes the copied chart contract
from bridge commit `ac77afea030bc9b256e36c5f28a3f41ed1fdd864`; integration owns
every chart read and write. The combined follow-up wires the drawer from the
floating Layers action and Settings → Navigation → Chart presentation. Opening
either entry closes the previous sheet. Chart overlays respect its occupied
region; Layers hides when the compact chart would overlap the bottom tools.

The active immutable HTML `index.html:326` and canonical Windows
`reference/windows/capture.json` supplied the composition: 398×674 drawer at
(682,80), 352px body, 48px format/orientation tracks, 4px padding/gaps, 40px
segment buttons, and eight 52/63.5px rows. Content scrolls within the existing
drawer. Day/Dusk/Night use the existing native theme and control components.

Recorded correctness adaptations, agreed with the integration owner:

- Vector/Raster is observed current chart or quilt-reference format, not a
  clickable palette selector. The copied format reason remains visible below
  orientation; a quilt reference does not describe all members.
- Symbol labels is explicitly **ENC text labels**, the native master text
  preference. Supporting copy preserves the distinction from independent
  buoy/light choices. Raster text/soundings are image content.
- Chart symbols and Depth contours show **Managed**, with OpenCPN presentation
  and safety explanations. No false/off state or universal-contour visibility
  claim is invented.
- Route corridor, Wind vectors and Radar overlay show **Unavailable** and
  their provider reason. Unknown editable-layer observations likewise show
  Unavailable, never a fabricated off switch.
- North/Course/Head are explicit native mode requests. The visible supporting
  note says Course up needs current course and Head up needs current heading;
  a selected mode does not prove valid rotation or live source health.
- The prototype symbol-guide link is omitted because no verified product
  callback exists. An optional separate Chart palette preferences callback
  preserves access to existing XNav/Standard settings when the root wires it.

Commands use only returned observed state, including failure or an unchanged
successful readback. Dismissal and explicit Open invalidate deferred commands;
hidden controls cannot mutate chart preferences. Explicit Open resets scroll
and old feedback. Ordinary updates compare all copied values, theme and palette
callback availability, so unchanged ticks do not repaint or lay out the drawer.
Ordinary Present/Update retain scroll.

The committed offline `tests/chart_presentation_drawer_test.cpp` is a
non-installed synthetic fixture, requiring one output-directory argument. It
exercises three layer callbacks, native readback after failure/unchanged success,
all three orientations, raster restrictions, unknown values, palette callback,
pending-click invalidation across dismissal/reopen, quiet unchanged updates,
scroll retention/reset and containment at increased interface scale. It writes
four 1280×800 PNGs and `result.json`; the fixture has no chart/profile/device or
network access. Local evidence is under `docs/evidence/scrum14-chart-presentation-linux/`.
The focused Linux run passed 46 checks. The timer is single-shot and rearmed
only after each step completes, so capture/event pumping cannot reenter the
next step. The local build compiles the drawer and native UI dependencies
against the committed bridge header, reusing only the cached vessel library.

Visual inspection is limited to the component and truthful state adaptations.
The fixture backdrop is intentionally chart-free. Linux fonts do not qualify
Windows typography. Root entrypoint checks are separate from this component
record. Exact-revision native Windows review and boat-display acceptance remain
open; no whole-screen pass is claimed.
