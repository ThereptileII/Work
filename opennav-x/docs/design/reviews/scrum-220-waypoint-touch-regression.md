# SCRUM-220 Waypoint field touch target

The Linux `smoke-user-flows.py` regression observed the Create waypoint
Name field at 488×46 logical pixels. Its existing 48×48 minimum assertion
remains unchanged. `EditSheet` now gives single-line inputs a minimum height of
`max(48, DisplayFieldHeight(scale))`: 48, 52 and 56 logical pixels at interface
scales 100%, 125% and 150%. The modal sheet's multiline editor is unchanged.

`sheet_field_size_test` opens the real modal `EditSheet`, inspects its actual
wxTextCtrl minimum and allocated heights at all three interface scales, and
cancels without saving. It passed in the local Linux wxGTK/Xvfb build. This
component check is development evidence; the end-to-end smoke assertion and
native Windows rendering remain separate gates.

The failing source is published commit
`f1e2cde8fcbf92826d648007b267cc0f5320aa55`, integrated run
[36875827855](https://github.com/ThereptileII/Work/actions/runs/36875827855),
Linux job `110414873707`. Downloaded artifact `11169629856` was checked against
its 8,877,421-byte size and SHA-256
`b51228609a47fa990af263cdc4ab8907ad299af87204e2f98f8599925375b1fb`,
then checked for ZIP integrity and safe extraction. The actual waypoint
screenshot and report show the visible/enabled 488×46 input. Typing, saving and
later route checks were not reached; this is not evidence of navigation data
loss. The preceding 146 unit tests passed.

Implementation is local commit `8a938eccfe8ca05392d6ebe5e404ad1f702a76e5`.
Test-only follow-ups `1c15ab0d57e2d53a343b8ff83eebc1d24574a462` and
`2e55187a6d1a85726b2e143f44c483dfc2665827` bound modal failure handling and add
the component to the existing native composition workflow, with a 30-second
process timeout. Local measured expected/minimum/allocated heights were
48/48/48, 52/52/52 and 56/56/56. These are interface-scale checks, not proof of
Windows OS DPI qualification. Native execution remains pending.

Independent source review confirmed that the shared sheet also covers route
naming/editing, while font sizes, the Preferences token and multiline fields
remain unchanged.

The focused integrated build and install completed successfully on 2026-10-01.
The executable identifies implementation `8a938eccfe8ca05392d6ebe5e404ad1f702a76e5`;
test/docs HEAD was `54ebb26340527e3b531999d8dfaa3a0b50544f52`. The source was
verified against pinned OpenCPN and all nine current integration patches.
The unchanged full-application smoke passed all eight groups: orientation,
context dismissal, waypoint creation/Go To/stop, draft controls, naming/save,
Undo state, route cancellation and read-only SQLite persistence. Both exercised
Name fields measured 488×48. The database retained one two-point route and the
original waypoint identity/clicked position; no cancelled draft remained.

Four actual 1280×800 captures were reviewed: waypoint editor, saved waypoint,
route naming and saved route. Fields/actions fit, chart coastline remains
visible, and saved details render. See the [hash inventory](../../evidence/scrum-220-linux-user-flow.json)
for executable, logs, result and all nine captures. Screenshots remain in the
local ignored evidence directory. This is focused Linux development evidence;
native Windows, OS DPI, full replacement-candidate and boat qualification remain
open. Existing prototype and branding work is not accepted by this touch review.
