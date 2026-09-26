# Native 150% DPI observation repair

Build examined: `6160d3e4bcd924853462f96a32f1b502a72a2884`, native Windows
run `36277024981`. This is an investigation record, not acceptance of the next
candidate or a substitute for the remaining native DPI run.

The DPI test rejected the first rail value at `x=1129`, width `204`, height
`122`, against the newly resized 1280×800 window. The saved
`dpi-150-failure.png` shows all four values fully visible. Its subsequent
`failure_diagnostics` and independent native HWND inventory agree on a rail
from `x=1065` to `1269`, with values spanning `y=129..704` and heights of
143/144 physical pixels. Controls retain their 72-pixel minimum touch height
at the actual 144-DPI setting. The 100% and 125% sequences had already passed.

The test mixed observation times. `windows-ui.py::size_window` waits 0.5 seconds,
while the integration publishes diagnostics at most once per second. The old
`rail_geometry` accepted any retained observation containing four values and
compared it with current native window bounds. A new native frame could therefore
be paired with geometry captured before resize.

The repair waits for a subsequent diagnostic publication and requires its
Menu, Navigation and System rectangles to equal independently read current HWND
rectangles. Only then does the test check rail geometry. Existing four-value,
visibility, containment, ordering and 80-DIP minimum-height checks remain.
Rail fit is deliberately excluded from the synchronization predicate, so a
persistent overflow still reaches and fails those assertions. The native
controls are read again to reject movement during synchronization.

No rail width, typography, touch dimensions, AUI placement or product code was
changed on the strength of this stale observation. Eight portable tests cover
the observation barrier, including the actual current native geometry, retained
or mismatched observations, duplicate/missing controls and continued rejection
of the failing rail rectangle when paired with current chrome. Python syntax
and whitespace checks pass. Actual native 100/125/150% execution remains a gate.

Private downloaded evidence is under
`evidence/local/boat-beta2/windows-6160d3e4/unpacked/`: `dpi-results.json`,
`dpi-150-failure.png`, `dpi-150-failure.json`, and the corresponding native job
log `evidence/local/boat-beta2/ci-6160-windows.log`. No boat screenshot or real
chart material is added to the repository by this investigation.
