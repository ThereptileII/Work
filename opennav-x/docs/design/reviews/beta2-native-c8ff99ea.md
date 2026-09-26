# Native Windows candidate c8ff99ea — partial review

Commit `c8ff99eaf448147a17c02d99f3a71d430763a618`,
[run 36275173742 / job 108496882437](https://github.com/ThereptileII/Work/actions/runs/36275173742/job/108496882437).
The MSVC build and 102 integrated CTests passed. The loopback pilot smoke also
passed, recording the exact `−10°`, `−1°`, `+1°`, `+10°` course captions (the
buttons use an ASCII minus). No physical equipment was connected to these tests.

Reviewed `pilot-01-status-only.png` and `pilot-02-confirmed-manual.png` from the
downloaded native artifact at 1280×800. Degree symbols render correctly in the
course buttons and magnetic-heading units; the previous `Â°` defect is absent.
The status-only controls are disabled, confirmed simulated feedback is shown
separately, and the primary panel remains dark. This is native rendering and
loopback evidence, not boat-control acceptance.

The job then failed before the first object/chart screenshot. The Windows
desktop was 1280×800, but the actual application remained at its default
896×532. Retained configuration records that frame size; diagnostics contain a
686×379 chart and System at (771,472). The newly added object-layout wait ran
before the first `capture()`, which had previously performed the window resize.
Unlike Linux, this Windows startup path had not explicitly resized the frame.
No blank chart or clipping acceptance is inferred from this failed attempt.

The replacement harness explicitly sizes Windows before reading layout. It
requires an exact 1280×800 native frame, a chart dominant within the actual
client, four visible non-overlapping rail values, and visible controls contained
outside the chart. It also removes the Linux-specific `chart height >650`
assumption: a correctly decorated native frame can have a 647px-high chart.
Failure now captures the actual unsized/unsettled window without silently
resizing it. Existing land/water pixel assertions remain mandatory.

Pure geometry tests accept the native 647px chart and bare-Xvfb layout, and
reject the observed small frame, stale pre-resize geometry, clipped/hidden rail
values, overlaps and missing controls. The recorded actual Linux geometry also
passes this replacement check. A new exact-commit native run must still qualify
the harness correction and all subsequent gates.

The verified ZIP is retained privately under
`evidence/local/boat-beta2/windows-c8ff99ea/`; its SHA-256 is
`fe44f1ce87513af8f84d73b1f289d901cb9cc6fcdc83b3475baadb7cadbd4fa3`,
matching both the CI upload log and artifact metadata. No Beta 2 installer,
boat deployment or release acceptance is claimed by this partial review.
