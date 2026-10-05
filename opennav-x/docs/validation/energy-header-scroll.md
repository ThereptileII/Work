# SCRUM-226: Energy header follows scroll content

The retained c95 native `dpi-100-energy.png` places Close over the destination
card (screen y391–435), while the immutable prototype's `energy-day.png` places
it in the header. The DPI driver leaves Diagnostics scrolled before opening
Route then Energy (`tools/smoke-dpi-windows.py`, touch-scroll sequence). These
pages share `PreviewPanel`; Instruments uses a separate unscrolled child panel.

`PreviewPanel` laid out its child Close button at viewport y28 on every size
event. If the page was scrolled, this changed the button's content coordinate
by the scroll offset. Returning to the top preserved that wrong coordinate.
The correction converts the intended content position to the current scrolled
position before setting the child rectangle. It does not pin Close onscreen:
the header still scrolls out of view, matching the original HTML's normal
`.view-header` inside the scrolling `#fullView`.

Only that size handler changes in production. Energy estimation, stale and
missing data, navigation state, page switching and shared scroll behavior are
unchanged. The original HTML and native reference images are unchanged.

## Focused offline evidence

Base: `688bb713db885dbe78ca6db861382ce248c231f7`.
Worktree: `scrum226-energy-header`. Retained local evidence:
`evidence/local/header-repro/{negative,positive}/`.
Each directory contains the executable, `build.json` (exact commands and input
hashes), compiler/run logs, `captures/result.json`, actual screen images and
`captures/header-geometry.jsonl`. The negative and positive use the identical
extended `tests/energy_panel_test.cpp`; only `PreviewPanel.cpp` differs.
The test, PreviewPanel and Controls were compiled directly against existing
unchanged libraries; no full build or OpenCPN application was run.

| Observation | Unchanged product | Corrected product |
|---|---|---|
| Energy scrolled 144 px | Close `[982,-48,80,44]` | Same; header offscreen |
| Resize 1000→1014 px while scrolled, then return to top | Close `[982,240,80,44]`; strict header check fails | Close `[982,96,80,44]`; passes |
| Actual pointer Close, hide and reopen Energy | Not reached | One close; header unchanged |
| Diagnostics scroll 456 px, resize, Route then Energy | Not reached | Header restored `[982,96,80,44]`; second real pointer close |
| Result | Expected failure at 52 checks | Pass 88 checks |

The original seven capture names and energy/data assertions remain. All seven
canonical images are byte-identical between runs. Two supplemental captures
show the returned and reentered Energy header; the latter retains the actual
Close hover/tooltip following pointer interaction. The pointer path checks
full visibility and the exact target before move and before mouse-down; Windows
uses foreground frame identity plus `WindowFromPoint`, Linux uses wx hit testing.
It verifies the callback on the following timer tick. A single-shot timer rearmed
only after Step returns prevents input/capture event pumping from reentering it.

SHA-256:

- Test source: `acf52f74ddfc461e94fc8c52afe79bf9d91b777e8e46415dc5f4fc4340f1e03d`.
- Corrected PreviewPanel: `87ffb4712c0bedb6b0d53ae9e5f75f16c34c2bfa5314f10d39d8c37c770f0685`.
- Negative executable: `5ac941e95b7d4879067fab47235e44262e9b8f01d77959c8afa96ac99e1c0baf`.
- Positive executable: `8ccab076aa940c24ad5581d6d958088c0bf8a0f516b38c29309e251b940ad9a7`.
- Negative returned header PNG: `f4f624572cdb482da1ba9e964fb29355c26fac0857afce2cd272e0b6659588d7`.
- Positive returned header PNG: `49cf92c11718cf60a4136c7ff9f133b7b3a066621187acb73dfb4fdee15e1802`.
- Positive reentry PNG: `93f36ac6f79dd984c220fca02e4ea0137e5716ac6663dc15f384136745396568`.

This Linux GTK/Xvfb 1280×800 run proves the coordinate-drift mechanism and the
bounded correction. It does not establish the precise native resize-event
ordering in c95. Native Windows compilation, physical pointer/header checks,
the original Diagnostics→Route→Energy DPI flow, and boat display acceptance
remain pending. Linux font rendering does not qualify Windows typography.
