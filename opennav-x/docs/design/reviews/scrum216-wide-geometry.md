# SCRUM-216 large desktop geometry evidence

Measured the immutable `docs/design/prototype/index.html` in headless Chromium
at DPR 1 with a temporary derived copy and `getComputedStyle` probe. The probe
reported `window.innerWidth/innerHeight` to confirm the requested CSS viewport;
it did not modify the prototype. The active media cascade returned:

| CSS viewport | top/nav/rail/timeline | nav button | metric text | timeline padding | event gap/text | chart location | next turn | drawer/wide drawer |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1280×800 | 68/80/186/132 | 61 | 48 | 13px 25px | 23 / 13,10 | 28,25 | 28,102 | 398/432 |
| 1280×600 | 56/80/186/98 | 43 | 33 | 9px 25px | 17 / 11,8 | 28,14 | 28,77 | 398/432 |
| 1500×800 | 76/88/220/150 | 69 | 48 | 18px 32px | 29 / 13,10 | 34,32 | 34,116 | 398/460 |
| 1500×900 | 76/88/220/150 | 69 | 62 | 18px 32px | 29 / 13,10 | 34,32 | 34,116 | 398/460 |
| 1920×1080 | 76/88/220/150 | 69 | 62 | 18px 32px | 29 / 13,10 | 34,32 | 34,116 | 398/460 |

The values come from the final cascade, including its later global typography
rules: those set timeline event title/small text to 13px/10px after the
large-screen 14px/11px declarations. Likewise, the final drawer rule resets
the regular width to 398px; the large-screen wide drawer rule sets 460px. The
62px metric value applies only from 1500px wide and 900px high, not at
1500×800.

`tests/prototype_geometry_test.cpp` checks the browser-derived CSS-pixel values
for 1280×800, 1280×600, 1500×800, 1500×900, and 1920×1080. The portable
`display_prototype_geometry` CTest entry runs in the Linux and Win32 contract
jobs when `OPENNAV_BUILD_TESTS=ON`. To run only this focused target locally:

```sh
cmake --build build/contracts --target prototype_geometry_tests
ctest --test-dir build/contracts -R '^display_prototype_geometry$' --output-on-failure
```

The isolated local `build/contracts-focus` configuration built this target and
passed the focused CTest entry (1/1). This portable check does not replace
native Windows rendering or boat display acceptance.

The native Shell now applies top/nav/rail/horizon pane sizes, rail insets,
metric value and label sizes, and regular/wide drawer widths. Horizon reads
viewport-derived timeline padding, event gap and text sizes. A 1920x1080
X11/Xvfb component capture and 257 checks cover the painted Horizon; the
1500x800 to 1500x900 metric break is checked as source geometry and wired
through the Shell's resize-cache key. Chart-location and next-turn offsets
remain reference values: the product retains the real upstream OpenCPN chart
and does not render those synthetic HTML overlays. The native <=760px compact
pane still differs from the HTML rail/timeline layout. Native Windows
rendering and boat display acceptance remain open.

## Capture-wrapper follow-up (SCRUM-216 comment 10424)

Read-only inspection of frozen local `76bad1a` found that the component clients
had advanced beyond their native capture wrappers. Settings emits 14 states,
but its wrapper still expected 12 and inferred a theme from the final filename
word; `display-applied-125-chart` is actually the retained Night state. Horizon
emits a fifteenth `prototype-large-desktop-1920` state, absent from the wrapper's
exact allowlist, and its prepared desktop was smaller than that screen capture.

The isolated test-only correction checks the exact 14 Settings identities and
explicit per-state themes, includes the exact large Horizon identity, prepares
at least a 1920×1080 native desktop (and a matching private Xvfb screen), and
requires the large image to remain exactly 1920×1080. Existing component pass
requirements, canonical Day/Dusk/Night comparisons, one-pixel geometry bounds,
content/palette checks and control-containment assertions remain unchanged.
No reference image, product source or frozen candidate was changed.

Four inert wrapper tests pass against the current C++ capture declarations.
They reject omitted, duplicate and unknown states, cropped/resized large images,
and excessive geometry differences. Python syntax and diff checks pass. These
checks launch no application or desktop and render no image. Fresh native
component execution, screenshot inspection, OS DPI and boat acceptance remain
required; this correction is not a new native result.
