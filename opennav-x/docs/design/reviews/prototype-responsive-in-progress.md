# Prototype responsive layouts — native qualification pending

The immutable reference defines compact desktop layouts. Their exact computed
geometry is now recorded with the unchanged HTML at the logical sizes below;
these are additional references, not replacements for canonical 1280×800 DPR1.

| Physical target / scale | Logical viewport | Top | Navigation width | Rail width | Horizon | Navigation button height |
|---|---|---:|---:|---:|---:|---:|
| 1280×800 / 100% | 1280×800 | 68 | 80 | 186 | 132 | 61 |
| 1280×800 / 125% | 1024×640 | 60 | 70 | 156 | 112 | 51 |
| 1280×800 / 150% | 853×533 | 56 | 70 | 156 | 98 | 43 |

All geometry above is CSS pixels. The 125% value font is 40px; at 150% it is
33px. Compact labels are 9px; the rail preserves four primary measurements.
The original's smaller targets are still approximately 64 physical pixels
high at these two Windows scales. Its values are not rounded to the old Beta
layout. Platform fonts can change fractional text/row measurements.

Twelve Linux reference PNGs (Navigation/Preferences/Instruments, Day/Night,
both extra scales) are generated offline with original hashes verified before
and after. They do not qualify Windows native rendering.

The added native development probe uses the existing non-installed Windows
DPI/touch helper, now available when testing a fixture-free application too.
It checks actual DPI, an exact 1280×800 client, native pointer ownership,
touch-operated Settings/theme/Close, visible navigation/readings and real chart
land/water. Every scale retains screenshots and diagnostic geometry; any clipped
control fails. The old full release regression suite remains unchanged.

Native compact-shell implementation now updates only the XNav-owned fixed AUI
panes at a breakpoint or DPI change. The pinned wxWidgets 3.2.8
[LayoutAll implementation](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/aui/framemanager.cpp)
resets fixed dock dimensions from best/min sizes, so no pane detach, chart model
change, perspective rewrite or navigation processing is needed.

Observed before correction: Settings was 41px high at 1024×640 and zero height
at 853×533. The new exact compact geometry preserves all eight actions and four
readings. Two Linux resize passes (1280×800 → 1024×640 → 853×533 → 1280×800)
retain 20 images. The second exercises real wheel input to reach the lower
Preferences action at every size. Twenty-one canonical product captures and
123/123 integrated tests also pass. Evidence is
[recorded separately](../../evidence/prototype-compact-layout-local.json).

Remaining mismatches: lower vessel-profile component is not implemented, so
Settings' bottom position differs at primary/125% sizes. Preferences native text
widths wrap five tabs instead of the HTML's six on Windows. Content forms still
need migration. These are visible defects, not accepted rasterization tolerance.
The compact screenshot is a logical viewport on a Linux 1280×800 display, never
a rescaled substitute for physical/native DPI evidence.

Actual Windows compact-shell execution remains pending.
No higher-DPI or physical touch PASS is claimed. No boat display, service,
connection or actuator has been changed by this development probe.
