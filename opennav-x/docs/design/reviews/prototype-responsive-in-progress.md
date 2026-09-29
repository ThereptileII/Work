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

Native compact-shell implementation and test execution are still pending.
No higher-DPI or physical touch PASS is claimed. No boat display, service,
connection or actuator has been changed by this development probe.
