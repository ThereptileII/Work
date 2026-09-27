# Native candidate failures and scoped test repairs

Candidate `edd8da0a4bd386fbb2dbd249289b9f09eeae8dc3`,
[run 36295867893](https://github.com/ThereptileII/Work/actions/runs/36295867893),
passed native integrated/fixture-free builds and chart/plugin checks. Its complete
Windows gate failed; there is no accepted release artifact from this run.
Downloaded failure artifacts match their API/upload SHA-256, size and ZIP CRC.

The installer passed genuine 8e780 development-package → candidate → rollback
and retained historical maintenance/uninstall, as well as the corresponding
Beta 1 rollback and interrupted-migration recovery. The later test deliberately
pressed Cancel on the actual owned Maintenance window before executing any action.
Its measured native exit code was 1. The harness incorrectly required 0.
[NSIS documents](https://nsis.sourceforge.io/Docs/AppendixD.html) 1 for user
cancellation and distinguishes the relocated uninstaller from its bootstrap.
The repair accepts exactly 1 only for that identity/hash-checked Cancel action,
requires the distinct relocation wrapper to exit 0, and compares the complete
installation inventory before/after. The caller also checks stock/profile
inventories. Install, repair, update, rollback, uninstall and app close still
require measured success; unknown exits remain failures.

The DPI check observed the Diagnostics page name before the first paint computed
its virtual height. The retained later diagnostics and screenshot show a scrollable
page. The repair waits within the existing deadline for that same page's initial
position and `can_scroll_down`, then performs the unchanged actual touch Down/Up
and viewport-movement assertions. It does not skip a scale or relax clipping checks.

A new exact-commit native run must pass both repaired checks and the remainder of
the unchanged gates. These explanations do not retrospectively turn the failed
candidate into an accepted one.

## First native chart gesture — 9f592 replacement

`9f59209914f57ff97af7e184b201752b8f422f0a`,
[run 36299835767](https://github.com/ThereptileII/Work/actions/runs/36299835767),
passes all **45** installer lifecycle checks, including the exact maintenance
Cancel behavior above. Its separate actual mouse-driven user-flow suite opens
the chart card and completes all eight groups. Software/OpenGL chart/plugin
checks also pass. However, DPI fails at 100% before any scale change: the first
desktop right-click does not produce the four chart-context actions within the
existing 15-second deadline. The saved image shows Navigation and four visible
rail values; no context card is present. This is not an accepted DPI result.

The failed harness used `SetCursorPos` and immediate native mouse events without
establishing foreground ownership or checking the hit window. Its screenshot
used PrintWindow, which can capture an inactive/obscured window and therefore
cannot establish where desktop input went. Foreground/input delivery is a
supported hypothesis, **not a proven retrospective cause**: that failed run did
not record the foreground HWND. The separately successful user-flow helper
already waits for pointer movement and checks the actual hit process.

The replacement pairs fresh diagnostic geometry with current HWNDs, brings the
main application forward and requires observed foreground ownership. After
moving the pointer, it requires the actual hit window's process and rectangle to
match the copied chart canvas before sending **one** right-click. It records the
input evidence; no missing card is retried, no scale is skipped and all card,
touch and clipping assertions remain. Failure evidence additionally captures
foreground identity and actual visible pixels. Product code is unchanged by
this test repair; the complete replacement native run remains required.

Verified retained artifacts:

- DPI failure, ID `10925524789`, 421,887 bytes, SHA-256
  `79e5d5209d27ebee247513c4690f81ac68915079db85ee04cb1b220e3a1a26e2`.
- Full native evidence, ID `10926137333`, 23,899,129 bytes, SHA-256
  `ab9383eeb504db26c3f330aeebde0ab1cf0b3609a6687eaf6d5972b23583d95e`.

Both match their API metadata, upload log, independent download and ZIP CRC.
No failed candidate was deployed; its Windows endurance and product-publication
steps were correctly withheld. The independent Linux elapsed-time run is retained
without treating it as qualification for the replacement.
