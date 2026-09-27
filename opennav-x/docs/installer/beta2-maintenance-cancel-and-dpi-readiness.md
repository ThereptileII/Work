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
