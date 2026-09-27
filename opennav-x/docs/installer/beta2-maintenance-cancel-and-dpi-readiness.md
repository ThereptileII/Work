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

A subsequent source review caught a harness-local naming collision before boat
use: the new input-evidence dictionary shadowed the screenshot-directory Path.
It is renamed `input_evidence`. An isolated execution of the actual function
with mocked native boundaries reaches capture and touch-close checks at all
three scales. This only checks Python control flow; real Windows mouse, touch,
DPI and pixel acceptance still require the complete native gate. The earlier
candidate remains unaccepted, regardless of how far its pending run proceeds.

## Relocated uninstaller completion — 608756

The retained `608756` run reached 39 lifecycle checks and invoked the genuine
maintenance executable. Its NSIS bootstrap exited successfully, but the
relocated child was still removing verified files across the matrix's retained
generations when the harness's 120-second report deadline expired. The full
native artifact contains the later durable report: cleanup started at
08:35:53 UTC and completed at 08:38:24 UTC. In the meantime, the harness had
deleted its temporary stock installation, causing the child's final stock-hash
check to fail. This is directly evidenced timing and fixture-lifetime failure;
the failed run is not accepted retrospectively.

The local harness now allows a bounded 600 seconds for this multi-generation
uninstall's durable result, records measured completion time and still requires
`status: passed`, unchanged stock/profile hashes and removal/preservation checks.
Other maintenance actions retain their 120-second deadline. A failed fixture is
retained for the actual child's completion and diagnosis until the disposable
runner is destroyed. There is no retry, forced termination or manufactured report.
Six portable tests execute these actual harness functions: delayed completion,
missing report, failed report, unchanged Diagnostics deadline, fixture retention
on failure and cleanup on success. They do not establish native NSIS acceptance.
Product installer/application code is unchanged by this repair.

Verified `608756` evidence, API/upload/download SHA-256 and ZIP CRC:

- Installer failure `10928030623`, 523,579 bytes,
  `bede5456ef6dccd24b2297c8180d351e21a01f2cbd92863e9bdff5b942ef45eb`.
- Full native `10927099640`, 23,909,734 bytes,
  `5bb09610fc9927fb75bf8de30bdacd643b222022c1b49e016626fdb155480906`.
- DPI failure `10928110288`, 461,193 bytes,
  `408aeac9763626f65abf3114fc9404e0a35e1fce61249f1e73f0c0517728818f`.

The actual DPI visible-pixel image shows the GPS-free chart card after exactly
one right-click, with foreground/hit evidence matching the chart. The subsequent
failure is the already identified local dictionary/Path collision, corrected in
`7827acb`. This verifies first-gesture delivery only, not the remaining DPI gate.

## Direct cleanup completion — a5b290e

The replacement run `36307149573` reached all 45 named lifecycle checks,
including conventional uninstall, actual stock chart launch, reinstall and
same-version update. Operation 40 (relocated uninstall) completed successfully
after **150.406 seconds**, validating the earlier bounded report-wait repair.
The final direct PowerShell engine Uninstall (operation 43) still had a separate
120-second `subprocess.run` deadline and raised TimeoutExpired. Its failed
disposable fixture is retained. This run is not accepted.

Failure artifact `10928676941`, 572,436 bytes, SHA-256
`c05930cd9ff44f3ec5370d69ce867a9bcbbef08552ebe081799b75af51cca6ab`,
matches API metadata, upload-log digest, independent download and ZIP integrity.
The full native artifact `10929391865`, 27,761,183 bytes, SHA-256
`fb36fd49b87efa502979ebdb4a38c4fef58d00c6936147a3b7f0ed8445b15aa2`,
also passes all four identity/integrity checks. Its subsequent DPI gate passed
100%, 125% and 150%; that does not change the installer failure or qualify the
candidate. The complete earlier
`7827acb` run remains independent and continues its endurance test.

The direct uninstall now uses the same 600-second bound, records its actual
completion time, and requires both exit code zero and a successful durable
report. Other direct operations retain their existing 120-second limit. No
retry or missing-report success is introduced. Twelve portable timing/lifetime
cases pass, including delayed direct cleanup, nonzero exit, failed/missing
report, bounded timeout without retry, retained failed fixture and unchanged
diagnostic deadlines. Fresh native validation is still required.

The combined repair is in `79a95c4f39063c20ca4d5a98c3147080d9813077`,
[run 36311718794](https://github.com/ThereptileII/Work/actions/runs/36311718794).
Application/installer code remains unchanged from the boat's `7827acb` build.
The extra staging-tool test first exposed an explicit .NET assembly dependency
on Windows PowerShell 5.1; both compression assemblies are now loaded. The
earlier staging run is failed evidence, not a boat-qualified helper.
