# SKAGER status — 2026-10-19

## UI flicker — repaint corrections, native confirmation pending

The SCRUM-378 follow-up removes confirmed redundant 250 ms repaint requests
from unchanged alert-button roles, unchanged unavailable rail readings and
unchanged drawer themes/settings copy. Data assessment, observation timestamps,
source health and alert cadence are unchanged. A Windows-only buffered header
keeps native route/clock labels and sibling controls painted together during
real header updates; it does not include the chart or its OpenGL canvas.

The reported “No active route” flicker has not been reproduced on replacement
bytes: source inspection did not establish route-state oscillation, and wxMSW
already ignores identical native labels. Header buffering is a targeted
mitigation, not a verified diagnosis of that flash. Source review only; no build,
test, CI, deployment or boat operation. See the
[repaint review](design/reviews/scrum378-source-refinement.md#2026-10-19-ui-flicker-follow-up).

## Floating chart compass — source-only correction

The SCRUM-378 follow-up fixes the compass button's two-mode toggle. It now
cycles North up → Course up → Head up → North up through OpenCPN's existing
`MyFrame::SetUpMode`, with application-thread and missing-canvas guards. The
chart-layer selector already supported Head up. OpenCPN still owns rotation,
heading/course inputs and missing-input behavior; selecting Head up does not
establish that heading is available. No build, test or boat operation was run.

## Prototype refinement — source changes, existing boat UI inspected

[SCRUM-378](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-378)
adds opt-in Very Light chart presentation, effective interface/chart scaling,
four editable rail slots and named presets, source/age detail, coherent apparent
wind presentation, rounded owned surfaces and forecast refinements. The vessel
shortcut refreshes its identity and opens Vessel settings; the header uses the
active route name. Rail saves do not reconfigure sensors or disable the pilot.

The user-authorized RustDesk inspection compared the existing installed
`9feae59c` navigation/instruments screens with the immutable prototype and the
supplied Personalise screenshot. Native captures were **1920×1020 at 120 DPI**,
not a 1280×800 acceptance run. They confirmed dark depth fills, strong contours,
missing rail source/age detail, an off-screen vessel shortcut and the absence of
an apparent-wind compass fallback. The particular northern lighthouse still
needs an exact object/rule identification; a generic “Light” label does not
justify replacing every light with a tower symbol.

[The item-by-item review](design/reviews/scrum378-source-refinement.md) separates
source changes from remaining visual acceptance. No builds, tests, patch
application/checks, CI, restart, installation or hardware commands were run.
The installed version and SCRUM-376 startup-blocker status remain unchanged.

## Sailing-track controls — local source changes only

SCRUM-377 adds the approved Record/Recording header control, native recording
drawer, saved-track library/details and independent Tracks chart-layer filter.
OpenCPN retains recording/storage and daily/restart policy; SKAGER copies owned
state and withholds stale GPS samples. The filter leaves recording and each
track's stored visibility untouched, and covers software/GL drawing and chart
hit/rollover paths. Replay cannot mutate real track state.

See [the contract and deferred checks](track-recording.md). Source lists,
upstream patches and package/receipt inventories were updated and manually
reviewed. No build, test, patch check, CI or boat operation was run, as requested.
Compilation and runtime acceptance remain pending. The installed candidate and
startup blocker below are unchanged.

## Boat reconnected — startup crash blocks acceptance

The authorized 17 October check authenticated the installed `9feae59c` /
`0.4.0-beta2.5` executable and all nine retained runtime files. The desktop is
available and remote access remains healthy. Resuming the interrupted RTL-SDR
receipt correctly refuses because the user profile changed before this session;
the original DLL is already restored and the non-profile protected trees match.
Newer user settings/navigation data were preserved, not replaced to satisfy an
old hash. A fresh reviewed baseline is still required.

Actual earlier user-session evidence now shows the rebuilt builtin plugins,
o-charts, real sensor data, Online AIS and Intel OpenGL working, followed by
normal shutdowns. However, the first start crashed at **11:16:19 UTC**. Exact
dump/disassembly analysis locates serial navigation event delivery; the last
AIS-VD compatibility log line does not establish plugin causation. A concrete
shared-list locking defect is identified as a candidate mechanism, with pointer
lifetime still requiring investigation. This is
[SCRUM-376](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-376),
not a passing stability gate. SCRUM-17 remains open.

No application relaunch, build, deployment, profile modification or physical
command was performed in this check. The later source-only batch remains
unbuilt. [Exact evidence and limits](evidence/scrum17-9fe-boat-resume-20261017/README.md).

## Chart tools and first-start presentation — local source changes only

SCRUM-375 replaces the measurement-only yellow label with SKAGER presentation,
adds persistent tap instructions and Cancel/Done in the existing horizon slot,
and changes the map Waypoint action to choose a real chart position before its
existing Save/Cancel sheet. OpenCPN retains measurement math and navigation
storage. The first-start portion of SCRUM-341 adds the early SKAGER title and
themed complete navigation warning, while automatic XNav setup uses the existing
Boat Setup rather than launching the stock first-use wizard as well. Explicit
stock setup and Legacy/Safe remain available; consent and updater receipts are
unchanged. This does not claim to replace every plugin/recovery/error dialog or
resolve Windows Search identity.

See [scope, preserved boundaries and pending qualification](chart-tools-and-first-start.md).
Manual source review only: no builds, tests, patch checks, CI or boat operations.
The installed candidate and its acceptance state below are unchanged.

## Alarm settings discoverability — local source changes only

SCRUM-374 separates active notifications from **Alarms & warnings**, available
from the bell drawer even when empty and first under Settings → Navigation.
The overview explains automatic warnings, reads the current OpenCPN AIS
thresholds, and links to the existing AIS, sound, anchor, energy-reserve and
sensor-freshness settings. It does not introduce new alarm types, defaults,
suppression, acknowledgement semantics or control output. Chart safety depth
is explicitly distinguished from an unimplemented measured-depth alarm.
See [operational alarm settings](operational-alerts.md#finding-and-adjusting-alarms)
and the [presentation boundary](alert-presentation-contract.md).

Source review and test-source updates only, under the user's no-build/no-test
instruction. Native interaction, readback and replay qualification remain
pending. No CI, deployment or boat operation was performed; the installed
candidate and its acceptance state below are unchanged.

## Online AIS identity and age — local source changes only

SCRUM-16's user-feedback increment merges vessel-name reports per MMSI, includes
AISStream metadata names, and uses MMSI as the chart label when no name exists.
Online display aging/stale/lost/expiry becomes 3/10/20/30 minutes; local AIS and
vessel data keep their existing rules. Static data never refreshes position age.
See [the contract and unperformed qualification](online-ais.md#cache-and-time).
Source review and test-source updates only: no build, test, CI, deployment or
boat operation. The installed candidate and its outstanding gates are unchanged.

## Route passage settings — local source changes only

SCRUM-373 adds an optional per-route radial arrival distance and a separate
geographic passage width. Active-passage display defaults on, with a chart-layer
toggle. Existing routes retain stock waypoint rules until explicitly configured;
shared waypoint radii are not rewritten. The configured strict policy is shared
by the integrated XNav/Legacy/Safe modes. See
[route passage settings and unperformed qualification](route-passage-settings.md).
The user's no-build/no-test instruction remains in effect. These local source
changes do not replace or qualify the installed candidate.

## Track/Wind follow-up — local source changes only

The user requested native Track/Wind controls and controller Wind corrections
under SCRUM-313 while retaining the instruction to make code changes without
building or testing. The changes are separate from the installed candidate below.
Both final mode-command allowlists, Track's explicit measured-AUTO prerequisite,
guarded native route-data delivery and firmware apparent-wind handling are the
scope; AutoTrack's automatic heading loop is not copied. See
[the source-only contract and outstanding qualification](pilot-track-wind.md).
There is no new build, execution result, firmware upload or boat acceptance for
this work. Original release/boat evidence below remains unchanged.

## Replacement candidate — native Staging passed; installed, boat acceptance blocked

The user made the desktop available on 16 October. The same installed `854078a5`
process then provided fresh real chart content, initialized o-charts, native Intel
OpenGL, measured vessel data and connected Online AIS. No second launch or
physical command was sent. However, all four rebuilt builtin plugins were
refused: the dependency guard counted missing lookup candidates against its
256-node limit. **That previous candidate was not accepted.**

The exact three-root graph needs 270 probes for 62 found dependencies. Correction
`896ad520` separates 1,024 bounded lookups from 256 admitted dependency PEs and
the unchanged 256 MiB aggregate limit. All 9,930 portable checks and two
independent reviews pass; obsolete-TLS refusal and native custody are retained.
The replacement version is `0.4.0-beta2.5`. Its policy sources at
`0615e79ba122a32d5adb6193ab5c5b9bfad30ce9` pass the same 9,930 checks in
focused native Win32/MSVC
[38910455080](https://github.com/ThereptileII/Work/actions/runs/38910455080).
Authenticated artifact `12049428405` and its actual compiled source hashes match
that revision's Windows checkout. This is a policy
fixture pass, not application/plugin-load acceptance. The first integrated run
[38910703440](https://github.com/ThereptileII/Work/actions/runs/38910703440)
failed the distribution preflight because the coordinator bumped `Version.h`
without the matching installer `openNavVersion`. It was cancelled before the
Windows application build; the original failed job is preserved. The manifest
now matches, with its supported stock hashes unchanged. The existing local
distribution preflight passes all 16 required inputs and the one stock
configuration; the focused workflow now runs it before any compilation too.
This packaging-only correction does not change the already-tested policy sources.
Corrected candidate `9feae59ce0ef1f54107c3c51f920085297ea7bc8` is now selected in
[38911440527](https://github.com/ThereptileII/Work/actions/runs/38911440527).
The distribution preflight and native contract/commissioning checks pass. The
Windows updater prerequisite initially stopped at the unchanged 60-second cap
for an inert package test; no assertion failure was reported. The identical
predecessor suite passed in 56.119 seconds. One failed-job retry, after Linux
completed, passes the same 14 cases in 48.163 seconds and all 126 package cases
without changing code, assertions or deadlines. The original timeout remains
recorded. Native updater job `117028247900` passes in attempt 2. Windows producer
`117036304896` passes at 15:49:44 UTC with authenticated dependency reuse:
146 CTest cases pass in each native variant, and native loader/TLS checks,
recovery/Setup packaging and immutable input retention pass. Qualifier
`117061245460` passes at **16:18:43 UTC**, using those retained bytes without
recompilation: actual wizard completion, all **four packaged-updater checks**
(bootstrap, supervised update, fault rollback and restored startup), clean
installation/first-install rollback and functional charts pass. The complete
Staging run succeeds on attempt 2; successful independent prerequisites remain
original attempt-1 evidence. Original Linux job `117008195582` passes all
154 CTest cases in each build variant, actual chart/data/lifecycle checks and
both loader checks. No design or endurance sweep was selected.

The coordinator authenticated actual Setup, recovery and source downloads from
draft release `415154258`, tag
`skager-staging-0.4.0-beta2.5-run38911440527-attempt2-9feae59ce0ef`.
Setup is 187,490,120 bytes, SHA-256
`662821c1e3ceb87a915129c03b684bb33dd6486e8fdbee464c31766b1840dfb9`.
**Replacement boat installation passed; ordinary builtin-plugin startup remains
pending.** The authorized update installed `9feae59c` / `0.4.0-beta2.5` in
generation `cd6a841a0a574c9f872457e09e82c51f`. The 16:39:33 UTC cold observation
verified the executable, adapter, three networking libraries and four builtin
DLLs against authenticated recovery bytes. Stock OpenCPN and the original INI
hashes were unchanged, and previous generations remained available.

The boat unexpectedly rebooted at 16:40:25 UTC before any new application launch.
The exact stock RTL-SDR DLL had already been moved back, but the final restoration
receipt was interrupted. Reconnected inspection verified its original bytes and
the durable restoration intent. The 16 October continuation safely refused while
the interactive Windows desktop was unavailable. Kernel-Power 41 / EventLog 6008 establish an
unexpected shutdown, not its cause; no deployment command requested a reboot.
The 17 October resumption above supersedes the desktop-only blocker: actual user
profile changes prevent the historical preservation assertion, and a captured
startup crash now blocks acceptance. Preserve the journal and current user data;
review a new baseline rather than overwriting either. Do not reinstall or rebuild
merely to repeat already captured evidence.
[Installation and interrupted recovery evidence](evidence/scrum17-9fe-install-20261016/README.md).

Native CI and cold file identities do not establish ordinary boat startup,
protected-host B0/B1 or physical GPU acceptance. Public access remains closed;
this is a private draft Staging delivery, not Production promotion.
The original timeout and earlier failed candidate evidence remain preserved.
This docs-only update from `95f503ed` does not change frozen product `9feae59c`.
All 30 SDK-bound inputs match the locked producers, and this run passed its
fresh native environment verification.
[Correction, exact inputs and limits](evidence/scrum209-plugin-probe-budget-20261016/README.md).

The observed application closed normally with exit 0. Four missing owned builtin
DLLs were recreated cold from the authenticated original 854 recovery archive,
with exact hashes and unchanged profile bytes. The ordinary hash-bound
commissioning restore then passed: original INI and plugin inventory restored,
all other current profile files preserved. A 13:48:47 UTC observation confirms
the exact stock executable and original INI hashes, no active commissioning
marker, no application/helper processes or running SKAGER tasks, and running
SSH/Tailscale/RustDesk services. The application remains closed; restoring the
original output configuration is not authorization to launch or send commands.
Original failed evidence remains preserved. That recovery does not establish
installation or ordinary startup acceptance of the replacement candidate.
[Actual startup, failure and recovery evidence](evidence/scrum17-854-ordinary-startup-20261016/README.md).

## Previous candidate — private composed Staging delivered, boat refusal retained

Frozen product `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`, version
`0.4.0-beta2.4`, was built once in
[38765418126](https://github.com/ThereptileII/Work/actions/runs/38765418126),
attempt 1. Linux passes both 154-case CTest executions and its chart, lifecycle,
synthetic-input and fixture-free checks.
[Original Linux results](evidence/scrum22-linux-854078a5-20261015/README.md).
Windows producer `116536443214` passes both 146-case CTest executions, native
AIS/TLS checks and packaging.
[Original native scopes and counts](evidence/candidate-854078a5-native-outcomes-20261016/README.md).
All five original Windows artifacts and the
Setup/recovery/source inventories are authenticated.
[Exact package custody and limitations](evidence/candidate-854078a5-custody-20261016/README.md).

The original Windows qualifier `116562831922` fails later. The corrected
installer commits its generation and completes the wizard; first updater
bootstrap then fails while waiting for authenticated startup. The application
logs thirty seconds of normal processing, but that local recovery checkpoint
does not prove delivery, receiver acceptance or persistence of the startup
receipt. The launcher reports a generic supervision failure and keeps its
error dialog open; the helper subsequently reaches its unchanged 240-second
limit. The child PowerShell error was not retained, so its exact cause remains
unestablished. Moving the wait after application closure or raising the timeout
would not repair this contract.

Helper-only revision `2ff32b3439365b86a67b66a54c7baf95cd4cdb21` adds a bounded
diagnostic in the existing retained workflow. Native
[38878511171](https://github.com/ThereptileII/Work/actions/runs/38878511171)
restores the exact 854 package and installs successfully, but its WOW64 host-path
query returns an empty buffer and fails before invoking the supervisor. This is
a helper failure, not new evidence of the product cause. The underlying API
behavior has not been established. Corrected helper
`293eae1213e44adc6eed5728ee6f62a438760b94` validates a bounded host query before
restoring inputs; all 21 focused checks pass. Its single corrected native run,
[38880343949](https://github.com/ThereptileII/Work/actions/runs/38880343949),
passes that early I386-host preflight, then the original supervisor exits 1:
opening `transaction.lock` fails because another process holds it. No timeout
or receipt is recorded; all observed installed inputs are unchanged.

Both the original qualifier and corrected diagnostic retain a Finish screenshot
with **Open SKAGER checked**. The helper tries to clear only the obsolete caption
`Launch SKAGER Beta 2` and does not assert discovery or state. The frozen installer
wires this checked option to the installed launcher, allowing an additional
untracked start before the helper's explicit launches. The checkbox mismatch is
confirmed; the exact lock holder and attribution of the original healthy
checkpoint to a launch are not established.
[Original diagnostics, checked-control evidence and limits](evidence/scrum22-wizard-launch-20261016/README.md).
Helper `3fe8866fa191a72eb47290368311cbed11463664` now requires the unique
owned native launch checkbox, clears it, and revalidates the same handle and
unchecked state immediately before Finish. All 34 focused checks and independent
review pass. The actual retained installer/charts run
[38882281891](https://github.com/ThereptileII/Work/actions/runs/38882281891)
passes using the original package: all nine installer checks and four real
updater mechanics, including authenticated startup, same-version pending update,
pre-launch corruption refusal, guarded rollback and restored clean startup.
The Finish checkbox is recorded checked before and unchecked after. The separate
charts fixture passes; requested OpenGL falls back to software on the hosted
runner, so physical GPU acceptance remains open.
[Authenticated replacement suffix](evidence/scrum22-installer-charts-854-20261016/README.md).
Diagnostic results cannot grant
qualification. All original failed attempts remain failed; no application
rebuild or boat deployment follows from this investigation.

The exact new-capture PluginHandler gate
[38881652663](https://github.com/ThereptileII/Work/actions/runs/38881652663)
passes all 13 cases and rejects the ignored-failure mutant. Trust removal and
cleanup pass with zero forbidden HTTP requests. Its authenticated capture and
same-image host do not qualify other runtime combinations. The final package's
three replaced MSVC DLLs have separate post-copy selftest/restart and seven
production-recovery checks, including actual fixture-free XNav/Legacy/Safe
launches over the verified 1,046-file package; no extra CRT-specific run is
selected. This is not a per-DLL loaded-hash census or a transfer of all 13
PluginHandler cases to the final DLL combination.
[Exact component acceptance and package distinction](evidence/scrum211-retained-854-20261016/README.md).
[Composition 38885572956](https://github.com/ThereptileII/Work/actions/runs/38885572956),
attempt 1 at `686c2c553e40cb207c468e1ce01400cb9bb2d811`, now passes both
assembly and draft-release jobs. It binds the original 21 reports and replacement
five reports while preserving the original failed qualifier and frozen product.
The actual Setup downloaded from [private draft 414814496](https://github.com/ThereptileII/Work/releases/tag/untagged-6d43932bcfbfd132b178)
has the original 187,484,268-byte size and SHA-256; all 11 release asset metadata
records match the manifest. Assembly records `productRebuilt=false` and no
application execution. The history-preserving merge retains strict historical
schema-2 compatibility; all 25 focused composition checks and independent review
pass. Earlier 15 composed-delivery and 20 release-delivery checks remain recorded.
[Composition, draft download custody and exact boundaries](evidence/scrum290-composed-854-20261016/README.md).
Protected signed B0/B1 and interactive UAC acceptance, fresh boat validation and
physical GPU acceptance remain open. Design/endurance runs remain off, no
Production promotion occurred and public access remains closed.

The authorized boat Update on 16 October failed **before atomic commit** when
the exact stock `rtlsdr_pi.dll` was imported: it requires the absent
`wxbase312u_vc_custom.dll`. This is the previously recorded wx 3.1 compatibility
gap, now an actual installer refusal. At 10:35:52 UTC, the installed `f1a1e68c`
generation, executable, state, stock executable and profile INI hashes remain
unchanged; no application/helper is running and all three remote-access services
are running. Before the attempt, the complete current profile and existing
recovery copy matched across 4,206 files. The original 850-byte failure report
is retained privately with SHA-256
`cd58d5b19e54dc003868e5338235c59683abea3ded4f0d526a5e4fd96fb2fefa`.
No automatic retry or incompatible wx DLL substitution is authorized by that
result. SCRUM-17/22 track a narrow reversible hold/restoration of this exact old
plugin and review of the remaining imported PE closure. The existing
commissioning quarantine cannot be used during maintenance.

The bounded read-only scan of that failed generation completed over **103 PE
files**: both missing imports belong to the same RTL-SDR entry DLL; no other
missing PE import was reported. Original source and PE inventories remained
unchanged. This is dependency diagnosis, not a successful load or launch.
[Exact scan and preserved failed diagnostics](evidence/scrum17-rtlsdr-854-20261016/README.md).
The narrow hold/restoration helper now passes **49 native checks** in
[38891027391](https://github.com/ThereptileII/Work/actions/runs/38891027391),
including the boat's native scheduled-task action shapes. Its earlier malformed
INI fixture failure and separate 38-check pass remain preserved. No application
rebuild was required.
[Versioned native helper evidence](evidence/scrum22-rtlsdr-hold-20261016/README.md).

The subsequent actual boat Inspect/Hold/Update/Restore sequence **passed**.
At 11:16:41 UTC the selected installation is frozen product `854078a5`, and the
exact stock RTL-SDR DLL is restored. The original profile, stock installation,
managed plugin tree and previous rollback generation are preserved. The installer
mandatory loader/resource self-test passed; no ordinary navigation launch is
included in this result. Fresh inventory confirms unchanged stock/managed trees
and only the four expected owned plugin DLL changes in the new generation.
Independent source/identity review passes. The guarded passive cold launch
started exact 854 at 11:47:20 UTC. The parent dispatch timed out during
prelaunch recovery verification; the same child later returned an authenticated
pass, without a second launch. Native logs confirm Intel UHD / OpenGL 4.6
initialization. Read-only window inspection confirms the existing Swedish
OpenCPN welcome dialog; no new-version diagnostics or chart-content acceptance
is claimed yet. The corrected helper accepts only the supported owned version
matching the embedded build, including `0.4.0-beta2.4`. Native
[38897699400](https://github.com/ThereptileII/Work/actions/runs/38897699400)
passes 127 warning and 33 dispatcher-budget contract checks at `bc2d1d5a`;
the other eight jobs are skipped. Its 164 exact Windows tool files are staged in
a separate immutable directory, preserving the active restart tools. The same
application remains the subject of guarded warning review. Actual Focus refused
before mouse input because Windows Widgets covered the caption. Separate normal
activation/inspection then refused an unavailable input desktop, with no capture
or acknowledgement. The user has been asked to make the ordinary desktop
available; no repeated UI attempt or generic input is selected. An older owned
read-only whole-log diagnostic was conclusively identified and stopped after it
reached 26.1 GB private allocation; the application/profile/services were untouched.
This is not evidence of a product memory leak. The commissioning remains input-only
and its recovery records are intact.
[Actual refusals and diagnostic cleanup](evidence/scrum17-warning-desktop-20261016/README.md).
[Native evidence and limits](evidence/scrum17-installed-warning-20261016/README.md).
[Actual Update and restoration evidence](evidence/scrum17-854-install-20261016/README.md).

SCRUM-211's certificate-verification software criteria were independently
reviewed and accepted on 16 October; Jira is **Done**. SCRUM-212's explicit
unavailable-peer containment alternative is also **Done**, supported by exact
854 model, CLI, concurrent-instance and mode-preservation evidence on both
platforms. This does not qualify an authenticated sharing protocol.
[Containment evidence and precise limits](evidence/scrum212-containment-854-20261016/README.md). This closes that defect,
not the separate installer, protected-updater or boat gates. The shorter Staging
result is supplemented by the retained package's Repair/Uninstall and
retained-addition lifecycle evidence below; ordinary installed module/plugin
observation remains pending for SCRUM-209. The focused full retained installer-lifecycle run
[38889555934](https://github.com/ThereptileII/Work/actions/runs/38889555934)
passes 35 grouped lifecycle checks and all four updater checks, then stops in a
qualification helper: its extracted SelfTest definition calls the omitted
`Assert-LifecycleMutation` dependency. Repair, both historical migrations and
retained obsolete-TLS refusals pass; the final current-product uninstall/recovery
suffix is not reached. Helper `d742a20c` includes the real production
mutation/protected-enrollment/armed-update guard dependencies and a distinct
retained `installer-lifecycle-tail` scope. Independent review confirms the
original suffix assertions are preserved. Native
[38896539839](https://github.com/ThereptileII/Work/actions/runs/38896539839)
now passes 16 native import/guard checks and all 16 suffix labels over fresh real
installed state using original 854 bytes. The 15 operation records include
expected negative refusals. Missing-DLL/error-mode handling, fault recovery,
current uninstall, stock startup/resource preservation, reinstall/Update and
final direct Uninstall pass. Its original artifact and frozen product/source
identities authenticate. This distinct result does not relabel the original
failed full run or claim the historical prefix was replayed.
[Authenticated suffix and exact limits](evidence/scrum22-lifecycle-tail-854-20261016/README.md).
No product rebuild or Production promotion occurred.
[Original failed run, passing prefix and exact cause](evidence/scrum22-full-lifecycle-854-20261016/README.md).
SCRUM-22 is **Done** for supported installation and unsupported-build preservation,
with the additionally selected lifecycle evidence above. Ordinary boat acceptance
and the protected customer-UAC/B0/B1 path remain separate open gates.


## Installer trust correction — native boundary and replacement installation pass

Product correction `890ca0ab`, with owned-child lifetime support `b7f0329e`,
removes the installer/launcher transaction-lock collision. Trust initialization
starts only after the generation is committed; the existing native image owner
pins the launcher and its physical ancestors across the lock handoff. The child
revalidates the selected generation under its own exclusive lock. The parent
waits for exact child exit, reacquires the lock and checks installation identity.
Only the exact unattended `--initialize-trust` command suppresses the blocking
failure dialog; ordinary startup retains its error guidance. The operation
deadline remains fifteen seconds, with separate retained custody until exit.

Focused native [38764328627](https://github.com/ThereptileII/Work/actions/runs/38764328627),
attempt 1 at `b375e6bd3d743993a1415729becdc73ee3e47575`, passes six Go tests
and 28 subtests without skips, then all 13 boundary cases under each actual
x86/x64 Windows PowerShell 5.1 host. Real first/repeated trust initialization,
missing configuration and nonmodal refusal use the freshly built GUI launcher;
separate inert children test custody, intervening mutations and timeout cleanup.
The original artifact, launcher identity and both native receipts authenticate.
[Exact focused evidence and limitations](evidence/scrum22-installer-trust-20261015/README.md).
The focused check alone is not installed-package or boat acceptance. The new
launcher and lifecycle engine were then included in replacement candidate 854
above, whose initial installation and retained startup/updater journey now pass.
All 30 SDK source inputs remain unchanged, allowing the existing strictly
verified two-image dependency selection. Design and endurance runs remain off.

## Previous Staging candidate — retained failures, not deployable

The previous coherent candidate is
`e80cb962e1a32d98bb75b964dca6eb516f7a5e23` /
[38744236920](https://github.com/ThereptileII/Work/actions/runs/38744236920),
attempt 1, still `0.4.0-beta2.4`. It includes the focused native installer PE32
extent and AMD64 helper fixes, the Linux Dashboard readiness fixture correction,
and the authenticated two-image SDK selection described below. The original
full run was requested once with private Staging trust; extended/design suites
are off. Linux job `116453779566` now passes: 154/154 integrated CTests and
154/154 fixture-free CTests, the original software/OpenGL chart/mode gate, and
all five fixture-free loader/resource checks. The original artifact digest and
3,984 member CRCs verify. The corrected Legacy-to-XNav readiness assertions pass
without polling pane state until green. Native Windows contracts also pass
115/115 CTests and all ten additional restart repeats.
[Exact original Linux/component evidence and limits](evidence/scrum22-linux-e80-20261015/README.md).

The original Windows producer `116461918332` passes both 146-case CTest
executions, native AIS/TLS and private chart module binding, recovery packaging
and installer compilation. [Authenticated original native outcomes and scope](evidence/scrum209-native-e80-20261015/README.md).
The downstream qualifier `116490177486` stops before installer lifecycle in
`smoke-navigation.py`'s object workflow: the second waypoint-edit dialog is not
found after the preceding edit/save and delete/cancel interaction. The first
edit completed; a static dialog-name mismatch is not established. Helper
`842ded95` now waits for the exact owned delete modal to close and its owner to
re-enable after the existing single Cancel. Seven existing and 17 focused checks
pass, including an event-queue reproduction that rejects the former sequence.
The original failure's precise input ordering was not captured and is not claimed.
One [retained full runtime retest, 38757982655](https://github.com/ThereptileII/Work/actions/runs/38757982655),
used that separate helper revision against the unchanged e80 package. The waypoint
interaction now passes, followed by recovery, user-flow and product smoke checks.
The retest fails later in fresh installation: the Lifecycle engine holds its
exclusive transaction lock while its trust-initialization child attempts the same
lock. The child reports failure through a blocking startup dialog; the parent
records a 15-second timeout. The lock/error-dialog chain is source-supported;
the child error text was not captured in the original log. This is a product
installer/launcher defect, not a reason to increase test timeouts. A focused
lock-handoff correction and native cross-process check are in progress before
a new product package is requested. The e80 installer is not deployable.
Setup, recovery and source leaves, their sealed inventories and exact hashes
remain independently authenticated.
The separate retained PluginHandler run
[38756849052](https://github.com/ThereptileII/Work/actions/runs/38756849052)
refused both attempts before native tests: the allocated newer Windows image
selects a different authenticated SDK from the one bound to this older-image
candidate. Exact TLS binding correctly stops; there are no further allocation
retries. Reviewed correction `724adadc` separates current-host tool reprobes from the
candidate's original authenticated subject SDK. Thirteen subject-policy and
38 orchestration checks pass; original SDK payload and source bindings verify.
[Native retained probe 38761760564](https://github.com/ThereptileII/Work/actions/runs/38761760564)
passes all 13 PluginHandler cases, the ignored-download-failure mutant and
trust cleanup. Its actual host and subject both use the original older-image
SDK; it does not claim execution of the newer-host/older-subject combination.
Application SDK/image verification remains strict. [Original custody, both refusals and runtime boundaries](evidence/scrum211-retained-e80-20261015/README.md).
Exact packaged lifecycle, the focused native gate and fresh boat acceptance remain
pending. The full-runtime composition extension was withdrawn in `591fc9ad`: it
was designed for a modal-helper-only repair and cannot qualify this defective
frozen installer. Its work and focused results remain in history; the existing
accepted delivery path is unchanged. Both original and retained full runs remain
failed, not relabelled. [Retained prefix and installer failure](evidence/scrum22-retained-e80-20261015/README.md).
The boat remains on its previous installed revision; a passive availability
check confirms SSH, Tailscale and RustDesk running with no OpenCPN/chart-helper
process. No actuator output, Production promotion or public release is
authorized by this run.

## Previous candidate — preserved qualification failures and focused corrections

`1ffd09ddf2fa6588def2907485f6e14e89bc3a7d` /
[38722465573](https://github.com/ThereptileII/Work/actions/runs/38722465573)
is the single replacement candidate, still `0.4.0-beta2.4`. It incorporates the
bounded installer refusal-report correction `df02dec2`. The actual original
catch path reproduces the missing report; independent review finds no blocking
issue. Before requesting this application build, native
[38722080504](https://github.com/ThereptileII/Work/actions/runs/38722080504)
passes the corrected engine under both x86/x64 Windows PowerShell 5.1: 39 new
report checks per architecture, including real foreign-shell COM and signed
barriers, plus the existing filesystem, shortcut and protected-broker checks.
[Authenticated focused result](evidence/scrum22-installer-operation-report-20261015/native-receipt.json).
The application run retains private Staging trust, unchanged verified SDK and
all relevant package/security gates; extended/design suites are off. No second
application build is active. Packaging remains necessary because the fix changes
the installed lifecycle engine. Native helper success does not qualify either
the replacement package or the failed original package.

The Windows producer `116388013514` now succeeds. Its original retained-input
qualifier `116405193249` fails fresh installation because the installer's PE
reader rejects the packaged wxWidgets export directory. The earlier foreign
Start Menu refusal/report case passes. Source inspection finds the separate
installer reader still contains the 1 MiB export/RVA assumption already removed
from the application reader. Correction `5a3d0ebb` accepts the fully backed
1,158,912-byte wx export extent while retaining PE32/file/section and independent
import/EAT/forwarder/name limits. Focused native
[38734026147](https://github.com/ThereptileII/Work/actions/runs/38734026147)
at `f880ef8f` passes x86/x64 Windows PowerShell 5.1 fixtures and the authenticated
retained wx DLL, without building the application. All 88 final packaged PEs are
also covered by inert local parsing: 78 unchanged compiled PEs plus the ten
changed/new package files. The original package remains unqualified; the corrected
installer still needs integrated lifecycle acceptance.
[Original candidate and corrected native parser evidence](evidence/scrum209-candidate-1ffd09dd-20261015/README.md).

Fresh passive boat inventory also identified a retained RTL-SDR AMD64 executable
helper which the former x86-only installer scan refused. Correction `6786a1e5`
adds bounded PE32+ import/forwarder inspection and per-edge architecture checks,
without changing the x86 application/plugin ABI. The child helper uses its own
application directory; a finite native Microsoft VC14 exception requires valid
Microsoft signatures and matching architecture. The focused suite and original
retained wx bytes pass native Windows PowerShell 5.1 in both x86/x64 hosts at
[38740580580](https://github.com/ThereptileII/Work/actions/runs/38740580580).
Seventeen exact retained helper/CRT files also pass inert local parsing with
unchanged hashes. These are parser/closure checks, not external-helper TLS-version
approval or installed-product acceptance. No boat program was launched or
profile changed. [Authenticated native and local evidence](evidence/scrum209-installer-pe64-20261015/README.md).

Linux job `116380307497` builds and passes preceding checks, then fails the
OpenGL Legacy-to-XNav workspace assertion. Its tick-3 diagnostic still reports a
visible Dashboard pane. Initial helper correction `9833ddbc` requires a newer
publication but does not establish deferred Dashboard readiness. Correction
`8b4497d9` now exposes the existing presentation phase only in fixture builds
and binds the returned diagnostic to the actual new process, that phase and a
pre-restart publication baseline. The first ready snapshot still faces every
ENC, pane, rail and native-visibility assertion once; a failing pane is never
polled until green. Fourteen helper tests, 18 actual Dashboard component checks,
fixture-enabled/disabled compilation and independent review pass. The phase
does not promise native repaint completion. The original failure remains
preserved; actual corrected Linux execution and the skipped fixture-free build
remain required.
[Original failure](evidence/scrum22-linux-chart-readiness-20261015/diagnosis.json);
[focused phase correction](evidence/scrum22-linux-workspace-readiness-20261015/verification.json).

The focused SCRUM-283 exposure fixture now also passes native MSVC x86 at
`63abf84b` / [38735688962](https://github.com/ThereptileII/Work/actions/runs/38735688962).
All four actual-source variants pass, retaining 26 baseline and 288 exposure
records, 93 original/corrected differences and the required original failure.
The independent source-provenance findings are corrected by `66cdad37`:
closed staged includes, exact private inventories and owned-header identities.
The previous host-selection failure is retained; the corrected lane selects a
single Python/Perl executable. Only consumed locked wx inputs are prepared,
with all 1,612 extracted files verified unchanged. No application or private DLL
was built. [Authenticated native receipt](evidence/scrum283-exposure-parity-20261015/native/native-receipt.json).

The separate actual associated-area query fixture (`d625c6b6`, reviewed original
`f70c0eb8`) now also passes Linux and native x86 MSVC. Native
[38740153247](https://github.com/ThereptileII/Work/actions/runs/38740153247)
at `2f9c41cb` executes 29 original/repaired query cases with 113/125 checks;
both original missing-callback and polygon-bypass negatives fail as required.
Actual pinned query, polygon and coordinate bodies retain original semantics
and connect through the repaired callback to isolated-danger/DisplayBase output.
All 1,612 wx files remain unchanged. Test geometry is pre-tessellated fixture
data: chart ingestion, tessellation generation, whole private DLL ABI/lifecycle,
software/OpenGL drawing and real-chart/boat acceptance remain open.
[Authenticated actual-query evidence](evidence/scrum283-actual-area-query-20261015/native/README.md).

The guarded older-image dependency producer at `bfbfdca7` /
[38735490052](https://github.com/ThereptileII/Work/actions/runs/38735490052)
now succeeds on `20261004.326.1`: OpenSSL 4,283, curl 1,569 and zlib 13 checks
pass. Its original artifact `11976566007` is authenticated, with all 296 ZIP
members and 295 payload files checked. All 30 bound inputs match the current
candidate. The independently reviewed exact-image selector (`b8f0a012`) passes
27 focused checks and now selects this SDK or the existing authenticated
`20261012.333.1` SDK by the actual runner identity. Unknown images still refuse;
no image, source, toolchain or artifact check is relaxed.

All four maintained runtime DLL hashes differ between producers; inert byte
comparison locates the differences in build metadata/date bytes. This supports
unchanged source/code evidence but does not transfer an exact-byte native
PluginHandler receipt. The next coherent candidate must retain its selected
inputs and complete any required focused native runtime qualification. These
SDK/helper results do not qualify the failed application candidate above.

## Previous candidate and preserved installer failure

`b4c40e6e9166d7e8340dec2879d2a8ad510b7458` /
[38705678784](https://github.com/ThereptileII/Work/actions/runs/38705678784)
is the previous candidate. It includes the native-tested installer observer
correction and the authenticated retained-plugin results below. Private Staging
trust is selected; extended and design suites are off. The unchanged verified
SDK is reused subject to the existing fresh native source/environment checks.
Attempt 1 Windows integration refuses before compilation: hosted image
`20261004.326.1` differs from the SDK's required `20261012.333.1`. Windows
updater verification and native contract jobs pass. Original Linux integration
job `116324544446` now passes, including both executions of the 154-case CTest
suite; its original artifact is authenticated. GitHub declined the first
failed-job-only rerun request while the overall run remained active, then
accepted it after terminal state. Attempt 2 job `116340382423` reruns only
Windows integration and its dependents on the same source, but receives the
same older image and refuses at the same early gate. A dependency-only producer
then receives the newer `20261012.333.1` image and refuses the explicitly
requested older image before any build. Independent inspection confirms all 30
bound SDK inputs still match the frozen candidate. On that new allocation
evidence, a selected Windows-only attempt 3 is requested: job `116350139778`
passes SDK authentication, image/Python/source preflight and native AIS/TLS,
both application builds, installed security checks, recovery packaging and
installer compilation. Its two CTest executions each pass the same 146 distinct
test names. Exact retained-input qualification job `116364569049` fails the
foreign Start Menu group refusal case: Install returns the expected failure
code, but its operation report is missing. Source inspection identifies the
failure-report writer requiring an installation root which the refusal correctly
never created. The native exception itself was not retained, so that mechanism
was then reproduced by the focused actual-catch regression. The bounded fix and
independent review passed under SCRUM-22 before the replacement above was
requested. Installer compilation alone does not qualify installed behavior.
No SDK, source or environment check is relaxed. The proposed multi-image selector is retained on
its separate branch, unused without a second qualified SDK.
[Dependency allocation and unchanged SDK binding](evidence/scrum209-candidate-977d37c7-20261015/dependency-old-image-preflight-receipt.json).
[Original allocation failure](evidence/scrum209-sdk-image-allocation-20261015/b4c40e6e-receipt.json).
No installed-product or boat acceptance follows yet.
The earlier `977d37c7` package remains failed rather than being relabelled.
[Authenticated native producer evidence](evidence/scrum209-candidate-b4c40e6e-20261015/native-producer-receipt.json).

## Focused recovery and installed-updater qualification tools

The historical WMM reference fix is integrated as `0defee7a`, independently
reviewed from `87949177`. It retains the prepared-parent and current generation
identities and derives a separate third-generation proof only from the original
profile bytes. Its 226 portable checks pass. Native-only
[38717837403](https://github.com/ThereptileII/Work/actions/runs/38717837403),
exact helper/workflow `ab722554`, passes **229 native checks** with native
profile-lock/ACL coverage. It tests the changed reconciliation contract without
repeating unchanged transaction/adoption suites or compiling the application.
[Authenticated native result](evidence/scrum17-cold-reconciliation-20261015/historical-wmm-native-receipt.json).
The same 161 tool files are staged and hash-verified on the boat. A fresh
observation matches the independently reviewed 18 changes and three WMM
generations, with the original profile unchanged and no application running.
An initial private-review staging attempt refused an unprotected new child
directory before Plan; read-only inspection confirms the original inspection
directory remains protected and unchanged. The review now uses a fresh sibling
created by the maintained private-directory helper, without changing the
original ACL. Actual Plan `60666477…` binds the unchanged profile baseline;
its single Commit completed successfully. Postverification confirms all four
original quarantine paths restored, their quarantine copies absent, the current
profile unchanged and the active commissioning marker removed. A fresh identity
check at 12:08 UTC retains installed `f1a1e68c`, stock OpenCPN and the exact
profile hashes, with no application running and SSH/Tailscale/RustDesk healthy.
[Actual boat reconciliation receipt](evidence/scrum17-cold-reconciliation-20261015/boat-completion-receipt.json).
The original timed-out Plan observation is preserved, without inventing an exit
status or repeating the operation. Cold reconciliation is accepted; a new
candidate installation, fresh read-only commissioning and navigation acceptance
remain separate gates. No physical output was sent or authorized by this work.

A fresh read-only boat inventory at 12:46 UTC binds all 13 plugin candidates
across three complete trees to the same unchanged profile. No application was
launched or profile changed. The first invocation was refused by the session's
PowerShell execution policy before script execution; the corrected invocation
uses a process-only policy without changing permanent system settings. Exact
vendor helper/source review and installed-candidate qualification remain
separate from this inventory.
[Sanitized inventory receipt](evidence/scrum17-cold-reconciliation-20261015/fresh-inventory-receipt.json).

Installed-updater helper `14ff2896` (reviewed original `02a943f6`) adds explicitly
bound interactive self-hosted admission for a disposable Windows target.
The original declaration hash, independent approval and one-use consumption
remain mandatory. Its 222 focused checks pass; independent review found a
preparation request could expire during host observations. Follow-up `84d79bc7`
(original `0ba45caf`) closes that gap with a post-observation deadline check;
the new expiry test reproduces the old failure and passes after correction,
alongside the valid path. Actual external target, qualified B0/B1 and observed
human UAC execution remain required. `a66826ea` explicitly classifies only five
reviewed qualification helpers; 20 classifier checks pass, with runtime and
unknown paths retaining conservative build selection. These helper commits do
not change or relabel the frozen `b4c40e6e` application.
Follow-up `9b3c4a32` requires actual `RUNNER_ARCH=X64` for the explicit
interactive self-hosted path; 70 affected checks pass, including early refusal
for missing/ARM64 architecture. Hosted behavior is unchanged. Native interactive
target and human UAC acceptance remain outstanding.

## Replacement candidate after native updater job timeout

Candidate `e538bbc8` / [38633971333](https://github.com/ThereptileII/Work/actions/runs/38633971333)
exceeded the Windows updater job's former 20-minute limit after all executed
steps succeeded. Windows application integration never started. Correction
`38fca523` preserves tests and allows 35 minutes on Windows, retaining independent
matrix results. Its already uploaded operator kit passes original artifact,
inventory and payload verification; it is not a qualified application package.

The single replacement is `89854110b41286cab0c6bf9748f37882a595a366` /
[38638855800](https://github.com/ThereptileII/Work/actions/runs/38638855800), with
private Staging trust and without extended/design suites. The verified SDK is
unchanged. The superseded run's remaining Linux compilation was cancelled;
no separate verifier-only build or dependency rebuild was selected.
[Failure, bounded component evidence and correction](evidence/scrum311-updater-budget-20261014/README.md).

Replacement attempt 1 passes Linux integration, including the fixture-free
build/loader gate, and the Windows updater job. Its application job was
allocated older hosted image `20261004.326.1` and correctly refuses the SDK's
required `20261012.333.1` before compilation. After the original run ended,
the coordinator requested only failed Windows job `116117298706` and its
dependent jobs again. Attempt 2 job `116128956769` passes the exact SDK
image/Python/source preflight and completes native compilation and retention on
the same frozen candidate. Successful Linux/updater results are retained; Linux
records **154 fixture CTests, 154 fixture-free CTests and 19 Python tests**.
No dependency rebuild or image-check relaxation is selected.
[Original allocation failure](evidence/scrum209-sdk-image-allocation-20261014/README.md).

Attempt 2's downstream qualification job `116150037326` fails the mode smoke's
Day-to-Dusk diagnostic assertion. Its original evidence remains failed; this is
not a compilation failure. Helper `c1dd0268` uses checked foreground/pointer
input for Day → Dusk → Night → Day and preserves all destination and chart
assertions. Five focused portable checks pass. Retained-only
[38684111006](https://github.com/ThereptileII/Work/actions/runs/38684111006)
tests the exact `89854110` application/package bytes with this separate harness.
It fails before sending the Day click: the native window inventory shows a
startup error dialog listing all four bundled plugins and blocking the main
frame. The test correctly refuses the covered control; dismissing the dialog
is not selected as a workaround. The original result archive authenticates.

The original mode log also shows `chartdldr`, `dashboard`, `grib` and `wmm`
refused before loading with `Invalid export directory extent`. The shared
import parser applies that check to dependencies as well as each named plugin.
This is not evidence that obsolete TLS was detected or that plugin compatibility
passes. The authenticated captured `wxmsw32u_core_vc14x.dll` has a fully backed
1,158,912-byte export directory, exceeding the parser's arbitrary 1 MiB extent
ceiling. Product correction `630d04d8` removes only that ceiling; file/RVA,
entry, forwarder and name budgets remain unchanged. Focused Linux checks pass
9,920 import and 17,625 forwarder assertions, including ASan/UBSan. All 91
distinct retained DLL/EXE byte sequences and eight declared plugin dependency
walks pass the corrected reader without executing the files.

Native guard [38685332524](https://github.com/ThereptileII/Work/actions/runs/38685332524)
passes on exact `630d04d8`: 17,625 forwarder assertions, 282 TLS byte-policy
assertions and all 58 required loader groups. Original artifact identity,
CRC and all 28 source/workflow hashes verify. The normal-startup smoke now also
requires all four bundled plugin filenames; Safe Mode is exempt. These focused
results do not qualify the old `89854110` bytes. A new integrated application
candidate is required; package, installed lifecycle and boat acceptance remain
open. No design or endurance suite is selected.
[Authenticated failure and correction boundary](evidence/scrum209-plugin-export-extent-20261015/README.md).

The new frozen candidate is `977d37c79f652fd9b9e28e55f26ec259c35eea8e` /
[38686209123](https://github.com/ThereptileII/Work/actions/runs/38686209123),
dispatched once with private Staging trust, extended tests off and design
validation off. Independent review confirms the parser bounds and unchanged SDK
producer inputs. Final helper correction `977d37c7` waits for one newer native
diagnostic publication after startup readiness, then checks plugins immediately;
it never waits for a failing plugin list to turn green. Eleven plugin/freshness
and five palette helper cases pass. The dispatch did not install on the boat. Compilation and downstream outcomes
for this frozen candidate are recorded below; it remains an unqualified package.

Attempt 1's Windows integration job `116266638089` is allocated older image
`20261004.326.1` and refuses the selected SDK before native setup or compilation.
The original failure remains recorded. Its Windows updater job succeeds with
1,060 top-level Go tests, 14 skipped tests and 133 Python checks. Original Linux
job `116261621665` completes with **154 fixture CTests, 154 fixture-free CTests
and 19 Python tests** on exact `977d37c7`; its original log and job identity are
retained in the [candidate receipt](evidence/scrum209-candidate-977d37c7-20261015/linux-receipt.json).
After attempt 1 ended, only the failed Windows integration/dependent jobs were
requested again. Attempt 2 job `116274722975` succeeds with 146 fixture CTests,
146 fixture-free CTests, 12 earlier Python checks and seven retention checks.
Successful Linux/updater results are retained; no dependency rebuild or
image-check relaxation is selected. Downstream qualification `116290535162`
fails installer Preflight: PowerShell reports the native NSIS temporary
`System.dll` as an invalid assembly. The original log is retained. Source inspection identifies the first cold
`Add-Type` in the protected-broker observer: the NSIS extraction directory can
shadow framework assembly resolution with its native `System.dll`. Correction
`f259a731` compiles from the trusted framework directory and restores both caller
directories in `finally`. Focused native run
[38700086122](https://github.com/ThereptileII/Work/actions/runs/38700086122) on
`22f91084` passes PS5.1 x86/x64 cold success, actual compiler-path failure and
restoration, 44 filesystem checks per host, shortcuts and protected-broker
refusals. Its earlier negative-fixture classifier failure remains recorded.
These checks qualify the changed observer boundary, not the immutable failed
`977d37c7` installer. A new packaged candidate is required; original package
assembly/delivery is skipped. [Focused installer and original peer runtime
evidence](evidence/scrum209-candidate-977d37c7-20261015/README.md). [Native and downstream receipt](evidence/scrum209-candidate-977d37c7-20261015/native-receipt.json).

Independent review verifies all 1,585 retained plugin-input file hashes, exact
source projections, 17 maintained TLS bindings and 11 Win32 runtime closures.
After the producer becomes terminal, one
[retained native PluginHandler gate](https://github.com/ThereptileII/Work/actions/runs/38697150458)
is dispatched with reviewed inventory hash `7666db6cce687cb045c7b58b4145bd1bb312c09daf6cd69bafc76047635fdfd9`.
Attempt 1 refuses an older hosted image before SDK restore. Attempt 2 receives
the exact selected image and authenticates the retained inputs, but its separate
PluginHandler probe fails to compile because archive headers include Windows
Winsock 1 before wxWidgets selects Winsock 2. Windows-only include correction `36dc0e65` passes
[38704113599](https://github.com/ThereptileII/Work/actions/runs/38704113599),
using separate harness `b096faa2` and the same immutable producer/inventory.
The harness authenticates its own exact revision and 99 consumed inputs, allowing
only explicitly reviewed probe/console differences with both hashes retained.
Independent review closed three missing transitive authentication imports before
dispatch; focused 36 workflow and 39 capture checks pass. No SDK is rebuilt;
production downloader slices and assertions remain unchanged. This is not an
application build failure. Original artifact `11957991338` authenticates by
identity, size, SHA-256 and CRC. All 13 native PluginHandler cases pass, the
ignored-failure mutant is rejected, cleanup is verified, and 58 loader, 17,625
forwarder and 282 TLS byte-policy checks pass. The gate does not rebuild the
application or qualify the failed installer. Independent source audit confirms
the SDK's 29 maintained inputs and producer workflow remain unchanged; the next
coherent application/package build reuses that authenticated SDK with fresh
native environment checks.
[Independent input review](evidence/scrum209-candidate-977d37c7-20261015/plugin-input-review.json).

## Bounded AIS helper and boat recovery preparation

Cold reconciliation helpers `998f7521` / `fc150311` keep the old commissioning
transaction and the later user-installed generation distinct, preserving the
current profile and restoring verified quarantine bytes only to original paths.
Independent review found and corrected interruption gaps before atomic profile
publication and after final marker removal. Exact portable helper checks pass
51 reconciliation and 127 baseline-adoption cases. A fully prepared journal-owned
stage resumes; incomplete or changed staging still refuses with evidence retained.
The [original native/Linux gate](https://github.com/ThereptileII/Work/actions/runs/38692225251)
on `fc150311` passes Linux (86 session cases, including 51 reconciliation, plus
127 adoption) but fails Windows at the exact creation-time/attributes assertion;
adoption never starts and the native transaction step is skipped. Original
artifact identity, size, SHA-256 and CRC authenticate. The journal stored a
DateTime using PowerShell 5.1's millisecond JSON representation; correction
`87b55cbb` stores exact decimal ticks instead. Exact native metadata/ACL and
lineage assertions remain unchanged. The original log lacks expected/actual
values, so final causal attribution awaits native confirmation. Independent
review passes and 56 corrected portable reconciliation cases pass.
The [corrected focused gate](https://github.com/ThereptileII/Work/actions/runs/38693208374)
on exact `87b55cbb` passes both platforms; neither run rebuilds the application.
Windows passes 86 session cases with 59 nested reconciliation cases, 131 adoption
cases (including four native invocations) and 27 separate preservation transaction
groups. Linux passes 86 session cases with 56 nested reconciliation cases and
127 adoption cases. Original artifacts authenticate by run/commit, size, SHA-256
and CRC. These are helper/file-operation results, not whole-product or boat
acceptance. [Native receipt](evidence/scrum17-cold-reconciliation-20261015/native-receipt.json).
[Correction and retained failure evidence](evidence/scrum17-cold-reconciliation-20261015/correction-receipt.json).

The qualified helper's actual boat Inspect now completes with a private full
profile backup and 18 reviewed differences. The entire current connection value
matches the original direction, so preserving this profile requires no INI write.
Plan/Commit remain unperformed. Integrated helper `f6234063` adds a separate
v2 retained-current-state review for the exact owned WMM resources, bounded
saved pilot/configuration fields, AIS radius, Weather flag and monotonic
Dashboard distance. It cannot publish or change the current profile, qualify a
saved pilot binding or authorize output; ordinary v1 session policy stays strict.
Restrictions survive later baseline lineage, and subsequent passive launch
requires a fresh source review bound to the exact build/profile/transport and
plugin decisions. The initial native Windows/Linux gate
[38701955180](https://github.com/ThereptileII/Work/actions/runs/38701955180) passes
Linux (86 session, 131 nested reconciliation, 127 adoption) but Windows stops
at the combined passive-source binding assertion. No real Plan/Commit follows.
A pure synthetic PS5 probe reproduces different ordering for raw Hashtable and
JSON-decoded plugin records. Correction `d397ebfb` projects the same seven fields
before sorting, with stable permutation/roundtrip and per-field mutation checks;
both synthetic native permutations now agree. All original identity comparisons
remain, with field-name-only diagnostics. The newly transitive reconciliation
module is also pinned in fresh restart tool copies. Local reconciliation 133
and broker-closure 28 checks pass.
[Original refusal](evidence/scrum17-cold-reconciliation-20261015/v2-native-refusal.json)
and [isolated diagnosis](evidence/scrum17-cold-reconciliation-20261015/v2-canonicalization-probe.json)
are retained. Corrected native gate
[38703828242](https://github.com/ThereptileII/Work/actions/runs/38703828242) passes
on exact `d397ebfb`: Windows 86 session cases with 136 nested reconciliation,
131 adoption and 27 explicit transaction groups; Linux 86/133/127. Both original
artifacts authenticate by run/commit, size, SHA-256 and CRC. Full installed
restart acceptance remains separate. The exact 161-file helper copy is staged
on the boat, and fresh Inspect completes with a full private backup: 4,229
profile entries, 4,483 ACL observations and the same 18 differences/hash. Its
4,525,948-byte report exceeds the generic reader's 4,194,304-byte limit, so the
subsequent read-only review-input read refuses before Plan/Commit. Integrated
helper `f26e1123` adds a dedicated 16 MiB owned-inspection reader/producer,
hashing and parsing the same bounded bytes. The producer verifies readability
before reporting success. The generic 4 MiB limit and full tree/ACL checks
remain unchanged. Independent source review finds no confirmed defects;
165 focused portable checks pass, including complete large inventories, the
exact size ceiling, invalid UTF-8, wrong record/context/hash and retained
tree/ACL refusals. Exact helper run `38709570067` / `1f17fc80` passes Linux and
the Windows prerequisite phase. The Windows job then reaches its 10-minute
limit during the separate transaction phase; original prerequisite logs/artifact
are retained. Explicit transaction-only continuation
[38711468946](https://github.com/ThereptileII/Work/actions/runs/38711468946)
passes 27 transaction and 23 native profile-lock labels on `177f2203`.
All 161 helper files match `1f17fc80`; the original run remains cancelled.
[Composed helper evidence](evidence/scrum17-cold-reconciliation-20261015/inspection-bound-transaction-receipt.json)
does not rebuild or relabel application candidate `b4c40e6e`.
Exact `1f17fc80` tools are staged on the boat and read the original inspection
successfully. Independent review of all 18 actual differences finds a distinct
historical WMM resource reference in the original profile: it is neither the
prepared-parent nor current generation. Read-only inspection verifies that
third generation's ownership and all three source-pinned resource files;
the current profile remains unchanged. The strict existing two-generation
review refuses this case, so Plan/Commit has not started. An explicit historical
reference proof is needed; substituting review values or rewriting the profile
is not permitted.
[Actual size refusal](evidence/scrum17-cold-reconciliation-20261015/v2-actual-size-refusal.json).
Boat checks confirm the same profile/active-marker hashes, no running OpenCPN
and all three remote-access services running. The current profile and active
transaction are unchanged, no application was launched, and no physical command
was sent. Retention must not qualify the saved pilot binding or grant launch authority.
[Sanitized actual inspection receipt](evidence/scrum17-cold-reconciliation-20261015/boat-inspection-receipt.json).


Separate helper `f1b6fc18` adds three fixed Online AIS settings actions with owned
window/control checks and fresh post-click observations. Independent review and
364 portable policy checks pass. Native helper run
[38687593659](https://github.com/ThereptileII/Work/actions/runs/38687593659)
fails before the first new click because its inert heading panel lacks the
accessible name already supplied by the existing AIS-list fixture. The original
artifact's SHA-256/size/CRC verify; no application was launched and cleanup reports
no errors. Fixture-only correction `5f1d0cc1` supplies the same heading name and
checks the direct-child accessibility shape. Runtime guards remain unchanged.
Native rerun [38688203646](https://github.com/ThereptileII/Work/actions/runs/38688203646)
on helper revision `0dbc87b3` passes all **78 inert-window cases**, including the
**14 new Online AIS cases**, plus 364 policy and 192 restart-policy checks. The
original artifact authenticates and both reported native source hashes match the
exact Git revision with Windows checkout line endings. Actual product accessibility
and boat AIS responsiveness remain pending; neither enabling a preference nor
disabling it proves a network-outage test.
[Bounded native receipt](evidence/scrum301-online-ais-actions-20261015/native-helper-receipt.json).

Read-only boat observations on 2026-10-15 confirm the selected application is
`f1a1e68c`, while the still-active commissioning record identifies older
`c0d8d85f`. The old generation and all four quarantined DLLs/backups remain intact;
their original paths are empty and their parent directories exist. Current
connection configuration exactly matches the recorded original baseline, including
COM8 input/output. Existing restoration correctly refuses the changed installation
context. A bounded reconciliation path is being prepared to preserve current
settings and authenticated original-path backups; the old record will not be
rewritten or blindly discarded. No boat profile, file or physical output has been
changed by these inspections.

## Disposable desktop provisioning remains insufficient

Helper `81cd0821` uses documented Windows-managed desktop access only for the
fresh test account. Focused checks pass **52 probe / 43 customer-helper cases**.
Native [38639838756](https://github.com/ThereptileII/Work/actions/runs/38639838756)
still times out without child entry markers or exit. All caller desktop/UAC
before/after checks pass unchanged, but no child desktop report is available.
Original artifact and source bindings verify. No unchanged probe retry is
selected, and no product crash or customer/UAC acceptance is inferred.
[Native evidence](evidence/scrum311-limited-managed-desktop-20261014/README.md).

The next focused diagnostic retains the original child and observes bounded
thread CPU/I/O, fixed module presence and window classes before resume and at
timeout. It changes no launch or privilege policy. A separate create-only,
disconnected ProgramData profile adapter exercises the actual Windows `.ini`
and navigation observer. Existing or partial profiles refuse without repair or
cleanup. **60 probe / 17 profile-adapter checks pass**. Native
[38643305138](https://github.com/ThereptileII/Work/actions/runs/38643305138)
passes profile preparation but the original child still times out. Its original
primary thread reports no accounted CPU or pending I/O; only NTDLL is observed
among the seven retained module names. This narrows the startup observation but
does not identify an exact wait/cause or a SKAGER crash. Both original artifacts
verify. No equivalent retry, application rebuild or boat change is selected.
[Native observation and boundaries](evidence/scrum311-startup-observation-20261014/README.md).

Helper `90ef6cf9` adds a bounded, read-only `SystemProcessInformation` query at
those same two observation points. It reports only the original process thread
count, primary-thread state and conditional wait reason, with original-handle
identity/liveness checks before and after. **68 focused probe tests pass**;
independent source review found no material findings. Native
[38645101455](https://github.com/ThereptileII/Work/actions/runs/38645101455)
still times out but the query succeeds: one original thread, waiting state 5,
wait reason 5 before resume and 17 at timeout, with 15.625 ms accounted user CPU.
This rules out calling it never resumed/scheduled; the blocking cause remains
unidentified. No launch policy, privileges or application inputs changed.

The subsequent host-only console discriminator `e97356e1` /
[38645846804](https://github.com/ThereptileII/Work/actions/runs/38645846804)
requests `CREATE_SUSPENDED | DETACHED_PROCESS` for the file-only worker. Windows
refuses error 87 before creating a child. No fallback occurs; owned disposable
cleanup passes, caller desktop/UAC stays unchanged and profile preparation passes.
This rejected flag combination will not be retried; it grants no acceptance.

The fixed, retained `pythonw.exe` sibling follow-up `7d0e051a` /
[38647259035](https://github.com/ThereptileII/Work/actions/runs/38647259035)
passes 73 focused checks and independent review. Native Windows creates and
resumes the matching original LIMITED child once, then records exit 1 with no
file markers/report. The exact worker failure remains unproved; cleanup and
disconnected profile preparation pass. Both original archives authenticate.

Further hosted launch variants are not selected for customer-UAC acceptance:
independent review confirms the unchanged hosted policy silently elevates and
the genuine signed-in customer FULL/LIMITED primary-token context is absent.
That acceptance needs an explicitly admitted disposable interactive Windows
host with normal human secure-desktop consent, qualified B0/B1 and the same
account/profile through the journey. Do not use the boat as a disposable host
or spoof the current hosted-only admission. This is a test-host limitation,
not a passing installed-product gate or evidence of a SKAGER crash.

The separate callable installed-journey CLI is integrated as `b5c5da7f`:
**13 CLI / 23 coordinator groups pass**. It preserves independent input and
operator approval and performs no signing, publication or connectivity changes.
Qualified B0/B1, original artifact authentication and a usable Windows host
remain required before invoking it.

## Installed journey coordinator passes focused native streaming gate

`e0f20d1e` / `fc76ee9e` / `03e5bcc1` join the existing approval transport and
original-input checks to a native one-use record and one maintained installed
journey. The qualification-only native helper retains the same protected parent
across the approval wait. The Python adapter observes the original receipt file
through an independent cursor on the same native object. Existing/partial records,
changed originals, timeout or uncertain effects refuse without retry/reset.

Focused portable checks pass **18 coordinator groups, 3 streaming-contract groups,
5 native-bridge groups / 17 subcases**. Windows x86 test-executable compilation
passes. The one focused native streaming run is
[38636051433](https://github.com/ThereptileII/Work/actions/runs/38636051433) at
`6424f9706341e8132b359ec316a8f8fb78114255` and passes **10 native top-level
groups / 31 including subcases**, all five mandatory groups, zero failed/skipped.
The actual Python streaming adapter passes; original archive hash, length, CRC
and member bytes verify. [Native evidence](evidence/scrum311-coordinator-native-20261014/README.md).
Actual installed/UAC acceptance is not granted by that fixture. These later helper revisions do not change the frozen
application candidate `e538bbc8` / [38633971333](https://github.com/ThereptileII/Work/actions/runs/38633971333).
[Coordinator contract](installer/installed-updater-live-coordinator.md).

## Combined candidate blocked by obsolete assertion; corrected local suite passes

Exact `f30b92e70b5357e505925547015155ef38837e73` in
[38632343277](https://github.com/ThereptileII/Work/actions/runs/38632343277)
fails `TestRollbackAndMissingChannel`: its old text assertion does not recognize
the authenticated-absence error type introduced by `6671af7a`. The correction
checks that exact error type and absence of release/payload, preserving the
existing rollback rejection. No runtime behavior changes.
The corrected complete Linux updater suite passes **790 top-level cases** across
**13 packages**, zero failed/skipped cases; three command packages contain no
tests. [Failure and repair evidence](evidence/scrum311-typed-absence-repair-20261014/README.md).

Original Windows job logs prove both new native fresh-cache tests and all five
refusal subcases passed without skips before matrix fail-fast cancelled the job.
That is bounded execution evidence, not a passing updater gate. Windows application
integration did not reach its SDK guard. The failed candidate and original logs
remain retained; no candidate package or boat installation is accepted from it.

## Fresh enrollment cache preparation is integrated for the next candidate

`924787ec` adds the explicit independently approved operator preparation missing
from the maintained fresh-B0 journey. It exclusively creates an absent protected
cache and exact original root, with flush/readback; existing, damaged, partial or
redirected caches refuse. Existing customer discovery never initializes or resets
trust state. The joined journey forwards this step before Setup/enrollment.
Independent source review found no blocking defect.

Focused Go preparation/parser tests, **39** B0 composition cases, **12** joined
journey cases and **12/14** broker/provisioner source-package cases pass. Windows
x86 compilation passes; the two new native fixture cases still require execution
in the combined candidate. CI must fail rather than skip them if elevation is
unavailable. No separate full build was requested per subtask.
[Preparation contract](installer/broker-bootstrap-policy.md).

## Installed customer component update passes focused native gate

Admission `04ea88e1`, command/native launch `7391a0a0`/`fc5733b7`, and
Setup/Maintain action `f08b2911` now join the fixed installed helper to its
independently verified successor. The dedicated **Update service components**
action never invokes legacy application mutation or claims an application update.
Normal protected application updates retain the existing startup offer and consent.

Independent review caught a detached failure-drain goroutine that would lose
custody at command exit. The corrected path keeps the command alive until the
original child handle proves exit; no resumed kill or retry is introduced.
Shared native cache refresh locking also preserves retained-root continuity.

Combined focused Linux checks pass **22 top-level / 82 including subcases**,
zero failures/skips. Broker/provisioner source-closure suites pass **12/14**.
Exact `7dde3b150bddfbb1e8a6c41a7439928227ddbf8d` passes
[38628998074](https://github.com/ThereptileII/Work/actions/runs/38628998074):
**391 Windows x86 / 275 Linux top-level passes**, no failures. All **138/102**
mandatory cases pass without skipped subcases; five earlier Windows skips remain.
Original artifact/stream/source receipts verify.
[Evidence](evidence/scrum311-installed-customer-20261014/README.md).
NSIS wizard behavior, actual installed UAC/SCM journey, full application packaging
and boat acceptance remain pending. No public
or boat deployment has been performed for this path.
[Updated contract](installer/broker-code-upgrade.md).

## Limited-account standard-handle repair does not resolve hosted startup

Helper `ad274297` removes the proven invalid supplied standard handles and uses a
bounded fixed file in the actual token's new profile. Focused checks pass
**43 customer / 45 probe cases**. Native [38636933058](https://github.com/ThereptileII/Work/actions/runs/38636933058)
still times out after one verified resume, with zero diagnostic bytes and no
observed exit. The original artifact and exact-source hashes verify. This is not
a diagnosed SKAGER failure or usable interactive-host acceptance. No unchanged
probe retry is selected; next diagnosis must distinguish a native startup block.
[Evidence](evidence/scrum311-limited-profile-file-20261014/README.md).

## Hosted ordinary-account probe identifies invalid child standard handles

The next bounded diagnostic `fe21e95a` passes **45/34** focused portable cases.
Native [38634616586](https://github.com/ThereptileII/Work/actions/runs/38634616586)
checks the original suspended child before resume: all three supplied stdin,
stdout and stderr handle values refuse duplication with `ERROR_INVALID_HANDLE`.
Original child identity checks passed; it was not resumed. This establishes a
handle-binding defect, not the cause of the older timeout or usable UAC/desktop
capability. No SKAGER installation or boat mutation occurred.
[Authenticated observation](evidence/scrum311-limited-child-handles-20261014/README.md).

### Earlier missing-progress observation

The diagnostic-only change `66ec07ee` passes **42/34** focused portable cases.
Native [38630784527](https://github.com/ThereptileII/Work/actions/runs/38630784527)
resumes the original limited child once, but observes neither its flushed entry
marker nor an exit before the unchanged timeout. Authenticated original artifacts
retain the bounded zero-byte progress and deferred cleanup. No SKAGER install was
attempted, and no concrete product defect or stall function is established.
Actual customer/UAC acceptance stays pending on a suitable interactive host;
unchanged probe retries and permission/timeout weakening are not selected.
[Evidence](evidence/scrum311-limited-host-progress-20261014/README.md).

## Customer discovery: retained-root correction passes focused checks

Independent signed-fixture reproduction found direct bootstrap/code-upgrade
discovery can overwrite retained root v2 with the original v1 anchor when the
remote rotation link disappears. Ordinary installed release checks preserve v2
through their authenticated generation state. Fix `cfb9fc64` preserves the
retained root before go-tuf construction; damaged existing caches refuse without
reset. Focused Linux checks pass **20 top-level / 96 subcases**, no failures or
skips. Windows x86 compilation passes; native customer qualification remains pending. Shared cache-refresh
serialization is now integrated in the customer increment above. No public or boat deployment uses
this new customer path. [Evidence](evidence/scrum311-root-continuity-20261014/README.md).

Optional-target absence is now a typed result only after live authenticated
nondelegated targets prove absence. Offline, expiry, signatures, rollback,
delegation and missing advertised bytes remain errors. The focused discovery
absence regression passes all eight cases locally; it awaits the next combined
native customer-admission gate.

## Authenticated upgrade recovery and original cache: focused native gate passes

Exact `3039528767aa71b801a63f258e4fb3d829e21e6e` passes
[38621115904](https://github.com/ThereptileII/Work/actions/runs/38621115904):
**366 Windows x86 / 255 Linux top-level passes**, no failures. All **113/82**
mandatory cases pass without skipped subcases; five existing explicit Windows
skips remain. Original artifacts, digests, lengths, CRCs and exact-source receipts
verify. The first fixture-setup failure remains preserved below.

SCRUM-311 now discovers the exact signed successor through the existing TUF
authority without manufacturing successor pins from local files. Complete,
already-validated DEMAND successors get one durable configuration attempt before
the existing independently fenced normal start. Early intent/completed-stage
interruptions can restore original-reader eligibility only after verifying the
unchanged running original pair and publishing a distinct terminal abort; partial
unproven copies still refuse. Configuration, start and query-only rights remain
separate. The customer Setup/Maintain admission and genuine desktop journey remain
open; these components do not qualify installed recovery by themselves.

The combined focused Linux check passes **28 top-level / 94 including subcases**,
with no failures/skips. Broker/provisioner source-closure suites pass **12/14**;
the integrated Windows x86 test binary compiles. First native run
[38619809617](https://github.com/ThereptileII/Work/actions/runs/38619809617) passed
253 Linux and 361 Windows top-level cases, with one Windows fixture-setup failure
and five existing skips. The empty demand-attempt fixture now creates its exact
protected descriptor at birth; production checks and refusal assertions remain.
Independent review checked shared dispatch, ownership and reader exclusions;
deferred cleanup uncertainty now also revokes terminal-abort classification.
[Contract](installer/broker-code-upgrade.md).

New enrollment binds the original protected TUF cache into schema-2 completion;
existing schema-1 maintenance stays compatible. The customer upgrade must retain
that exact cache instead of resetting its floors. Reviewed source and focused
portable checks and the corrected recovery/new native cache components pass.
Actual customer admission/elevation is still being
joined. [Evidence and limits](evidence/scrum311-code-recovery-20261014/README.md).

## Frozen candidate refused newer Windows host before application compilation

Candidate `9b12442d9de9ad1fd91780f74cdfa43eba261dc6`,
[run 38615517743](https://github.com/ThereptileII/Work/actions/runs/38615517743),
received hosted image `20261012.333.1`. Its early SDK gate correctly refused the
selected producer's `20261004.326.1`; native application compilation and packaging
were not reached. This is an environment qualification failure, not an observed
application crash. The Linux integrated regression job completed successfully: **154/154 tests**
in each fixture-enabled and fixture-free application configuration. Original
artifact source/hash/length and ZIP CRCs verify.
[Linux evidence](evidence/scrum209-linux-candidate-9b12442d/README.md).
The original job log is retained
privately; the missing evidence-directory upload is a secondary failure after
that early refusal.

The explicit dependency-only refresh passed at source `68c83517` in
[38618335277](https://github.com/ThereptileII/Work/actions/runs/38618335277) on
`20261012.333.1`. Original artifact `11924018896`, its SHA-256, full inventory,
source/recipe inputs and native dependency test evidence verify. The lock now
selects this exact SDK. Fresh native reuse passes at `6752261e` in
[38630308437 attempt 2](https://github.com/ThereptileII/Work/actions/runs/38630308437),
including five native tool reprobes and runtime checks, with no dependency or
application compilation. Attempt 1 correctly refused the older concurrent hosted
image before setup; its evidence remains preserved.
[Artifact audit](evidence/scrum209-sdk-refresh-20261014/README.md). The new guard passes five
focused actual-workflow-program checks. It does
not force GitHub's allocation or bypass the consumer's exact environment checks.
No duplicate full application build or blind retry is authorized by this result.

## Stopped successor recovery: focused native gate passes

Exact `9b12442d9de9ad1fd91780f74cdfa43eba261dc6` passes
[run 38615096156](https://github.com/ThereptileII/Work/actions/runs/38615096156):
**344 Windows x86 / 240 Linux passes**, zero failures. Five earlier explicit
Windows skips remain; all 91/67 mandatory maintenance cases pass. Original
activation and clean AUTO_START/STOPPED recovery now share an exclusive durable
single-start fence. Query-only finalization retains the same actual process;
configuration downgrade and repeated uncertain starts refuse. The initial empty
fixture-construction failure and its exact-byte evidence remain recorded.
[Scope and receipts](evidence/scrum311-stopped-start-20261014/README.md).

One full private Staging candidate at this source was dispatched in
[38615517743](https://github.com/ThereptileII/Work/actions/runs/38615517743), reusing
the unchanged authenticated SDK subject to same-job environment verification.
No design/extended run was requested. Its Windows environment refusal is recorded
above; full application/installed/boat acceptance remains pending. Later recovery
increments are described above and cannot qualify that frozen candidate.

## Earlier running successor finalization gate


SCRUM-311 now recovers the bounded interruption after an exact complete successor
pair is already AUTO_START/RUNNING but before final ready publication. A separate
native capability observes the same retained process/token/image and authenticated
history with query-only service rights, then exclusively publishes the absent
marker. It cannot configure, start or stop the service. Existing disabled rename
recovery is preserved. Cooperative executor-query access and its inherited
abrupt-death limitations are documented; this is not a claim of zero effects.

Exact `39f5b7e60d18498c329c37f34471dcb1a03a3969` passes
[run 38602308806](https://github.com/ThereptileII/Work/actions/runs/38602308806):
**340 Windows x86 / 237 Linux passes**, zero failures. All 87/64 mandatory
maintenance cases pass, including five new portable and four inert native tests.
Five existing explicit Windows skips remain. Original artifact hashes, lengths,
ZIP CRCs and exact-source receipts were verified. Source-closure checks pass
12 broker / 14 provisioner cases. No full application rebuild was required.
[Evidence](evidence/scrum311-running-finalize-20261014/README.md).

Normal Setup/Maintain activation, actual installed SCM/customer/UAC/boat
qualification, early partial staging, DEMAND validation, AUTO+STOPPED and partial
immutable-publication recovery remain open. This is bounded primitive acceptance,
not complete customer recovery or public release approval.

## Earlier checkpoint: existing broker code upgrade operator foundation

SCRUM-311 now separates existing-code replacement from initial enrollment with
a distinct signed TUF target, exact predecessor broker/helper hashes and an
independently approved successor tuple. The publisher retains those exact
approved bytes across ordinary application releases; conflicting or corrupted
authority refuses before publication reservation. Original enrollment, user
profile, trust/floors and installed startup publication cannot be reset to make
an upgrade succeed.

The native operator increment supports one immutable successor and authenticated
resume through the four file-rename boundaries while the original service is
disabled and stopped. It remains **unqualified and disconnected from normal
Setup/Maintain**. Early partial staging and interruptions during store validation
or after service activation still refuse; complete customer recovery is not
claimed. Focused native kernel/reader tests are required separately from the
actual SCM, same-account UAC, installed upgrade and boat journeys.
[Contract and explicit limits](installer/broker-code-upgrade.md).

Exact `eceef0843df49d8a930623cf4edaf83406eabec7` passes focused native
[run 38597858395](https://github.com/ThereptileII/Work/actions/runs/38597858395):
**331 Windows x86 / 232 Linux passes**, zero failures. All 78/59 mandatory cases
ran and passed; five existing explicit Windows skips remain unaccepted. The
first run's foreign-file reread conflicted with its own retained DELETE handle;
the fixture now releases that probe and additionally compares the original full
file identity. Publisher-case requirements match their Linux-only lock contract;
all earlier Windows controls remain. No full application build or installed
customer acceptance is inferred.
[Passing receipts and preserved first failure](evidence/scrum311-code-upgrade-native-20261014/README.md).

## Customer Repair integrated; focused native gate passes

The reviewed Repair batch is integrated through `8e0331b9`: exact retained-source
restoration, cold restart profile custody, and explicit continuation of one
authenticated interrupted copy. The new journal binds its prior original reader
before its first fact, so an interruption at that complete durable boundary can
be continued without reopening an old journal for append. Independent review
found no blocker in that scope. Malformed/torn writes, lost private authority,
damaged bootstrap and old fixed broker/helper compatibility remain explicit
limitations. [Contract and remaining gates](installer/customer-protected-repair.md).

All 12 new production Go modules are mandatory in the corresponding-source
closure; customer rollback sources are also explicitly required. Both package
verification suites pass locally (12 broker + 14 provisioner cases). Exact
`de87b132f6f8dbbb7711b586854e412a8e23ea9d` passes short native
[run 38587874517](https://github.com/ThereptileII/Work/actions/runs/38587874517):
**317 Windows x86 and 214 Linux top-level passes, zero failures**. Windows has
five explicit skips; all required 64 Windows / 41 Linux maintenance cases ran
and passed without skipped/failed subcases. Original artifacts, hashes and CRCs
verify. [Evidence and preserved first failure](evidence/scrum27-repair-native-20261014/README.md).
The test-only fix released the previous baseline before independent sealing and
used the original source ACL for an identical replacement file; both original
NTFS failures are resolved. No installed/customer/boat acceptance or new full
application build is claimed. Existing older fixed broker/helper replacement
remains a separate authenticated-upgrade gap tracked in SCRUM-311; ordinary
application updates cannot replace that pair or reset its original trust state.

## Earlier customer rollback native helper gate passes

Exact `4fe70d8ae80c5f71c0d127bd1ab6c6eb0d0d54b2` passes the existing short
[Linux/Windows gate 38580830570](https://github.com/ThereptileII/Work/actions/runs/38580830570).
Authenticated downloaded receipts contain **188 Linux** and **283 Windows x86**
top-level test passes, zero failures. Windows records five explicit skips: four
non-Windows boundary cases and the disposable-host-opt-in SUBST remap case.
All 15 Linux / 30 Windows newly mandatory maintenance cases actually ran and
passed; skipped cases are not claimed as covered. The Windows job took under
two minutes. [Exact artifact evidence](evidence/scrum27-maintenance-native-20261014/README.md).

The integrated changes join the customer Maintain Rollback action to protected
selected history and add an actual pinned stock OpenCPN reopen observer after
Maintain uninstall. These helper tests do not establish genuine interactive
customer UAC, the complete installed journey, historical-version compatibility
or boat acceptance. This earlier gate predates the Repair implementation described
above and cannot qualify it. No full build,
boat change or public release was performed for this gate.

The preceding full candidate `bd47aa729aa728c6834072047eec7e00a65c6f13`,
[run 38563357629](https://github.com/ThereptileII/Work/actions/runs/38563357629),
remains **failed and unqualified**. Its same-job SDK check passed, but Chocolatey
returned HTTP 503 acquiring a Poedit dependency before application compilation.
Both original Linux executions passed the same 154 cases with no failures or
skips. [Authenticated original Linux evidence](evidence/scrum209-27-linux-bd47-20261014/README.md).
No newer helper result qualifies that application candidate or its security gates.

A single bounded [native prerequisite run 38581902769](https://github.com/ThereptileII/Work/actions/runs/38581902769)
now passes at `edd04b37b4a257f0f4307f5e240342fe3529a0f7`: 25 contract tests,
Poedit 3.9.1 acquired on the first attempt, both real gettext 0.26 operations
and UTF-8 catalog verification pass. Original artifact hashes/CRCs and exact
input hashes verify. [Prerequisite receipt](evidence/scrum209-gettext-20261014/README.md).
The 76-second job establishes current acquisition availability only; the full
candidate and genuine customer/boat acceptance remain open.

## Windows SDK authenticated; same-job early environment gate selected

Exact `1107c04f55a5a51660e424493bf083fc83106443` passes
[native helper run 38548714640](https://github.com/ThereptileII/Work/actions/runs/38548714640):
**44 tests**, plus the native file-metadata probe. Original artifact bytes and
all CRCs are authenticated. Independent path/descriptor mutation baselines
preserve identity, link, reparse, size and checksum protections. The failed first
focused check and corrected Windows path/newline fixtures remain recorded in
[the evidence](evidence/scrum27-installer-receipt-20261014/README.md).

That exact full [Staging candidate, run38549237034](https://github.com/ThereptileII/Work/actions/runs/38549237034)
passes the early receipt contracts, but Windows refuses the authenticated SDK:
hosted image `20261012.333.1` differs from producer `20261004.326.1`. Windows
application compilation and packaging were not reached. Linux completed successfully:
154 tests pass in each of the fixture-enabled and fixture-free runs, with no
failures or skips. [Authenticated original Linux evidence](evidence/scrum209-27-linux-1107-20261014/README.md).
The overall candidate cannot qualify from this attempt.

Root selected one dependency-only refresh at `e80f0b92`,
[run38552041879](https://github.com/ThereptileII/Work/actions/runs/38552041879).
It completed successfully. Original artifact `11890887394` is 117,984,619 bytes,
SHA-256 `b528229d8ec2f3f2272053da5c3baae98ab206d867c327f0bdd407303f7ab4d3`;
all 296 ZIP CRCs and the complete 295-file payload inventory verify. Native
producer tests passed: OpenSSL 4,283, zlib 13 and curl 1,569. Both its original
GitHub runner log and bundle record the older image `20261004.326.1`, so this
does not establish compatibility with `20261012.333.1`.

Change `30c0b9b043e454af687eb01c6529748450cc9132` selects that exact authenticated
SDK and moves its existing image/Python/source/inventory verifier immediately
after checkout, before native setup, in the same job that would build the
application. The later restore and native reprobes remain unchanged. Twenty-six
workflow checks and four focused bundle refusal checks pass locally. A mismatched
runner fails cheaply without a cold fallback or automatic retry; a separate
preflight job cannot reserve an image for a later build. Fresh native consumption
and application qualification remain pending. No image check was bypassed.
[Original failure and refresh scope](evidence/scrum225-image-refresh-20261014/README.md).
Later helper/evidence commits do not replace the product's source identity.
Genuine customer installation/recovery, boat and public readiness remain open.

## Installed updater adapters integrated; native journey remains open

The actual four-phase observer now launches the fixed protected startup path
using the original same-account limited token, checks prompt/application tokens
before UI input and retains original ownership through close. Bounded GitHub
HTTPS callbacks are also implemented. **145 focused tests pass**, with independent
source review. [Implementation evidence](evidence/scrum311-live-adapters-20261014/README.md).

The exact frozen product's Linux repository operator is prepared and its public
trust matches the existing repository; no signing/publication was performed.
The joined admission/update journey is now implemented at `3b879097`: one
Setup/enrollment, initial B0, no-update/Later/Now/offline, then Maintain uninstall.
It uses explicit external publication/connectivity handoffs, fresh native
observers, original-profile comparisons and retained broker ownership; refusals
stop without retry or cleanup. **69 focused portable tests pass**, with independent
source review. The [driver contract](../tools/test-installed-protected-updater-windows.md#joined-installed-journey)
records its scope. A suitable disposable signed-in Windows host and actual
native execution remain pending; portable tests are not installed acceptance.
The manual approval-record producer is registered at `c907d876`, but no live
approval has been authored or dispatched. No extra application build was started.

## Earlier candidate: compiled checks pass; installer receipt failed

One combined full Staging candidate, exact
`faa211b568f0e79e1d1065ba3fb76c3b1e801fd6`, failed after native installer compilation in
[CI 38529654044](https://github.com/ThereptileII/Work/actions/runs/38529654044).
The authenticated Linux integration job passes: both application suites have
**154 cases, zero failures and zero skips**; applicable integration steps pass.
[Partial Linux receipt](evidence/scrum209-27-candidate-linux-38529654044/README.md).
Both native Windows builds and their **146-case suites** passed, with no failures
or skips. NSIS then produced an installer, but the receipt helper refused its
path/descriptor metadata before recording the package hash. The failed job
does not retain that Setup or the final package; the full candidate is not
accepted. A focused native helper check is repairing this boundary before any
new application build. Extended preview
scenarios and endurance were deliberately skipped; design remains off. Later
approval/host helper corrections do not replace that compiled source revision.
The authenticated prepackage recovery contains both compiled applications and
16,790 verified files, but no final Setup, payload or source/recovery ZIP. Safe
delivery from it would require a new cross-run packaging/source/sealing contract;
it is not the existing sealed-input retest path. The coordinator selects one
new candidate through the existing workflow **after focused native helper
qualification**, rather than adding that new release mechanism during this gate.
The original failed run and retained compiled identities remain preserved.
Setup approval now binds original policy, installer hash/size and local file
metadata (25 focused checks). A short fresh-account probe produced real split
tokens but refused their primary/impersonation type mismatch; its own cleanup
passed. Failed evidence is retained below and the ordinary-customer gate stays open.

The actual boat now reaches the private HTTPS origin with native certificate
validation: HTTP 200, no redirect, and all 1,577 public-root bytes match the
retained SHA-256. [Read-only evidence](evidence/scrum311-private-origin-20261014/README.md).
This resolves boat HTTPS reachability only; hosted CI connectivity and actual
installed update/recovery acceptance remain separate.

Exact `404cc83bd0873a6aa4231c1ba2ae2e4a1f8a3499` passes
[focused run 38528886583](https://github.com/ThereptileII/Work/actions/runs/38528886583):
Linux **143 top-level / 422 nested**, native Win32 Windows **216 top-level /
574 nested**, zero failures and three existing Windows platform skips.
Original artifacts and source closure are authenticated. All three original
shortcut roles survive publication and restoration through their retained
native handles. The failed exact-error assertion and its stronger state-based
replacement remain recorded in [the evidence](evidence/scrum27-original-shortcuts-20261014/README.md).

The B0 driver now uses genuine Setup and the original same-account customer
launch; the earlier driver limitation below is historical. Its actual installed
journey is still pending. The read-only hosted probe confirms a real limitation:
GitHub's built-in Administrator has no linked normal token and cannot qualify
ordinary customer/UAC behavior. No host policy was weakened. A suitable disposable
interactive host, authenticated live enrollment, complete application package,
installed recovery journeys and boat acceptance remain required. SCRUM-27 stays open.

## Fixed customer maintenance passes focused Linux and native Windows

Exact source `91d647c383ce662815b0e4e9fab79c5ad0f32ad6` passes
[run 38521029142](https://github.com/ThereptileII/Work/actions/runs/38521029142):
Linux **140 top-level / 422 nested** and native Win32 Windows **206 top-level /
565 nested** cases, zero failures. Three existing non-Windows checks skip on
Windows; all 65 explicitly checked current/prior native-run controls pass.
Both original artifacts match provider source/run/attempt, hashes, lengths and
ZIP CRCs. The original failed run `38520213641` is preserved; deriving canonical
names from the original image handle fixes its Windows path-normalization
failure without relaxing identity checks.

Original enrollment now retains the approved maintenance helper and per-install
completion. The fixed customer Uninstall path accepts no caller cleanup
authority and joins the existing broker/installed cleanup owners. Same-account
elevation, parent-loss cancellation and original-owner close outcomes govern
completion. [Exact evidence and boundaries](evidence/scrum27-customer-maintenance-20261014/README.md).

**Still open:** actual installed Maintain/UAC, genuine B0/update/rollback/uninstall
journeys, full application package and boat acceptance. The existing B0 driver
does not yet install the LocalAppData integration or exercise the unelevated
customer launch. Old packages cannot be relabelled to qualify these new bytes.
SCRUM-27 remains open; no full application rebuild, delivery or boat operation
was performed in this component cycle. Earlier evidence remains below.

## Installed cleanup owner passes focused Windows and Linux

Exact source `b93da59cf0749157dfca84c562cc8c39713ece68` passes
[run 38516605460](https://github.com/ThereptileII/Work/actions/runs/38516605460):
Linux **117 top-level / 347 nested** and native Win32 Windows **171 top-level /
448 nested** cases, zero failures. Three existing non-Windows-boundary checks
skip on Windows; all 30 specifically required native controls pass without skips.
Downloaded original artifacts match provider hashes, lengths and ZIP CRCs.

The same owner now joins approved installed inventory, original shortcut
retirement, closed completed generation lineage, service shutdown and bounded
installed file/registry cleanup. Native registry transaction visibility, conflict,
rollback, timeout and exact commit run in disposable private keys. Original
stock, modified/unknown members and recovery are preserved by the contract.
[Authenticated evidence and limits](evidence/scrum27-installed-owner-20261014/README.md).

**Still open:** customer Maintain integration and a complete actual installed
B0/update/rollback/uninstall journey, application package and boat acceptance.
This component pass does not close SCRUM-27. No new full application build or
boat operation was requested. The entries below preserve earlier failures and
checkpoints.

## Corrected protected-uninstall sequence passes focused native Windows

Exact source `d7be409dc5c3abc31b82315510891adf53de4486` passes
[run 38512820510](https://github.com/ThereptileII/Work/actions/runs/38512820510):
Linux **104 top-level / 298 nested** cases and native Win32 Windows
**139 top-level / 356 nested** cases. Zero failures; three existing non-Windows
boundary checks skip on Windows. All eight specifically required new native
controls pass without skips. Both original evidence ZIPs match provider hashes,
lengths and CRCs; broker-source closure passes.

The original rename failure remains preserved. The correction uses one bounded
same-broker conversation: terminal fence, original shortcut restoration and
app/launcher handle drainage, original generation/registration retirement, then
matching final witness before SCM effects. Original root and recovery custody
remain held. Failed or interrupted handoffs cannot retry, fall back or claim
success. Independent review and focused tests complete this component gate.
[Exact evidence and remaining boundaries](evidence/scrum27-two-phase-uninstall-20261014/README.md).

Full installed terminal cleanup/uninstall, updated lineage, repair/rollback,
a new complete application package and boat acceptance remain open. No new full
application build or boat operation was performed for this component check.
The entries below retain original failures and earlier checkpoints.


## Windows compiled checks pass; SDK capture blocks packaging

Frozen [Staging run 38455915079](https://github.com/ThereptileII/Work/actions/runs/38455915079)
at `cd8a76e7d0e2a1ab62e1277606e9ccdb4d9d6495` is **failed**, after successful
fixture and fixture-free Windows compilation/security/unit steps: the same
**146 tests** pass in both builds, zero failures or skips. The plugin SDK capture
then refused the legitimate wx include `wx/arrimpl.cpp`. Packaging and installed
qualification were not reached. Linux at the same commit passes 154 CTest entries
in each build; this does not qualify the missing Windows package.

The complete locked wx headers/Dev archive inspection also identified
`listimpl.cpp`, `thrimpl.cpp` and selected `mswu/build.cfg`. Integrated correction
`575898e1` permits only those four exact per-root paths, retaining their hashes
and all existing identity/mutation/size controls. Linux passes 39 capture plus
30 runner checks; actual 1,509-file include and three-file setup trees pass.
[Focused native run 38466239261](https://github.com/ThereptileII/Work/actions/runs/38466239261)
passes 37 tests with two existing portable-only fixture skips and the native
metadata probe. Original artifact digest/length and all four CRCs are verified.
No replacement full application build has been requested for this fix.

Original failed B0 `38442801510` and all subsequent failed observations remain
preserved. Both diagnostic compiled archives lack the SDK/control custody and
supported restore path required for qualification; re-fetching pinned archives
cannot be relabelled as the original captured SDK. [Exact evidence](evidence/scrum209-plugin-capture-20261013/results.json).
Installed B0/B1, complete uninstall/repair, release and boat acceptance remain open.

The independently reviewed baseline uninstall join is integrated at `8a706e80`:
retain original startup custody before retirement, authenticate the same moved
objects, then restore shortcuts before service removal. Its first focused native
[run 38465784927](https://github.com/ThereptileII/Work/actions/runs/38465784927)
passes Linux but fails Windows. The native directory move returns access denied
while original descendant handles are open; delete sharing does not make this
ancestor rename legal. This is a product sequencing defect, not an assertion to
relax. A bounded same-broker restoration/drain/retirement correction is under
review. The failed increment is not installed or accepted. The explicit workflow required-name patch is
retained locally at `db355047` pending workflow-edit permission; unchanged existing
selection already runs the new cases and their actual events remain mandatory.
Earlier entries below preserve their historical state.

## Protected retirement component passes; frozen application candidate continues

The combined protected-retirement component at
`cddaebbbd40bba19a4726205e5e9fdbd03c8910d` passes focused Linux and native
Win32 [run 38448552844](https://github.com/ThereptileII/Work/actions/runs/38448552844).
Linux passes 85 top-level and 219 nested cases; Windows passes 112 top-level and
264 nested cases, with three intentional non-Windows-boundary skips and no
failures. Both downloaded artifacts match their original GitHub digest/length
and pass ZIP CRC verification. [Original and replacement evidence](evidence/scrum27-protected-retirement-20261013/results.json).
It retains original registration and baseline-generation identities through a
read-to-DELETE handoff, moves them to protected recovery siblings, and supplies a
distinct broker witness before shortcut restoration and service shutdown. A
review-discovered concurrent shutdown handle leak was corrected and explicitly
tested. The first native run `38447749381` remains failed: its temporary fixture
used the same principal for broker and client, correctly rejected by production.
The replacement preserves that rejection and exercises the native transaction
with the fixture's separately validated original security descriptor.
Updated/pending generation lineage and full installed Lifecycle cleanup remain
unsupported; the existing maintenance guard stays in place. This component pass
does not close SCRUM-27 or claim installed uninstall acceptance.

The application candidate remains **`2c61369080c358d5ef9b8952d6f2881b52891b13`**,
version **0.4.0-beta2.4**, in the original private-trust
[Staging run 38442801510](https://github.com/ThereptileII/Work/actions/runs/38442801510).
Its Linux integrated gate and native updater job have passed; Windows application
compilation/qualification is still running at this update. The same-commit
operator kit has been downloaded and authenticated, but is not yet paired with
a qualified recovery package. No replacement application build was dispatched.
The later driver-only repair `717d170d` now observes OpenCPN's actual ProgramData
profile and bounded navigation-store migration, with 33 focused checks passed.
Those helpers do not change or relabel the frozen candidate. Installed private
channel B0/B1 execution and boat acceptance remain open. Web deployment still
awaits provider access; no new Stripe secrets were saved.

## Reviewed prerequisites integrated for one trust-enabled Staging candidate

The original-shortcut restoration, explicit decommission CLI and protected
registration identity correction pass exact focused run
[38438377758](https://github.com/ThereptileII/Work/actions/runs/38438377758)
at `4843113c492792442f67296146db5669541990ad` on Linux and native Win32 Windows.
The earlier native run `38437716546` exposed an actual Win32 rename-wrapper
refusal. The correction uses the documented native retained-directory operation;
the same move, collision, custody and partial-failure controls now execute and
pass. [Original failure and replacement evidence](evidence/scrum27-native-restoration-20261013/results.json).
This is component acceptance, not a completed installed repair/uninstall or
private-channel B0/B1 exercise.

The actual OpenCPN plugin-root handoff now passes focused native Win32 run
[38439761709](https://github.com/ThereptileII/Work/actions/runs/38439761709) at
`fd9c6a005893ebb5eae1c6f91ec6e3ec762d77f1`: 58 distinct loader groups with no
group skips, 17,602 forwarder checks and 282 byte-policy checks. The downloaded
artifact digest/ZIP CRC and all 28 source identities are verified. Nested-root
legacy refusal, clean-sibling masking refusal, benign first mapping and custody
through unload pass; this is component evidence, not full SCRUM-209 acceptance.
The separate resource-only job was conditionally skipped.
[Authenticated results and remaining scope](evidence/scrum209-libdirs-native-20261013/results.json).
The same-version installer helper correction is integrated as `18d89c2a`, after
8 focused trust checks, 27 completion checks and independent source review. It
verifies Lifecycle's intentional trust initialization and retained identity;
the initial incorrect uninitialized-state assumption was caught before a build.
It retains all startup/update/corruption-recovery checks and explicitly leaves
protected B0/B1 acceptance pending. One combined private-trust Staging build is
selected with design and extended tests off. Full product and boat gates remain
open. The separate protected driver ProgramData/navigation-store observation
repair continues without delaying application compilation.

The private origin now runs as a pinned user service with verified automatic
failure recovery. Local and actual boat certificate-checked HTTPS responses
match the original public root; no boat app/profile/hardware changes were made.
[Origin service observations](evidence/scrum311-origin-service-20261013/results.json).
The separate web routing correction at `0e1ce6b3` passes
[CI 38439045828](https://github.com/ThereptileII/Work/actions/runs/38439045828)
(88 Django and 42 Edge tests). Provider access, deployment and live login/payment
acceptance remain pending; the user's private-beta configuration approval is
recorded in Jira and does not open public access.

## SDK consumer verification passes; plugin closure correction in progress

The full replacement run [38386718400](https://github.com/ThereptileII/Work/actions/runs/38386718400)
failed before application compilation at its authenticated SDK compatibility
check. Linux integration and native updater/prerequisite gates passed, but no
new installer or recovery package was qualified. The reviewed producer-specific
consumer correction subsequently passed native verified-reuse
run [38393796503](https://github.com/ThereptileII/Work/actions/runs/38393796503)
at `853cb5b3`, in 80.45 seconds without an application build. Downloaded evidence
matches the live GitHub artifact digest and ZIP CRC. Application consumption
and installed runtime qualification remain open.

The independent native observation
run [38393906703](https://github.com/ThereptileII/Work/actions/runs/38393906703)
at `d24c6b2f` passes 52 inert checks and proves a gap: the sibling-only plugin
inspection accepts an application-directory helper which imports a harmless
legacy-name stub. This is a reproduced security coverage failure, not product
acceptance. A shared two-root walk and 55-group native refusal/compatible-mapping
fixture now passes exact native run [38422671169](https://github.com/ThereptileII/Work/actions/runs/38422671169)
at `e27f60cd`: 55 loader groups, 17,602 forwarder checks and 278 byte-policy checks.
All 27 source identities and the downloaded artifact digest/CRC are verified.
[Original receipts and scope](evidence/scrum209-two-root-native-20261013/README.md).
Broader loader and integrated product qualification remain open.
The separately reviewed post-publication shortcut binding and native restoration
are integrated and component-tested as recorded above; installed repair and full uninstall remain open. No Jira product
issue is closed by these component results; boat, design and endurance tests
have not been started for this increment.

## Native prerequisite repairs pass; full replacement candidate not yet qualified

The first B0 product request [38382520660](https://github.com/ThereptileII/Work/actions/runs/38382520660)
stopped before native application compilation on two test-fixture defects, not a
proven application crash. The Windows profile-lock fixture passed an invalid null
backup path to `File.Replace`; subsequent junction cleanup could mask an earlier
exception. The AIS collector fixture attempted POSIX unlink while its emulated
Windows handles remained open. Product safety code and failure assertions are
unchanged; original failed evidence is retained.

Focused replacements pass on both platforms: [profile preservation](https://github.com/ThereptileII/Work/actions/runs/38385062815)
on helper `4c2eb2705bc2ac13297b9267aa0eab4bf6da3242`, including actual disposable
Windows closed-session transactions and interruptions, and [AIS collector](https://github.com/ThereptileII/Work/actions/runs/38384993606)
on helper/workflow `bc328e191986ddb40cd828dafd714e35f8778ea5`, **23 cases per platform**.
Downloaded original evidence artifacts match GitHub SHA-256 digests. SCRUM-370
closes only this collector fixture defect.

After owner HTTPS activation, private tailnet Serve and the pinned origin return
certificate-checked HTTP 200 with the exact original 1577-byte root from Linux
and the actual boat Windows PC. This supersedes earlier pending transport notes
below. No public Funnel or remote-access changes were made. The boat application,
profile and hardware are untouched. Transport and component passes do not qualify
installed no-update/Later/Now/offline behavior, full uninstall, or a replacement
application package. The reviewed fixture fixes are integrated for one replacement
B0 request; design review and endurance remain off. [Exact focused evidence](evidence/scrum311-native-prerequisites-20261013/focused-results.json).

## Fresh same-commit Windows and Linux updater component gates pass

Combined run [38302791125](https://github.com/ThereptileII/Work/actions/runs/38302791125)
on exact `4966e8ad47da13f84bc6a341d4fbd1ef01dec0b8` succeeds on both platforms.
The functional suite passes **601 top-level + 2603 nested** Linux cases and
**782 top-level + 2875 nested** native Windows cases (13 Windows top-level skips,
zero failures). Native launcher, broker, provisioner, baseline-handoff and operator
kit production, exact corresponding-source checks, startup/interruption records,
historical installer transactions and inert signing boundaries all pass.
Fresh real SCM enrollment also passes, including both stopped zero-exit broker
operations and fixed service configuration/DACL/SID readback. The downloaded
operator-kit artifact and every inner payload size/hash match their original
exact-commit manifests; no downloaded executable was run locally.

This supersedes the pending fresh-component gate below. Earlier failed evidence
remains unchanged. It does **not** qualify a full application package, real
installed no-update/Later/Now/offline flow, networking/security issues
SCRUM-208/209/211/212, protected full uninstall or boat deployment. Private HTTPS
still requires account-owner activation; the expired earlier setup process was
checked before recovering its activation link. Serve remains unconfigured and
no public Funnel or remote-access configuration was changed. The existing boat
installation is untouched. No full OpenCPN build, design review or endurance run
was started. [Exact run, counts and downloaded-artifact evidence](evidence/scrum311-startup-native-repairs-20261012/fresh-component-pass.json).

## Actual native service enrollment passes with corrected helper

Retained native run [38302376379](https://github.com/ThereptileII/Work/actions/runs/38302376379)
on helper `c4fec9c5bf0db8aba88b0e307cf256027c7923aa` succeeds with the exact
original `0ddb649f` broker. Downloaded artifact and provenance hashes are verified.
All 7 selected top-level tests and 18 nested cases pass, including actual fresh
file/service creation, stopped zero-exit `initialize-store`, broker-principal
stopped zero-exit `verify-store` and fixed service configuration/DACL/SID readback.
The registry-template and fresh-before-first-start corrections preserve system
ACLs and strict post-start zero-exit checks. Failed original runs remain preserved.

This qualifies the retained broker with the corrected enrollment helper, not a
newly compiled provisioner or installed application updater. One combined fresh
same-commit component production is next; full OpenCPN rebuilding and boat changes
remain deferred until prerequisites are ready. Actual B0/B1 registration, protected
startup and no-update/Later/Now/offline qualification still await their separate
operator/HTTPS gates. [Verified native evidence](evidence/scrum311-startup-native-repairs-20261012/retained-native-enrollment-pass.json).

## Fresh native components pass; actual SCM permission refusal under investigation

Replacement component run [38298416414](https://github.com/ThereptileII/Work/actions/runs/38298416414)
on exact `0ddb649ffaa06c8b6e1fc313fa6a087edf9914dd` passes Linux (**598 top-level +
2601 nested**) and native Windows functional tests (**776 top-level + 2857 nested,
13 top-level skips, no failures**), including the corrected cleanup-error fixture.
Windows vet, source packaging, all native component producers, authenticated
startup/interruption records and historical installer transactions pass.
Actual fresh native file enrollment now succeeds. The separate real SCM fixture
then refuses `untrusted machine mutation permission` during service initialization;
the original log does not identify the rejected SCM/registry/service object or
ACE. Two independent source reviews agree that relaxing permissions now would
be unproven. Same-held-descriptor diagnostic evidence is the selected next step,
reusing the retained exact component bytes rather than rebuilding OpenCPN.
The original run remains failed, signing is skipped, and no native service or
product acceptance is claimed. Boat installation and remote access are unchanged.
[Original run evidence](evidence/scrum311-startup-native-repairs-20261012/fresh-component-scm-run.json).

The bounded retained-only diagnostic [38300951246](https://github.com/ThereptileII/Work/actions/runs/38300951246)
authenticated the original `0ddb649f` broker and invoked the real fixture with
helper `21ad7b27`. It identifies the same held refusal as registry ancestor 1
(`HKLM SYSTEM`), BUILTIN Administrators owner, `S-1-3-0` CREATOR OWNER allow ACE,
mask `0x000f003f`, flags `0x02`. This is specific evidence for investigating Windows
registry inheritance-template semantics; it is not permission to trust that SID
as an owner/principal or change system ACLs. The next correction must remain
registry-specific and retain actual descendant owner/ACL checks. No full component
or application rebuild was needed for this observation.
[Retained diagnostic evidence](evidence/scrum311-startup-native-repairs-20261012/retained-machine-acl-run.json).

Retained-only [38301585978](https://github.com/ThereptileII/Work/actions/runs/38301585978)
on helper `4dd56a93` passes the registry-template, descriptor-diagnostic and
SCM/service-rights matrices (3 top-level and 16 nested checks), original component
authentication, machine/initial-service ACL checks and exclusive service creation.
It then refuses `new service was not stopped` before first start. The log does
not emit returned status fields. Microsoft's SCM protocol requires exit code
1077 before any start attempt; the implementation incorrectly applies a completed
zero-exit predicate at both fresh-service checks. A fresh-only distinction and
same-read refusal diagnostic are selected, keeping all post-start completion
checks strict. This remains a failed run, not native service acceptance.
[Original evidence](evidence/scrum311-startup-native-repairs-20261012/retained-registry-template-run.json).

## Private HTTPS preflight available; origin activation still pending

The read-only preflight pins the approved public trust configuration and original
public root, uses direct certificate-checked HTTPS, refuses redirects/proxies and
writes exclusive sanitized transport-only evidence. Its 11 deterministic tests
pass. Original input checks pass, but the live request refuses. Current read-only
Tailscale Serve status is `{}` and no loopback origin process is observed, so
previous loopback evidence is not current HTTPS availability. The earlier
account-owner Serve setup remains pending; no network configuration or keys were
changed. Hosted-runner tailnet access and actual B0/B1 installed startup tests
remain separate open gates. [Current observations](evidence/scrum311-private-origin-20261012/preflight-20261012.json).

## Native ProgramData proof passes; corrected cleanup-error fixture awaits native gate

Fresh component gate [38297499754](https://github.com/ThereptileII/Work/actions/runs/38297499754)
on `0d888edcf2866416108d62f06f16a050aae0aea7` passed Linux (**598 top-level +
2601 nested**, no failures). Windows passed **775 top-level + 2857 nested**,
with 13 top-level skips and one failed cleanup-error fixture. Actual retained-child
NTFS reparse/removal prevention, actual ProgramData known-folder capture and the
broker-store guard tests passed. The failing test supplied `windows.InvalidHandle`
to `CloseHandle`, but that value aliases the current-process pseudo handle and
did not produce the intended error. `3f8078c4` uses NULL and requires both the
actual `ERROR_INVALID_HANDLE` and classified cleanup error, preserving idempotence
and closed-guard denial. Windows/x86 vet and compilation passed; replacement
native execution remains pending. Production guards
are not relaxed. Component packaging and SCM enrollment were skipped after the
failure; no native service/product acceptance is claimed. [Original evidence](evidence/scrum311-startup-native-repairs-20261012/programdata-guard-native-run.json).

## Native component producers pass; SCM ancestor permissions remain blocked

Exact focused component run [38288556379](https://github.com/ThereptileII/Work/actions/runs/38288556379)
on `4dcee9b15922e62ea43a97e003e1278eead71b2f` passed both platform functional
jobs' test steps, Windows static analysis, exact-source packaging, native launcher,
broker/provisioner/baseline production, retained operator-kit packaging, startup
transactions and historical installer checks. Its Linux job succeeded. The Windows
job failed **before invoking the fresh SCM enrollment fixture**: the relative
log/script path resolved beneath the module directory. This is neither a native
service acceptance pass nor an observed product crash. The signing step was
skipped. The original run remains failed.

The selected helper repair uses absolute workspace paths. A separate SCM-only
workflow authenticates the original failed run, exact successful producer steps,
job/attempt, artifact identity and SHA-256, safely extracts only the existing
broker installation and re-verifies its original package/source identity. It
rechecks provenance before privileged fixture effects. Original product commit
and later helper commit are recorded separately; no component is rebuilt or
relabeled. Normal Staging gates are unchanged. [Retained-test contract](installer/retained-broker-enrollment.md).
Native service acceptance remains pending. The separate retained run
[38292790551](https://github.com/ThereptileII/Work/actions/runs/38292790551)
on helper `718b65c0d24e0b00f5bc1e89d6be9b0bd9ce1099` authenticated the original
bytes, rechecked provenance and actually invoked the native fixture. It refused
`createFreshFiles` with `untrusted enrollment ancestor mutation`, before service
initialization. Follow-up [38293693131](https://github.com/ThereptileII/Work/actions/runs/38293693131)
on helper `dcef6f6b569ca2907b14a9db8082f89a99a33ecd` observed the original held
descriptor: SYSTEM owner, BUILTIN Users allow ACE, mask `0x116`, flags `0x02`.
This grants creation and EA/attribute writes. The role diagnostic in
[38295243035](https://github.com/ThereptileII/Work/actions/runs/38295243035), helper
`b0f5f82a5327748d3ac746574d5645ecbe2e901b`, identifies `data-root`, component 1,
`allowAdds=true`: the actual ProgramData ancestor. It still refused before service
initialization. A bounded original-child custody correction is now implemented;
shared system ACLs are not changed. Both enrollment and the broker's own store
need the correction, so its next native gate must build fresh same-commit
components instead of qualifying the old retained broker.
`8efc6bd5` and `6fc293d3` retain an existing, nonempty Common Start Menu path
beneath the actual Windows ProgramData location. Every affected observation
rechecks original file identities and parent bindings before permitting only
ProgramData EA/attribute writes. Delete, child-delete, ACL/owner mutation and
generic write permissions remain rejected; SKAGER's own directories retain
their strict policy. No shared Windows ACL is modified. Focused Linux checks
and Windows/x86 vet and compilation passed. The native NTFS proof and fresh
same-commit SCM enrollment remain pending. The explicit corresponding-source
closure now includes both guards, the terminal-decommission implementation and
the operator diagnostics, so missing source members cannot silently pass packaging.
All 76 focused package checks passed with the completed closure. Independent
combined review found no further concrete custody or selected-source omission.
The preceding helper-only
PowerShell failure remains preserved. [Separate attempts](evidence/scrum311-startup-native-repairs-20261012/retained-scm-attempts.json).
No full application rebuild or boat change has occurred.

Earlier exact component run [38287535018](https://github.com/ThereptileII/Work/actions/runs/38287535018)
on `c75c3519f4ac1ee2732e8c38f0fc63334943cdf5` passed Linux (**569 top-level +
2556 nested**) and Windows functional contracts (**729 top-level + 2799 nested,
12 top-level skips, no failures**), plus independent native MSVC protected/ordinary
restart and popup interaction jobs. Its `go vet` failure and skipped packaging/SCM
remain preserved in [the original evidence](evidence/scrum311-startup-native-repairs-20261012/protected-routing-native-run.json).
The test-only `c9023127` repair passed the replacement run's native Windows vet.

`eef4ca5b` adds a bounded read-only selected-history diagnostic under the original
broker lock, protected registration and elevated peer. It reports selected
qualification/closed-history data, not mutation authority or the internal reason
for a metadata-check outcome. Focused Linux and Windows/x86 compilation passed;
actual installed endpoint acceptance remains pending.

## Service-only terminal decommission source joined

`b0f5f82a5327748d3ac746574d5645ecbe2e901b` joins the explicit elevated operator
action to exact singleton deployed inventory, the original authenticated broker
witness and the original retained SCM/process owner. It permanently fences new
startup, disables the same service, proves its clean stop and marks it for
removal. Code, profiles, charts, histories, shortcuts, backups and trust floors
remain preserved. It reports `marked-for-removal`, never observed SCM absence or
full uninstall. A prior structured-output bug is fixed: selected-history and
decommission JSON no longer receive unrelated trailing completion text.

Focused checks across six Linux packages passed; Windows/x86 vet and the joined
provisioner test compilation passed. Independent review found no concrete defect
in corrected inventory/stage custody, witness lifetime or terminal reader drain.
Actual native installed decommission, protected repair and full uninstall remain
pending. [Bounded contract](installer/protected-service-decommission.md).

## Private updater handoff joins ready for native verification

`9ba7bb93` preserves the independently approved trust file byte-for-byte through
the baseline producer and initial/retained request parsers, including its final
newline. It does not trim, replace the approved digest or guess whitespace
variants. Focused Linux race checks and Windows/x86 cross-compilation passed;
native producer acceptance remains pending.

`2c65f1df` joins the private origin's fixed broker-bootstrap route/artifact names
and paired, independently approved bootstrap policy/hash options in the existing
keyring publisher. Eight origin and twelve keyring tests passed without real
keys, signing or publication. Direct private HTTPS and the installed sequence
remain unaccepted. `d3ba0ff3` adds an inert native protected-restart fixture;
independent review identified wait-proof and owned-cleanup gaps which are being
corrected before component CI. No full application rebuild or boat change has
occurred in this increment.

## Protected restart and retained operator delivery joined

`fe9bf9cf` routes installed XNav/Legacy/Safe restarts through the corroborated
same-generation broker launcher. Original qualification fences remain latched;
commissioning markers cannot divert a protected restart into the old helper path.
Independent review found no further concrete issue in that bounded path. Native
MSVC compilation and actual inert companion dispatch are pending.

`17792d5f` corrects the inaccessible-directory fixture's private token to retain
only directory traversal privilege and verifies its actual privilege list. The
parent-access, child ACCESS_DENIED and production-refusal assertions remain;
this is not a native pass until the replacement run confirms them.

`60ffef92`/`2c97d643` package an inactive operator kit from the same-commit
launcher and already-produced native broker/provisioner/baseline components.
The reuse path performs no compiler invocation or rebuild fallback. Sixteen
focused packaging tests passed; native packaging remains pending. The kit is
separate from the normal product and must be paired with its independently
authenticated same-commit recovery/launcher prerequisite. It grants no enrollment
or mutation authority. [Operator delivery contract](../tools/protected-operator-kit.md).

Protected repair/decommission and the actual installed no-update/Later/Now/
offline sequence remain open; neither component checks nor old-layout installer
checks close those acceptance gates. The existing boat installation is unchanged.

## Latest component result — one native fixture remains

Exact component run [38283605275](https://github.com/ThereptileII/Work/actions/runs/38283605275)
on `cfca0a9855d87b430083f5b25513f1ba187d4596` passed Linux and native startup
interactions. Windows recorded **724 top-level + 2780 nested passes, one
top-level failure and 12 top-level skips**. The original maintenance-capability
revocation test now passes. The remaining inaccessible-directory test fails its
parent-access premise under its private privilege-disabled token; native denial
has not yet been established. Production directory checks remain unchanged.
Fresh SCM enrollment was skipped after that failure. Historical installer
transactions passed 47 inert checks; these do not qualify protected maintenance.
[Original evidence](evidence/scrum311-startup-native-repairs-20261012/closed-modes-native-run.json)
is retained separately from the earlier runs. The boat's existing application
has not been replaced. No full application rebuild is claimed.

## Protected mode startup joined; native maintenance repairs pending

Component run [38281497298](https://github.com/ThereptileII/Work/actions/runs/38281497298)
for exact `ea3c2bffa493915bf48b68834d6c1cc508ffb44c` passed Linux and native
startup interactions. Windows recorded **703 top-level + 2756 nested passes,
2 top-level failures and 12 top-level skips**. The earlier physical-image and
profile-DACL repairs no longer fail. Remaining failures were closed maintenance
capability error ordering and an inaccessible-directory fixture whose read was
still permitted. Fresh SCM enrollment remained skipped after the prerequisite
failure. [Original artifact and failures](evidence/scrum311-startup-native-repairs-20261012/manual-maintenance-native-run.json)
are preserved. `c8a4ab50` moves the original lifetime check before backend
readiness and attempts to establish real ACL denial under a private,
privilege-disabled test token; it does not change the process primary token or
weaken production directory observations. Native confirmation is pending.

`32a15f7f` binds closed XNav/Legacy/Safe modes through protected selected startup;
qualification stays XNav and recovery modes skip only the update offer.
`025a0451` publishes three fixed shortcuts through the same approved baseline
launcher with independent retained link identities and partial-publication
evidence. Joined focused Linux race checks passed **21 top-level + 77 nested
cases**, no failures/skips. Native mode/link execution is pending.

`da4bb0be` refuses old LocalAppData Lifecycle mutation when the fixed protected
service, registry or machine namespace is present or uncertain. Diagnostics
remain read-only. Historical installer fixtures must run before fresh fixed SCM
enrollment; the service is never deleted to make an old-layout test pass.
Protected setup migration, repair and uninstall remain explicit integration
gates, not covered by historical installer results.

The running broker now joins a separate, bounded manual-selection endpoint to
its existing Store/operation lock. It requires the original elevated operator,
exact selected-history pins, retained original application exit, unchanged stock
compatibility and a frozen CURRENT profile. It never launches an application or
restores historical navigation data. The one-shot operator client restores its
temporary folder-query ACL before releasing ordinary query custody. Normal
startup remains query-only. [Contract and explicit cleanup limits](installer/broker-execution-contract.md#explicit-manual-selection-and-retained-ordinary-application).

Joined focused Linux race checks passed **42 top-level + 181 nested cases** in
four packages, with no failures/skips; Windows/x86 host, pipe, launcher and
provisioner test binaries compiled. The four affected packaging suites passed
**64 cases**. These are focused source checks, not native Windows or installed
acceptance. [Recorded checks](evidence/scrum311-startup-native-repairs-20261012/manual-maintenance-source-checks.json).

Independent inspection confirmed the older installer must not be joined by
blindly adding a provisioner call: its LocalAppData shortcut ownership/repair
path conflicts with the protected shortcut. Protected in-app mode restart and
owned install/repair/uninstall integration remain open. No full application
build or boat modification is claimed; SCRUM-311 remains In Progress.

## Protected startup/Now source joined; exact native component failures isolated

Latest component run [38267062370](https://github.com/ThereptileII/Work/actions/runs/38267062370)
on `abce9b32` passed Linux, native startup interactions, authenticated startup/
interruption records and installer interrupted/history transactions. Windows
component verification passed **643 top-level + 2595 nested cases**, with **2
top-level + 1 nested failures and 11 top-level skips**. The remaining failures
are maintenance process image-path binding and the profile DACL mutation fixture.
Actual fresh SCM enrollment was skipped after the prerequisite failure and has
not passed. [Original downloaded evidence](evidence/scrum311-startup-native-repairs-20261012/maintenance-native-run.json)
is retained separately from earlier failures.

`200a2832` compares the process-reported image through its opened canonical
physical path and full native identity, retaining strict original image custody.
`560ec201` gives the test-only DACL mutator read/control rights and independently
corroborates its mutation before requiring production refusal. Native confirmation
of both repairs remains pending. `ea4273d2`/`750af80b` add the operator baseline
handoff producer: exact approved recovery ZIP copy, bounded validation and separate
request approval. Its focused Linux checks passed; this is neither enrollment nor
installed updater acceptance.

Source through `7e3c5e09` joins authenticated normal-startup/update offers, the
protected prompt/Now path and explicit initial operator startup activation.
Unknown/crashed prompt exits now refuse rather than masquerading as Later.
Staging also requires the existing native private-chart plugin-loader gate.

Exact component run [38261504482](https://github.com/ThereptileII/Work/actions/runs/38261504482)
on `daf9c3a4` passed Linux and the native startup interaction harness, but Windows
component verification failed: **624 top-level + 2535 nested passed; 8 top-level
+ 12 nested failed; 9 top-level skipped**. The failures isolate initial profile
watch notifications and native shortcut factory/publication. A separate installer
fixture omitted the required installation root; the production guard correctly
refused it. `3abc9bd4` changes watch arming/complete validation without ignoring
events, `c8b5a89b` repairs native shortcut factory/rename custody, and `b92eaaa5`
fixes the stale installer fixture call. Replacement component run
[38264329533](https://github.com/ThereptileII/Work/actions/runs/38264329533) on
`307b66bc` passed Linux and native startup interactions. Windows now has **631
top-level + 2548 nested passed; 2 top-level + 2 nested failed; 9 top-level skipped**.
Remaining failures concern short/long shortcut-path expectations and the fixture's
security mutation. The installer fixture reached authenticated child reception,
then failed because it omitted the production `CreatedRoot` initializer.
`e87229ca`, `182e5a99` and `ca368ac4` repair these fixture cases without changing
production guards; replacement native validation remains pending.

The older installed lifecycle has not yet joined protected enrollment and
repair/rollback/uninstall. Actual protected service operation, combined package
security/lifecycle and exact boat acceptance remain open. No combined Staging
application build or boat change is claimed. See [source, failed native evidence
and remaining acceptance](evidence/scrum311-startup-native-repairs-20261012/README.md).

## Protected startup admission joined; normal installed updater remains open

Source `b2b92829` joins the initial-only approved-baseline startup gateway,
authenticated read-only original profile-archive reopening and bounded fresh
launcher pipe handoff. Profile archive validation/closure and cold-profile pin
release happen before the fresh launcher starts. Joined focused Linux race:
**65 top-level + 287 nested passed**, zero failures/skips. Windows/x86 service
component compiles; native execution remains pending. Fixed package source
completeness suite: **12 passed**. [Exact evidence and limits](evidence/scrum311-startup-admission-20261012/README.md).

Normal installed shortcut activation and protected Update Now are now the active
parallel integration tasks. Existing installed default still uses the historical
user-owned supervisor. No combined Staging candidate, whole-product Windows or
boat acceptance is claimed. Reopened archives cannot recreate original child
exit authority after service restart. SCRUM-311 remains In Progress.

## Native updater service joined; end-user and native acceptance remain open

Source `aafae707` integrates completed profile capture/restoration, explicit
selected-launcher relay/current startup, and both actual SCM service endpoints.
It independently binds the original child before health-pipe observation and
uses retained teardown/recovery authority when the update server becomes
unavailable. Candidate launch/selection still require current signed eligibility.
Focused joined Linux race: **54 top-level + 217 nested passed**, no failures/skips;
actual joined Windows/x86 brokerhost compilation passed. These are component
checks, not native service/product acceptance. [Evidence and open boundaries](evidence/scrum311-execution-integration-20261012/README.md).

Review caught a fresh-launcher pipe handoff race; its bounded preconnection
correction is in progress. Installed normal shortcuts and Update Now still enter
the previous user-owned supervisor path. Protected startup admission, original
archive host reopening/service-restart recovery, remaining legitimate profile
mutations, actual SCM access, packaged security qualification and exact Windows/
boat lifecycle remain open. No combined full candidate, boat change, physical
command, design review or endurance run has started.

The previously pending C++ fresh-profile fixture now passes exact native Windows
run [38244815589](https://github.com/ThereptileII/Work/actions/runs/38244815589),
revision `63f79bcb`. Its original downloaded summary/logs are retained in the
linked evidence. It is separate from the newer Go service source and does not
qualify the whole updater. SCRUM-311 stays In Progress.

## Qualification plugin bookkeeping repaired; native fixture rerun pending

The candidate-entry plugin guard now runs before OpenCPN can create a persistent
crash/load stamp. A focused compiled probe of the actual patched upstream prefix
passes restricted and ordinary discovery behavior; removing the guard fails its
negative control. All 13 pinned patches apply. [Evidence and scope](evidence/scrum311-plugin-deferral-20261012/README.md).

Focused Windows run `38243098619` on `175449f6` failed compiling the inert native
profile fixture (wx/Win32 header ordering), rather than crashing the application.
The header repair is integrated as `fae99f9c`; MSVC rerun remains pending.
The service execution transaction and full-profile restoration are still being
joined. SCRUM-311 and packaged security/Windows/boat gates remain open; no
combined candidate or installed boat state changed in this increment.

## Updater executable qualification and interactive execution joined in source

Source through `d2c0a287` integrates exact linked pre-spawn recovery history,
original executable capability validation, the authenticated interactive
executor, restricted broker process/token query access, and an explicit native
fixed-profile fixture in the disposable Windows startup harness. The executor
consumes each launch role before native creation and retains the original child;
lost conversation never retries or enters ordinary startup. Qualification
intentionally suppresses core outputs and defers all plugins for that process.
Normal selected startup needs its separate nonqualification path; changing a
live qualification process into normal operation is not permitted.

The package producer now requires those capabilities from the actual executable
self-test and binds its fixed sidecar to the copied executable. All 13 upstream
patches apply in order to the pinned OpenCPN revision. Focused package tests pass;
new native profile, service-token and execution checks remain pending.
This is source integration, not a qualified product or release.

SCRUM-311 remains In Progress. Actual service execution/selected commit,
protected full-profile backup/restoration, initial baseline qualification,
ordinary startup/shortcuts and exact Windows/boat acceptance are unfinished.
The latest full pipeline `38143022102` compiled the Windows application and
installer but failed later regression qualification. No combined replacement
build, installed boat/profile/service change, design/endurance run or physical
command has occurred in this increment.

## Restart originals, permanent journals and interactive peer checks pass

Source `43a9cdb6` joins original protected generation reopening and permanent
execution journals. Focused Linux race: **18 top-level + 107 nested passed**.
Exact native Windows/x86: generation reopening **3 top-level + 14 nested**, journal
**3 top-level + 4 nested passed**, no failures/skips. The existing interactive-peer
binary passed **2 top-level + 7 nested** after the original session-0 refusal was
preserved and the SAME bytes were dispatched in desktop session 1; its temporary
limited-user task was removed. [Exact evidence and scope](evidence/scrum311-restart-journal-peer-20261012/README.md).

Actual staging now retains its native peer across the operation (`204de84b`).
Closed execution commands bind exact generation and startup-health fields
(`264efdf1`, isolated `463f80dd`); portable component race passed **27 top-level +
133 nested**, Windows/x86 component compiled, native transport fixtures pending.
C++ Windows startup-health least-rights changes require the integrated MSVC gate.

SCRUM-311 remains In Progress: protected selection/pending, general-profile
commissioning, consumed child spawn, actual healthy commit and restoration/
fallback integration remain unfinished. No full application build, installed
product/profile/service change, design/endurance run or physical command occurred.

## Original interactive launcher image passes native identity check

Source `b1bed3d4` binds a sealed executor-image capability to the fixed retained
`app/skager-start.exe`. Actual native process creation identity, image path and
full physical file ID must agree with original read-only custody. Focused race
**3 top-level + 5 nested passed**; exact native Windows/x86 **1 fixture passed**.
The fixture creates a copied test image suspended, never resumes its entrypoint,
then terminates/drains only its own original creator handles.
[Exact source/native evidence](evidence/scrum311-executor-image-20261012/README.md)
records the limits. The actual approved SCM registration path now publishes the
initial baseline loading view after private registration writers close (`d18df036`);
its Windows/x86 host component compiled, actual SCM enrollment remains pending.

This is not executor-session, selection, commissioning, spawn, healthy-start or
restore acceptance. Authenticated pipe capability, protected generation reopen
and durable operation journal source are being integrated before the actual
execution transaction. SCRUM-311 remains In Progress; no combined full build or
installed updater/boat qualification has occurred.

## Atomic history and commissioning source retention pass joined Windows checks

Integrated source `8ee7e633` uses atomic read-only candidate materialization in
the actual broker adapter. Retained live generation histories adopt only the
exact newly published successor; uncertain publication revokes affected owners.
Recovery production and readers retain the exact twelve original commissioning
files in a separate versioned namespace, preserving the three-engine contract.
Joined Linux race: **20 top-level + 123 nested passed**; exact native Windows/x86
component: **6 top-level + 19 nested passed**, no failures/skips. Producer and
fixed source-package suites passed **35** and **12** tests respectively.
[Exact original logs, source/binary receipts and limits](evidence/scrum311-history-commissioning-20261012/README.md)
retain the isolated reader's initial fixture compile failure and corrected run.

SCRUM-311 remains In Progress. Actual protected selected/pending state, interactive
execution, general-profile commissioning, one-use spawn, authenticated healthy
commit and restore/fallback remain unfinished. The next source work binds the
live authenticated executor to the exact retained launcher and reopens completed
generations without adopting paths or receipts. No installed product/profile,
service or physical output changed; OpenCPN PID1852 stayed responsive. No full
application build, design or endurance run occurred.

## Approved initial baseline loading view passes native component checks

Source `d3d653ba` (isolated `de86d6fa`) adds a fresh protected public copy of the
live completed approved baseline, with original protected registration evidence,
all original sidecars/helpers, full physical IDs and distinct typed provenance.
Its read-only loading custody survives private-source closure. Focused Linux
generation race passed **12 top-level + 65 nested**; Windows/x86 component compile
and **3 top-level + 5 nested native fixtures passed**, no final failures/skips.
The initial copy-helper panic, source repair and exact corrected evidence are
[retained](evidence/scrum311-approved-baseline-view-20261012/README.md).

SCRUM-311 remains In Progress. Live sibling-history evolution, actual protected
selected/pending state, interactive executor/commissioning, one-use creation and
authenticated healthy/fallback are incomplete. This does not qualify a selected
previous version or the full installed updater. The separate manual retained
plugin qualification workflow (`0bbb54a2`, documented in `e4496989`) is wired but
has not run against a new producer. Full MSVC application and installed/boat
gates remain pending. No full app build/release, installed product/profile change,
visual/endurance run or physical command occurred. OpenCPN PID1852 remained
responsive. Earlier sections retain their original revision and scope.

## Current increment: native read-only generation custody passes

Source `bb2dde1b` (isolated `fcec6f21`) adds the actual one-way writer-to-read-only
transition for completed protected generations. Original file identities, bytes,
descriptors and private creation history remain retained; failure consumes the
attempt and preserves completed evidence. Revocable member reads expose no native
writer handle or caller-selected execution path. Linux focused generation race
passed **8 top-level + 38 nested**; Windows/x86 component compilation and
**3 top-level + 3 nested native fixtures passed**, zero fail/skip. The native PE
fixture mapped a copied test image without executing it, verified read/execute
compatibility and write/delete refusal, and covered actual close-gap cancellation
and changed-original rejection.
[Exact source and native read-only evidence](evidence/scrum311-generation-readonly-20261012/README.md).

The earlier existing-writable-mapping stock hypothesis was tested against native
Windows: initial stock custody refused with a sharing violation while the mapping
remained live. **3 exact inert stock fixtures passed**, zero skips. No metadata
bypass was reproduced and no unnecessary production rehash patch was added.
[Exact stock fixture and evidence](evidence/scrum311-native-stock-integrity-20261012/README.md).
OpenCPN PID1852 remained responsive and no owned fixture process remained.

SCRUM-311 remains In Progress. The initial approved baseline needs a protected
public loading view; retained previous/candidate owners need authenticated sibling
history evolution. Actual selected/pending state, interactive executor binding,
commissioning/restoration, one-use creation and authenticated healthy/fallback
remain incomplete. Component success is not an integrated MSVC/installed update
qualification. No full app build, release, product/profile change, visual/endurance
run or physical command occurred in this increment. Earlier sections below retain
their original revision and scope.

## Protected update generation copy joined; independent review recorded

Product source `ac96ef54` joins the signed candidate's fresh protected application
view and actual host call before staging completion. Original sidecars and the
three execution helpers are preserved; authenticated original parent identities
support subsequent fresh generations. No generation is selected or launched.

Final joined race checks passed **14 top-level + 64 nested cases**, with no
failures/skips. Enrollment race checks passed **19 top-level + 67 nested**;
**68 source-package tests** passed with retained raw logs. Windows/x86 component
test compilation passed. Native inert boat checks passed **2 top-level + 4 nested**
for the application-root boundary and **4 top-level + 11 nested** for generation
copy/history/mutation behavior. Two prior native fixture failures and their
test-only corrections (`f8fce3e5`, `d2a45ee7`) are preserved. Production security
validation was not relaxed. OpenCPN PID 1852 remained responsive; its installed
application/profile and physical controls were untouched.
[Exact source, checks, failure history and native receipts](evidence/scrum311-protected-launchview-20261012/README.md)
record the limited scope.

The user approved the exact nine-file Claude transfer. The attempted review hit
Anthropic's weekly limit before returning findings; reset is October 15 at 06:00
Stockholm time. No automatic retry or expanded payload was used. The independent
GPT-6.1 Sol fallback found no confirmed defect in that frozen scope. One native
mapped-file custody hypothesis remains unproven and tracked in SCRUM-311; three
subsequently changed source files are explicitly excluded from frozen acceptance.
[Review comparison and failed-attempt receipt](evidence/scrum311-client-registration-20261012/review-comparison.md)
are retained. Earlier sections describe their original dates/revisions, including
then-pending consent; this paragraph records the current review state.

SCRUM-311 stays In Progress. Actual pending/selection/spawn, read-only loader
custody, interactive launch, profile restoration/commissioning, authenticated
healthy-start and installed update/fallback qualification remain open. No full
application build, CI dispatch, product deployment, release, visual/endurance
suite or physical command occurred for this increment.

## Original installer helpers retained through signed acquisition and storage

Source `aa3e6bfe` now preserves the three exact NSIS helper snapshots in Recovery,
then copies and validates their original bytes through signed staging and approved
baseline import into protected candidate/installed custody. Missing, partial,
aliased, tampered or foreign helper sets refuse; historical no-helper evidence
supplies no execution authority. The broker/provisioner source closure also
requires every actual imported production member.

Focused checks passed **48 top-level + 229 nested Linux Go tests**, **25 receipt
tests**, and **68 delegated source-package tests** (the latter session-reported,
without a retained raw logfile). Windows/x86 component compilation passed.
With runner `700e5dfd`, **5 top-level + 22 nested native inert tests passed**;
OpenCPN PID 1852 stayed responsive and no owned test process remained.
[Exact source, raw logs and native receipts](evidence/scrum311-engine-preservation-20261012/README.md)
record the scope and prior failed fixture. This does not complete the real update,
rollback/interactive launch, provisioning or healthy-start path. SCRUM-311 remains
In Progress. No full application build, CI, product installation/profile change or
physical output occurred. Claude's nine-file comparison still awaits exact-scope
transfer consent. Earlier evidence retains its original revision and scope.

## Protected installed baseline source joined; inert native checks pass

Production source `67665e18` joins fresh operator publication of the independently
approved baseline ZIP, strict original-evidence import, permanent protected
installed-store completion and retained host staging custody. Registration cannot
complete before the baseline; missing/partial/existing state refuses without
adoption or reset. Original manifest/inventory/product pins keep their exact
meanings. This does not attest an older running LocalAppData installation or
supply transaction, selection or launch authority.

Combined final Linux component checks passed **131 top-level + 477 nested tests**
with zero failures/skips. Four Python packaging suites passed **68 tests** in the
recorded coordinator session; no raw Python logfile was retained. Windows/x86
broker/provisioner compilation passed at `67665e18`. The initial brokerstore test
compile failure is retained; test-only mask correction `f2a9e051` passed without
changing production bytes. **10 top-level + 6 nested native inert tests passed**
on Windows 10.0.22631 with zero failures/skips. The evidence collector's later
local extraction failure is also retained; recognizing ZIP directory entries
fixed retrieval without rerunning tests. Existing OpenCPN PID 1852 stayed
responsive, with no owned test process left.

[Exact source receipts, raw logs, failure history and boundaries](evidence/scrum311-installed-baseline-20261012/README.md)
are recorded. Actual privileged provisioning/first execution, a real
update/fallback transaction, interactive launch handoff, commissioning and the
healthy-start handshake remain open. SCRUM-311 remains In Progress; these inert
checks do not accept the release. No full application build, CI dispatch, service
activation, application/profile change or physical command occurred in this
increment. Earlier records below retain their original revision and scope.

## Approved client registration and retained trust corrections integrated

Product source `3404a8d2` joins the independently approved request to stopped-only
native registration before service readiness. Original approval bytes, protected
state and completion evidence remain bound together; partial/existing enrollment
is refused. The retained reader now distinguishes an expired original bootstrap
anchor from currently validated rotated trust. Cleanup failures and uncertain
publication remain explicit, and corresponding-source producers require the
new runtime members. Contract documentation is recorded in `7f542766`.

Combined focused Linux race checks pass **117 top-level / 352 nested tests** with
zero failures and zero skips. Four delegated package suites pass **68 tests**;
the integrated tested inputs match exactly. Windows/x86 inert test compilation
passes. With runner `bd9c05c7`, **14 top-level / 2 nested inert Windows tests pass
natively on the boat**, with zero failures/skips. Exact member hashes and native
PowerShell parsing pass. The initial transfer timeout and failed missing-file
preflight are retained; the compressed transfer subsequently passed. The original
OpenCPN remained responsive and no owned test process remained. These checks do
not modify the running boat installation/profile or qualify actual privileged
enrollment, installed application custody, MSVC/application behavior or release.
[Exact source hashes, checks, review and remaining boundaries](evidence/scrum311-client-registration-20261012/README.md)
are recorded. SCRUM-311 stays In Progress. The new nine-file Claude comparison
awaits exact-scope transfer consent; no additional private source was sent.
No full application build, CI dispatch, release or physical command occurred.

The SCRUM-209 plugin-loader ownership repair and shared model layout are now
recorded in [focused evidence](evidence/scrum209-plugin-custody-20261012/README.md).
Native MSVC loader and installed dependency-closure qualification remain open.

## Native machine enrollment source joined; inert boat policy checks pass

`c185300d18e81b317358e172d04a1ee7e412689e` joins independently pinned signed
bootstrap verification and retained broker acquisition to actual fresh protected
KnownFolder creation, fixed virtual-account SCM enrollment and stopped-only
initialization/verification. The fixed producer/verifier binds the provisioner to
same-commit launcher/broker prerequisites and corresponding source. Existing or
partial machine state refuses; startup does not invoke this operator component.
Independent Codex review corrected cleanup-error classification before integration.

Affected Linux race checks pass **32 top-level tests / 138 pass events including
nested cases**; **81 inert packaging tests** pass. Windows/x86 component/test
cross-compilation and vet pass. With runner `a5d7b613`, **nine exact allowlisted
Windows-only inert policy tests pass natively on the boat**. The initial runner
attempt was blocked before execution by PowerShell's script policy; the same
reviewed bytes then ran with a process-only policy, without changing machine
policy. The original OpenCPN process remained responsive. [Exact commits,
results and limitations](evidence/scrum311-native-enrollment-20261012/README.md)
are retained, including the failed initial attempt.

These native checks do not create directories/services through the provisioner,
start a broker, modify the application/profile or send hardware commands. Actual
privileged enrollment, independently authenticated first execution, protected
client registration, installation/loader custody, installed update handoff and
Windows/boat release qualification remain open. No full application build or CI
dispatch occurred. SCRUM-311 stays In Progress. The completed earlier Claude
acquisition review remains distinct; the expanded private broker review awaits
exact-scope external-transfer consent and has not run.

## Read-only AIS attribution source path integrated

`092c51c6` joins an explicit opt-in owned connection diagnostic, strict portable
validator and actual read-only native Windows collector. Three registered focused
Linux CTest cases pass: **153 C++ checks, 31 validator tests and 23 inert collector
tests**. Independent Codex review and root integration corrected address, parser,
handle ownership, output cleanup, tuple ambiguity and deadline defects before
accepting this source increment. [Exact source, corrections and bounded evidence](evidence/scrum227-readonly-attribution-20261012/README.md)
do not qualify native Windows, actual installed module closure or live boat
attribution. The result explicitly grants no filtering authority and leaves the
inert-only WFP guard unchanged. No full application build or boat change occurred;
SCRUM-227/16 remain In Progress. Claude has not reviewed this AIS increment; the
expanded private broker review still awaits exact-scope transfer consent.

## Fresh broker creation and stopped-only SCM initialization integrated

`50227181` adds actual broker-principal-only creation of the fixed protected store
under an independently protected-at-creation machine parent. A permanent parent
fence is synchronized before any child creation; partial attempts remain consumed.
`e227d0c3` joins explicit SCM initialization to a stopped-only path and requires
completed creation evidence before normal service startup. Independent Codex review
identified the partial-tree completion gap during implementation; the completed
fence and exact-tree validation address it. Combined affected Linux checks pass
**34 top-level tests / 143 pass events including nested cases**, with race detection,
zero failures and zero skips. Windows/x86 component and test compilation plus vet
pass; **native execution remains pending**. [Source, review and focused evidence](evidence/scrum311-fresh-initialization-20261012/README.md)
do not qualify privileged enrollment, client registration or installed updates.
No full application build, boat change, service activation or physical command
occurred. SCRUM-311 remains In Progress. The expanded Claude comparison remains
pending exact-scope external-transfer approval; no additional source was sent.

## Purpose-bound broker policy publisher and reader integrated

Source `75a770b1` separates the machine broker declaration from ordinary app
updates, using the existing signed TUF repository and independent exact policy,
provisioner and public-config pins. Explicit operator publication retains original
bytes and a separate version floor across app upgrades and interrupted attempts.
Independent Codex review found two publication-collision defects; both were fixed
and rechecked. Focused affected Linux checks pass **37 groups / 438 test pass
events including nested cases**, with no failures or skips. A deliberately removed
preflight makes both new collision regressions fail, as expected. Actual
publisher-to-reader TLS integration, Windows/386 component compilation and vet
pass. [Exact source, results, review and negative control](evidence/scrum311-bootstrap-policy-20261012/README.md)
do not qualify native Windows or installation. Initial privileged verifier trust,
protected service enrollment and actual installed update handoff remain open.
No full application build, boat change or physical command occurred. SCRUM-311
remains In Progress; expanded Claude review still awaits exact-scope transfer consent.

## Focused TLS policy wiring verified; boat compiler unavailable

At product source `eede2fe9`, the actual portable CMake target and registered
CTest case pass 241 known-obsolete-TLS byte-policy checks. Only that component
was built and tested, using existing offline dependencies. Read-only discovery
on the boat found registered Visual Studio Build Tools but no installed MSVC
C++ component, `cl.exe` or `vcvarsall.bat `; native compilation/execution did not
occur. PID 1852 remained present and no application/profile change occurred.
[Exact inputs, commands, failures and scope](evidence/scrum209-native-byte-policy-20261012/README.md)
are retained in `2b9a60b4`. Native CI and complete installed dependency-closure
qualification remain required; this is not a Windows pass or new candidate.

## Route draft completion and DPI selection fixes integrated

`e5ac8b37` preserves XNav's explicit Finish/Cancel policy across native modal
chart interactions; focus loss can no longer overwrite that policy and bypass
the naming step. `3a11990e` scales enlarged route-segment targets using the
querying canvas's actual physical DPI. Legacy and other selectable objects keep
their existing radii. `36bcd5a7` registers both patches in CI, build/source and
dependency inventories. Combined focused checks pass: 250 desktop plus 250 macOS
timer assertions, 295 selection assertions, and two affected packaging/input
tests. [Evidence and preserved fixture failures](evidence/scrum298-route-interactions-20261012/README.md)
separate these results from pending native Windows/boat interaction acceptance.
No full application build or boat application/profile change occurred.

## Broker-owned acquisition and indexed cleanup joined

`ce226959`, `07ad7922` and `663b8c0d` connect the actual broker to a durable private
acquisition lease, native retained artifact owners and acquisition-only startup
cleanup. Unknown or mismatched residue refuses readiness; candidate payloads and
request IDs are never adopted or reused. Storage passes 7 focused Linux groups;
the verifier and affected existing paths pass 12. The host handoff passes 6 groups
plus 13 fault subcases with race detection. Five native Windows groups plus nine
nested cases pass on the boat PC using inert temporary files. Joined Windows/386
component compilation passes; no full application rebuild was needed.

[Exact source, preserved fixture failure and native evidence](evidence/scrum311-broker-acquisition-20261012/README.md)
separate this result from unfinished actual service-principal/provisioning,
installation custody and installed update acceptance. Independent Codex review
found no concrete integration defect. Claude's expanded review still awaits
exact-scope private-source transfer consent. The installed application/profile,
remote access and vessel controls were not changed.

## Guarded lighthouse information helper integrated

`da7dddee` and `193e3130` now join actual final raster observations to core/private
chart Info, including the CA-light bypass. Observations are bounded and copied,
matched to chart/viewport/presentation state; partial results remain unavailable.
Root review reproduced and fixed a partial-CA diagnostic defect. All 12 core and
3 private patches apply together; 122 paired dispatch/copy/bounds assertions
pass. `59152587` registers explicit core-only Linux CI checks. [Renderer evidence](evidence/scrum308-render-observation-20261012/README.md)
does not qualify native canvas/GPU/private-DLL execution or identify the actual
northern lighthouse; those gates remain open.

`8e074bf7` adds narrowly guarded chart-position Info and bounded read-only
chart-information observation. The isolated helper passed 285 portable checks
and compiled its C# source; root reviewed the change before integration. Native
UI execution and identification of the actual northern lighthouse remain open.
An operator must first open the intended chart-position context; no guessed
center/menu command was added. Normal chart details do not prove the resolved
conditional renderer symbol. See the [helper evidence and limitations](evidence/scrum308-chart-info-helper-20261012/README.md).
No application build, boat/profile change or physical command occurred.

## Store lifetime lock — focused native checks passed

The protected native store lock in `7c2ceed4` passes four distinct Windows test
groups plus four nested refusal subcases across retained results. The original
three passing groups were not rerun. The cleanup group exposed two fixture-only
errors: an injected process pseudo-handle did not fail, then fixture cleanup
double-closed the replacement null handle. Both failures are retained; separate
pin ownership fixes the fixture without changing production bytes or weakening
assertions. The final failed-only native retry passes.
[Exact-artifact evidence](evidence/scrum311-store-lock-native-20261012/README.md)
separates this component result from service/update qualification. No application
build, installed application/profile change, service activation or hardware
command occurred. Authenticated provisioning, installation/loader custody,
orphan recovery and exact-package Windows/boat acceptance remain open.

## Existing private Staging trust selected

`110e61f7` commits the unchanged existing operator public configuration after
hash/provenance, root-signature, expiry, retained-state and stock-tuple checks.
`49264c91` joins it to the explicit manual `private_staging_trust` packaging
option, default off; push builds remain unconfigured. Nine focused workflow
tests plus three affected existing policy checks pass in the isolated source.
The selected public bytes remain identical under Windows-style checkout
conversion. No private signing keys were accessed or regenerated. See
[selection and preserved provenance refusal](evidence/scrum311-private-trust-selection-20261012/README.md).

This corrects the earlier investigation's missing-input finding: real retained
public trust already existed, although it had not been committed as a package
input. The existing loopback-only origin now serves the exact retained public
root. Actual Tailscale activation reports that the tailnet owner must first
enable private HTTPS Serve; its mapping remains empty. The owner step is pending,
and this is not HTTPS acceptance or a persistent deployment. See
[activation observations](evidence/scrum311-private-origin-20261012/README.md).
HTTPS activation, signed offer publication, authenticated privileged
enrollment, installation/loader custody, orphan recovery and exact-package
Windows/boat acceptance remain open. No full application build, CI dispatch,
public push, service activation or physical command occurred.

## Broker cleanup retains uncertainty

`4fc0704e` fixes two reproduced owner-close defects: repeated close no longer
erases cleanup errors, and concurrent store close waits for the original drain.
Nine focused Linux test groups, including thirteen fault subcases, pass with Go
race detection; independent Codex source review found no concrete defect.
[Evidence](evidence/scrum311-close-results-20261012/README.md) records the failing
before cases, changed expectations and exact source. Native qualification and
crash/orphan recovery remain open. The requested expanded Claude investigation
has not run because exact private-source transfer approval is pending. No full
application build, CI dispatch, boat change or physical command occurred.

## Inactive broker package/source prerequisite implemented

Source revision `3c0033fc` adds a native probe for a separate, inactive broker
bundle while preserving the original launcher-only package transfer. The broker
reuses the exact existing corresponding-source archive, now including its build
scripts and required shared helper. Its consumer checks actual embedded Go
identity, Win32 GUI format, exact commit and source correspondence; authenticated
successful remote producer provenance remains the caller's responsibility.

Focused source checks pass **73 distinct Python tests**. A single retained
Windows/386 cross-compiled executable exposed an incorrect linker-metadata
expectation, repaired without rebuilding. Actual module-graph inspection also
exposed sparse optional checksum fields; the producer now audits the retained
provisioning checksums offline. The actual full-archive/native producer gate is
still pending. [Evidence and preserved failures](evidence/scrum311-broker-package-source-20261012/README.md)
separate source implementation, component compilation and native qualification.

No application build, CI dispatch, boat deployment, service activation or
physical command occurred. The installed boat application remains unchanged.
Public trust is now explicitly selected as recorded above; HTTPS activation,
privileged provisioning identity, installation/loader custody, orphan recovery and exact-package Windows/boat
acceptance remain open under SCRUM-311; this is not a ready updater or release.

## Native directory-custody failure corrected

Actual Windows execution exposed a retained-directory defect: metadata-only
handles did not enforce the claimed rename exclusion. `aa9fe37c`, `31175848`,
`11486212` and `cd85b15e` add directory list access to the existing protected-store,
pipe, stock and launch owners without relaxing sharing/ACL/identity checks.
Empty-directory tests require refusal while held and successful rename after
release; conflicting pre-existing delete access also refuses acquisition.

Focused native boat tests pass: broker Go/Win32 **11 groups + 26 subcases**;
launcher Go/Win32 **3 groups + 4 subcases**; installed launch-barrier **60 checks
per x64/x86 Windows PowerShell 5.1 host**; actual C# directory-owner opens **12
checks per host**. The latter uses helper-only correction `054f9a79` against
unchanged `cd85b15e` production bytes. Failed fixture/wrapper results are retained.
The running application, profile and hardware configuration were not changed.
These are native component results, not a native MSVC application build or full
authenticated service/update qualification. See the
[exact-source evidence](evidence/scrum311-directory-pins-native-20261012/README.md).
SCRUM-311 remains In Progress; provisioning/trust, installation/loader custody,
orphan recovery and exact-package acceptance remain open. No full application
build, CI dispatch, deployment or physical command occurred.

## Authenticated protected staging joined in source

Source commit `3d0b68d3` joins real `PrepareUpdateSet` and retained Recovery
extraction to the new distinct-principal Windows store and authenticated local
pipe. The operation accepts no caller paths or execution commands, reserves a
durable unique ID, reads protected registration, remeasures the exact stock
OpenCPN baseline and writes only authenticated installed app/docs files.
Artifact cleanup must succeed before its staging receipt; final owner/namespace
failures withhold a successful response. Its sole result is
**staged-awaiting-qualification**, never installation or startup success.

Combined-root focused checks passed **37 top-level test groups and 118 subcases**
across extraction, broker operation, protocol and storage. Windows/x86 host test
and service-command compilation, plus Windows-targeted vet, passed; these are
**compile/static checks, not native execution**. The independent bounded Codex
review found an interface ID-length mismatch, corrected before integration.
Storage additionally rejects unsafe physical ancestor permissions and caps
durable candidate entries rather than silently deleting partial evidence.

This service is not provisioned, packaged, activated or used by the installed
launcher. Native service/ACL/IPC qualification, authenticated provisioning and
trust initialization, actual installation/loader custody, interrupted staging
recovery and exact-source Windows/boat update acceptance remain unfinished.
No full application build, CI dispatch, boat deployment, visual/endurance suite
or hardware command was performed. SCRUM-311 remains In Progress. See
[source evidence](evidence/scrum311-protected-staging-source-20261012/README.md)
and the [staging boundary](installer/signed-update-session-contract.md#protected-candidate-staging--source-increment-not-enabled-updater).

## Independent acquisition review and one-attempt cleanup

Claude Opus 5 completed the requested independent read-only review of 11 exact
source/test files at `8d8b6a07`. It agreed that acquisition uses one authenticated
offer and actual caller cleanup failures withhold startup. Full native-source
comparison rejected its prompt-drain and installer-cancellation hypotheses.
One latent repeated-close pathname deletion hazard was accepted and fixed in
`135cab1f`; cleanup is attempted once and repeats retain the original result.
Three focused download tests and Windows/x86 compilation passed in isolation;
native execution remains pending. Crash-left artifact reconciliation and the
full signed broker are still unfinished. See the
[finding-by-finding comparison](evidence/scrum311-acquisition-claude-comparison-20261012/README.md).

## Direct boat launch and restart deny armed installations

Commit `6004c3db` adds the missing restriction at the actual direct application
launch, startup-launcher creation and commissioning restart permit/ALLOW paths.
The shared helper retains the existing native image/ancestor owner, checks the
fixed marker under that held image's physical managed root, and rejects a
changed requested image alias. The installed custody module is imported from
bounded, held, hash-verified bytes; it is not reopened after hashing.

The combined-root focused helper passed **39 portable checks**. The isolated
increment also passed 44 existing startup checks, 309 restart checks, 26 broker
fixture checks and eight qualifier tests. Native inert-file/SUBST cases were
unexecuted at that source increment. The focused follow-up above now records
their exact-source native component evidence; actual-entry/product integration
still requires qualification. These remain point-in-time
absence checks, not continuous namespace custody, no-child proof or signed
broker authority. No boat launch, application build or hardware command was
performed for this increment.

## Installed updater acquires one authenticated artifact set

Commit `7f9ca7d3` wires `PrepareUpdateSet` into the actual installed launcher's
Update Now preparation. One fresh signed check and exact consent supply both
Recovery and Setup; Recovery's exact Lifecycle sidecar is required before Setup
download. Both read-only owners remain held through the existing operation and
timeout handling, with independent cleanup failures preserved. The combined-root
focused check passed five new tests and nine subcases; the isolated broader
preparation/launcher regression and Windows/x86 compile checks also passed.
Follow-up `e75a2c55` fixes the actual caller swallowing those cleanup failures:
fixed redacted error identity now withholds startup only for uncertain cleanup;
clean cancellation/network/old-sidecar refusal retains ordinary fallback. Three
additional combined-root tests and seven subcases passed. The partial-removal
fixture is Unix-only; no Windows sharing result is inferred from it.
No native execution, signed broker authority or Setup embedded-manifest proof is
inferred. Protected-from-creation loader custody and actual creator-bound denial
remain required source work before full updater qualification. See the
[acquisition contract](installer/signed-update-session-contract.md#authenticated-recovery-and-exact-installer-identity).

## Chart forecast value restored in source

Commit `ef724724` restores the forecast speed/direction to the expanded chart
forecast box after per-arrow values were removed (SCRUM-360). The value is
explicitly located at the vessel using assessed real Live/Aging position and
actual forecast coverage; missing position/coverage remains unavailable. Forecast
time, stale/source and advisory labels remain separate from measured wind.
Amendment `fba4d604` suppresses the overlay when its mandatory forecast marker
cannot fit, so narrow viewports cannot show anonymous forecast arrows.
The existing forecast-view suite passed 670 assertions, including nine added
cases; changed overlay syntax passed. The larger integration unit's syntax check
stopped at the existing missing `shapefil.h` dependency. Combined native build and
boat product behavior remain pending; no new application build was started.

## Authenticated Recovery and installer-manifest identity integrated

Commits `e4bf7627` and `f44d3764` join authenticated Recovery acquisition to
the exact existing installer manifest's identity. The Go reader repeats signed
update verification and exact consent, then reads bounded inventoried content
through the retained authenticated archive handle. The producer retains unchanged
`installer/package.json` after successful Setup generation, outside the payload,
and rebuilds Recovery/inventory/checksum before signing. Missing sidecars remain
explicitly unavailable. Focused acquisition checks passed seven groups/49
subcases; producer checks passed 19 tests. The existing release-boundary runner
now includes the producer checks without another workflow or full build.
These are acquisition/identity boundaries, not an enabled signed broker or proof
of installation/launch. Actual packaged native/boat updater qualification remains
pending. See the [identity contract](installer/signed-update-session-contract.md#authenticated-recovery-and-exact-installer-identity).

## Real previous-generation restoration integrated

Commit `b6f2667b` integrates the signed-session operation with the real existing
InspectRestore/Restore engine. It requires exact current inspection and, for
changed settings, the existing independent preservation proposal. It verifies
actual restored profile/plugin files and permissions, selected generation and
marker removal before publishing `PreviousRestored`. Failed postchecks or journal
persistence leave denial in place and poison the live store even when Restore
itself completed. The combined focused check passed 49 assertions across 11
temporary-file transactions against store source SHA-256
`83928135ce3196565a7cedb322508edfccda5d12f38e83f28c62c5bb65105a92`.
The portable signed-session gate now includes this check. It uses explicit
unsupported-Windows adapters. A separate exact-source native temporary-file
fixture has since passed 106 assertions under each Windows PowerShell 5.1 x86/x64
host on the boat PC (212 total). Real NTFS handles/ACL/SACL/mutex and restoration
code execute, with synthetic bounded identity/context/closed-process/baseline
substitutions explicitly recorded in the [native receipt](evidence/scrum311-previous-restore-native-20261011/README.md).
The native qualifier now registers that fixture in a fresh process per host;
actual installed updater behavior remains pending. This
operation supplies no install/spawn authority and is not product-wired. See the
[operation contract](installer/signed-update-session-contract.md#concrete-previous-generation-restoration).

Independent review then identified a cleanup exception after durable event
append that could leave the live store usable. The amended operation poisons
custody on release failure, attempts both independent lock releases and emits
success only after cleanup. The extended real-engine fixture passed 55 checks
across 12 temporary-file transactions, including actual restored state with a
consumed event but no usable custody after failed release.

## Physical signed-store path correction

The source batch now binds installed arming's transaction lock, owner/state
reads and marker creation, plus private-journal creation, to physical paths
derived from retained native handles. It pins physical ancestors above mapped
drive targets and verifies the held marker/journal's parent. Initialization
cleanup retains the original failure and attempts all independent releases,
including when a failed stream flush also makes disposal fail. Focused checks
passed 236 private-store and 38 installed-store assertions. Initial attempts
failed because the system temporary directory exhausted its quota; the same
checks passed using the project temporary directory. The exact five source/test
hashes were then verified on the boat PC, where both native Windows PowerShell
x86/x64 hosts passed 42 installed-store and 245 private-store checks each,
including the new SUBST-remap fixtures (574 assertions total). See the
[native store receipt](evidence/scrum311-store-boat-native-20261011/README.md).
No application/profile/hardware was accessed. This is a store correction,
not complete loader custody or signed broker acceptance. See the
[physical path contract](installer/signed-update-session-contract.md#physical-installed-store-paths).

## Integrated dependency provenance refinement

Source batch `3bc1478b` connects verified updater/adapter source and notice
records, wxWidgets archive-member evidence and the MSVC runtime receipt to the
generated package inventory. The preceding commits retain the actual selected
Visual Studio product/catalog identity (`c6ae26d1`), verify installed wx DLLs
against the locked runtime archive (`61283d05`) and pass that archive explicitly
to packaging (`201ffe3c`). The combined focused check passed 59 tests. Native
Windows version APIs, PowerShell argument transport, 7z decoding and complete
package assembly remain pending. These records establish specific byte/source
associations, not redistribution entitlement or legal clearance; SCRUM-59
remains open. See the [inventory contract](compliance/component-license-inventory.md).

## Route-detail names and commissioning source integration

The remaining route-detail naming omission is integrated at `69bedee2`: planned
waypoints reuse the existing guarded name editor and OpenCPN persistence action.
Successful saves retain the route page/scroll and other unfinished drafts;
Cancel/rejection keep existing behavior. The isolated implementation passed 63
existing naming checks and its changed wxWidgets unit syntax check. Native and
boat edit/save/reopen, protected-point and multi-draft interaction remain pending.

Commit `bd79eb42` adds shared physical-profile exclusion to cooperating
commissioning Apply/inspection/restoration and real direct/startup/restart/stock
launch paths. It changes no profile bytes or filesystem ACLs merely to acquire
the lock. This is not recursive loader namespace custody, protection against
caller DOS-alias retargeting, durable recovery authority or a complete signed
updater broker; those limits are explicit in the
[session contract](installer/signed-update-session-contract.md#shared-commissioning-profile-exclusion).
The focused portable commissioning gate passed its 12 groups plus the new exact
C# compile/unsupported-platform refusal. Restart dependency closure passed 26
checks. New native contention, identity, SUBST, reparse, abandonment and
preservation fixtures are wired into the existing commissioning gate but have
not executed on Windows. The focused startup-policy gate passed 44 checks.
No full product build or boat modification has occurred.

## Integrated source batch: armed-update denial and plugin bytes

The separate source batch now integrates durable installation-root arming
(`cad36fc1`), installed Go launcher refusal (`2aa25e1f`), PowerShell
startup/maintenance refusal (`fa7cabd3`), known obsolete plugin-byte refusal
(`f75682bd`) and portable/native test registration (`57168cd3`). The combined
portable persistence gate passed 61 receipt, 129 operation, 236 private-store
and 34 installed-store checks. Combined transaction checks passed, including
56 new barrier policy checks. Python test-runner syntax and diff checks passed.
These portable checks do not execute Windows file/ACL/process behavior.

The fixed installed marker is a durable denial signal, never recovery or launch
authority. Missing/corrupt/ambiguous custody cannot silently resume; there is no
automatic disarm. Independent Claude/Codex findings and dispositions are in
[the comparison record](evidence/scrum311-armed-claude-comparison-20261011/README.md).
A source-proven mismatch between a held ordinary executable and a remapped
original drive alias is corrected in `1bb4e9b5`: final checks use the exact held
image's physical installation root, and other presence checks capture the drive
once before physical lookup. The agent's offline launcher suite passed 34 groups
and 207 subtests; Win32 fixtures cross-compiled but have not executed. The
complete four-stage signed boat broker and native/package qualification remain
open. No new full application build, boat change or Production promotion has
occurred for this batch.

## Current exact-commit Staging run

Commit `ec51b9261d2854d8600672ea27c97af5abeb3f6d` completed integrated
Staging [run `38143022102`](https://github.com/ThereptileII/Work/actions/runs/38143022102)
with an overall **failure**.
The Linux and Windows contract jobs, native updater checks, chart-loader guard,
restart boundary and boat-recovery contracts have passed. The integrated Linux
application compiled and its 152/152 CTests passed, but the overall Linux gate
failed in the first-start mode-cycle smoke: a third setup field was below the
initial 1280×800 scroll viewport, and the harness incorrectly required it to be
visible. This is a harness refusal, not evidence of an application crash. The
native Windows producer built both application policies and the installer,
but its downstream qualification stopped in the peer harness: a test-only
export fault did not match a Windows-normalized temporary path. Correction
`27e4a1d3` resolves both paths and deliberately exercises a path alias on every
host; all 19 focused tests pass with export/redaction assertions retained.
The existing binaries are under a separate no-rebuild native retest,
[run `38158246826`](https://github.com/ThereptileII/Work/actions/runs/38158246826),
using harness `e1d6f5a6`. It passed the corrected peer check and subsequent
application/recovery checks, but the installer test then waited for obsolete
selection-page wording. Its captured native screenshot proves the page was
open. Helper correction `d8bd6c86` checks the exact launched enabled frame and
current page/control identity; 11 focused observer tests pass while the original
pointer, deadline, default-action and preservation assertions remain. Only
installer/chart checks were repeated in no-rebuild
[run `38160318032`](https://github.com/ThereptileII/Work/actions/runs/38160318032)
against the same retained `ec51b926` application bytes. That selected native
installer/chart run **passed** on 2026-10-11; the artifact hash, all 1,985
member CRCs and 19 retained raw members have been independently verified in
[the native evidence receipt](evidence/scrum98-retained-native-38160318032/README.md).
Nine named installer checks, seven operation receipts (two passes and five
expected refusals), four nested updater checks and 18 functional chart captures
passed. Requested OpenGL was rejected by the hosted environment, so both chart
phases exercised software fallback. The official prerequisite returned exit
1223 despite verified installation postconditions; this anomaly remains recorded.
This is a selected exact-byte retest, not an
integrated qualification of the newer source batch. The original Linux failure
remains. This commit has **not** yet
produced an accepted boat
package. The installed
boat application remains at `f1a1e68c` and no Production promotion is in scope.
See [the exact Linux failure record](evidence/2026-10-11-staging-linux-setup-harness.md).

The next source batch is isolated from this frozen candidate. It carries the
reviewed offline FreeSans original/editable source, converter and notices
(`15e00b94`), corrected retained-updater evidence (`7b7856b2`), and a
fail-closed secret-scanner correction for unscanned Git submodules (`be83c6b9`).
A factual MSVC runtime redistribution/provenance note (`1532d23b`) records the
remaining edition, entitlement and exact-package review evidence without
asserting legal clearance. The Linux setup helper correction (`4fbd8f92`)
accepts the two visible identifying fields and complete action row, while
still refusing partial identity and stale post-close controls; 13 focused
tests pass. A bounded SCRUM-317 route-marker optimization (`2f722714`) removes
one proven duplicate active-route traversal per eligible marker, with
deterministic baseline/output comparison. It does not establish a fix for the
boat's reported zoom hang; native frame and boat evidence remain outstanding.
A second bounded SCRUM-317 change (`11a6abd9`) avoids rescanning the same
route style for repeated occurrences of one waypoint pointer within a single
marker call. The 4,095-repeat control falls from 16,773,121 to 8,191 node
advances; 4,095 distinct points remain at 8,191, with 16,106 differential
checks. This still does not qualify chart responsiveness on Windows or the boat.
The batch also carries a bounded SCRUM-209 original-plugin guard (`465e0c33`,
extended by `840e1932`)
that refuses declared normal/delay imports of legacy TLS DLLs before the
shared original load path, including configured external plugin roots. It
also follows declared imports through same-directory sibling DLLs while
holding their inspection handles; extensionless module imports are normalized
before sibling lookup. It does not establish general Windows loader resolution,
dynamic/helper closure or full application compatibility. The bounded PE reader
passed 9,366 portable checks. All 42 focused native MSVC Win32 loader groups
passed in [run `38146240771`](https://github.com/ThereptileII/Work/actions/runs/38146240771)
at `840e1932`; the benign sibling was preloaded in that proof. An additional 43rd
group (`1f756424`) additionally tests first mapping under live inspection
handles. Its first native attempt,
[run `38158753749`](https://github.com/ThereptileII/Work/actions/runs/38158753749),
compiled and passed the prior 42 groups but refused its own DLL-search
precondition before the new load. The corrected bounded string query
(`e615446e`) passed all **43 groups** in
[run `38159170047`](https://github.com/ThereptileII/Work/actions/runs/38159170047),
including first mapping, exact mapped sibling identity/hash, live write/delete
locks, unload and search-state restoration. This is focused guard evidence,
not an integrated application or plugin-closure acceptance. A reviewed extension
(`f5ebd020`) now inspects bounded declared export forwarders and follows eligible
same-directory dependencies using the same retained-handle policy. Both portable
compiler checks passed 9,366 existing plus 17,496 new assertions. Native
[run `38162532440`](https://github.com/ThereptileII/Work/actions/runs/38162532440)
stopped at MSVC linking of the harmless forwarder fixtures, before the new
51-group runtime gate could execute. Fixture-only correction `eba68bf7` now
constructs the nine fixed inert export forwarders in bounded reserved metadata;
15 portable constructor methods pass. Native
[run `38164290605`](https://github.com/ThereptileII/Work/actions/runs/38164290605)
compiled and constructed them, and passed seven forwarder groups, including
actual named/ordinal legacy-TLS targets and explicit `.dll` resolution. The
final multi-dot grammar positive control failed native resolution with its
prepared filenames. Test-only correction `b8d6dc05` adds the exact private
`forward.clean` filename; production forwarder grammar is unchanged. All **51
native loader groups** and 17,496 reader checks then passed in
[run `38164763565`](https://github.com/ThereptileII/Work/actions/runs/38164763565)
at exact `b8d6dc059deedf3c061681558406a37fd82e7b0b`. The multi-dot case mapped
that exact private file, returned value 409, and passed live identity/hash,
write/delete lock, unload and search-state assertions. All 50 artifact member
CRCs and 56 retained records were independently audited in
[the native evidence](evidence/scrum209-forwarder-native-38164763565/README.md).
This qualifies bounded guard mechanics, not the integrated product or general
plugin dependency closure. The FreeSans upstream conversion and
original license evidence is integrated as `3ea17639` without a final legal
clearance claim. Focused source/notice and real-scanner tests passed, but these
newer commits have no integrated native package result and do not clear final
license, checkout-secret, plugin dependency-closure or updater commissioning
gates.
The peer-containment CLI also now emits a fixed failure diagnostic rather than
printing an exception chain that could contain test credentials (`c0ee37c1`).
All 19 focused peer-harness tests pass, including subprocess stderr checks;
SCRUM-212's product/native acceptance remains open.

The opt-in SCRUM-227 report-silence fixture (`8be05c69`) passed actual-provider
loopback TLS on IPv4 and IPv6. Each withheld position reports for 121.24 real
seconds while ping/pong remained responsive, observed stale/lost status without
refreshing retained receipts, then verified TCP teardown, cleared connection
observation, bounded reconnect, confirmation-only still-lost state and fresh
report recovery. Exact inputs, runtime metrics and the early-exit negative
control are retained in [the focused evidence](evidence/scrum227-real-clock-report-outage/README.md).
This fixture is excluded from ordinary builds/suites and does not qualify WFP
packet loss, helper-death, product/process/DNS binding, native Windows or boat
continuity. CMake was unavailable locally; the retained direct compiler recipe
built the actual provider and pinned patched IX source.

The test-only WFP safeguard is recovered from its accepted historical lineage
and extended by `3bb00adb`: a fixed 122-second blocked-retry window, 135-second
native helper watchdog, independent 150-second helper-only job deadline and
fresh 180-second inert clients. Eleven portable refusal/evidence methods pass;
watchdog exit and forced helper death remain distinct. The manual-only focused
native [run `38161766040`](https://github.com/ThereptileII/Work/actions/runs/38161766040)
at `4f423a80` **passed**. The independently audited artifact retains all eight
client logs, strict dual-family filter readbacks and exact-GUID cleanup audits.
Both 122-second blocked-retry windows passed without marker success; controls
remained connected. The watchdog exited 124 at 135.047 seconds, and forced
helper termination exited 1 at 123.047 seconds; both recovered with new marker
connections. The configured 150/180-second outer timers did not expire and are
not claimed as exercised kill paths. See [the native receipt and limits](evidence/scrum227-long-native-38161766040/README.md).
It only targets newly compiled loopback marker clients on a disposable hosted
Windows runner. It does not build the application, filter product/boat traffic
or qualify product endpoint binding, provider behavior or boat continuity.

A separate narrow updater change (`5792c4e9`) refuses files with more than one
hardlink using metadata from the actual acquired native handle, before granting
an owner. The focused native Win32
[run `38159970075`](https://github.com/ThereptileII/Work/actions/runs/38159970075)
passed at exact source `1c4d3c88`: all four read/hold/measure/startup-lock cases
refuse both aliases, release refused handles and accept the restored one-link
file. The downloaded artifact's ZIP hash, CRC and actual named test receipt
were independently verified. This run built no application and skipped the
broader suites. The later source batch integrates the native PowerShell image
owner (`8e93c5cc`): held physical NTFS image/ancestors, explicit suspended
creation, raw child identity, a single resume attempt and versioned interruption
records. Portable policy/embedded-source checks pass. Its focused native
[run `38163791374`](https://github.com/ThereptileII/Work/actions/runs/38163791374)
at `01ac92a6` passed actual file/ancestor protections but failed CreateProcessW
with Win32 87 before resume after the SUBST remap. Diagnostic `517c0679`, wired
by `3689a75c`, ran ten suspended-only inert-marker cases in
[run `38165504516`](https://github.com/ThereptileII/Work/actions/runs/38165504516)
at exact `afa5fbcf33cbf93f0f6c3f74fd72acf0b22086bf`. All seven DOS
application/command-path cases created the expected held image, including a
GLOBALROOT current directory. All three GLOBALROOT application/command-path
cases failed with 87 regardless of current directory. Application and argv[0]
spelling change together in these rows, so neither parameter alone is yet
proven causal. No marker was resumed; child and exact-owned alias cleanup
passed. The unchanged production owner still failed with 87 and the overall
run is **failed**; no native launch acceptance is inferred. A compatible
physical spelling is being investigated without adding a mutable DOS fallback.
Ordinary Go creation (`948d43e3`, source `5a803d94`) now also receives the managed
image hash and holds the physical image/ancestry through suspended creation and
one resume. Its independent source review found no additional concrete defect;
27 Linux groups / 143 including subtests and both Windows cross-compiles pass.
Its actual Windows execution and GLOBALROOT image/CWD compatibility remain
pending, including the separately documented counterpart failure.
The expanded seventeen-row diagnostic at exact `3d13f1e9`,
[38166517938](https://github.com/ThereptileII/Work/actions/runs/38166517938),
then separated application spelling from argv[0]/CWD: eight DOS application
rows passed, five GLOBALROOT and four held-file-derived volume-GUID application
rows failed with 87. No child resumed; exact-owned cleanup passed. The
[audited failure](evidence/scrum311-createprocess-matrix-38166517938/README.md)
rules out the tested GUID remedy without asserting an exact Windows internal
cause. [Independent scoped Claude/Sol comparison](evidence/claude-native-investigation-20261011/README.md)
records accepted findings, rejected claims and capture limits.

Corrections `7e133776` / `255fc30c` now select one DOS application spelling from
the held image, corroborate its root/image against physical pins and retain
physical argv[0]/CWD, suspended raw identity checks, one resume and no retries.
The DOS namespace is rechecked, not claimed pinned. Focused native
[38167390164](https://github.com/ThereptileII/Work/actions/runs/38167390164),
exact `6445d67e27c6b6f4254849556efea28248204711`, passed the Go Win32 owner
(ten top-level groups, two subtests, no skips). PowerShell x64 now created the
correct held image after SUBST remap, but its first normal-close test failed on
the hidden/owned inert WinForms fixture; supervisor health was not reached.
The x86 harness did not run because its custom-shell temporary file lacked a
`.ps1` extension. Overall this run remains **failed**; test-only window/driver
corrections are pending, and no PowerShell owner acceptance is inferred.

The same exact source passed the changed plugin module-name gate in
[38167423509](https://github.com/ThereptileII/Work/actions/runs/38167423509):
51 native loader groups and 17,602 native forwarder-reader assertions. Correction
`b77dedb2` refuses reserved device stems before sibling lookup and preserves
original filename case. The 9,885 import assertions were run with portable
ASan/UBSan, not by that Windows harness. These bounded results do not qualify
general plugin closure or the integrated application. The
[changed-source native receipt](evidence/scrum209-module-name-native-38167423509/README.md)
retains the exact inputs and results.

Final focused owner confirmation
[38168539396](https://github.com/ThereptileII/Work/actions/runs/38168539396)
**passed** at exact `b08f9a1cf3360ff10e075b8b0134e5c39ac15f73`.
PowerShell 5.1 x64 and x86 each completed 79 owner assertions and 60 transaction
summaries; Go Win32 completed ten top-level groups and two subtests without
skips. The final fixture correction explicitly compiles/validates x86 inert
applications and preserves the first error when cleanup also fails. The
production owner/cleanup source is unchanged by these fixture repairs. Both
artifact digests, eight CRCs, three architecture-specific passed receipts and
25 unique source hashes are verified in
[the exact native evidence](evidence/scrum311-owner-native-38168539396/README.md).
This is selected owner/transaction qualification; it does not qualify a new
integrated application, signed commissioning broker or boat package.
General process/script/helper/module custody and authenticated signed
successor/fallback boat commissioning remain open. Existing boat guards and
the installed boat package remain unchanged.

SCRUM-211 now also has bounded SDK/runtime retention (`69711a06`), wired after
the native production build by `afa5fbcf`. All 29 focused capture checks and 19
existing runner-contract checks pass. The capture binds the actual production
CMake SDK selection, authenticates the TLS inputs and retains the declared
runtime DLL closure. Its generated inventory hash is informational; an
independently reviewed external pin is still required before retained-input
qualification. No full application build was dispatched for this helper.
Native capture and actual retained production-runner execution remain pending;
see [the capture contract](architecture/plugin-guard-input-retention.md).

## Exact-byte Staging native retest passed; replacement source is not built

Commit `5dd795be16081b4b0e521315bfc45f23a9af8251` compiled and
passed the Linux integrated regression gate, Windows integration build,
Windows contracts, native boat recovery/commissioning contracts and startup
updater interactions. The overall workflow is **failed**: its retained native
Staging qualification stopped in `boat/test-signed-update-artifacts.py` before
later application/installer checks or Staging publication. Windows denied a
test-only replacement of an open file with `WinError 5`; the original test
recognized only sharing errors 32/33. This is not evidence of a product crash
or a qualified package. The boat installation remains unchanged.

Development commit `74972445` extends that test to recognize the Windows
access-denied sharing case only after proving the open snapshot stayed intact
and the same operation succeeds when its handle is closed. All 16 focused
portable artifact tests pass. Native no-rebuild retest
[`38139852479`](https://github.com/ThereptileII/Work/actions/runs/38139852479)
passed the artifact check against the exact retained `5dd795be` application
bytes, then stopped at the next peer smoke. The ordinary instance's owned
first-run setup dialog was visibly open, but the first post-canvas diagnostic
snapshot had no setup controls; the test attempted to resolve those controls
before their next publication. The development harness now waits only for a
newer diagnostic tick from the same witnessed dialog, and still refuses partial,
ambiguous or missing controls without clicking. Eleven focused startup and
fourteen peer-contract tests pass locally. The next exact-byte native retest,
[`38140234560`](https://github.com/ThereptileII/Work/actions/runs/38140234560),
passed the corrected first-run step but exposed a different harness failure:
the second portable process remained alive behind OpenCPN's missing tide/current
harmonics modal. Its startup log showed portable resource paths made relative
to its explicit profile while the process inherited the CI checkout working
directory. The peer harness now starts that portable process in its own profile,
matching the pinned upstream path convention. The final retained native run
[`38140837201`](https://github.com/ThereptileII/Work/actions/runs/38140837201)
passed the complete selected Windows qualification against the authenticated
`5dd795be` executable bytes, with harness `efec7a4e`; it includes two 144-test
native XML suites, concurrent ordinary/portable peers, packaged updater,
installer, chart, mode-cycle and recovery checks. The original workflow remains
red; this is separate exact-byte retest evidence, not a rebuilt or newly
published package. See [retained retest record](evidence/2026-10-11-retained-staging-retest.md).

Subsequent SCRUM-317 route-label decluttering, SCRUM-209 read-only module
observation, SCRUM-59 source/font/MSVC inventory and provenance receipts,
SCRUM-57 packaged notice/source gate, SCRUM-60 About/attribution, SCRUM-64
first-run disclosure, SCRUM-14 vessel-setup row and SCRUM-212 failure-safe
peer evidence remain on the development branch for one combined Staging build.
Focused tests pass; none has native package acceptance yet. The secret audit
reports unresolved checkout matches and unsupported file coverage, and does
not qualify public delivery. The boat installation is unchanged; no Production
promotion or physical command is involved.

## Current Staging qualification — correction batch not yet built

The exact `bf68b355` Staging run `38122286949` passed its early Linux
contracts and fixture-free compilation but failed the integrated pilot smoke:
a fresh, bidirectional TCP status source made **Enable control** clickable
although product policy forbids TCP pilot output. Independent review found no
wire-output bypass. The compiled integration now uses the same endpoint policy
as the final output sink before offering control setup, while preserving the
qualified serial confirmation flow. Focused checks passed across three build
policies, including 216 offline drawer assertions; the unchanged product smoke
and native Windows gate still require a new exact-commit build. That run's
native Windows build succeeded but its retained qualification failed in the
concurrent peer smoke before later application checks. Profile 1 stayed alive
without a startup log or diagnostic; profile 2 reached ready state. The pinned
OpenCPN/wxWidgets Windows single-instance mutex ignores the profile directory,
so the original two-ordinary-process harness cannot establish its claimed
independent profiles. The corrected pending harness starts one ordinary and
one separately configured portable process, retains simultaneous boundary
assertions, and captures bounded owned startup evidence on failure. This
topology correction and the pilot product fix both await native execution.

The next Staging source batch also includes virtual-time Online AIS outage
checks (251 focused session assertions), an inert passive zoom collector,
signed-update custody foundations and their native x86/x64 qualification
wiring, plus an installer refusal for unqualified non-x86 executable helpers.
The helper foundations do **not** yet authorize a live update. Native
PowerShell, installed plugin loading, full installer lifecycle and boat checks
remain open. No new package is accepted and the boat installation is unchanged.

Before the next combined build, focused review found and corrected a Windows
metadata mismatch in two read-only security observers: Python can report
different creation/change timestamps and suffix-derived execute bits for a
path and its open descriptor. Both observers now compare common identity fields
across those sources and retain separate full baselines for mutation checks.
The focused artifact, plugin-inventory, change-selector, peer-topology and
plugin-guard orchestration checks pass locally. The plugin guard's real Linux
TLS matrix passed 13/13, including a rejected guard-bypass mutation. Its new
native orchestrator is not yet a Windows acceptance result: candidate SDK and
runtime provenance must be independently reviewed before execution on a
disposable Windows runner. The pending Staging selection requests one product
build and reuses the unchanged dependency producer; no new Staging run or boat
deployment has occurred for these corrections.

## Replacement Staging candidate — source integration pending native gates

The first combined Staging run, `38121327021` at `b3cef57c`, was deliberately
cancelled before its native application build. Its early checks passed, but a
focused audit found that a C6v2 producer could expire after automatic sensor
selection, temporarily hiding a healthy alternative source. `SCRUM-368` now
assesses current producer validity on owned copies before selecting or reporting
health. The focused 83-check regression includes heartbeat and group expiry,
fallback, explicit pins, fresh recovery and retained timestamps; integrated
Linux, native Windows and boat acceptance remain pending.

The replacement source also binds any future retained peer retest to the
authenticated producer receipt and executable hash, and adds a focused
PluginHandler/Downloader download-to-archive guard. Its 13 Linux loopback
cases pass; native Win32 execution, full plugin routing and installer acceptance
are still open under `SCRUM-211`. These source checks do not qualify a package.
The boat remains on the earlier `f1a1e68c` installation; no new software has
been installed and no physical output has been sent in this batch.

## Current boat-feedback integration — source checks, not a new candidate

The boat still runs the previously installed `f1a1e68c` prototype-parity
candidate. Its Staging run failed the Linux pilot smoke assertion after the
native Windows integration had passed; the one-line assertion correction is
the base of the current integration branch (`70b16aff`). No replacement
application has been packaged or installed on the boat in this batch.

The in-progress integration branch adds the observed 1280×800 rail-visual
clearance, readable Following chip, complete header health label, closer
ownship chip, direct saved-route entry from the header, chart-derived names for
new numbered route points, a wider XNav-only route-segment touch target, the
correct OpenCPN Connections tab for **Add a sensor**, independent measured
apparent wind on Weather, and a Windows SKAGER taskbar/relaunch identity.
Jira records the changes under SCRUM-361, 364, 298, 320, 365, 366 and 367;
native Windows and boat acceptance are still pending. Focused source checks
passed: 63 navigation naming assertions, measured-wind quality presentation,
25 delivery-workflow checks, and ordered application of all ten pinned OpenCPN
integration patches. These checks do not qualify an installer or UI rendering.

The lighthouse north of the boat remains unresolved: current evidence does not
identify that object's chart class, attributes, effective S-52 rule or selected
symbol. The existing guarded light/landmark mapping must not be broadened by
guesswork. A read-only Object Query on the exact installed candidate is the
next evidence step. The next-turn card and SCRUM-317 zoom responsiveness also
need boat observation. The earlier salmon cold-start/disclaimer observations
remain deliberately deferred per user feedback.

One combined Staging build follows the remaining scoped source fixes; no
Production promotion or public publication is implied. The boat's Legacy/Safe
recovery and remote-access paths are unchanged.

The combined security preflight also rejects retained byte-identical stock
OpenSSL 3.0.5 runtimes after installer addition preservation (SCRUM-208/209),
closes a proven `https:/` wxCurl HTTPS-to-HTTP redirect bypass in core and
private o-charts (SCRUM-211), and adds an exact-build concurrent two-instance
peer-containment gate (SCRUM-212). The TLS redirect repair passed 17 actual
wxCurl and 12 Downloader cases on Linux; the peer harness passed five focused
contracts. Native Windows, full packaging and real two-instance acceptance
remain pending. No source check here qualifies a Production release.

## Earlier 2026-10-06 Staging batch — SCRUM-311/312/313/28/29

The user selected these five issues for parallel implementation and explicitly
authorized actual manual autopilot commands. The replacement candidate is
`0.4.0-beta2.3`; native integrated and boat acceptance remain pending. The `.2`
candidate compiled and packaged but failed native first-start interaction:
main-window activation cancelled a held setup button before release. The
replacement adds the missing modeless setup guard and keeps the existing
focus-loss cancellation. No new boat
installation or physical command has yet occurred. The existing `c0d8d85` boat
application remains the last installed candidate.

The retained `.2` native failure is run `37440993866` (helper `be33cd4`, product
`0db45cb`). Its trace records pointer down, focus, blur while pressed, then
pointer up without activation. The exact activating callback was not traced;
the source and focused red/green test identify the unguarded delayed-recapture
path. Ten preceding isolated startup checks passed but do not override this
complete-run failure. The narrow guard passed the native Win32 red/green check
in run `37443443071` at `dcab13c`: original focus loss cancelled activation;
the corrected guard preserved focus and produced exactly one activation.
The downloaded artifact hash and replacement-source comparison are recorded in
[native evidence](evidence/2026-10-06-staging-batch/modeless-recapture-native.json).
One replacement product build follows this component proof. See
[focus evidence](evidence/scrum28-modeless-setup-recapture.md).

- **SCRUM-28:** six-step first-run boat setup, explicit Later/reset, live source
  health and atomic preference saving. Existing profiles remain read-only until
  the user saves. The dialog scrolls within the display workarea at increased
  scaling, with its action buttons kept visible.
- **SCRUM-29:** bounded, versioned settings export/import with preview and
  transactional restore. Credentials, charts, navigation files and pilot
  permission are excluded; restore turns control off and retains local binding.
- **SCRUM-312:** the already qualified native navigation-warning/healthy-startup
  fix is included in the combined application candidate.
- **SCRUM-313:** selected Actisense serial transport supports only the six
  explicitly enabled manual commands. Confirmation requires the exact command's
  completed write and subsequent fresh physical pilot feedback. Queue expiry,
  reconnect isolation and joined worker shutdown are covered by focused tests.
  Fresh installations and every new session start with control off. TRACK/WIND
  and autonomous control remain unavailable. Physical qualification is pending.
- **SCRUM-311:** the private repository is initialized with four distinct keys
  in the existing encrypted Secret Service collection. The bounded loopback
  origin and key custody checks pass. HTTPS activation remains pending and the
  actual installed updater/boat commissioning handoff is not implemented. The
  four-phase receipt reducer is only an offline contract. This candidate keeps
  its existing no-trust startup behavior; it does not claim live signed-update
  acceptance or ship test trust. See [private evidence](evidence/2026-10-06-staging-batch/private-origin.json).

Combined setup/backup checks pass (6/6). Fourteen pilot contracts, three
actual-source sanitizer harnesses, exact-ticket regression and complete Linux
component compilation pass within their recorded scopes. Narrow setup-preservation
checks (73) and existing preservation checks (85) pass; the actual native
restoration remains a CI gate. [Pilot evidence](evidence/2026-10-06-pilot-serial/README.md)
retains the acknowledgement race and worker-lifetime findings and their fixes.
Protected-key custody tests (8), origin tests (6), versioned output-policy checks
and package-input checks pass. These are focused preflight results, not a new
Windows, boat or release acceptance claim.

One integrated Staging build is requested after the combined source preflight.
The native setup/backup checks and serial harness run alongside the existing
required build/package gates. No design campaign, endurance run, Production
promotion or website work is included. The later physical permission supersedes
older requests for consent below; measured identity/feedback and qualified
transport gates still apply before any command.

The `.1` source correction at `584e0c824eeca3d37be8a6c1c20fbf34c97210b0`
passed native setup/backup/startup interactions, all three native serial
harnesses and native boat maintenance/restoration. Its Windows contract preflight
then exposed a test harness import omission: the extracted installer self-test
called the new output-policy function without importing it. The harness now
imports that actual function; the failed assertion was not removed. Windows
application compilation had not started. Candidate `.2` includes that correction
and requires fresh integrated qualification. Neither `.1` attempt is an accepted
boat package.

The subsequent `.2` preflight found a partial-download cancellation/EOF race;
the download still failed closed. A deterministic red/green case and the Go
suite cover the correction. Run `37418273537` then passed native startup,
serial, boat recovery and Windows contract checks, but refused SDK reuse before
Windows application compilation. The only newly differing SDK fingerprint input
adds the application pilot patch to a same-job inventory. Exact reviewed hash
compatibility preserves the immutable dependency bytes and all native probes;
see [the source comparison](evidence/2026-10-06-staging-batch/dependency-consumer.md).
The separate manual pilot child transaction passed native 32/64-bit PowerShell
qualification and independently verified downloads; [helper evidence](evidence/2026-10-06-pilot-manual-tools/native.json)
does not claim actual UI actions or physical pilot feedback.

The exact SDK correction then passed its native restore/reprobe run `37419757974`
without rebuilding dependency libraries. The earlier integrated Linux run also
found a pilot status regression: stale feedback correctly revoked control but
overwrote an outstanding command's timeout. Automatic revocation now keeps the
original timeout while cancelling output and permanently revoking confirmation
for that request. Focused red/green coverage passes. The smoke test replaces its
obsolete stale-STANDBY exception with no-request/no-byte assertions and requires
fresh feedback plus explicit re-enable. Run `37421283197` now passes the
integrated Linux build and pilot smoke, plus the preceding live-data checks. Its
next navigation-object smoke stopped because the new first-start boat-setup
sheet had not been deferred before inspecting permanent chart controls. This
is being corrected without removing the existing layout assertions. Native AIS
transport/TLS/session and cold full-application configure preflight passed at
`dd31ae64`, run `37422324401`; dependency libraries were reused, not rebuilt.
[Downloaded-artifact evidence](evidence/2026-10-06-staging-batch/integration-followup.json)
records the exact boundaries and the incomplete full candidate.
The one-action manual UI and completed-child binding-preservation helpers passed
native 32/64-bit PowerShell at `a134215c`, run `37421498782`; actual boat UI and
physical feedback remain unperformed. A subsequent bounded helper correction
will preserve an inert retained route GUID only with persistence OFF and fresh
explicit runtime NoActiveRoute proof; it needs its own native qualification.

## Previous increment — SCRUM-312 navigation warning and secure startup

Selected after the user asked for the next software feature. Implemented finite
authenticated human-wait/continue/cancel phases around the original OpenCPN
warning; healthy startup still requires the existing 30-second ready shell and
durable checkpoint. Focused Linux/native qualification passes on `4cd191cb`
([native run](https://github.com/ThereptileII/Work/actions/runs/37377435744)); integrated
package and installed acceptance remain pending. [Evidence](evidence/2026-10-05-startup-warning/README.md). The boat's c0d8d85 app remains
unchanged; no actuator commands. [Contract](installer/startup-human-wait.md).

Pilot component follow-up: 14 native contracts, 6 presentation cases and 161 button
checks passed; the serial harness Windows-path correction subsequently passed
its focused native run without repeating those suites. [Evidence](evidence/2026-10-05-pilot-integration/README.md). Physical pilot identity/response and
serial reconnect/output qualification remain open in SCRUM-295/313/19.

## Autopilot follow-up — not yet physically qualified

SCRUM-295/307/313 are being investigated against the actual existing AutoTrack /
OpenCPN Actisense serial connection. Read-only boat inspection confirms live
marine input, but the installed `c0d8d85` candidate has no accepted pilot identity
or mode feedback and its product control sink remains closed. No physical
commands, plugin restoration, connection-direction change or app restart was
performed. The current application is left running.

Source inspection establishes an event-driven address-claim discovery gap when
joining an already-running bridge. New bounded passive diagnostics distinguish
missing identity, vendor traffic, rejected envelopes and stale/conflicting
feedback, without manufacturing identity or enabling output. A focused serial
patch corrects an ASan-reproduced short-request memory overread and escaped
payload bounds. It does not qualify serial reconnect, queued output or physical
delivery. Six manual command encodings are tested against the actual pinned
bridge parser; all button events and disabled-state boundaries are exercised in
an isolated component. TRACK/WIND remain unavailable.

Focused Linux and native component checks pass; integrated Windows/product
qualification remains pending for this increment.
These source/test changes are not yet a new installed product. Physical
STANDBY/AUTO/course tests require the user's separately requested secured-vessel
confirmation and a qualified transport; neither is inferred from simulated
results. See [inspection and scope](pilot-boat-integration.md) and the corrected
[commissioning procedure](boat-commissioning.md). Jira retains the remaining
acceptance gates; no pilot issue is marked Done on these results alone.

## Secure startup updater — implementation and focused qualification

Current Staging candidate `c0d8d85fb602e86d40e2f3f1be32307919702408`
([run 37330218586](https://github.com/ThereptileII/Work/actions/runs/37330218586))
has passed Linux integration (152/152 tests in each variant), both native Windows
builds (144/144 each), security checks and installer/recovery packaging. Exact
compiled inputs are sealed and retained. The original runtime job passed real
launcher startup, supervised Setup startup and corrupt-candidate rollback, then
failed the restored application's normal-close check. Its failed evidence remains.
The helper now waits for fresh OpenCPN deferred initialization before issuing the
single close request, without changing the 30-second exit deadline.

The separate retained retest
([37344321589](https://github.com/ThereptileII/Work/actions/runs/37344321589),
harness `1bd62fd727dfe712889aed3910befb75179f7314`) passes all four actual
updater checks, nine bounded Staging installer checks and the previously skipped
chart gate against those unchanged binaries. Restored startup was ready at 3.656
seconds and closed by 4.422 seconds. The downloaded evidence SHA-256 is recorded
in the linked receipt. The original 17 passing runtime checks are retained;
there was no application rebuild. Provenance composition in
[37348369038](https://github.com/ThereptileII/Work/actions/runs/37348369038)
passes assembly of the frozen candidate. The draft uploader retained ten exact
assets but failed on the final corresponding-source ZIP. Publication-only
[continuation 37350987228](https://github.com/ThereptileII/Work/actions/runs/37350987228)
now passes at helper `b1b407d3d0570cec75b6d1bb7fdf9805390528d6`: all eleven
original assets match the frozen manifest and a separate authenticated transport
receipt records completion. The [Staging release remains a private draft](https://github.com/ThereptileII/Work/releases/tag/untagged-17c2e26c8768e056f05b).
The failed upload stays recorded; no rebuild or native retest was performed.

The original actual-Lifecycle step also passed all 46 checks,
including a deliberate six-second child startup delay, authenticated after about
7.8 seconds. Suspended-loader healthy/wrong-hash cases also pass. This supersedes
the pending focused corrections below, without turning inert fixture results
into packaged application or boat acceptance. The earlier isolated green probe
did not execute the delayed Lifecycle script; that omission remains recorded.
SCRUM-310 is **Done** for the separately reviewed closed-session preservation.
All 17 reviewed saved-setting changes were retained. A complete private backup
contains 2,400 files / 1,181,602,021 bytes; the only live INI recovery edit restored
the original COM8 direction byte from `0` to `1`. Original plugin inventory and
permissions were verified before durable restoration and preserved-baseline
receipts completed. Their full hashes are in [the sanitized evidence](evidence/2026-10-05-secure-updater.json).
This completion grants neither automatic migration nor launch permission.

Exact helper `10371eb06f481fce72791a1dcfaa2b6fbdc883ee`, exported from local
`73e6702`, passes [run 37362804219](https://github.com/ThereptileII/Work/actions/runs/37362804219):
Linux 85 preservation + 127 adoption checks, Windows 85 preservation + 178
adoption + 27 actual transaction checks. The original `3d8eaf7` read-only boat
inspection correctly refused Linux LF resource pins against Windows CRLF files,
before mutation. Both actual stock and installed WMM resources were verified
byte for byte against the pinned source's Windows checkout. Only exact Windows
hashes were corrected; runtime normalization and alternate LF hashes remain
forbidden. Earlier failed native fixture runs 37357797892/37358227937 and the
LF mismatch remain retained. See [the preservation contract](installer/session-preservation.md),
including its explicit stock-to-parent WMM path limit.

The qualified `c0d8d85` Setup has now been installed manually on the boat. All
980 managed-file hashes match ownership, the supported stock executable remains
unchanged, and fresh Inventory/Prepare/Apply completed input-only commissioning
from the preserved baseline. This is not a signed offered-update transition.

**The first guarded bootstrap failed acceptance.** Exact interactive observation
shows the standard OpenCPN navigation warning waiting for human **Agree**, before
`InitTimer` and deferred initialization. The supervisor's 90-second health bound
and 120/150-second monitoring bounds expired; no authenticated healthy startup
was established, and no crash is proven. SCRUM-312 is In Progress for finite,
authenticated human-wait handling. The warning must not be bypassed or treated as
healthy startup. The original timeout evidence is retained.

The incident was then recovered without relaunching the application. After exact
review of the standard navigation caution, a separate one-use helper delivered
Agree. Deferred initialization completed and the application logged 30 seconds
of normal processing. A bounded native observation found the enabled, visible
SKAGER main window. The old launcher's failure message was closed normally; it
returned its expected failure exit code 1. At 20:44:43 UTC the same exact
candidate was still responding and the failed launcher was gone. It is left open
for the user. The recovery helpers passed 10 and 21 focused policy checks each
on Linux and native Windows; those checks do not qualify a product correction.

This recovery does **not** repair SCRUM-312, change the failed bootstrap result,
create an authenticated known-good receipt, or prove a signed update transition.
The original typed-variable helper refusal and bounded child-window inspection
refusal remain retained. No application rebuild, visual-design review, restart or
hardware command was performed for the recovery. The Desktop guide
**SKAGER - Test c0d8d85.txt** provides the [five-minute startup/settings case](installer/boat-updater-test-case.md).
Its user result is pending; no Update Now/Later offer is expected yet.

SCRUM-311 tracks only the private signed Staging channel dependency of
SCRUM-72/73. Explicit public-root packaging, an operator-side TUF publisher and
source-derived increasing Beta versions are implemented, with focused local
checks. No real signing keys or public-root configuration have been provisioned.
Private HTTPS requires the user's Tailscale sign-in, currently pending. Native
qualification of the new publisher/package wiring and a real increasing-version
offer remain pending. The physically connected boat also requires the separately
documented [pre-launch commissioning boundary](installer/boat-signed-update-boundary.md)
for successor and fallback generations. Do not remove the existing bootstrap-only
guard or equate an unconfigured startup with signed-offer/Now/Later/offline
acceptance. No new application build is requested before those concrete inputs
are ready.

SCRUM-23/25/26 are Testing. The new installed launcher composes retained
signed trust, measured OpenCPN compatibility, explicit Update Now/Later consent,
verified installer custody and the existing transactional lifecycle. Candidate
acceptance requires an authenticated live-process startup receipt and a durable
recovery checkpoint; failed/interrupted startup restores the verified previous
generation. Legacy, Safe and portable recovery keep their direct startup paths.
See [the implementation contract](installer/secure-updater.md).

The earlier integration attempt was **not packaged or qualified**. At
`c330f33883a9af0e7fc7b95d4cbcf916a3ff576d`,
[run 37317835346](https://github.com/ThereptileII/Work/actions/runs/37317835346)
passes both Linux builds (152/152 tests each), both Windows builds (144/144 each),
and the compiled security checks. Packaging then correctly refuses untracked
source: the published repository lacks the local upstream gitlink, so its
explicit OpenCPN clone dirties the integrated checkout. The narrow correction
reuses the exact native launcher/source artifact already built by that run's
successful clean updater job, retaining all source/commit/hash checks. Early
compiled-output retention is also being added; the failed run retained diagnostic
evidence but no recoverable application package. No boat installation changed.
The transfer/recovery correction passes focused native
[run 37326068134](https://github.com/ThereptileII/Work/actions/runs/37326068134)
at `e0ae050cdd59fe1309628f020344ccd8ffdbd10b`. Downloaded original and
copied launcher/source bytes match exactly; seven native early-retention fixtures
pass. The same implementation tree is now requested as Staging candidate
`c15ecddc0fadd7ba18bde97976f04ea03da9df06` in
[run 37326986679](https://github.com/ThereptileII/Work/actions/runs/37326986679).
Its Windows prerequisite stopped in the isolated actual-Lifecycle fixture:
a generated inert child timed out connecting to the startup pipe and cleanup
masked the original receiver failure. This is not evidence of a real application
crash. Failure diagnosis and fixture cleanup are being corrected without changing
production authentication or timeout rules; Windows application compilation has
not started for this request. Packaged runtime and boat acceptance remain pending.
The isolated diagnostic run
[37329174360](https://github.com/ThereptileII/Work/actions/runs/37329174360)
identifies a test deadline defect: the inert CLR child reached its entry point
after the fixture's five-second listener had already closed. The healthy fixture
now allows 30 seconds and deliberately delays six seconds before connecting;
production remains at 90 seconds. A separate source-supported early-loader race
is corrected by deferring full executable inspection until the connection is
established, before reading the receipt; native suspended-process success and
wrong-hash cases are required. Neither correction is accepted until that focused
native gate passes.

The first focused native run,
[37294573113](https://github.com/ThereptileII/Work/actions/runs/37294573113),
at `891acf93408a8d094811f144bd3373337f4865b9` passed Linux Go tests but
found an elevated-Windows directory-owner mismatch and a queued popup-choice
race. Both have source corrections; that failed run is not acceptance of the
corrections. Local combined Go checks and actual PowerShell contract checks pass;
native authenticated pipe/process, installer and actual application gates
remain pending. The second focused run
[37298291972](https://github.com/ThereptileII/Work/actions/runs/37298291972)
at `658c70d93c549a47e79ca380bbef01857da35d1a` verifies the owner and choice
corrections: all 67 top-level Windows Go tests (285 including subcases) and all
96 native dialog checks pass. It still fails overall: compiler-source packaging
rejects setup-go's provisioned junction, a receipt helper inherits a PowerShell
module-path problem, and fast candidate startup fails authenticated receipt.
These failures are retained; the listener-order correction is a hypothesis until
its native delayed-receive case passes. See [the focused evidence](evidence/2026-10-05-secure-updater.json).

The third focused run
[37300372946](https://github.com/ThereptileII/Work/actions/runs/37300372946)
at `4dc7bb97bdb2708180f6ae52fb3dc4a85c5bae67` contains those corrections,
the download/cancel window, isolated signing preparation and independent native
installer-recovery checks. Source packaging and authenticated startup pass;
46 actual-Lifecycle assertions pass but the helper incorrectly retains the last
intentionally failed child's exit code. The progress helper times out without
preserving its case; it also used a posted key message which does not represent
native Escape input. Helper-only corrections preserve all assertions and bounds.

The corrected focused run
[37301261353](https://github.com/ThereptileII/Work/actions/runs/37301261353)
passes at `7f1686d60b89740fe2c8783b648517e9f32313a7`: 63 top-level Linux
Go tests (288 including subcases), 72 Windows Go tests (302 including subcases),
96 native dialog checks, 26 prompt/progress protocol cases, five actual receipt
sender/receiver scenarios, 305 shortcut checks and 46 actual-Lifecycle checks.
Native launcher/source packaging and the inert signing boundary pass as well.
Downloaded evidence hashes are recorded in the linked receipt.

The combined Staging candidate `4bdae3d5873adc94b9ea98651a6888b63d3c861c`
([run 37302067258](https://github.com/ThereptileII/Work/actions/runs/37302067258))
adds real packaged launcher bootstrap,
supervised installation, authenticated application startup and an explicitly
faulted disposable candidate's guarded rollback to installer smoke. Those
integrated results and boat acceptance remain pending. No product trust root,
signing secret or public update endpoint is invented or bundled. Production
activation remains a separate gate.

This Staging attempt stopped before Windows application compilation: early Go
setup changed `PATH`, so the immutable dependency reprobe correctly rejected
the environment. The retained OpenSSL facts differ only in `PATHSha256`; all
recorded tool binaries, versions and other fields match. Go setup is moved after
the existing dependency/application checks, immediately before updater packaging.
A focused ordering regression passes; no SDK identity check is relaxed and the
approved dependency bundle is unchanged. Native confirmation passes in
[37304055112](https://github.com/ThereptileII/Work/actions/runs/37304055112)
at `4adb0004f8e1e9df12b814260cdbb754be7ca275`: both dependency reprobes,
eight TLS lifecycles, 18 transport scenarios and 216 session checks. The earlier
Staging Linux job also passed 152/152 tests in each of its fixture and production
builds, plus its functional runtime checks. No Windows package was produced by
that failed attempt. Automatic run
[37306797494](https://github.com/ThereptileII/Work/actions/runs/37306797494)
correctly selects helpers only for the workflow correction and skips product
jobs; it is not an integrated acceptance or a new package. An explicit Staging
build request is needed to qualify the corrected packaged application.

The explicit replacement request
[37308001424](https://github.com/ThereptileII/Work/actions/runs/37308001424)
at `c8023dc6ca77e760229ef2e47269dae3b2c55e8c` again passes both Linux
builds (152/152 each), functional runtime checks and all focused updater checks.
The guarded stock-warning case initially refused a foreground mismatch; the
same-commit single-job retry passes without changing its guards. Native Windows
then stops during configuration, before application compilation: an early
maintained-TLS cache file satisfies the pinned upstream batch's whole-stock-bundle
sentinel, so stock LibArchive headers/library are absent. The correction is a
consumer-only, verified cache preparation step, followed by an actual native
configure-only preflight before another full build. The immutable SDK and its
identity checks remain unchanged. No Windows package or installed updater
acceptance is claimed from this run.
The consumer correction passes focused native run
[37316217945](https://github.com/ThereptileII/Work/actions/runs/37316217945)
at `4540022d67dae14b9067b57e2af21c27404964d2`: both SDK reprobes,
AIS security checks, stock provisioning, authenticated TLS restaging and complete
Win32 CMake configuration pass. Downloaded evidence proves producer prefixes
unchanged and LibArchive selected from the stock cache. This probe omits the
optional private chart adapter and does not build/install the application.
The corrected product snapshot is now explicitly requested for combined Staging
packaging; the boat remains unchanged pending its result.

The frozen boat-feedback Staging run
[37283380247](https://github.com/ThereptileII/Work/actions/runs/37283380247)
failed before application compilation because relative dependency paths changed
meaning in a child process. The narrowly scoped helper correction passes twelve
focused tests. See [the retained failure evidence](evidence/2026-10-05-boat-feedback/dependency-reprobe-path-defect.md).
The boat installation has not been changed by updater development. No Production
promotion, broad design review or endurance run is scheduled.

## Remaining boat feedback — coordinated Staging batch

All 16 reported items have been investigated; **15 code fixes are implemented**
and await native/boat confirmation. SCRUM-300's callback-lifetime correction is
included in the replacement; SCRUM-306 still needs installed target reception
confirmed. The changes include
passive pilot discovery, coherent anchor distance, anchor/route transitions,
route/waypoint naming and contextual cards, actual XTE, chart-object information,
AIS list scrolling and the reported chart/anchor/ownship presentation defects.

SCRUM-306 remains In Progress: its radius-control upgrade is implemented, but
the installed missing-target symptom is not yet proven fixed. An isolated live
probe received real traffic; that does not qualify the current application's
chart/list path. Coordinate-free commissioning diagnostics are implemented to
separate incoming reports, subscription state, cached positions and radius
filtering without exposing the credential or vessel position.

The focused shared suite passes 11/11 components plus the added actual-source
anchor-renderer test. These receipts identify a dirty development worktree, not
a release. The initial native preflight found and corrected an obsolete build-test
parameter-order assertion before full Windows compilation. The corrected
helper passes native run 37267103145. Integrated Linux/native Windows Staging
qualification with authenticated SDK reuse remains pending. The boat remains on the previously
installed candidate until a qualified replacement is available. No Production
promotion or broad design/endurance campaign is requested. See the
[batch evidence](evidence/2026-10-05-boat-feedback/README.md).

Pre-deployment review found that synchronous OpenCPN plugin callbacks can change
the selected route during the new anchor-to-route transition. Candidate
`ff7ae5d84884b8369acdd5eba9666152869ad497` is held from boat installation. The
replacement re-resolves and validates state after those callbacks. Its focused
actual-function fixture passes 33 mutation cases plus normal/failure paths;
the previous source fails with the expected stale-pointer error. The held candidate's contract
gates passed 97 Linux and 94 Windows tests; integrated runs were superseded.
Those results do not qualify the corrected replacement.
The equivalent inherited watch-set callback is also corrected and covered by
seven additional cases in the same fixture. Four native fixture entry-point
settings found in the superseded logs are corrected; application code is
unchanged by that build-setting repair. The next candidate includes all three
corrections, without another design or endurance campaign.

Candidate `30a0bb7853c1fa879d682cdc70b2d72204eb5977` compiles on native
Windows and passes 144/144 integrated CTests. Twelve of thirteen new shared
components pass; the AIS scroll fixture failed because an ordinary native owner
mouse-motion event contaminated its lifetime counter. The isolated native
diagnostic reproduced the extra event, then the corrected fixture passed all
47 checks in run 37273670351, preserving the original assertions and adding
three exact-dispatch checks. Application code is unchanged by this correction.
See [downloaded native evidence](evidence/2026-10-05-boat-feedback/ais-scroll-native.json).
The new component step is moved after immutable build retention, so future
runtime-helper failures retain the compiled inputs. Replacement integrated
packaging and boat acceptance remain pending; the failed run did not produce
a qualified installer.

Replacement `982a2b54d06bea4de9507b11e445dae00651d06f` also compiles
on Windows and passes 144/144 CTests. Its following AIS dependency reprobe
stops because it tries to reuse the initial build's curl diagnostic directory;
the diagnostic correctly refuses overwriting earlier evidence. This is a build
orchestration failure, not evidence of an application crash or TLS failure.
The producer SDK remains unchanged. A focused repeated-reprobe correction must
pass native Windows before the next application build. The AIS security gate
is moved before expensive application compilation so it fails early.

The repeated native correction now passes at `7397e057` in
[run 37282487804](https://github.com/ThereptileII/Work/actions/runs/37282487804).
The downloaded, hash-verified artifact contains two completed native reprobes
and passing AIS results: 8 provider TLS lifecycles, 18 transport scenarios and
216 actual session checks. Earlier run 37277511537 passed only one native
invocation because its first invocation was skipped; it is not repeated-native
proof. The corrected workflow explicitly requires both success transcripts.
See [the receipt](evidence/2026-10-05-boat-feedback/dependency-reprobe-native.json).
Staging selects the same `7397e057` revision for integrated packaging. The
preceding `982a2b54` Linux job completed both variants with 152/152 CTests each;
this does not qualify the replacement. No boat application change is claimed.

Separate helper corrections pass 199 delivery-policy checks on each platform
and 64 inert native window cases, including modern AIS row selection. Their
receipts identify helper revisions separately from the application. The boat
source checkout is `d8ab001`; all 118 helper files match reviewed bytes. The
installed application remains `0da2c64`; no live AIS acceptance is inferred.

## Online AIS freeze: first narrow repair — SCRUM-301

The provider no longer holds its UI/state lock while sending a subscription.
Pinned IX can synchronously call back after a write failure; the old lock scope
could deadlock both the worker and the next UI read. Subscription reservation
now precedes the unlocked send, preserving immediate confirmation, viewport
changes and newer enable/disable intent.

All **8 Linux provider TLS lifecycle scenarios pass**, including two new
send-concurrency cases. The old provider fails the same reentrant-read test by
the expected bounded timeout. The corrected launcher/runtime helper suite
passes 11 tests on both platforms. The focused native Windows
[run 37237278099](https://github.com/ThereptileII/Work/actions/runs/37237278099)
passes at `e4273b68d4024ed1d38a3fcc3f5550221135a11a`: MSVC Win32 build,
8 provider TLS lifecycles, 18 adversarial transport scenarios and 178 session
checks. Downloaded evidence and compiled-source identities were verified.
It reuses the authenticated SDK and builds only AIS test clients.
Integrated application/boat reproduction remains pending; this is not yet a
confirmed resolution of the user's boat symptom. SCRUM-301 remains Testing.
See [focused evidence](evidence/2026-10-04-ais-freeze.md).

This increment does not request a full application build, deployment, design
review or Production promotion. The installed boat candidate is unchanged.
Missing AIS targets/radius and list scrolling remain separate Jira work.

## Further delivery streamlining — SCRUM-225 / 292 / 293

Implementation now selects CI from actual changed inputs, retains compiled
Windows packages before desktop qualification, and supports targeted retesting
against the original bytes with a separately identified test-helper revision.
Library reuse is an immutable authenticated producer bundle, with fresh native
toolchain/source checks; failed or mismatched inputs never fall back silently.
New packages remain Staging by default. Production and design review still need
explicit instructions. See [build efficiency](build-efficiency.md).

Focused delivery checks pass **190 tests on Linux and 190 on native Windows**
at `0e6ee68aa70f82a5d29561cf8de346e0b28d7422`
([run 37235438463](https://github.com/ThereptileII/Work/actions/runs/37235438463)).
Native dependency-only production and cross-run reuse both pass. The measured
SDK stage drops from **50m30s** cold to **1m14s** authenticated reuse, with fresh
native tool/source checks. This is a dependency-stage measurement, not a full
application-build timing. The retained archive's timestamp defect is corrected
and tested across forced clock changes.

SCRUM-292/293 helper gates pass; SCRUM-225 and the complete Staging delivery gate
remain Testing pending the next real consuming application candidate. No extra
application build is requested merely to close those gates. No boat, design or
Production qualification is claimed. The installed boat candidate is unchanged.
See the [delivery-efficiency evidence](evidence/2026-10-04-delivery-efficiency.md).

## Staging delivery and explicit Production promotion

The user has adopted versioned Staging delivery followed by a separate
Production-readiness flow. Promotion must perform only release-readiness work;
design validation, prototype comparisons, aesthetic refinement and design
screenshot/DPI review sets require explicit user instruction. Older blanket
visual-promotion requirements do not override this decision. Functional,
navigation/data-validity, security, installer/profile-preservation, recovery and
package/source checks remain. Unrequested design review is not recorded as a
pass. See [the delivery policy](delivery-workflow.md) and
[SCRUM-290](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-290).
The workflow split and release tooling are implemented under SCRUM-290:
ordinary delivery defaults to Staging; both channels create versioned draft
GitHub Releases while public access is closed. Production is manual-only,
requires a named Staging release and the user's instruction, and qualifies the
retained application/package without recompilation. Installer readiness,
profile preservation, recovery, navigation and security remain functional gates.
Prototype comparisons and design-only DPI/painter runs are opt-in. The complete
installer matrix runs in Production; Staging checks installation/startup,
profile preservation and rollback. Evidence is bound to both product and test
helper revisions, package hashes and the original producing run attempt.

Focused local verification passes **75 tests** across release inventory,
transport, qualification/workflow policy and installer helper suites. Edited
workflows pass actionlint and duplicate-key checks; Linux build scripts parse.
These are delivery-tool results, not application or installer acceptance.
Native delivery-tool [run 37221989304](https://github.com/ThereptileII/Work/actions/runs/37221989304)
passes **75 Linux and 75 Windows checks**, including Windows script parsing,
at `311ddabafde937ab7764ba1c396730ac2d3e4648`. The default-branch workflow
registration preserves the existing firmware files; `staging` is the software
starting point. The first real Staging release/promotion execution remains
separate verification; no application release is claimed from helper CI.
See [delivery implementation evidence](evidence/staging-delivery-workflow/README.md).
No application build, design review, Production promotion, boat change or public
opening was performed for this process update. Historical evidence below retains
its original outcomes and the installed candidate is unchanged.

## Boat candidate installed and left open

The exact **0.4.0-beta2 / 0da2c64379d5a9cc4b9b2bd068de6e0b69816577**
candidate is now installed on the boat PC and launched through the audited
read-only path. The process is responsive and left open for the user.
**Startup still awaits the OpenCPN welcome acknowledgement**, obscured by a
Windows feature-update reminder: select **Remind me later**, then **Acceptera**.
No normal navigation frame/fresh canvas-completion marker is claimed yet.
The incompatible inherited RTL-SDR plugin was safely excluded from the new
candidate; original files and the previous installation remain recoverable.
Read-only commissioning remains applied and physical output remains disabled.
No additional visual/endurance tests, Windows update or reboot were performed.
See [the precise handoff record](evidence/boat-handoff-0da2c64/README.md).
The full CI/installer and release gates remain open.

## Current user handoff instruction

The user now requests: finish the current Windows build, install the candidate
on the boat, verify that SKAGER starts and responds, and leave it running for
manual testing. Do not start additional visual, chart-comparison or endurance
runs. Do not dispatch the prepared supplementary package-review workflows for
this handoff. Existing in-flight CI remains the package producer; this direction
does not qualify unperformed visual, security, hardware or public-release gates.
Verify the downloaded package identity, compatibility, recovery backup and
read-only launch boundary before installation/startup. Keep the safe read-only
configuration while leaving the application open; do not restore output-capable
configuration behind a running application. Record the exact installed build and
handoff state. Broader acceptance issues remain open, rather than being marked
Done from successful startup alone.

## Current replacement — installer test boundary correction

The active frozen replacement is `0da2c64379d5a9cc4b9b2bd068de6e0b69816577`
(local `f1ea102cb362ed719cef50bb2c2506f1c470dd12`),
[run 37207119257](https://github.com/ThereptileII/Work/actions/runs/37207119257).
Its [6,881-entry mapped tree](evidence/skager-installer-test-f1ea-publication.json)
is independently reconstructed. Only the installer harness and failure-only
binary retention change execution behavior. Application, production installer,
chart resources, typography, logo and TLS policy are unchanged. Twelve existing
completion tests pass; the actual Windows helper parses without execution.
The [replacement contracts](evidence/contracts-0da2c64/README.md) now pass
94 Linux and 91 Windows cases (94 unique), plus 20 existing lifecycle repeat
executions. The [original same-run restart receipt](evidence/restart-0da2c64/README.md)
passes independent hash/CRC and exact commit/run/attempt checks, with actual boat
acceptance false. The [exact Linux integrated artifact](evidence/linux-0da2c64/README.md)
now passes independent original digest/CRC verification and byte comparison of
all 27 retained reports. Both fixture and fixture-free builds pass the same 147
cases (294 executions); actual core Downloader/wxCurl pass 12/13 cases. Four
unchanged chart captures show coastline/ENC content after mode return in software
and llvmpipe OpenGL. Their glyph rasterization differs; neither individual symbol
families nor Windows/boat font conformance is accepted by this wide-view check.
The exact native fixture-free application build passed at 15:30:24 UTC, followed
by successful real-host private-module loading, Windows Downloader trust and
peer-buffer checks. The fixture navigation/restart/scenario and AIS transport
steps and recovery packaging also pass. The installer suite stopped at 15:52:38
UTC after 35 completed checks because the new standalone test helper uses
`Get-FileHash`, unavailable in its inherited PowerShell environment. Its error
occurs before the separate `SelfTest` call. The exact candidate's clean install,
Beta 1 upgrade/startup, profile preservation, modes, rollback/repair and missing
import guard had passed. The [helper-only correction](evidence/installer-selftest-hash-0da2c64/README.md)
reuses the unchanged production `Hash` function; native execution of that repair
and the rest of the suite remain unverified. The CI run is not qualified.

Under the user's startup-only handoff instruction above, recover the unchanged
original Setup/recovery/source from the failure-retention artifact for the
limited boat test, after exact byte/source verification and read-only preparation.
The 700,027,780-byte original exceeds the connector's 512 MiB limit, so a
transfer-only workflow will retain a smaller selection of unchanged payloads.
It does not rebuild, retest or turn the failed qualification into a pass. Broader
installer/security/visual/public-release gates remain open. Endurance stays
skipped. The later installation/handoff above supersedes this earlier pending state.

## Preceding 55ef51e — chart binding passes, installer test correction

The preceding replacement is `55ef51e4944e8570f6a391dad20b0db8447a7443`
(frozen local `e768b06bf5c130038abfd6e1166da0078037cb5b`),
[run 37199379614](https://github.com/ThereptileII/Work/actions/runs/37199379614).
Its [complete 6,862-entry mapped tree](evidence/skager-chart-mutex-e768-publication.json)
is independently reconstructed. Publication followed the bounded native
before/after mutex proof and independent review. The private adapter now uses
the pinned host's existing MSVC compatibility definition; explicit early-check
stages retain diagnostics if loading fails. Chart artwork, UI, typography and
TLS policy are unchanged. Independent trust/peer checks can finish after a
successful production build even if the module check fails, but packaging still
requires every original gate. Exact-commit Linux qualification has passed as
recorded below. The actual native product/module, trust, peer, recovery-package,
DPI and chart gates now pass. The installer test stopped after 34 checks when
its intentionally missing DLL was correctly refused by the earlier PE-import
guard: the test still expected a later loader error. No eligible delivery was
produced and no boat installation has changed. The [test correction](evidence/scrum217-installer-dependency-gate.md)
requires the exact missing import and retains a separate actual missing-DLL
loader check. Application and installer implementation remain unchanged.
Endurance is explicitly skipped.

The replacement's [completed contracts](evidence/contracts-55ef51e/README.md)
pass 94 Linux and 91 Windows cases, with no failures or skips, plus ten additional
executions of the existing restart test on each platform. Its independently
verified [same-run restart receipt](evidence/restart-55ef51e/README.md) retains
the exact source/run/attempt and all four prerequisite successes. These do not
replace the native application or the required package/boat gates.

The [exact replacement Linux artifact](evidence/linux-55ef51e/README.md) now
passes independent original size/digest and 4,251-entry CRC verification. Both
fixture and fixture-free applications pass 147 cases; actual core Downloader
and wxCurl pass 12 and 13 cases respectively. Product reports fixtures disabled,
status-only and zero loopback output. Endurance is explicitly skipped. Four
original chart captures retain ENC/coastline after Legacy return in software
and llvmpipe OpenGL. Their glyph forms differ; no actual preferred font face,
individual symbol family, physical GPU or private-chart acceptance is inferred.
The original native Windows artifact is retained as `11305580491`, SHA-256
`747e38e8d88c4e49fca6b6efb181101db10824ac87cf55f6e0be80011e8bec7a`.
Installer completion, eligible delivery and boat review remain open.

### Preceding ecf7e0c candidate and verified chart binding correction

The preceding replacement is `ecf7e0c46609cf4cb29141964d7c4bde98b002b7`
(frozen local `1988df7a8a0ae8ddc6365ca46026d32fccfa0bdc`),
[run 37191400051](https://github.com/ThereptileII/Work/actions/runs/37191400051).
Its [complete 6,833-entry mapped tree](evidence/skager-console-ownership-1988-publication.json)
is independently reconstructed. The narrow logger ownership repair has passed
its native before/after proof and independent source review. Only the console
helper, probe-result flushing and focused proof changed; application, chart and
TLS policy inputs remain unchanged. The existing functional, security, package,
installer and boat gates remain required. Endurance is explicitly skipped.
The native fixture-free product build passed at 11:07:29 UTC. The following
real-host private chart-module check failed at 11:07:33 UTC. The run has now
completed with failure. Its [original Windows artifact](evidence/windows-ecf7e0c/README.md)
is independently digest/CRC verified: 139 fixture and 139 production cases,
12 actual Downloader and nine private wxCurl cases pass. The repaired console
helper completes teardown. The positive private-module child exits
`0xC0000005`, before its JSON report; there is no retained faulting instruction.
Default/invalid-option early checks pass, and profiles are unchanged. Public
Downloader, recovery-package and installer gates were skipped, so this run
cannot produce an eligible candidate. Independent DPI/chart checks pass with
software fallback, not an actual OpenGL result. No package or boat acceptance
is claimed.

The private adapter omits the mutex compatibility definition used by pinned
OpenCPN while its first binding call locks a global mutex against the older
staged MSVC runtime. SCRUM-259's [small native before/after proof](evidence/scrum259-native-binding-runtime/README.md)
now reproduces the first binding lock's access violation against the exact
14.12.25810.0 CRT and passes the full guarded binding lifecycle. The original
13-member artifact and all seven source identities are independently verified.
The guard matches upstream; explicit early-check stderr stages preserve
diagnostics if module loading still fails. Neither change replaces stock or
boat runtime files. SCRUM-217's next workflow also retains module failures
immediately and permits independent trust/peer checks to finish; packaging
still requires every original gate. One combined replacement follows this
bounded native proof and independent review; full-host acceptance remains open.
Endurance remains skipped.

The exact replacement's [contract audit](evidence/contracts-ecf7e0c/README.md)
passes 94 Linux and 91 Windows cases with no failures or skips, plus ten
additional executions of the existing restart test on each platform. Its
[same-run restart prerequisite](evidence/restart-ecf7e0c/README.md) is verified
against the original artifact and all four required success values. These
results qualify prerequisites only; the integrated application, actual TLS,
package and boat gates remain separate.

The [current Linux integrated artifact](evidence/linux-ecf7e0c/README.md) is now
independently digest/CRC verified: both builds pass the same 147 cases, and the
actual core Downloader/wxCurl suites pass 12/13 cases respectively. The product
loader identifies this exact commit, fixtures disabled and status-only output;
its loopback pilot record contains zero output. Endurance is explicitly skipped.
Four retained Day/Legacy-return captures show ENC/coastline content, neutral
structural colours and the smaller logo. Linux software/OpenGL chart glyphs
differ; native Windows/boat typography remains unaccepted. Linux OpenGL uses
llvmpipe, so this is not physical-GPU evidence. Windows and boat gates remain open.

### Preceding d29da37 candidate and verified ownership correction

The preceding candidate is `d29da372af86c01cb2817f932891fd9408d882fe`
(frozen local `615118f351abdea7b79a042db63e58fe0a635e0a`),
[run 37184477492](https://github.com/ThereptileII/Work/actions/runs/37184477492).
Its complete 6,781-entry mapped tree
`06beb1e9a06af7d7ee30abf32556a08602a165b6` is independently reconstructed.
The native production step failed; the independent DPI and ENC steps subsequently
passed, and the final evidence upload completed. This candidate is ineligible for installation. The
[original failure audit](evidence/windows-d29da37-production-failure/README.md)
confirms 139 passing production cases and a successful owned-CA import, followed
by Downloader probe exit `0xC0000005` after GET/HEAD return. The former timeout is
gone, but no structured result or completed TLS case is produced. There is no
eligible installer. The [bounded native ownership proof](evidence/scrum211-native-console-ownership/README.md)
now directly observes wxWidgets deleting the previous helper's owned startup log
during initialization. The corrected helper establishes wx first, then owns its
persistent logger. All five proof cases pass, including 16 complete teardown
cycles and retained fatal assertions. The original artifact and all six frozen
source identities are independently verified. Actual Downloader/wxCurl results
now flush before teardown. No production TLS, application or chart behavior
changed. One combined replacement will retain the existing gates; this narrow
proof does not establish actual TLS acceptance or the original AV instruction.
Its [completed contracts](evidence/contracts-d29da37/README.md) pass 94 Linux
and 91 Windows cases, with zero failures/skips, plus ten additional executions
of the existing restart test per platform. The independently verified
[same-run restart receipt](evidence/restart-d29da37/README.md) binds the exact
commit/run/attempt and all four required success values. These close package
prerequisites, not integrated application, TLS, installer or boat acceptance.

The [exact replacement Linux integration audit](evidence/linux-d29da37/README.md)
now verifies the original 19,751,200-byte artifact and all 4,251 ZIP entries.
Fixture and fixture-free builds each pass the same 147-case suite with zero
failures/skips; the actual Downloader's 12 and core wxCurl's 13 cases also pass.
Production is status-only with zero pilot output, and endurance is explicitly
skipped. Native Windows production qualification failed as described above. Linux does not exercise the
Windows console helper, native trust store or private wxCurl integration.

The preceding `bccdbb1` run completed with a native production-gate failure;
it produced no eligible installer. Its [original failure evidence](evidence/windows-bccdbb1-production-failure/README.md)
contains 139 passing production cases followed by the bounded 30-second
`downloader-valid` timeout. The earlier owned-certificate import passed in
0.34 seconds. No navigation-application crash is established by this timeout.

The [small native console proof](evidence/scrum211-native-console/README.md)
then reproduced a visible wx logging message box in a standalone process.
The shared explicit stderr logger/initialization passes file staging/rename;
an actual assertion exits 86 with its diagnostic, rather than blocking for UI.
Both Windows TLS probes now use that helper and flushed stage messages.
Production Downloader, wxCurl and certificate-verification behavior are
unchanged. The helper is included in the private adapter's source-input manifest.
Focused local checks pass: 17 preparation cases and 12 actual Downloader cases.
One combined replacement will run the existing functional/security/package
gates. Endurance remains explicitly skipped. The native proof is not TLS or
boat acceptance, and the previous full run will not be retried unchanged.

The [retained native visual review](evidence/native-visual-bccdbb1/README.md)
shows neutral structural paint, prototype font fallback and the smaller logo.
Both chart phases actually used software rendering, and the private adapter
was unavailable. Individual symbol recognition, actual private ENC, boat GPU,
fonts and physical display remain required checks; complete fidelity is not
claimed from these screenshots.

The [fresh boat font inventory](evidence/scrum263-boat-fonts-20261004/README.md)
confirms the prototype's preferred families are installed; actual native glyphs
remain a visual gate. The [separately qualified review tools](evidence/scrum289-native-review-tools/README.md)
pass 75 native fixed-action cases and are now staged as one verified 117-file
bundle at `C:\XNav\scripts\review-47c0a789967c29a618db1f0249a42ade756bf460`.
This adds the real prototype zoom and System/recovery paths without widening
the action boundary. No application was installed or launched by that staging.
The saved older launch attestations remain expired; fresh post-install
commissioning is required for the replacement generation.

### Preceding bccdbb1 candidate

The preceding candidate is
`bccdbb11cef827d3b63731fe1e874bc72bf47d2d` (local frozen
`1a6733a1cbcc817aa0f13fa5acc41aac62a54146`),
[run 37177738716](https://github.com/ThereptileII/Work/actions/runs/37177738716).
The complete 6,709-entry mapped tree
`ffd4c39e28dfaa29caaaa16619da32a2752e9158` is independently verified.
Application and chart inputs remain the reviewed 17ab044 source. Execution
changes repair the proven unattended certificate prompt, bound probe subprocesses,
and skip endurance by explicit user direction. Native functional, actual TLS,
package and boat acceptance remain pending. No duplicate full run was started.
The [same-commit contract audit](evidence/contracts-bccdbb1/README.md) now
confirms 94 Linux and 91 Windows cases, with no failures/skips, and ten additional
executions of one existing restart test per platform. The
[same-run restart receipt](evidence/restart-bccdbb1/README.md) is independently
verified against its original artifact. Integrated application/package and boat
acceptance remain separate; these counts do not qualify the whole candidate.

## Preceding 17ab044 evidence and proven blocker repair

Frozen local `0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4` is published as
`17ab044a1e5222dc71791ac8118454219efe8734` in
[run 37164360050](https://github.com/ThereptileII/Work/actions/runs/37164360050).
The [6,658-entry mapped tree](evidence/skager-symbols-0a52-publication.json)
is independently reconstructed and preserves all eight unrelated root files.
The reviewed application source is identical to the successful `98d2c45`
preflight below; only documentation follows it. This one combined candidate
includes supplied symbol artwork, neutral structural colours, prototype font
selection, the smaller approved logo, classified light outlines and the compact
unchanged-trigger warning. It also includes the AIS traversal correction that
passed its short original-file native proof before this run began.

The [completed contract jobs](evidence/contracts-17ab044/README.md) pass at this
exact commit: 94 Linux and 91 Windows CTest cases, with no failures or skips.
Both also pass ten additional executions of the same restart test. Root verified
the original decoded log identities, result rows and frozen source hashes;
these counts do not include the ongoing integrated or package gates.
The [same-run restart prerequisite](evidence/restart-17ab044/README.md) is also
independently acquired and verified: exact commit/run/attempt, original artifact
digest/CRC and all four native success values. It explicitly records no boat
acceptance and does not qualify the eventual application package.
The original native artifact is now retained and digest/CRC verified. The
application builds, 139 fixture-enabled tests and 139 production tests pass;
the maintained-TLS AIS runtime passes. After the actual Downloader/wxCurl probes
link, the trust harness produces no first-case receipt. Its last certificate
output is at 02:11:25 UTC; the user-directed cancellation is at 04:16:57 UTC.
The [bounded native diagnostic](evidence/scrum288-native-trust-import/README.md)
reproduced a visible unattended Windows Security Warning in CurrentUser root
import. The corrected disposable-machine import and bounded helpers now pass
four native cases in run 37177584391; import takes 0.344 seconds and exact cleanup
restores the trust inventory. No application/TLS validation policy changed.
The single replacement candidate combines this proven harness repair with the
explicit endurance skip; actual TLS and package gates remain required.
No eligible installer or Windows overall acceptance is claimed.

The user explicitly changed the active objective on October 4 to deliver a
stable SKAGER boat-test candidate quickly and **skip endurance testing**.
[The narrow policy change](evidence/scrum217-user-directed-endurance-skip.md)
records endurance as skipped, never passed or shortened. Functional, security,
installer, chart and recovery gates remain. Named/public release stays withheld.
SCRUM-287's parallel-endurance preparation is deferred in Idea. The obsolete
run was cancelled normally; its complete native evidence was preserved.

The [read-only boat connection check](evidence/boat-connection-20261004.json)
confirms all three remote-access services were running with no navigation
application running. No installation, launch or retirement occurred in this cycle.
The [read-only source-review preparation](evidence/boat-source-review-preparation-bccdbb1.json)
now verifies the 14 retained review-note copies and the unchanged current stock
and managed DLL identities. Four built-in plugin sources and all three Dashboard
bridge files match their prior review. The private adapter adds presentation and
owned observation behavior, with its existing closed vendor-helper boundary
explicitly retained. This reuses equal source evidence; it is not a fresh launch
attestation or candidate binary acceptance. Post-install inventory and boat
rendering still require the new package.

The separate package-security tooling is now ready at
[`f288b03`](https://github.com/ThereptileII/Work/commit/f288b031fa83990cc1ad3986e545581651ac9908)
(local `f6b7dff`), with its complete 1,457-entry mapping independently verified.
It preserves the SKAGER naming/native-path fixes and later unsupported-Setup/
profile-restoration checks. Its exact owned-certificate helper also passes the
[separate native proof](evidence/scrum288-native-trust-import/README.md#separate-candidate-package-helper),
including refusal and exact trust cleanup. No candidate request exists yet:
actual packaged PluginHandler TLS/peer and profile-preservation acceptance still
requires an original eligible replacement artifact. The failed bccdbb1 run is
ineligible. This test-tool update changes no
application input and caused no replacement application build.
The [exact producer-label comparison](evidence/scrum211-candidate-prerequisite-label.json)
also corrects one stale upload-step name after the endurance policy change;
all 19 required names match the frozen producer and actual native job. Nine
adapter cases and two request cases pass, including rejection of the old label.
Strict success, commit/run/attempt, digest and restart requirements are unchanged.

### Current native prerequisites verified

The [original native maintenance artifact](evidence/boat-maintenance-bccdbb1/README.md)
contains 20 passing PowerShell suite receipts and 11 unique Python navigation-copy
tests. Nested assertion totals are retained separately. The
[original private-loader artifact](evidence/native-ocharts-loader-bccdbb1/README.md)
contains 38 passing native groups using inert DLLs. Root verified both original
archives and their frozen-source identities. These are completed same-candidate
prerequisites, not actual product/boat runtime or visual acceptance. No test was
rerun to produce these documentary audits.

### Current Linux gate passed

The [original bccdbb1 Linux audit](evidence/linux-bccdbb1/README.md) verifies
artifact `11295011188` against its API/upload digest and all 4,251 ZIP CRCs.
Both fixture-enabled and fixture-free production builds pass the same 147-case
suite, with zero failures/skips: 294 executions, not 294 unique cases.
Functional inputs, modes/persistence, charts, recovery and production status-only
output checks pass. Root verified the original bytes, unchanged report subset,
production identity and explicit endurance-skipped receipt; returned software
and llvmpipe chart captures retain coastlines. This closes the Linux gate for
this candidate. Native Windows, package/security and boat acceptance remain open.

### Completed prior Linux gate

The [original Linux artifact audit](evidence/linux-17ab044/README.md) confirms
147 fixture-enabled and 147 fixture-free production CTest cases, all passing
without skips, plus 10800.118 seconds of actual endurance. Both groups are test
executions of the 147-case integrated suite, not 294 unique tests. Recomputed
resource deltas are resident memory −223232 bytes, handles 0 and threads 0; average
CPU is 3.353% of one core. The audit retains 90 dropout recoveries and 765 route
progress changes. Root reviewed coastlines in start/end and returned software/GL
images. The GL renderer is llvmpipe; this does not qualify the boat GPU or native
Windows appearance. One zoom-in observation leaves the scale unchanged with a
changed centre, so every opposite zoom pair is not claimed to restore its viewport.

The completed Linux endurance above is retained historical evidence. No further
endurance runs or parallel-endurance infrastructure are required for the current
user-directed boat-test objective. The earlier
[read-only collector](evidence/native-log-diagnostic-12d7058/README.md) could not
retrieve a live log; the terminal original native log is now available and
bounds the harness delay as described above.

### Bounded build-tool closeouts during this run

SCRUM-255, SCRUM-272 and SCRUM-273 are now Done against their own acceptance
criteria: [early gettext prerequisite](evidence/scrum255-closeout/README.md),
[native trust-probe path normalization](evidence/scrum272-closeout/README.md),
and [duplicate source-cache publication](evidence/scrum273-closeout/README.md).
Original failures, native proofs and stronger later private-package evidence
remain linked. The gettext audit also corrected one stale source-order test
after the shared-helper refactor; its single focused case passes and both
deliberately wrong orderings are rejected. This later test-only maintenance is
not in frozen `17ab044`, does not change application inputs, and did not trigger
another full build. Full maintained-TLS, product/package, chart and boat gates
remain open; these issue transitions do not qualify the current candidate.

SCRUM-285's [generated-project traversal closeout](evidence/scrum285-closeout/README.md)
is also Done after root reviewed the original eight-case native proof, unchanged
frozen execution inputs and the subsequent actual AIS-stage success. Detailed
runtime artifact audit and the remaining candidate/package/boat gates stay open.

## Prior full candidate — 4ddf1f3 native AIS gate failed

Frozen local `48c2f8b8d5d4103bcbaacc45a18c4e6a238d4c52` maps to remote
`4ddf1f383e495150551946dd36103cd77cea85eb`, with the
[5,662-entry tree independently reconstructed](evidence/skager-parent-context-publication.json).
The [short native proof](https://github.com/ThereptileII/Work/actions/runs/37155593528)
passed; its [original artifact and audit](evidence/scrum278-native-parent-context/README.md)
retain the negative rejection and successful exact restoration. Root independently
verified the archive, all 25 CRCs and unchanged receipts. Both negative/positive
PATH hashes exactly reproduce the original 63c1029 failure pair; only that field
differs. This proves the parent-context correction, not AIS or TLS acceptance.

The full branch now uses that **same source SHA**, with no intervening changes,
in [run 37155858878](https://github.com/ThereptileII/Work/actions/runs/37155858878).
It includes the previously checked chart metric consistency and neutral building
alias corrections below. Actual AIS runtime, remaining product/package gates and
boat qualification are pending. No eligible package or boat modification is claimed.

At 23:21 UTC on October 3, the same run's Linux integrated functional stages
have passed through the public ENC/plugin rendering gate; its actual elapsed-time
endurance stage is running. The native Windows application build and integrated
mode checks passed, followed by source reproduction, installed peer-key refusal
and staged-loader checks. Actual chart gestures, repeated crash recovery, fixture
UI and same-job dependency capture passed. Native AIS step18 then failed;
fixture-free production, TLS and package stages19–27 were skipped. Independent
DPI and public ENC checks subsequently passed. The original artifact now proves
OpenSSL/zlib parent and child checks and AIS configuration passed, then the AIS
wrapper raised `KeyError: 'Include'` while treating MSBuild configuration metadata
as a project dependency. AIS compilation/runtime never began. SCRUM-285's narrow
ItemGroup traversal repair and [retained-project checks](evidence/scrum285-native-ais-project-closure/README.md)
are integrated. Its [short native proof](evidence/scrum285-native-ais-project-closure/native-ab92dc7/README.md)
passed all eight cases in run 37162505696 at exact `ab92dc7`; root verified the
original artifact, all CRCs and execution-input identities. This qualifies the
parser correction before a replacement build, not actual AIS compilation or TLS
runtime. A bounded downstream review of the original eight projects, 55 source
paths, expected JSON headers and 155 retained dependency files found no further
concrete setup defect. It did not execute native binaries or reuse producer
receipts across jobs.
This is a separate wrapper defect, not the earlier PATH mismatch or an observed
application crash. The separate pristine
Linux/native Windows baselines, recovery/restart/guarded-mode prerequisites and
both contract jobs passed. The [terminal contract evidence](evidence/contracts-4ddf1f3/README.md)
records 94 Linux and 91 Windows CTest cases; these are not the still-pending
integrated application or package totals. No duplicate full build was dispatched.

Supplied artwork is integrated on the separate symbol-completion branch, without
changing this candidate: SCRUM-279 marina, SCRUM-280 rock/wreck, SCRUM-281 cable
waveform and SCRUM-282 fishing-stake area. The [combined resource proof](evidence/scrum279-283-combined/README.md)
passes: exactly 505 new pixels per theme, all other RGBA/PNG metadata unchanged,
and complete prior XML recovered after only the reviewed transformations.
The [combined source/object review](evidence/scrum281282-combined-review/README.md)
proves ordered patch composition and normal optimized core/private compilation.
Root reviewed the three-theme glyph/painter comparisons. The [combined actual
Linux canvas evidence](evidence/scrum279282-45d-linux-canvas/README.md) now passes
40 captures, ten clean sessions and ten exact unmasked whole-chart Day returns;
root verified all 292 retained file identities and independently recomputed those
returns. Actual unknown-depth rock selection is proven. The sampled awash rock
and wreck correctly retain OpenCPN's isolated-danger symbol; they do not qualify
the alternative supplied glyphs. The retained charts contain no matching marina
or fishing-stake polygon. Native Windows, private DLL and boat acceptance remain
open. The oversized cable
experiment was rejected; the final 24-pixel-equivalent waveform is implemented.
The fishing motif retains native repeat spacing and now uses the actual private
adapter compile guard. These changes are not present in `4ddf1f3`.

SCRUM-283 is a newly observed private-chart safety qualification blocker. Focused
execution of original pinned conditional procedures shows that private
`_UDWHAZ03` does not call the chart's associated-depth-area query, while core
OpenCPN does. The retained negative case has UWTROC/WATLEV3 with missing VALSOU,
a 5 m safety contour and an associated 10 m DEPARE: core selects ISODGR51 and
DisplayBase; the original private branch differs. This source-level result is
inherited, not caused by the new artwork and not yet an observed boat-chart
failure. The original failure remains evidence; per-source resource preservation
must not be reported as core/private safety parity. The narrow adapter-only
callback correction is now integrated, with [actual-source safety and lifetime checks](evidence/scrum283-private-hazard-association/README.md)
passing. Both original failed point cases now select ISODGR51/DisplayBase. The
original private reference-point/first-area limitation remains explicit; native
private-DLL and boat qualification are still required before closing SCRUM-283.
The running `4ddf1f3` build remains development evidence, not final private-chart
navigation qualification.

SCRUM-284 separately addresses the remaining heavy ordinary all-round light arc.
The prototype has no custom all-round/tower glyph; any treatment must preserve
the upstream full-circle/range-band meaning while applying its paint hierarchy.
The [outline-only refinement](evidence/scrum284-all-round-light/README.md) is now
integrated as `fc6348a`: 339 actual-method assertions per core/private renderer
and both complete optimized renderer objects pass. Root reviewed its 36 controlled
images and verified 75 evidence identities. The [actual `fc6348a` chart comparison](evidence/scrum284-fc6348a-linux-canvas/README.md)
now passes: normal 29-step incremental build/link/install, 16 captures, four
clean sessions and four exact Day returns. All eight Standard chart images are
unchanged; SKAGER differences occur only around the old light ring. Root inspected
Day/Night comparisons and verified all 142 retained file identities. Native/boat
conformance is not claimed.

SCRUM-286's [compact OverZoom warning](evidence/scrum286-overzoom-warning/README.md)
is integrated as `98d2c45`. The original warning trigger and stock fallback remain
unchanged; the new presentation uses existing prototype warning-callout roles.
Its 260 focused assertions, eight controlled images and three optimized actual
production units pass. Root reviewed the source and verified 45 evidence files.
The [actual `98d2c45` application comparison](evidence/scrum286-98d2c45-linux-canvas/README.md)
now passes: normal 38-step single-job incremental build/link/install, 16 captures,
four exact whole-chart Day returns and four clean exits. All eight Standard chart
controls remain identical to `fc6348a`; SKAGER changes are confined to the old
top-left warning region. Root inspected Day/Night comparisons and the complete
Day chart, and verified all 143 evidence file identities. The combined source is
ready for one full native replacement; no eligible Windows package is claimed yet.

The [current identity audit](evidence/scrum246-15-identity-33f9fd1/README.md)
confirms the 124-DIP approved logo, nine-size Windows icon and customer SKAGER
captions. The original native `4dd` font probe resolves Segoe UI because that
host lacks Variable Display, as permitted by the prototype's stack. The boat's
HTML reference resolves Variable Display; actual native boat typography still
needs review. Internal compatibility identifiers and immutable design evidence
retain their original names.

## SCRUM-278 — focused Windows environment correction

The shared parent setup is integrated as `970671d`, with the existing Gettext
receipt bound to dependency reuse in `7568bf7`. Build and AIS callers preserve
the same native-Perl/Gettext prefix order; AIS verifies existing tools without
installing them or rewriting captured identities. The 13 helper checks, 12
initial receipt/order cases and one additional receipt-tamper case pass locally.

`eab2e95` adds the short native proof on its own CI branch, using the actual
unchanged tool-facts helper and exact producer setup span. Thirteen preparation
checks passed before the successful native run. The full candidate was held at
failed `63c1029` until that focused native evidence was verified.

## Prior full qualification — 63c1029 native AIS gate failed

Frozen local **`d18da7ccc94b175d1f4a09a0019ab7e4b4f55d31`** is published as
**`63c1029584a325a961fa89794071cbf053c2e966`** in
[run 37147671879](https://github.com/ThereptileII/Work/actions/runs/37147671879).
The [independently reconstructed5113-entry tree](evidence/skager-combined-d18-publication.json)
preserves all eight unrelated root files. This single full replacement starts
after the focused certificate/native-source checks and the combined normal
49-step Linux Release build/link/install pass. Application/resources/patches
match exact preflight326daf7; later changes are documentation and optional
native capture tooling. The native integrated build/mode checks, fixture UI and
dependency receipt capture passed, but step 18 (same-job AIS runtime gate) failed.
The [original native artifact](evidence/scrum274-native-63-path/README.md) proves
that OpenSSL parent verification rejected only `environment.PATHSha256`; all
other tool and source facts match. AIS configure/build/runtime had not started.
Root independently verified the original archive hash, all 13,395 CRCs and the
sole-field difference. SCRUM-278 owns restoring the same parent initialization
used by the successful dependency build, with a short native proof before a
full replacement. Exact environment verification will not be relaxed.
Production, subsequent TLS and product packaging were skipped.
No eligible boat package or replacement full run is claimed.

Both actual software light/fog and R/W/Gsector theme sets pass16captures with
clean exits. Actual MesaGL now dispatches the previously missing XNLIT013 light
point and preserves FOGSIG01. The mixed-sector GL set passes8 SKAGER/Standard
captures, including exact whole-chart Day returns and three successful fan
builds/draws. Full light/fog GL theme comparison remains **failed**:355 changed
pixels in numeric26.2 near the chart edge, separate from the light/fog glyphs.
Original images/assertions are retained; it is not accepted as mere antialiasing.
A [single read-only actual metric trace](evidence/scrum268-326-elevation-metrics/README.md)
confirms LNDELV28 elevation8m→26.2ft: the first atlas miss changes average width9
to16, while subsequent hits keep9, shifting otherwise unchanged text7px left.
All four traced images exactly match the original failed captures. SCRUM-268 now
owns the minimal core/private consistency correction, integrated locally as
`a8481a8`. Its [actual corrected Linux chart gate](evidence/scrum268-5bb-linux-canvas/README.md)
passes at exact `5bb7e05`: normal29-step incremental build/link/install,12
OpenGL captures, three exact unmasked Day returns and three clean exits. Root
verified all116 retained evidence files and inspected Day/Night/Standard images.
The initial Day chart is unchanged; subsequent differences are confined to the
diagnosed elevation label. Native/private-DLL/boat acceptance remains open;
this correction is **not** in the frozen63c1029 Windows candidate.

That visual investigation continues independently of Windows qualification.
The observed brown point below the light is now identified as generic BUISGL36.
The [isolated default-building alias](evidence/scrum265-building-point-alias/README.md)
is integrated locally as `78ef9ff`, after 128 focused checks, independent
source review and verification of 26 input/output identities. Only Simplified lookup 1091 uses prototype neutral
inks; all 81 alpha values, geometry, original/Paper/conspicuous symbols and
classification remain intact. The [actual 27e93e4 Linux canvas gate](evidence/scrum265-27e-linux-canvas/README.md)
now passes: eight software/Mesa captures, two exact unmasked Day returns and
two clean exits after the normal 28-step incremental build. Root independently
verified all 146 evidence identities and recomputed the eight before/after image
comparisons: zero changed chart pixels outside the two actual building tiles.
The single loader and combined-resource checks also pass; their assertion
totals are not separate test-case counts. Brighter
Dusk/Night generic ink relative to the preserved conspicuous tile is an explicit
unaccepted readability boundary. This addition also remains outside frozen candidate 63c1029.
The [all-round light review](design/reviews/scrum264-all-round-light-boundary.md)
confirms that the prototype supplies no full-circle range-ring replacement;
its retained upstream outline and Night paint remain visible differences.
No package, full visual conformance, private renderer or boat acceptance is
claimed. The boat installation and frozen candidate application stay unchanged.

## Pre-publication correction record — original GL light point failure

The native certificate-fixture repair passes its [short Windows gate](https://github.com/ThereptileII/Work/actions/runs/37145287262)
at exact `a191102c6f150f47980bdb1c774878483c203b99` / local `827cf9b`.
[Seven helper checks and the independently audited original artifact](evidence/scrum277-native-fixtures-a191/README.md)
prove original CRLF rejection, corrected issuance and specific expiry rejection.
The runner tool is OpenSSL 3.6.4; maintained 3.5.9 full Downloader/wxCurl TLS
acceptance is still mandatory. No new full candidate has been dispatched.

The corrected `c24c154` Linux Release application builds and links with
unchanged warning policy. Eight real-chart software SKAGER/Standard theme
captures pass. [Actual Mesa GL failure and original diagnostics](evidence/scrum275-c24-linux-canvas/README.md)
showed the added central light missing because an unrelated SOUNDG multipoint
container has no scalar coordinate. OpenCPN deliberately stores its geometry in
arrays; the presentation inventory incorrectly read the unused scalar instead.

Local `326daf7` corrects both inventory passes without changing soundings or
independent-point refusal. The regression failed against the original helper;
[147 core / 147 private / 145 no-GL focused checks](evidence/scrum275-multipoint-sounding/README.md)
now pass at normal O3/Werror. Independent source review checked parent/clone
initialization. The warm application is advancing to this exact combined source
for actual software/Mesa light and sector captures. This is not yet a GL pass.

Root now composes reviewed SCRUM-274/275 plus scoped-lifetime repair and the
[SCRUM-276 compact sector fan](evidence/scrum276-compact-ca-fan/README.md).
576 focused software/Mesa checks pass for each actual core/private method
variant, with exact Standard/oriented/uncertain image fallback. Original sector
geometry remains authoritative. Full-canvas clipping and near-cap tile cost
still require actual candidate review. No all-round/expanded-light or complete
lighthouse-family conformance is claimed. Fonts, smaller logo and neutral
structural palette remain in the batch. The boat installation is unchanged.

## Combined replacement: certificate-fixture setup failure isolated

Native job `111243795980` failed after the application built and all **139/139
production tests passed**. Actual Downloader and private wxCurl trust probes
configured and linked, closing the prior CMake path failure at that boundary.
Before TLS assertions started, OpenSSL rejected the expired-certificate fixture's
CA index (`could not load/parse file`). `Set-Content -Value ''` writes a newline;
the CA database requires an initially zero-byte file. **SCRUM-277** owns the
small correction and native fixture-only proof before another full candidate.
The [original failure artifact](evidence/scrum272-native-154-expired-fixture/README.md)
is retained, independently hashed and inspected. No TLS acceptance is claimed.

Integrated modes, chart gestures, repeated recovery, fixture UI and dependency
capture passed. Independent DPI/ENC checks passed with software fallback; actual
native OpenGL remains open. Module/package/installer gates were skipped, so no
eligible boat package exists. A next-run workflow-only correction immediately
uploads existing production failure records; every original step, failure status
and package eligibility guard is preserved. Its ordered YAML comparison passes.

Local **`96c0c2705aacae511b6f8c22afaa6618dbabceff`** is published as
**`15452e512fd073090b1a4cea7c9010eac0874118`**. The complete 4,782-entry mapped
tree `3c53cff0332dfc09babb8e5b8daa7986f77a9c84` was independently reconstructed
with eight unrelated root files preserved. One
[combined full run](https://github.com/ThereptileII/Work/actions/runs/37136712793)
started at 16:24 UTC after the focused proof below. The
[publication receipt](evidence/skager-combined-96c0-publication.json) keeps exact
identities. Application source is frozen during qualification; no replacement
application has been installed, launched or retired on the boat in this cycle.

The [focused native run 37135967220](https://github.com/ThereptileII/Work/actions/runs/37135967220)
passed at exact source `71470d07d8dd9ac6a3050e2b64093e0892473c97` / local
`1c4d817c373b85838bc7c1b5d97d11bd026b3454`. SCRUM-272 normalizes native
CMake paths while retaining the original production targets and TLS assertions.
The first short run exposed a source-cache publication failure;
SCRUM-273 prevents competing duplicate writes by giving each locked blob one publisher, preserving parallelism and
each source path's hash/size verification. The original failure is retained.

Six cache regressions, 24 path checks, the original native configuration failure
control and ten actual core/private x86 objects passed. The
[independent original-artifact audit](evidence/scrum272-native-714-compile/README.md)
binds the source/header closure and four generated projects. Root separately
verified the archive hash, all 171 CRCs and all ten object bytes. This is compile
proof, not link/TLS or application acceptance. The next full candidate retains
those mandatory gates and batches the already native-compiled SCRUM-271
notification styling; no repeated standalone dependency build was needed.

Supplementary package review now supports
[named lateral/cardinal IHO scenes](evidence/scrum264-named-iho-review-scenes/README.md)
with locked provenance and unchanged yellow-pair checks. These require actual
image review and do not claim complete symbol-family conformance. Boat native
chart/font/logo acceptance and replacement installation remain pending.

A subsequent read-only lighthouse review identified a remaining presentation
gap: ordinary sector and long-range lights often emit upstream `CA` arcs rather
than the `SY(LIGHTS11–13)` commands covered by the new compact aliases. They
therefore still lack the prototype's independent central point. **SCRUM-275**
tracks a bounded follow-on in an isolated worktree, retaining actual sector
bearings, colors, ranges and visibility. The frozen candidate is unchanged and
must not be described as complete lighthouse conformance. Prototype fan styling
and pin/focus/readout interaction are separate open SCRUM-14/15 concerns.
**SCRUM-276** now owns a separate ordinary R/W/G compact-fan paint increment:
the prototype wash and fine translucent lines, retaining upstream geometry and
navigation distinctions. Software and GLSL need bounded alpha-capable paint;
changing shared symbol colors alone cannot reproduce the reference. This work
is isolated from 15452e5 and does not include expanded/pinned interaction.

The isolated CA-point implementation `ef266211e214c9ad57b14c7283b3ecd2fe71c67a`
is followed by `5085641e1c7af3f6294ce99bd42820622d48f3d8`: ordinary mixed-color
groups now use a generic prototype location point while each original sector
remains independent. An exact offset FOGSIG lookup is allowed; tower/pile
groups still retain stock presentation because the added point would obscure
their structural symbols. The follow-on passes 121 core, 121 private and 119
no-GL focused checks, and four affected production compilations. Root reviewed
the delta and independently verified source/output identities and unchanged
painters. These are not actual canvas results. The 32,768-object bound remains;
the roughly 55 ms maximum all-light desktop case needs boat responsiveness
review. SCRUM-275 is in Testing, with native/private-DLL/boat visuals and
scaling/rotation still open. No change entered the failed 15452e5 candidate.

In parallel, **SCRUM-274** implements an isolated read-only
AIS transport-observation dependency for SCRUM-227. The pinned transport has no
public positively attributed endpoint copy. A value captured inside its accepted
connection path, with immutable time and generation, avoids UI/socket lifetime
leaks. Independent review also found rejected-Open and unlocked credential-read
races that the new observation must guard. This work is not part of 15452e5;
no firewall, real outage, hardware command or boat action has been performed.
The separately reviewed implementation `20ee0b566527ceab5d46663df348d3f96fd905e8` is now
in Testing: 178 session checks, six actual TLS/provider lifecycle cases and 18
existing transport scenarios passed on Linux. Root reviewed the boundary and
independently verified 12 source inputs, 129 compiled IX records and four
retained outputs. Native Win32 runtime and actual outage/boat gates remain open;
the frozen build excludes it. It is now composed locally for the next candidate.
Its isolated continuation `e4400d8acabf10e012398caf164d0e9973567b46` prepares a
small native runtime gate for the next required candidate, reusing verified
same-job TLS dependencies. Three offline guard checks pass; actual native
configure/link/runtime execution is still pending. No separate dependency or
application rebuild was launched for this preparation.

The separate source-composition rehearsal `e6ef3843dc604368d32bc823c770bec76d890b69`
combined these two reviewed workstreams without modifying 15452e5. Root now
incorporates that reviewed composition and the lifetime correction below.
Ordered core/private patches, committed-byte preservation and owned-header
closure pass; the only original conflict was two appended documentation sections,
both retained. Isolated input results are not reclassified as combined runtime
proof. This is preparation for the next required candidate, not a published or
accepted replacement, and it does not close remaining lighthouse-family styling.

The exact e6 Linux Release attempt failed `-Werror=dangling-pointer` in the
scoped light inventory before link. [Original failure evidence](evidence/scrum275-e6-linux-build-failure/README.md)
retains full compiler output and verifies all 4,610 donor fingerprints unchanged.
Isolated repair `eaff747` uses scoped stable ownership, restores the previous
borrow before destruction, and preserves stock paint on allocation failure.
Focused actual-source checks at `-O3 -Werror` pass 133 core, 133 private and
131 no-GL cases. The previously failing actual Release object now passes unchanged flags; the
same warm application build/link and canvas proof continue separately. No warning
is suppressed and no e6 application/canvas pass is claimed.

## Production application builds; trust-probe configuration blocks candidate

Local **`8a0ed1f646e2551a55639c1cc3fb609cc2652464`** is published as
**`442960ba55277845e171f9b95a38838f66c23981`**, independently reconstructed
tree `3b74065f9f9886ebe73f33f85b71ca8bb469865e` (4,689 mapped entries; eight
unrelated root files preserved). The [corrected full run](https://github.com/ThereptileII/Work/actions/runs/37126951293)
started after the focused SCRUM-270 proof below passed. The
[publication receipt](evidence/scrum270-preview-validity/publication.json)
keeps the exact identities. Application source, patches, chart assets and CMake
are unchanged from 1835/9d98. The difference includes the coherent preview
predicate, immediate failed-fixture upload and separately qualified review tools.
Native/full/boat acceptance is pending; the current source is frozen and no
new application has been installed or launched on the boat.

The exact native job `111215213199` passed integrated build/exercise at
15:10:14 UTC and the formerly failing full preview scenario suite at
15:15:05 UTC. Startup/loader, pointer chart interaction and repeated crash recovery
also passed. Its same-job dependency closure and fixture-free product build then
passed, including **139/139 production CTests**. At 15:30:28 UTC the private
downloader-trust probe failed CMake configuration on Windows backslash paths
(`Invalid character escape '\a'`). No TLS assertion ran. **SCRUM-272** tracks
the narrow path-boundary correction and native proof before another full run.
The [original failure evidence](evidence/scrum259-native-442-trust-configure/README.md)
retains the complete production transcript and independent artifact audit.
No application/setup/probe executable was retained; the private DLL alone does
not make a review package. Packaging, installer and boat gates remain open.
Independent DPI/public ENC checks passed with software fallback only, not native
OpenGL acceptance. The Linux elapsed-time gate continues in the background.

The boat machine produced one isolated **HTML reference** capture in Day, Dusk
and Night. [Original captures and actual font observations](evidence/scrum263-boat-html-reference-5884701/README.md)
confirm Segoe UI Variable Display for the visible root typography and Segoe UI
for geographic land labels. A dual-class landmark resolves to Segoe UI Semibold;
this does not establish normal-only LIGHTS-description weight. The original HTML
and its old illustrative logo are unchanged. Existing browser processes and the
normal navigation profile were preserved; no navigation application was launched.
This establishes same-machine reference fonts, not physical native UI acceptance.

The supplementary actual-package chart collector now preserves the application's
portable-profile guard: it runs a verified disposable copy with its own clean
profile/logs and leaves the audited original package untouched. Pre-dispatch
inspection caught the former external-profile request and wrong diagnostics
path. Twenty focused offline cases and four checks linked to the unchanged
production path guard pass; native launch remains pending. See the
[bounded collector correction](evidence/scrum264-native-recovery-collector/README.md).
Its separate native preflight caught asymmetric canonical path comparison before
CMake or any application launch. The original artifact is retained; the narrow
plain-path-then-canonical correction and real equivalent-path regression now pass
[native run 37130959195](https://github.com/ThereptileII/Work/actions/runs/37130959195):
21 Python cases without skips and four actual MSVC Win32 portable-path checks.
The downloaded original evidence and exact source hashes are retained. This
closes the collector path defect, not application/visual acceptance. No additional
full application build was started.

## Native application passes build; preview sample timing blocks packaging

The exact `1835d1b` / `9d98a500` native job has now completed with a retained
[fixture-suite failure](evidence/scrum270-native-9d98-preview-failure/README.md).
The application build, all 139 native CTests, pointer route gestures, repeated
crash recovery, 100/125/150% DPI/touch and public ENC checks passed. Independent
renderer inspection confirms software and permitted fallback only: the hosted
machine rejected OpenGL, so this result does not qualify native hardware GL.
The [renderer receipt](evidence/scrum270-native-9d98-preview-failure/renderer-proof.json)
retains the exact report identity and observations. Independent
artifact inspection confirms the fresh five-export private DLL, its complete
source/resource identity and identical private/host presentation resources.
The actual GDI probe selected Segoe UI for both the UI and ordinary chart text;
the hosted machine lacks Segoe UI Variable Display and uses the prototype's
declared fallback. These are native checks, not boat visual acceptance.

At simulated second 115, the preview test accepted a decreased-SOC snapshot
without checking route validity. The product correctly reported
`ActivePointChanged` / unavailable and withheld both remaining distance and
arrival SOC; the test then indexed the absent distance. **SCRUM-270** now has a
[narrow coherent-sample predicate repair](evidence/scrum270-preview-validity/README.md):
the actual failed diagnostic and genuine fixture outputs at seconds 114/115/116
prove the old exception and new valid-sample selection. The twelve-second timeout
and every original comparison remain unchanged. No production navigation behavior
changes. The original failure, screenshot and diagnostic
remain immutable. The fixture-free build, real-host module check and packages
were skipped; this run supplies no installable candidate. Immediate fixture
failure upload is added so a future failure can be inspected while
independent display checks continue. No blind full rerun or boat installation
has occurred. The current Linux elapsed-time gate continues separately.

## Preceding combined candidate (native fixture failure retained above)

Local source/evidence **`1835d1b84df89aff42220ac8bb535e4262034a54`** is published
as **`9d98a500916e8a7f59dac9735427dde6d3c7d2e5`**. All 4,649 mapped blob/mode
entries independently reconstruct `bc9e631e57c71992c573f5b5110b497096b23867`;
the eight unrelated root files are preserved. The
[full Windows/Linux run](https://github.com/ThereptileII/Work/actions/runs/37120549213)
started only after the native resource repair and the actual SKAGER software/GL
yellow-symbol checks below passed. The [publication receipt](evidence/skager-symbols-1835-publication.json)
keeps exact identities. Native runtime, fresh five-export private package and
boat acceptance remain pending. No product change will be mixed into this run.

One read-only boat refresh at 11:44 UTC found no meaningful change: no navigation,
helper or active commissioning process; stock/installed/profile/recovery identities
and all 117 qualified tools match. Remote access remains healthy. No application
has been installed, launched or retired on the boat in this cycle.

Separate review tooling is being prepared against that unchanged application:
the [audited-package collector](evidence/scrum264-native-recovery-collector/README.md)
can capture the existing Windows payload with public ENC and the exact official
IHO test cell, without another application build. Its dedicated workflow refuses
to run until a produced artifact has been independently audited and its exact
identities recorded. Original licensed test data is excluded from uploads.
The [guarded palette review](evidence/scrum269-guarded-palette/README.md)
(SCRUM-269) now follows the actual Layers interface and binds one XNav/Standard
choice to the normal restart broker. The separate [native tooling run](https://github.com/ThereptileII/Work/actions/runs/37124439890)
**passed** on exact `70537545011ef3fd2ecb288a404ae3dc94d5d019`, attempt 1:
14 existing mode HWND cases, 16 palette/reveal HWND cases with normal fixture
exits, 13 actual broker cases and five Prepare/Arm cases. The downloaded
[three-gate/bundle receipt](evidence/scrum269-guarded-palette/native-corrected-run.json)
independently binds all 117 tested Windows operator bytes. This is tooling-only
proof, with harmless windows and marker/helper fixtures; actual application
palette selection, native visuals and boat recovery remain pending.

After separate authorization, the existing qualified `ffa2de31` staging operator
placed those exact 117 files in a new owned versioned directory. Independent
[staging and preservation checks](evidence/scrum269-guarded-palette/boat-staged-only.json)
matched every file and completion record; stock, installed ownership/executable,
profile/state, cold/recovery records, old tools and remote-access health were
unchanged between 13:09 and 13:12 UTC. No navigation/helper/commissioning process,
running product task or active commissioning marker was observed. An initial
read-only stdin wrapper timed out; a subsequent invocation omitted the existing
per-process execution-policy flag and refused the old helper import before any
write. Both are retained; corrected transport/staging followed. No new operator,
application or installer ran, no persistent policy changed, and no physical
output was issued. The frozen application and all launch/recovery guards remain
separate from this tooling PASS and staged-only result.

## Windows resource repair verified; combined candidate awaiting full gates

The exact d5/b8 [full candidate](https://github.com/ThereptileII/Work/actions/runs/37114216075)
failed native host configuration at 10:58 UTC. OpenSSL passed, zlib passed 13/13,
curl passed 1,569/1,569 and the private chart DLL linked successfully. The next
host configure rejected **“Adapter chart resource manifest differs”**. The
failure is tracked in SCRUM-259; no application crash or linker failure is
inferred. The [downloaded failure audit](evidence/scrum259-full-b8cf-failure/README.md)
verifies the linked four-export private package and all 5,812 artifact CRCs.
Its host generated resources were not retained. The build used Python 3.12.10
for initial generation but CMake selected 3.14.7 for the host; those Windows
versions use different PNG compression implementations. The
[entry-interpreter repair](evidence/scrum259-resource-python-repair/README.md)
pins both paths and preserves strict byte checks. Its
[focused native proof](evidence/scrum259-resource-python-native/README.md)
passed on remote `c7f3616c5b9d385d39edf25b51f89a3508906071`, mapped exactly
from isolated `eced8da`: the original interpreter difference is reproduced,
decoded pixels remain identical, and both explicitly pinned configurations
produce all seven files byte-for-byte identically. The combined replacement
run above began after that focused proof. No candidate from this
cycle has been installed or launched on the boat; the existing Linux elapsed-time
gate continues independently.

## Isolated symbol and private-observation follow-ups

The supplied generic beacon is implemented in `85ea050` (isolated source
`2b90485`), with [exact prototype/resource evidence](evidence/scrum264-generic-beacon/README.md):
2,118 focused checks, two original Simplified generic selections, preserved
classified/Paper consumers and exact Day/Dusk/Night comparisons. The yellow
body and explicitly fitted X from `5ad1c94` are integrated as `559d161`, with
[classification and source proof](evidence/scrum264-yellow-special/README.md).
Combined application source **`33d90c3`** passes the
[integrated Linux build and 147 regressions](evidence/scrum264-combined-33d-linux/README.md).
The separate [combined resource batch](evidence/scrum264-combined-resource-33d/README.md)
passes 79,177 resource checks and 17 private preparation tests. These suites are
distinct from the 147 integrated cases. The
[17-tile actual-loader check](evidence/scrum264-combined-loader-33d90c3/README.md)
passes 46,918 checks across all three themes. Its methods match the separately
tested seven negative controls, which were not redundantly repeated. These are not Windows or boat visual
acceptance; the documented positive IHO objects are official test geography.
Actual software canvas validation then caught a missing yellow X: the real
loader represents an empty instruction as U+001F, whereas the new topmark guard
expected a zero-length string. The failed capture and runtime values are
retained in the integrated evidence. The narrow canonical no-op repair in
**`e1d0136`** passes [both actual pinned parsers](evidence/scrum264-yellow-empty-instruction/README.md)
(150 checks and six mutation refusals). Its affected-target build/install and
[actual yellow body/fitted-head software and OpenGL captures](evidence/scrum264-yellow-e1d-linux/README.md)
pass all SKAGER themes and exact whole-chart Day return. Standard software also
passes. The 147-case batch and complete resource suite were not repeated.

The same review found an inherited Standard OpenGL light-label shift. One
frozen-33 control reproduces all 503 changed pixels; corresponding entire chart
regions match e1 exactly in every theme, without masking. Its failed assertion
is retained, not reported as a pass. **SCRUM-268** tracks the original text-cache
behavior and pending native applicability. No Standard renderer correction was
introduced in this candidate. SCRUM-264 enters Testing; native private-renderer
and boat recognition/readability remain required.

`a6b2d01` (isolated source `f5fbd3a`) closes the private diagnostic-observation gap:
[copied actual private table state](evidence/scrum267-private-diagnostics/README.md)
is lifetime/thread gated and separate from core state. Focused contract, package,
PE refusal and affected Linux-object checks pass; SCRUM-267 is Testing pending
native/boat acceptance. The [incremental lifecycle source review](design/reviews/scrum259-ocharts-e1-lifecycle-source-review.md)
verifies all 219 locked inputs and the four new observation-only activity calls;
a fresh five-export package/runtime receipt is required. These follow-ups are
**not** in the frozen d5/b8 candidate
below. Its running results must not be attributed to the newer source.

## Full candidate after focused Windows repair

Frozen application **`d5d71356d806ea8c3518644d10728a24f1334d1d`** is published
exactly as **`b8cfbf809450208f723095ffb4e00d7800b619a5`**, mapped tree
`111fac745157fd70cc83be81d09b0f3a6da7ecd5` (4,249 verified blob/mode entries).
The [full native/Linux candidate](https://github.com/ThereptileII/Work/actions/runs/37114216075)
started only after both [audited native compilation checks](evidence/skager-final-font-native-9dee9b1/README.md)
passed: 70 private-renderer and 23 core chart objects. Those checks apply to
the preceding 9dee source; the final two-line [Land label face correction](evidence/scrum263-geographic-face/README.md)
has separate actual-resolver/object proof and will be qualified by the full run.
The noncritical face-choice adjustment did not cause another 93-object preflight.

The prototype explicitly uses Segoe UI for Land annotations while Water names
inherit the main stack. Both core and private renderers now preserve that
distinction, with no size, tracking, opacity or resource changes. The preceding
[9dee integrated Linux evidence](evidence/scrum263-chart-face-9dee-linux/README.md)
passes 147/147 tests and four exact chart comparisons; the initial isolated
Wayland/Xvfb launcher failure and corrected font-probe retry remain recorded.
The [final d5 integrated Linux build, font probe and two original Day captures](evidence/scrum263-land-face-d5d-linux/README.md)
pass. Both full chart and identity-panel comparisons are exactly equal to 9dee;
the 147-test suite was deliberately not repeated for the two-line face choice.
The native run then reached private DLL linking but failed the host resource
identity check described above. Host runtime, packaging, actual font resolution
and boat rendering remain pending. No candidate has
been installed or launched on the boat in this cycle.

The [same-run native restart receipt](evidence/final-d5-restart-qualified/README.md)
has been downloaded and independently verified: all four gates passed, exact
candidate/run/attempt identities match, and `actualBoat:false` is preserved.
The [read-only boat refresh](evidence/boat-readiness-final-d5-20261003.md)
verifies the unchanged real installation/profile, complete recovery hashes and
117 qualified staged tools. Retain that exact tool bundle for the later review;
neither receipt is an application-package or boat-rendering acceptance.

## Final chart typeface correction under qualification

Application source **`9dee9b148f4d6ebdd20bb4c49229fe19340df209`** is published
as **`aa95750b0abbd9ffa646407019b3393f2e7b5bce`** with all 4,174 mapped
blob/mode entries independently verified. The [ordinary chart-font correction](evidence/scrum263-ordinary-chart-face/README.md)
selects installed Segoe UI, then Arial, only for a newly verified SKAGER
presentation library. Ordinary text retains its existing size, weight, style,
content and positioning. Geographic and generated-LIGHTS roles retain their
own handlers and deliberate fallbacks. Font creation failure retains stock;
Standard/Legacy and stored user preferences remain unchanged.

Focused actual-method, production-object and fallback checks pass. The exact
9dee Linux integration and audited native core/private compilation pass. The
existing Windows font component now checks the selected HDC face for ordinary
chart text as well as the UI stack; that result and physical boat review remain
pending. No all-symbol or screen-level visual acceptance is implied.

## Current focused Windows repair and classified buoy candidate

Frozen source **`ccc0faad089e89a5b3a0b1f2994f4fa4ee18053d`**, published as
**`1c3e32d68b2e12892a62e6fe28d61cfaa2377a45`**, adds the reviewed private
CMake path repair and the [classified white/orange pillar derivative](evidence/scrum264-white-orange-pillar/README.md).
All 4,074 mapped blob/mode entries independently reconstruct the remote tree.
The [short native gate](https://github.com/ThereptileII/Work/actions/runs/37112484119)
passes the original Windows escape reproduction, normalized paths and all 70
actual private production translation units. The [independent artifact audit](evidence/scrum259-native70-ccc0/README.md)
verifies all actual I386 objects and source/resource identities. It does not
qualify dependency producers, DLL linking, runtime or packaging.
The [integrated Linux build and real-chart review](evidence/skager-chart-ccc0faa-linux/README.md)
pass 147/147 regressions, 16 captures and four clean exits. All Standard and
Day-return chart comparisons are exact; changes remain inside the two buoy bodies.

The buoy alias is restricted to the inspected white/orange horizontal-band
pillar classification, verified SKAGER presentation and Simplified lookup.
Day and Night use the supplied prototype's stem/base geometry with preserved
classification. Dusk retains the original symbol: a tested brighter orange
lost its recognizable hue and was rejected. Unmapped fixed beacons, physical
lighthouse towers and directional/sector lights are not claimed to match.
No candidate from this cycle has been installed or launched on the boat.

## Latest bounded light and hatch correction

Application source **`5c05eb55c15b67d4014df55896d95452e9cc9d64`** combines
[orientation-preserving compact light aliases](evidence/scrum264-oriented-light-aliases/README.md)
and [neutral construction-hatch ink](evidence/scrum265-construction-hatch-ink/README.md).
Original LIGHTS11/12/13 vectors remain entirely stock. Only verified SKAGER
instances and lights without any ORIENT attribute may select the separate compact
red/green/white aliases. Directional data, conditional decisions, original Rule
metadata/lifetime and missing-alias fallback are preserved. The earlier white
bitmap substitution below is superseded; it is not approved for boat deployment.

Only 192 Day construction-pattern pixels change to prototype neutral ink. The
pattern geometry, alpha, dashed shoreline and all twelve original conditional
consumers remain unchanged. Dusk/Night retain their existing transparent tile.
Global CHBRN and other hazard usage are untouched. This corrects the identified
brown Pier57 hatch without painting a submerged ruined pier as ordinary land.

The [combined resource proof](evidence/scrum264265-combined-resource-proof/README.md)
passes 53,377 focused checks and the unchanged whole-resource inverse/negative
oracles. Exact integrated Linux build/install and 147/147 regressions pass.
The [Pier57 hatch review](evidence/skager-chart-5c05eb5-linux/README.md) and
[actual red/green lights](evidence/scrum264-colored-lights-5c05-linux/README.md)
retain 24 original software/OpenGL captures. Standard historical comparisons and
complete Day-return checks pass; light changes stay within the real light
neighborhoods and the hatch change stays within the construction feature.

Exact mapped publication **`159cbeff00d67fbd0b77442b7288407f5d93db2b`** has
3,805 independently verified blob/mode entries, including eight preserved
unrelated root files. The [focused Windows changed-unit preflight](https://github.com/ThereptileII/Work/actions/runs/37109776549)
passes. Its [downloaded artifact audit](evidence/scrum247-native23-159c/README.md)
verifies all 23 actual I386 objects, 447 product inputs, 1,439 patched upstream
inputs and seven generated resources against the frozen source/build. It is
compile qualification, not an application/runtime release gate.
Physical boat fonts/GPU/display and private-renderer acceptance remain pending.
No candidate from this correction has been installed on the boat.

## Preceding combined candidate: SKAGER chart fidelity and branding

The next correction batch includes the exact prototype
[ferry/cable-area ink](evidence/scrum260-area-ink/README.md),
[Day neutral marker ink](design/reviews/scrum261-day-neutral-ink.md), and
[floating chart-control border and complete navigation captions](evidence/scrum14-floating-caption/README.md).
These preserve navigational classifications, symbol geometry, soundings,
Standard resources and control hit areas. The Day mask changes only 40,482
unambiguously owned neutral pixels; off-palette wreck bitmaps remain unchanged.
The [final copy audit](design/reviews/scrum236-final-copy-audit.md) also closes
remaining operator/diagnostic product labels. Customer-facing identity is
SKAGER with the approved Jira artwork; immutable evidence, compatibility IDs
and required OpenCPN attribution remain.

The preceding frozen application source **`9632421f701c5ec74d7a1360bc713c0034faf9af`**
is published exactly as **`d2787c649268809a2d99a72c6ad3e104d504f461`**:
3,306 mapped blobs/modes match and eight unrelated repository files are preserved.
The [integrated Linux build/install](evidence/skager-chart-9632421-linux/README.md)
and **147/147** regressions pass. All **32** real NOAA ENC captures pass across
two scenes, software/Mesa OpenGL, SKAGER/Standard and Day/Dusk/Night/Day-return.
The entire chart returns to identical Day pixels in all eight cycles, including
the GL selector. All sixteen Standard historical chart comparisons remain exact
within their documented toolbar exception. The saved Paper preference stays
unchanged while SKAGER uses the effective Simplified table. The smaller approved
header matches its reviewed component at actual size.

The preceding `a3e8477` / `7a9e549` GL selector failure remains in
[its original evidence](evidence/skager-chart-a3e8477-linux/README.md). The
[cache correction](evidence/scrum266-selector-cache/README.md) now closes that
observed **Linux** reproduction without an added mask or tolerance; physical
GPU and Windows acceptance remain separate.

The preceding [native run 37100591834](https://github.com/ThereptileII/Work/actions/runs/37100591834)
passed all 1,569 executed maintained-curl upstream tests, then stopped at
private-adapter dependency-manifest resolution before application compilation.
It was not an application crash. Explicit producer roots and the complete copied
header closure now pass [fresh native proof](evidence/scrum259-native-prefix-proof/README.md):
17 producer/package cases, 15 preparation cases, 16 wiring refusals and 38 actual
harmless-DLL loader groups. The independently verified artifact belongs to the
same `d2787c6` source. The [full replacement qualification](https://github.com/ThereptileII/Work/actions/runs/37106815245)
passes the maintained dependency producers, then stops at private-renderer CMake
configuration: Windows backslashes in the wxCurl source paths are interpreted
as escapes. The [retained exact failure](evidence/scrum259-private-configure-d2787/README.md)
is before private DLL or host application compilation, not an application crash.
The bounded path repair and direct native configure/object preflight are in
progress before another full candidate. No tested dependency output binaries
were retained in this failure artifact, so they cannot be treated as reusable
verified producer output merely because their logs passed.

Actual private DLL linking/loading, Windows font selection, final
package/installer and boat rendering remain pending. No candidate has been
installed on the boat in this cycle. The later source5c05 above also corrects
the ORIENT boundary identified in this historical candidate. `d2787c6` remains
prerequisite/diagnostic evidence and is not approved for boat deployment.

The user's additional October 3 feedback is represented by SCRUM-263–267.
The [approved header wordmark is now 124 DIP](design/reviews/scrum263-header-typeface.md),
down from 148, with source artwork and Windows icon unchanged. The prototype's
Segoe UI Variable Display / Segoe UI / Arial font stack is already selected by
native UI policy. Canonical Windows browser evidence resolves Segoe UI; an
actual native HDC face probe is now in the Windows drawing gate. Its fresh
Windows result and generic ENC/user-font review remain open. Eleven supplied
[modern marine/light glyphs](evidence/scrum264-seamark-art/README.md) now have
bounded resource/loader proof: eight classified lateral aliases, isolated danger,
safe water and the prototype light circle/rays. An additional
[16 official IHO S-64 chart captures](evidence/scrum264-s64-9632421-linux/README.md)
now verify actual ordinary lateral, isolated-danger, safe-water, four cardinal
and white-light raster selection in software/Mesa OpenGL, with twelve normal
application exits. The ordinary red/green short flares and classified long-range
light treatment remain visibly stock. Preferred-channel actual-chart coverage
and full native/boat recognition remain open. The special-purpose buoy remains
stock because its actual white/orange classification cannot safely be painted
as the prototype's yellow mark. Generic beacons and unproved physical topmark
composition also remain open.

[Structural fill and outline changes](evidence/scrum265-structure-ink/README.md)
replace fourteen specific brown fills and six outlines with prototype land/shore
roles. Shared hazard brown is preserved. This is not a claim that every remaining
brown chart object can safely be recolored. The actual public ENC before-captures
are retained in [the Pier 57 scene record](evidence/scrum264-public-enc-scenes/README.md).

The stock default Paper Chart table would hide the new Simplified artwork.
Both the [core](evidence/scrum267-effective-symbol-style/README.md) and
[private renderer](evidence/scrum267-private-symbol-style/README.md) now use an
effective Simplified table only inside a verified SKAGER presentation instance.
The saved OpenCPN preference remains unchanged for Standard and Legacy. Actual
render/query/cache consumers share the same effective selection; diagnostics
expose saved and effective values. Focused compiled-method/object checks pass.
The combined Linux application now proves the retained-preference round trip and
whole GL theme return. Actual private-renderer execution, Windows fonts and boat
rendering still require fresh qualification.

### Preceding integrated candidate and corrected Windows prerequisite

The replacement source is `dfa7b721ef6eca3f084f77e95dd8e2adc20bde4b`,
published exactly as `c9ff4ae2c110d234807a326bc30c25084eb42a5d` (2,849 mapped
blobs/modes verified). Its [integrated Linux build/install](evidence/scrum259-linux-dfa7b72/README.md)
passes all five original drawing fixtures and **147/147** regressions. The
shared-model callback link correction and the initial isolated test-environment
failure are retained with their passing evidence. The
[corrected native preflight](evidence/scrum259-native23-corrected/README.md)
passes all **23 actual Win32 compilation units** with independently verified
downloaded objects. Its earlier missing-header failure is retained separately.

The [full replacement run 37097634494](https://github.com/ThereptileII/Work/actions/runs/37097634494)
stopped at its Windows private-loader prerequisite, before application
compilation: the preparation test expected deterministic LF output, but inherited
Windows Git settings produced CRLF. Thirteen of fourteen preparation cases
passed; the native loader did not execute, so its expected artifact was absent.
A bounded production preparation correction now passes its
[separate native gate](evidence/scrum259-native-final/README.md), run
37098440931 at exact published `5d6c5cf44cb72f8166b2ab67115bccd568336e35`:
14 preparation cases, 16 build-wiring refusals and 38 actual harmless-DLL
loader groups. The downloaded artifact and source identities were independently
verified. Git patch output now explicitly stays LF regardless of inherited
Windows settings. This does not qualify the private renderer DLL, application,
charts or final package; no application crash is inferred from the original run.
Other Linux/native jobs continue independently. The Windows real-module,
final-icon, installer, chart and boat gates remain open.

The [fresh read-only boat check](evidence/scrum17-readonly-20261003/README.md)
confirms unchanged validated stock OpenCPN, installed Beta 1, profile and recovery
state, with no navigation/helper processes. SSH and Tailscale remain healthy;
RustDesk's service is running, but interactive connectivity was not exercised.
No candidate was installed and no older generation was removed.

**Boat chart presentation gap (SCRUM-259):** the fresh read-only inventory found
581 `.oesu` files in the first configured chart root and two MBTiles files in the
second. Both roots are available; no chart contents, private paths or licence
material were exported, and the profile remained unchanged. The accepted
o-charts source constructs and renders with its own S-52 library from the stock
shared resource directory. Current core `CreateChartPresentation` does not
intercept that construction. Consequently the core NOAA captures below cannot
qualify SKAGER presentation on the boat's o-charts collection. The exact plugin
resource/API boundary has now been inspected under SCRUM-259. A separately named,
exact-source private renderer adapter and fail-closed host boundary are implemented;
the source/package recipe has passed focused preparation checks. Native ABI,
private downloader trust, complete application and boat rendering gates remain
open. The [actual Win32 loader boundary](evidence/scrum259-native-loader/README.md)
passes all 38 native groups at published `4609f31dba3e970802e6ba3fbe3438e129ab9aad`.
The next candidate also includes an explicit early real-host module check,
which blocks child-process creation and never invokes the plugin factory or
initialization. It has not yet passed natively. The installer gate now checks
the actual application's and Setup's PE icon frames and product metadata
against the approved Jira artwork; native final-binary results remain open.
No boat plugin or stock resources have been changed. Raster MBTiles
cannot receive object-level vector styling. [Availability evidence](evidence/boat-chart-types-20261003/summary.json).

The preceding chart-capture application source is `78eccb8b7f21b260ded57d3ba763f884d60c8180`,
published exactly as `61a0a7838b56ad841bb458af6fc62651464bdafe` (2,472 mapped
blobs/modes verified). Its [integrated Linux build and staging pass](evidence/skager-chart-78eccb8-linux-build/README.md).
This combines the prior palette/service artwork, the [four classified cardinal
glyphs](design/reviews/scrum256-cardinal-glyphs.md), and the [actual active-waypoint
name card](design/reviews/scrum257-active-name.md). Active-point symbol/blinking
and navigation semantics remain upstream-owned. SKAGER naming and the approved
Jira logo are wired into the product and Windows resources; compatibility IDs,
original source evidence and required OpenCPN credit are retained.

The [actual native Gettext prerequisite proof now passes](evidence/scrum255-gettext-native-pass/README.md):
25 contracts, successful bounded acquisition, exact-path tool identities and a
real UTF-8 catalog operation. Both earlier focused failures are retained.
The [final seventeen-unit native preflight](evidence/scrum-247-native-chart17-final/README.md)
passes with independently verified downloaded objects and resources. All
[sixteen final ENC comparisons](evidence/skager-product-fidelity-78eccb8-linux/README.md)
pass, and Standard pixels remain identical. The [software and actual-GL route
runs](design/reviews/scrum252-257-final-78eccb8.md) pass all 26 original assertions,
hot themes, active-name cards with preserved icon blinking, stale controls and
clean shutdown. A card-covered GL pixel sample required a documented collector
correction; its original failure and missing-stroke negative controls remain.
No application correction or weakened navigation assertion was involved.

The exact published candidate entered the [full Linux/Windows qualification
run 37088759582](https://github.com/ThereptileII/Work/actions/runs/37088759582).
Windows job 111105527254 built and installed the application, but failed the
offline geographic-name painter at 03:16:18 UTC: water-label pixels exceeded
the required alpha bound. That candidate remains unqualified. The isolated
[native diagnosis and correction](evidence/scrum243-windows-alpha/README.md)
now pass all 2,452 original Windows assertions in run 37094223771 at exact
`7687ce53c780bdfd53db30bb063f9bcc8f6404b7`. GDI+ grayscale coverage restores
translucent text without changing bounds, colors, opacity or any opaque pixels.
The original failed artifact and paired images remain retained. A complete
replacement application/package has not yet been qualified. Linux qualification
continues in the background. No
full release, physical GPU, boat visual pass or obsolete-version retirement is
implied. Whole-chart prototype conformance remains open.

Boat review preparation found a tooling integration gap: the already qualified
cold-baseline recovery checkout predates SKAGER window selectors, while the
application candidate's tools lack that completed-baseline reader. SCRUM-258
[combines those previously reviewed boundaries](installer/skager-boat-tool-composition.md)
without changing the frozen application. Its exact local `3da0e563de40f71fdb5c31df89c9ec65a04826b5`
is published as `ffa2de31ae02ca826ec5d3604f57a8c619035bb1` on the separate
`skager-boat-review-composition` branch: all 2,484 local blobs/modes match and
eight unrelated repository files remain unchanged. Independent source review and
945 focused portable checks pass. All ten jobs in [native tooling run 37090716935](https://github.com/ThereptileII/Work/actions/runs/37090716935)
now pass, with [six original artifacts independently verified](evidence/scrum258-native-tooling/README.md),
including actual dependency refusals and successful Prepare/Arm/Collect. It
built only small test helpers, not another OpenCPN application. At 03:13:19 UTC,
all 117 tools were staged separately and independently rehashed on the boat;
[the staging receipt](evidence/scrum258-native-tooling/boat-staging.json) confirms
unchanged stock executable, profile, installed state and qualified source checkout.
SSH, Tailscale and RustDesk remain running. No new tools were executed and no
application was launched or installed.

## Previous correction batch: chart labels, Night, land and service symbols

Local `f0976cc65ea63d3ed6f60ac53ec38a856d11b66b` is frozen and published as
`9a4231f45d978321c612a6a1de66d735d531bbaa`. Its combined Linux application
[built and staged successfully](evidence/skager-chart-f0976cc-linux-build/README.md).
The [seventeen-unit native preflight](https://github.com/ThereptileII/Work/actions/runs/37085103227)
passed, with downloaded object/source/resource identities independently verified.
It is compilation evidence only, separate from the preceding full candidate.
[Sixteen real-ENC captures](evidence/skager-product-fidelity-f0976cc-linux/README.md)
pass software/OpenGL Day → Dusk → Night → Day, with four clean exits. Exact
Night/land colors, matte-free logo and unchanged Standard chart pixels were
checked. The [active-route comparison](design/reviews/scrum252-integrated-f0976cc.md)
passes all 26 original assertions and hot-theme label checks in OpenGL, but
the first software capture lacks the expected name card, numbered circle and
route understroke. The [diagnosis and repair](design/reviews/scrum252-fixture-repaint.md)
trace this to direct test-scenario mutations missing the redraw notification
used by normal route commands. An ordinary theme repaint on the same executable
shows the correct cards, markers and current geometry. Only the test scenario
was corrected; all 26 navigation assertions remain intact. A fresh capture of
the corrected scenario remains required. No release or whole-chart conformance
is inferred from these captures.

This batch includes theme-aware approved SKAGER artwork, guarded route-point
name cards, effective prototype Night chart colors, the GTK chart-control
recapture fix, exact pilot-boarding/radar-beacon artwork, and the correction of
built-up-area fill to prototype land. It retains navigation classifications,
hazard contrast, Standard resources and Legacy/Safe boundaries. No new boat
installation or old-generation retirement has occurred.

## Previous full candidate: integrated failures retained

The frozen implementation is local
`1356fd1603aacbea04d7081d16331e9a181180bb`, published as
`9fcd3db54ee6913cc144ecfc09ec2152078a6eff`. On October 3 at 00:02 UTC it was
promoted once to the existing `opennav-x-beta2-ui` qualification branch, after
the targeted Windows failures were fixed and verified. Later local evidence and
corrections are separate from this frozen executable.
The exact-source [full run is 37080314681](https://github.com/ThereptileII/Work/actions/runs/37080314681).
Its Linux and native Windows contract jobs pass. The integrated Linux job built
the application and passed all 147 CTest cases, then stopped on the resource
scenario's obsolete `OpenNav` log-prefix expectation. The
[one-line correction and downloaded failure](evidence/scrum235-linux-resource-log/README.md)
retain all resource/profile assertions; a focused same-executable rerun passes
SKAGER, Legacy and Safe with three clean exits. The full job remains failed.
The native job also stopped before application compilation: Chocolatey's Poedit
download returned HTTP 504, which the upstream batch masked until the later
gettext check. [Downloaded failure evidence](evidence/scrum255-gettext-failure/README.md)
is retained. SCRUM-255 now owns early prerequisite verification and bounded
recovery; no blind full-job rerun is started. This is not an observed application
crash. Application/package/installer/rendering/endurance acceptance remains open.

The narrow SCRUM-255 repair checks both Poedit tools before expensive builds,
retries only bounded package-manager failures, and verifies their identities
again before configuration. Its [separate native prerequisite run](https://github.com/ThereptileII/Work/actions/runs/37086572916)
stopped in the offline contracts, before acquisition: the lexical-path guard
mistook canonical Windows path spelling for redirection. That guard is being
corrected to detect actual reparse components; the failed run is retained.
No full application build was started for this failed short proof.

The next local increment includes theme-aware compositing of the approved logo
(`8accdd9`): original asset bytes and letter geometry remain unchanged, the
baked-in rectangular matte is removed, and Night uses the prototype's reduced
brand ink. [Nine focused component drawings](evidence/scrum-236-theme-wordmark/README.md)
cover three themes and 100/125/150% device sizes. They do not qualify native DPI.
SCRUM-252 covers the remaining waypoint-name labels; SCRUM-253 covers the
confirmed difference between raw Night chart tokens and the prototype's
chart-only brightness rule. These changes are not included in run 37080314681.

The combined next increment now includes waypoint name cards (`559a070`),
effective Night surfaces and owned overlay ink (`fcc79aa`), and the confirmed
GTK owner-recapture repair (`1e0a7f1`). Focused source/raster checks pass; the
waypoint cache includes the two review corrections: failed GL replacement
keeps stock ownership intact, and transparent texture padding does not enlarge
the label's hit/cull bounds. The combined `6dd6665` Linux application linked
successfully in 98 incremental steps. This is build evidence, not visual or
Windows acceptance. Night hazard ink, safety contours and soundings deliberately remain
brighter to preserve the measured contrast gates. The [symbol source audit](design/reviews/scrum15-symbol-source-audit.md)
separates exact artwork candidates from decorative examples that would change
charted meaning. SCRUM-254 owns the narrow pilot-boarding/radar-beacon artwork
increment, now integrated as `585fd0f`: both exact prototype assets pass the
pinned native loader and resource-isolation checks. The built-up-area fill is
also being corrected from the earlier shore-color policy to the prototype's
land role. A final combined capture build and native preflight follow these
bounded changes. No newer combined executable is qualified yet.

- [Native sixteen-unit preflight](evidence/scrum-247-native-chart16/README.md)
  passed MSVC Win32, including the actual GL canvas and sounding renderer.
  Downloaded objects, source identity and archive integrity were independently
  verified. This is compilation evidence, not Windows product acceptance.
- The frozen Linux integrated build linked successfully after 104 build steps.
  Sixteen real public ENC captures cover SKAGER and Standard in software and
  Mesa OpenGL, each through Day → Dusk → Night → Day, with four clean exits.
  The strict repeated-edge check now passes; the older failing pictures remain.
  [Captures and independent review](design/reviews/skager-chart-1356fd1-linux.md)
  retain their precise identities and limitations. Two actual upstream route
  scenarios also pass 26 checks each, exact projected
  route-paint samples, stale handling and clean exit. These do not qualify boat
  GPU, Windows fonts or every route/AIS visual state.
  The stale-position GL route image omits floating chart controls visible in
  its software counterpart. SCRUM-228 traced this to the delayed owner recapture
  and now has a [focused repair](design/reviews/scrum228-recapture-repair.md),
  including an actual native stacking negative control. The newer integrated
  application still needs to demonstrate the correction.
- The [fresh read-only boat audit](evidence/boat-readiness-20261003.md) found
  remote access running, no application processes and matching stock/profile/
  recovery hashes. The connected display reports 1920×1080 with configured
  150% scaling; actual application DPI and the requested 1280×800 comparison
  remain unmeasured. The accepted Beta1 installation remains unchanged.

The candidate includes customer-facing SKAGER naming, the approved Jira logo
and Windows icon derivatives, chart/name/light/sounding typography, bounded
route/waypoint/healthy-AIS presentation, the chart framebuffer repair and the
scoped submarine-cable color. **Visual conformance is not accepted:** harbor
label density, small light-description readability, several special-state
symbols and the newer wordmark's integrated rendering still require review or
correction. Standard fallback and navigation meaning remain mandatory.

Full native application, fixture-free packaging, installer/recovery, DPI,
physical display and boat acceptance remain open. No replacement is installed,
no older generation is removed and no physical command is sent by this batch.
The previous candidate's [Linux job and three-hour endurance passed](evidence/scrum-224-linux-baseline-4c597955/README.md)
with both 147-case suites, 1,080 samples and 90 dropout recoveries. This older
source's pass cannot qualify the frozen candidate or these newer changes.

## Previous: SKAGER identity and prototype chart refinements

The latest user instruction requires the approved SCRUM-89 SKAGER artwork and
customer-facing name throughout the product. The combined development branch
now includes native captions/logo (SCRUM-235), Windows application/setup/maintenance
icons and SKAGER distribution/guide names (SCRUM-236), fractional default-route
foreground (SCRUM-237), and geographic chart-name typography/ink (SCRUM-238).
The integrated source is local `1a02ae083501eb05feeef7879e6ee450c076be69`,
published equivalent `280d5e2e28570fed4f2b84eaafc104bad9ee7c1f`. It also includes
the bounded healthy COG predictor (SCRUM-239). Its integrated Linux build and
six real-ENC software captures passed with two clean exits; see the
[visual review and exact evidence](design/reviews/skager-chart-integrated-1a02ae0.md).
Further focused integration now includes onboard AIS bodies (SCRUM-240), the
bounded route understroke (SCRUM-241), verified default waypoint markers
(SCRUM-242), geographic tracking/opacity (SCRUM-243), and the effective
prototype anchorage glyph (SCRUM-244). All nine combined upstream patches apply
to the pinned source. The first combined source `e14de49f444c56de28465416a83337838ad84413`
passed its Linux integrated link and twelve real-ENC software/Mesa-GL captures,
with four clean exits. The [comparison record](design/reviews/skager-chart-e14de49-linux.md)
retains exact identities and two initial collector failures. SCRUM-243's
text-overlap/DPI-bound correction is included. Visual inspection nevertheless
found a theme-transition OpenGL edge-wrap defect in both SKAGER and Standard;
SCRUM-250 is repairing the confirmed framebuffer/viewport mismatch. These
images therefore do not pass the OpenGL visual gate. They contain no active
route or AIS targets and do not qualify those painters. Later integrated
source includes light descriptions (248), sounding typography (249), and the
selector's deferred-fallback correction (246). Native and boat qualification
remain open; older Windows evidence does not qualify these newer changes.
Internal configuration/protocol identities,
existing user data, immutable prototype and required OpenCPN attribution remain
intact. See [native identity](architecture/skager-native-branding.md),
[packaging identity](installer/skager-branding.md),
[route foreground](design/reviews/scrum237-route-foreground.md) and
[geographic names](design/reviews/scrum238-geographic-names.md).

The preceding candidate `4c597955f96647a9c7b837139aa2732f2c3a3fa5` is in
[run 37063131823](https://github.com/ThereptileII/Work/actions/runs/37063131823).
At 22:12 UTC on October 2, integrated Windows job 111026476044 failed to link
`floating_surface_test.exe`: its console target expected `_main`, whereas
`wxIMPLEMENT_APP` supplied the Windows GUI entry point. This is an automated
fixture link failure, not an observed SKAGER crash. SCRUM-245 isolates the
entry-point repair and native lifecycle proof, including a required reproduction
of the original linker error, before another full candidate. No behavioral
assertion is removed. Failure artifact 11254867923 has SHA-256
`82e07a88d0e822a0556fcaa4eebc39ad7a33ca8e4d2a7491a9531097b077861b`.
Linux contracts passed 92/92 and Windows 89/89; 13 other jobs passed. The latest
22:25 UTC observation still has Linux endurance running, not accepted as a
three-hour pass. Windows UI, fixture-free packaging and installer steps after
the failed link were skipped and cannot be claimed passed.
The isolated SCRUM-245 correction then passed native MSVC Win32 in
[run 37073219984](https://github.com/ThereptileII/Work/actions/runs/37073219984),
commit `e67f70e7dafc31b6da9d34d5eab8690a9eab09c8`: the old entry point reproduced
the exact linker errors, and the fixed executable passed all 12 unchanged
Windows lifecycle checks. The downloaded artifact's size/hash/CRC, 130 source
inputs, executable and 11 runtime DLLs were independently verified; see
[the retained receipt](evidence/scrum-245-floating-entry/native-37073219984/verification.json).
This closes that fixture defect, not SCRUM-224 or full product qualification.
Long-running qualification continues in the background while these isolated
visual changes receive focused checks. No redundant full candidate was started
for the individual branding or paint changes.

Prototype conformance remains open. The e14de49 captures show the corrected
COG endpoint, geographic tracking and two vector-selector colors; labels and
several chart-object states still differ. Route/waypoint/onboard-AIS appearance,
native Windows DPI, actual boat GPU and boat-display comparisons remain gates.
The bounded implementations retain explicit special-state and geometry fallbacks.

The first focused native chart preflight [37076177181](https://github.com/ThereptileII/Work/actions/runs/37076177181)
stopped before compilation: Git's repeated-section mode handling rejected
`chcanv.cpp` under Windows file-mode configuration. The exact nine-patch failure
was reproduced; private pinned-index application and independent final-tree
verification fix it without changing the patch contents or tamper guards.
Combined local `19c900afdfca4c3272715c0f998b8941ec45c8df`, published equivalent
`6f22f688cc1156c8c1e2395c854fff1a38398338`, includes all nine verified patches and
15 actual chart production units. The corrected [native preflight37077779848](https://github.com/ThereptileII/Work/actions/runs/37077779848)
failed on the preflight helper's missing `OCPN_Sound.h` include path. The complete
production include set and actual generated sound configuration corrected that
helper; [failure evidence](evidence/scrum-247-chart-preflight/native-37077779848/receipt.json)
is retained. The subsequent sixteen-unit preflight above passes. This short
compile gate cannot substitute for a full application,
fixture-free package, installer, Windows UI or boat acceptance.
No physical
actuator commands or boat-install retirement have been performed by this batch.

## Previous: chart refinements and corrected candidate preparation

The previous stale-snapshot and touch failures below are retained historical
evidence. The corrected source now has two verified development gates:

- [Exact Linux preview](evidence/scrum-224-cf07197-linux-preview/coherence-identity.json)
  at local `cf0719749482e62d48daed02515f2b6c6d242b67`, published equivalent
  `e0e0e9ef71714932ab69d533d1f86cbcd48e33b3`: the 129-check freshness regression
  and one full preview pass. The preview took 213 seconds, retained 44 captures,
  exercised 60 pointer actions and all eight scenarios, and observed seven
  clean application exits through XNav, Legacy, Safe and direct Legacy return.
  Real coastline remains visible in the reviewed final return. This run used
  process/signal tracing; it does not retrospectively identify the older747
  SIGKILL cause. The interrupted build wrapper, brief overlapping Ninja and
  explicit object/cache recovery are recorded, not hidden.
- [Downloaded native touch proof](evidence/scrum-229-cf44-native-touch/verification.json)
  for `cf44e938a3539af6e97a03346542f04f3e7895a4` passes actual 125% Windows
  input. The original Edit target gives zero scroll. Three native body-hit pans
  move 480 px, leave Save and the lower action fully visible, and an exact tap
  activates the action once without saving or changing fields. The initial
  test's unsupported forwarded-message-count assumption was removed; native
  hit, movement, settled geometry and state assertions remain strict. Artifact
  hashes, 273 recorded input entries, 29 objects, 12 runtime DLLs and both test
  executables were verified. This is a component proof, not full-app DPI or
  physical touchscreen acceptance.

After those checks, the exact `cf44e938` source was promoted once to
[full run37057756273](https://github.com/ThereptileII/Work/actions/runs/37057756273).
Its Linux and Windows portable contract jobs found a missing shared-helper
binding in the older geometry test (91/92 and 88/89 CTest entries passed).
This is a test-harness `NameError`, not an application crash. The downstream
Windows integrated job was skipped; running upstream/Linux jobs remain intact.
The two-file repair at local `3b944213` / published `16187f407754a3dbe137d03311518e27445b03d7`
passes all 24 retained cases locally and in [native short run37058880373](https://github.com/ThereptileII/Work/actions/runs/37058880373)
under Windows cp1252. No product or behavioral assertion was removed.
Chart refinements are being batched separately before another full candidate.
No replacement package or boat deployment is yet qualified.

## SCRUM-15 / SCRUM-231 / SCRUM-232 — user chart feedback confirmed

The user's screenshot concern is valid: the active XNav chart presentation
still differs visibly from the immutable prototype. Functional chart tests do
not qualify chart appearance. The retained FFE ENC view uses XNav resources,
but its large mustard built-up areas retain stock CHBRN. Inspection of the
hash-pinned public US5SEAFL cell identifies the Seattle and West Seattle
BUAARE polygons. SCRUM-231 changes only their fill through a dedicated XNav
paint role using the prototype shore neutral. CHBRN also colours hazards and
light sectors and remains intact for those uses. [Matched local ENC evidence](evidence/scrum-231-built-area-local.json)
now confirms the correction in all three themes and pixel-identical Standard
chart regions. The strengthened resource guard passes 3,484 checks and leaves
the captured generated resources byte-identical. Native Windows, OpenGL and
boat acceptance remain open.

SCRUM-232 separately tracks the stock red default ownship versus the prototype
chevron, preserving OpenCPN position, heading, accuracy, user-icon and scaled
ship semantics. Heavy labels and chart-symbol/route/waypoint/AIS styling remain
open. Some illustrative prototype danger symbols omit required chart meaning;
they cannot be copied indiscriminately. No primary chart view is visually
accepted. The chart corrections are isolated from the frozen stability
candidate. The combined refinement source now includes:

- Dedicated built-area fill, with matched Day/Dusk/Night real-ENC captures
  and byte-identical Standard chart regions (SCRUM-231).
- Shared prototype ownship chevron at the existing OpenCPN position/rotation,
  retaining stock custom, scaled and low-accuracy paths. The three changed
  production units compile; 488 focused painter checks pass, including the
  Windows max-macro hazard and shared software/GL DPI factor (SCRUM-232).
- Available owned online AIS names, exact prototype baseline and theme ink,
  bounded placement and stale/lost label removal. 58 core checks and the updated
  26-check painter pass; the production overlay compiles (SCRUM-233). Independent
  review found OpenCPN's 500-pixel text-measurement clamp. The correction omits
  saturated metrics; its regression exercises a real 1,024-pixel name reported
  as 500 pixels. Exactly-500-pixel labels are conservatively omitted too.

The combined local `25297e3` / published `6e0117de` source completed one
136-step integrated Linux link and a short real-ENC Day/Dusk/Night run with a
clean exit. [Retained identity and captures](evidence/scrum-15-25297-integrated-chart/identity.json)
bind the exact executable and public chart. Root review confirms the neutral
built areas and east-facing prototype chevron. Controlled loopback input was
fresh; it is not real boat-data evidence. These captures precede the label
saturation repair and contain no online targets, so cannot qualify that repair.

The same comparison still shows prominent text, dense soundings, stock hazard
symbols and a red upstream course predictor. This is not prototype conformance.
The [remaining presentation review](design/reviews/scrum15-chart-comparison-20261002.md)
records the differences and source boundaries. SCRUM-234 now corrects a
separately confirmed GL-only online AIS stern-notch defect by changing only
the polygon's cyclic starting vertex. The original fails two topology checks;
the correction passes all three, including 1,326 interior comparisons and
unchanged software path geometry. Its production object compiles. This is
source/topology proof, not an actual OpenGL capture.
Windows, actual GL, physical boat review and remaining chart-symbol/label/route
refinements are still open. No replacement package is qualified yet.

## SCRUM-224 / SCRUM-228 / SCRUM-230 — clean GTK fix verified locally; stale snapshot blocks preview

The coherent Linux build at `a3edcaef2904bf1450e306f3e086164479ec7f6f`
contains the approved GTK floating-surface fix without temporary diagnostics.
Its short viewport check passed actual zoom/palette button activation, stable
chart center, correct painted positions and coastline/layout checks.
[Exact build and failure evidence](evidence/scrum-224-a3edcae-local-preview.json)
binds executable SHA-256
`cb195d7356f5891aa2c4831b64f3bc8f33f0fe9e4fc4670c696200858b404985`.
The prior747 and671 binaries and evidence remain preserved.

The one traced full preview then failed the existing stale-data assertion.
At5038ms, the exported snapshot marks samples stale and omits route distance,
but still includes an aging arrival-SOC estimate. The retained
[UI screenshot](evidence/scrum-230-a3edcae-stale-ui.png) already shows unavailable
values and “Battery SOC stale”; this is an inconsistent snapshot observation,
not proof that stale advice remains visible. SCRUM-230 is In Progress. No
assertion was relaxed and no rerun was performed.

Lifecycle checks were not reached. The original747 missing-window/SIGKILL
uncertainty remains open; its observed descendant was not proven to be the
restart child. Process tracing can alter timing and grants no waiver. This is
not a full preview, native Windows or boat pass, and no new full candidate was
dispatched by this local verification.

## SCRUM-224 / SCRUM-229 — ffe native build passes; preview and 125% touch gates fail

The frozen [ffe run](https://github.com/ThereptileII/Work/actions/runs/37041103887)
has completed its native job. [Downloaded terminal evidence](evidence/scrum-224-ffe-native-final.json)
verifies artifact11246267901 (43,226,505 bytes, SHA-256
`50ab9ff8307f85b792c9f1ee8ddb5fe771cb96c23527b1402af5c1e1459d853e`).
Native compilation and138/138 integrated tests passed, as did installed peer
and loader checks, pointer navigation/route gestures, three recovery sequences
and the public ENC/plugin gate. The requested OpenGL phase actually used
software fallback; this is not actual GPU/OpenGL acceptance.

The preview stops at the obsolete Settings → Display → Chart presentation
path already corrected in the prepared747 helper. The native125% DPI run
separately fails to reach the lower Vessel Preferences action by touch. Its
unverified pan start falls on the editable battery-capacity field; SCRUM-229
tracks a strictly checked scroll-body gesture and native replay. The100%
sequence completed,125% remains failed,150% was not reached. Fixture-free
product, installer, package-security, native endurance and boat gates were not
qualified; no replacement full candidate or boat deployment has started.

## SCRUM-224 / SCRUM-228 — local preview repair verified; two bounded failures retained

The coherent Linux build at `74761400862bc1d25102b2b741d14247dd343cdc`
verifies the corrected visible Diagnostics selection, all eight data scenarios,
60 pointer actions and XNav → Legacy with coastline content.
[Retained preview evidence](evidence/scrum-224-747-local-preview.json) records
the subsequent Legacy → XNav failure: an OpenCPN-named descendant received
SIGKILL before the expected replacement window/startup log appeared. Its empty
observed argv does not establish which forked child it was. The sender/cause is under
investigation; this is not a full preview or lifecycle pass. An earlier
interrupted run remains separate. No new full candidate was dispatched.

A separate three-input check of that same executable confirms an initial
Linux floating-chart-control visibility/geometry inconsistency after startup
resize. The chart receives the zoom-labelled inputs while controls are not
painted; palette input still works. [Evidence and images](evidence/scrum-228-747-local-viewport.json)
retain the actual input trace and unchanged scale. SCRUM-228 tracks the fix.
This does not explain the older c95 scale drift or establish a Windows failure.
The frozen ffe native build continues; package/boat promotion remains blocked
by its Linux preview failure. Boat files, profiles and remote access are unchanged.

## SCRUM-224 / SCRUM-14 / SCRUM-226 — combined focused Windows proof passes

Published `ffe4ecf11f43a916098fefe8f17071c63867a838`
(local `f220b31391815663c442d1f8dcbe98a54c03333c`) passed
[native run37039246758](https://github.com/ThereptileII/Work/actions/runs/37039246758):
**469 component checks** (Settings250, Search85, Chart46, Energy88). The strict
Layers and drawer geometry checks and runtime staging guards also passed.
[Downloaded proof](evidence/scrum-224-ffe-focused-native/verification.json)
verifies exact artifact11241214936,259 unique source/patch/lock inputs,
31 compiled objects,12 runtime files,four executables and30 canonical captures.
The immutable Search reference job was intentionally skipped.

Native review confirms the System intro/row order, truthful disabled states and
visible recovery/Advanced links. Title/subtitle ink remains2px/4px above the
canonical reference, and row font metrics remain unmatched; this is partial
System refinement, not whole-view visual acceptance. Search/Chart retain their
previous appearance (only Shell clock pixels differ). The Energy header repair
passes actual pointer-close/reentry after scrolling and resizing; both retained
header screenshots show the action in its correct upper-right position.

After the downloaded focused evidence passed, the exact same `ffe4ecf` commit
was promoted once to [full qualification run37041103887](https://github.com/ThereptileII/Work/actions/runs/37041103887).
It is frozen. The prior88 Linux layout-oracle failure remains retained below;
its native job subsequently failed the same obsolete Layers oracle after successful compilation and138 tests. Waiting for that already-disqualified
candidate's terminal native result is no longer a prerequisite to starting the
corrected candidate; no run was cancelled or restarted. No package or boat
promotion follows from the component-only pass. Exact full-shell DPI,
dependency/security, installer, recovery and physical-display gates remain open.

The exact ffe Linux job then failed in the preview interaction check after the
repaired Layers check, navigation/input, recording, recovery and user-flow
checks passed. [Downloaded failure evidence](evidence/scrum-224-ffe-preview-selector-failure.json)
shows a hidden previous Diagnostics control counted alongside the visible
Preferences action. No crash was reported. A focused selector repair is being
verified before any new candidate; the native job continues unchanged. This
candidate is not eligible for boat deployment or package-security probing.

The older c95 Linux job completed:146/146 fixture-integrated tests and the same
146/146 fixture-free product tests passed. Its downloaded
[three-hour evidence](evidence/scrum-224-c95-linux-final.json) contains1080 samples,
zero sustained handle/thread growth, resident-memory median change−106496 bytes,
and2.58% of one CPU core. The final soak image is all water after substantial
viewport scale/longitude drift; it is explicitly **not chart-content acceptance**.
Numeric endurance does not qualify ffe, native Windows, or the boat display.
Boat/profile/remote access and public payment/download access were unchanged.

## SCRUM-227 — disposable AIS outage boundary verified

The separate [native run37047212685](https://github.com/ThereptileII/Work/actions/runs/37047212685)
passes on exact source `97142aa1a99ed789380274f20ae7a5c5349d18aa`.
[Downloaded evidence](evidence/scrum-227-disposable-native-proof.json) verifies
all four source inputs,three Win32 executables and both IPv4/IPv6 phases.
Existing marker connections were interrupted, retries were blocked and traffic
recovered after both normal helper exit and forced termination. Independent
control traffic stayed connected without errors. Fresh independent queries
confirmed the uniquely owned filters and sublayer were removed in both phases.

This helper can target only inert loopback marker executables in a disposable
hosted runner. It does not target OpenCPN,AISStream or boat traffic. The initial
RPC type compile failure and strict readback refusal were retained; the latter
was documented Windows INDEXED metadata, now explicitly requested and compared
exactly without masking unknown flags. Actual-product binding and live
AIS loss/aging/reconnect remain open;SCRUM-227 is not Done. Product candidate,
boat installation, remote access and hardware-control policy are unchanged.

## SCRUM-224 / SCRUM-213 — native focus repair proven; integrated layout oracle correction required

Published source `88c141ba9fee5c28e12b5f4ac0f7e7550d1a8ca6`
(local equivalent `688bb713db885dbe78ca6db861382ce248c231f7`) passed the
[focused native run](https://github.com/ThereptileII/Work/actions/runs/37032588707):
230 Settings + 85 Search/Shell + 46 Chart checks, plus the corrected strict
large-desktop geometry oracle. Downloaded artifact `11238311244` was verified
against its SHA-256, 247 source inputs, 31 objects, 12 runtime files, three
executables and 23 canonical captures. Combined Linux Settings passed 181
checks with 14 captures; this does not replace integrated Linux qualification.

The retained `60929a5` negative control reproduces the actual failure:
restoring focus to Advanced battery model scrolls Sensors out of its pointer
target. Explicit Preferences reopening now focuses its current visible tab
before resetting scroll. The unchanged native pointer checks and before/after
trace prove stable geometry and real Sensors selection. The independent
1920-width failure was an obsolete test expectation: the immutable prototype
and native product require 460 DIP at the large-desktop breakpoint. No tolerance
was widened and no release gate was removed.

[Verified native evidence](evidence/scrum-224-88c-focused-native-proof.json) and
[scoped visual review](design/reviews/native88-focused-review.md) cover the
Search row/focus outline, shared Windows title/border, Chart supporting text
and System flow corrections. Original HTML is unchanged. Whole-view conformance,
other DPI/Back-header states and boat acceptance remain open.

After this evidence passed, the **same commit** was promoted to
[full Linux/Windows qualification run 37033702537](https://github.com/ThereptileII/Work/actions/runs/37033702537).
The candidate remains frozen. Its integrated Linux job failed before navigation-object interactions because the strict chart-layout oracle still listed the six pre-Layers floating controls. The actual new44×44 Layers button matches the immutable prototype. [Verified failure evidence](evidence/scrum-224-88c-linux-layout-failure.json) retains the real screenshot and layout. The isolated correction requires the exact seventh control and preserves one-pixel geometry bounds and arbitrary-overlay rejection; captured negative/positive replay plus nine targeted rejections pass. No application geometry changed. Native Windows continues independently; **this run is not eligible for boat deployment or security-package probing**. Full product/dependency, installer/recovery, native DPI and boat gates remain unaccepted.

The same commit's [native recovery subset](evidence/scrum-17-88c-recovery-tooling.json) has21 passing maintenance reports,384 marker-process checks,51 actual broker checks and56 Prepare/Arm/Collect checks, with downloaded bytes/source identities verified. These disposable tests do not imply real-profile or application acceptance.
The older c95d failed native evidence remains preserved; its already-running
Linux endurance continues independently. The boat installation and remote
access were not modified; older-install retirement remains gated by a known-good
replacement. No public payment/download access was opened.

## SCRUM-224 — c95d native UI gates fail; short prototype proof passes

The frozen `c95d3a0` [full run](https://github.com/ThereptileII/Work/actions/runs/37017351644)
passed native compilation and all 138 integrated tests, installed peer CLI,
staged loader, pointer route gestures and repeated crash recovery. It then
failed the Sensors Preferences pointer check and the 1920-wide Preferences
width assertion. [Verified failure evidence](evidence/scrum-224-c95-native-ui-failure.json)
records both artifacts and exact scope. The latter oracle expects 432px where
the immutable prototype and actual native drawer require 460px. The former
shows body scrolling after foreground activation and remains under investigation.
No product package, installer, installed security probe or boat promotion is
accepted. Linux endurance continues independently. Narrow reproductions and
corrections precede any further full candidate build.

Separately, `29ea06a358ed24cc29eb2be74b67b234a3ecaa6e` passes
[focused native run 37026807977](https://github.com/ThereptileII/Work/actions/runs/37026807977):
190 Settings, 82 Search/Shell and 46 Chart drawer checks (**318 total**), plus
compilation of all 28 production UI objects and the complete NavigationActions,
NavigationObjects and SettingsStore units. The downloaded artifact, 247 source
inputs, 31 objects, three executables/runtimes and 23 captures were verified.
[Exact proof and limitations](evidence/scrum-224-29ea-focused-native-proof.json)
remain component-scoped. Display track colors/geometry are supported; known
other Settings differences and physical boat acceptance remain open. This pass
does not qualify the full candidate or the later local System-flow correction.

## SCRUM-14/15/216 — bounded prototype follow-up; native proof pending

Published follow-up `34b8509` reached native CMake setup in
[short run 37025085247](https://github.com/ThereptileII/Work/actions/runs/37025085247),
which rejected an obsolete include directory before compilation or execution.
The [retained artifact and all-path audit](evidence/scrum-224-prototype-native-setup.json)
identify the narrow correction: remove `gui/src/s57/include`, absent from the
pinned source tree; retain strict checks for all 27 actual directories and all
production headers/macros. Only this short proof is retried. No application
crash, dependency qualification, Windows UI pass or boat acceptance is inferred.

Retry `9b0766b` / [37025995465](https://github.com/ThereptileII/Work/actions/runs/37025995465)
compiled both complete navigation-bridge units and the production UI library.
It stopped linking the standalone Search test because its Windows console
entrypoint was missing. Both new fixtures now use the established explicit
`main` → `wxEntry` pattern. Their local checks still pass (82 + 46); only the
short native proof is retried. Product sources and `c95d` remain unchanged.

Separate from frozen candidate `c95d3a0`, the October 2 follow-up restores rail
metric icons and the Display selector track, adds saved-route/waypoint Search,
and connects the native Chart presentation drawer to floating Layers and
Settings → Navigation. OpenCPN owns the chart preferences; commands display
observed readback, and unsupported/managed layers have no invented switch.

The combined Linux component build passed. Search and Shell entrypoints passed
82 checks, including rejected-action readback, Escape/Close and a short-chart
overlap boundary; the chart drawer passed 46 checks. Two synthetic Shell captures
and exact local source/executable identities are recorded in
[scoped evidence](evidence/scrum14-prototype-combined-linux/verification.json).
Earlier component-only Display and rail reviews retain their original scope.

A separate short native proof compiles the complete changed navigation bridge
and production UI, then runs the Settings, Search and Chart components. It does
not rebuild dependency producers, qualify an installer, or launch on the boat.
Windows and boat conformance remain pending. The full `c95d3a0` candidate remains
unchanged: at 15:04 UTC its Linux endurance step and native integrated build were
still running; no product artifact or boat replacement is accepted here.

## SCRUM-224/214 — native encoding proof passed; c95d replacement qualifying

Published replacement `c95d3a091bb2a0ce0e19d0146ab28dc848ef5290`
(local `24d0a5a`) passes [native Python-only run 37017228040](https://github.com/ThereptileII/Work/actions/runs/37017228040):
Windows Python 3.12.10 with default cp1252 completes all 24 tests in 0.303 seconds.
The GitHub comparison against `2060` confirms only the geometry test file and
its small Python-only workflow changed; product code is unchanged. Both AST
source reads now explicitly use UTF-8, with a regression for legacy Windows
decoding. Geometry, clipping and touch assertions remain intact.

The preceding `2060` full run failed Windows contracts at 86/87 tests: implicit
cp1252 decoding corrupted the Unicode minus label in the geometry harness,
failing thirteen subcases. Native integration was skipped. This is a test-source
encoding defect, not a product crash. Other Linux jobs from that revision remain
running; no endurance result is inferred. The earlier 190-check Settings
component proof remains valid within its recorded scope.

After the native Python proof passed, the integrated branch advanced to exact
`c95d`. [Replacement run 37017351644](https://github.com/ThereptileII/Work/actions/runs/37017351644)
is in progress. Full native application, CLI, DPI/visual, installer/recovery and
endurance acceptance remain open. See the [failure and replacement proof record](evidence/scrum-224-c95d-native-python-proof.json).
No product, boat or release acceptance is claimed.

## SCRUM-224/98 — retained 2060 focused native proof

Published source `2060f8f6b078db6e74e6f8b111e7439073ca1bce` (local `af33a05`)
passes [focused native run 37014950505](https://github.com/ThereptileII/Work/actions/runs/37014950505).
The downloaded artifact matches its API length and SHA-256; all 867 ZIP entries
pass CRC/path checks. Native Settings completes 190 checks, fourteen captures
and ordered interaction steps 0–23, including root reopening. The executable,
twelve runtime DLLs and fourteen captures match their retained hashes. All five
changed objects and 26 UI objects compile; both legacy macro negative controls
fail as intended. The native home-directory excerpt proof passes, but does not
execute the actual installed CLI. See the [scoped proof](evidence/scrum-224-2060-focused-native-proof.json).
All 256 retained source/input records correspond to exact local `af33a05`:
seven are byte-identical and 249 match the expected Windows CRLF checkout.

The preceding `f2704c3` probe failed because the Settings test's recurring timer
re-entered its interaction sequence. That failure remains retained. The `2060`
correction changes only `tests/settings_drawer_test.cpp`; production Settings
code remains unchanged.

After this proof, the integrated branch advanced once from `488f` to exact
`2060`. [Full candidate run 37015958242](https://github.com/ThereptileII/Work/actions/runs/37015958242),
created at 13:52:44 UTC, later failed Windows contracts and skipped native
integration as described above; separate Linux jobs remain running. Actual CLI, complete DPI/fullscreen,
chart/plugin, installer/recovery, runtime closure and native endurance gates
remain open. Captures still require visual review; this is neither product nor
boat acceptance. Publication to `ThereptileII/Work` was explicitly authorized
on 2026-10-02; the repaired source and earlier qualification evidence are
published. No boat deployment or retirement has occurred.

## SCRUM-224 — compile repaired; three native qualification failures isolated

Frozen `488fbbdf5161e986c986773187b1008a7f8684f5` has completed
[run 36984898997](https://github.com/ThereptileII/Work/actions/runs/36984898997):
14 jobs passed, native integration failed, publication was skipped. Application
compilation/linking now pass. Windows passes 138/138 application tests, native
chart/route pointer flows, repeated crash recovery and the public ENC/software
and OpenGL chart gate. These are not complete Windows product acceptance.

The native failures are now source-grounded: the peer CLI refusal test runs
after GUI processing has created the normal profile directory; root Preferences
reopening retains a scroll offset that hides its section tabs; and the 1920px
DPI oracle expects 61px navigation although the immutable HTML and actual
native rendering require 69px. Local repairs preserve the unknown-profile
refusal and strict UI assertions. The Preferences component passes 179 checks
and its no-reset negative control fails; 23 geometry tests distinguish the
prototype width breakpoint and reject deliberately incorrect sizes. The first
[focused run 37013831216](https://github.com/ThereptileII/Work/actions/runs/37013831216)
subsequently failed on test-timer reentrancy. Its retained failure and the passing
replacement proof are distinguished above; full candidate qualification remains
pending.

Linux integrated qualification passed: 146/146 tests in each fixture and
fixture-free configuration, plus the actual three-hour trip. The latter records
1,080 samples, 540 UI actions, about 1.1 MB resident growth, no measured
handle/thread growth and 3.21% of one CPU core. This is Linux evidence, not a
substitute for native endurance or boat acceptance. Both artifacts have verified
size, SHA-256 and ZIP CRC. See the
[terminal qualification record](evidence/scrum-224-488f-terminal-qualification.json).

No fixture-free Windows package or installer was produced; those dependent
stages were skipped. The boat is unchanged, old installations are retained,
and prototype fanout remains held. On 2026-10-02 the user explicitly authorized
publication of the reviewed source repairs and non-sensitive qualification
evidence to `ThereptileII/Work`. Fresh native acceptance remains required.

## SCRUM-224 — earlier focused compile proof and prerequisite results

Frozen candidate `488fbbdf5161e986c986773187b1008a7f8684f5` (local `548a376`)
passes [the focused native compile](https://github.com/ThereptileII/Work/actions/runs/36984238840).
The verified artifact reproduces both original Windows macro failures, then
compiles the four corrected networking units, production UI target and full
SettingsStore. All 31 retained I386 objects match their recorded hashes; this
covers 14 of the 15 changed XNav UI/integration source files. See
[exact-source proof](evidence/scrum-224-native-changed-units-488fbbd.json).
The first short probe stopped at a CMake path-escaping error before compilation;
the corrected probe passed without rebuilding the maintained dependencies.

The [integrated run](https://github.com/ThereptileII/Work/actions/runs/36984898997)
subsequently verified full application compilation/linking, then failed the
three later checks described above. The new early compile gate detects the
original macro failure before expensive producer suites. No boat installation
or retirement occurred.

Eleven early artifacts from this run have verified API sizes, SHA-256 and ZIP
CRC, including the same-run restart prerequisite receipt. Windows pristine
passes 60/60; Linux pristine retains its two named upstream failures (64/66).
These are prerequisites, not integrated product acceptance; see
[early evidence](evidence/scrum-224-488f-early-gates.json). The earlier 09:02 UTC
running observation is superseded by the terminal record above. Neither job
was restarted or cancelled to collect it.

The same candidate's completed contract logs confirm 90 Linux / 87 Windows
CTest passes, 43 dependency contracts per platform and ten additional restart
executions per platform, with no failures in those suites. See
[contract evidence](evidence/scrum-224-488f-contracts.json). These results do not
replace integrated application, installer, UI or boat qualification.

## SCRUM-22/27 — installer preservation checks prepared in parallel

The unsupported-build smoke assertion previously checked the supported sibling
directory rather than the rejected installation. Test-only revision `0fc08f9`
now snapshots the actual rejected tree and pre-existing seeded profile, verifies
the exact rejection reason and checks that installation/shortcut folders remain
absent. It records Setup, package-manifest and test-source hashes. Installer
product code is unchanged. This correction is separate from frozen `488f`;
native proof against its eventual verified package remains pending.

[The recovery coverage audit](evidence/scrum-27-real-release-coverage-audit.json)
confirms genuine accepted Beta 1 and genuine early Beta 2 installers are used
by the existing update/rollback harness. Historical 45-check recovery evidence
is retained separately from pending candidate acceptance. There is no need to
rerun another complete application build for this audit or the test correction.
Neither issue is Done and no boat retirement has occurred.

## SCRUM-24/25 — signed release policy passes native tests

The isolated update branch at `a3a7bac04a3bdda8ddd485450a7dab8aedbd2c4d`
passes [Linux and native Windows/386](https://github.com/ThereptileII/Work/actions/runs/36939227135).
Each platform passes 19 top-level tests and 30 subtests, without failures/skips,
plus `go vet`; both downloaded artifacts have verified length, SHA-256 and CRC.
The policy distinguishes upgrades, same release, downgrades, conflicting source
identities and unsupported OpenCPN installations. Strict JSON and Windows path
checks reject the case-alias and reserved-name defects found during review.
The read-only composition binds policy identity to actual signed TUF metadata;
no installer executes and no startup updater is enabled. See
[exact native evidence](evidence/scrum-24-release-policy-native.json).
Root/key/cache custody and transactional recovery remain open. The boat candidate
is still separately frozen at `45b8a9d`.


## SCRUM-25 — isolated signed-update verifier passes native feasibility

The separate verifier branch at `24bd013f4891638f0e59411836c67f271a66a645`
passes [Linux and native Windows 386](https://github.com/ThereptileII/Work/actions/runs/36936782032).
Both downloaded result artifacts have verified size, SHA-256 and ZIP CRC.
Each platform passes six top-level tests and nine subtests without skips, plus
`go vet`. The tests use real Ed25519-signed metadata generated only in temporary
fixtures and reject altered signatures/content, expired or older metadata,
wrong channels/identity, oversized data and cross-origin redirects.

This is a feasibility result, not a shipping updater. It neither executes a
package nor modifies the application or installer. Trusted-root deployment,
cache protection, application release/downgrade policy, dependency licensing and
transactional installer handoff remain open; see
[the scoped evidence](evidence/scrum-25-verifier-feasibility.json).
SCRUM-24 now owns the deterministic release-policy prerequisite. Product
qualification continues independently on frozen `45b8a9d`.


## Current replacement — verified early gates

All ten native boat-tooling jobs at exact `45b8a9d` pass. Twenty-seven
available early artifacts have independently verified lengths, SHA-256 and
all 1,237 ZIP entry CRCs. Supplemental AIS passes seven tests per platform;
the Linux network fixture passes 18 TLS/transport scenarios and 30 offline
checks. Prototype references capture 63 states per platform, without implying
native application visual acceptance. Pristine Windows passes 60 tests;
pristine Linux retains its two classified upstream failures out of 66 and is
not release acceptance. Integrated Linux endurance and the three native product
build jobs remain in progress at this observation; installation remains withheld.
See [the exact early evidence](evidence/scrum-224-45b8-early-gates.json).

## Current replacement — native certificate path repair

The application candidate is now frozen at
`45b8a9d2809d0d34171010ad1c00f4a3cd056a09` (local `9c4740f`).
The previous `37978e` composition and object-workflow builds failed before curl
configure because a Windows-backslash script path made MSYS Perl look for
certificate inputs in the output directory. The narrow caller repair uses the
verified MSYS Perl explicitly and passes the same slash form as upstream CMake.

[Focused native run 36935540968](https://github.com/ThereptileII/Work/actions/runs/36935540968)
passes the new actual-script path regression and retained host/certificate/tool
checks. Its downloaded artifact has verified length, SHA-256 and all 20 ZIP CRCs.
See [the scoped proof](evidence/scrum-224-45b8-native-path-pass.json).
Fresh [integrated](https://github.com/ThereptileII/Work/actions/runs/36935775514),
[prototype](https://github.com/ThereptileII/Work/actions/runs/36935778459) and
[native tooling](https://github.com/ThereptileII/Work/actions/runs/36935773834)
runs target that exact commit; they are not yet accepted. No deployment or old
installation cleanup has occurred. A read-only boat heartbeat at 22:24 UTC
confirmed zero OpenCPN processes and all three remote-access services running.


## SCRUM-218 — complete native diagnostic log retained

The separate read-only collector at `c42ff4c97ee71cfe84bab54873ee5dac18f15510`
passes [run 36932209206](https://github.com/ThereptileII/Work/actions/runs/36932209206)
and ten focused refusal/security tests. It retained all 1,738,273,051 bytes of
the cancelled b48 prototype job's log within the explicit 2 GiB / eight-minute
limits. The downloaded archive and uncompressed log have independently verified
hashes and lengths. No running product job was modified or restarted. This is
diagnostic evidence, not application acceptance; see
[the collection record](evidence/scrum-218-complete-native-log.json).

## Previous candidate qualification — 37978e

The previous frozen candidate is `37978e26517af810092ec2bc554b199d36a705d2`
(local `2a3c2c6b88945a50a7b552fd41c37658747ac5a4`). Its
[integrated run](https://github.com/ThereptileII/Work/actions/runs/36929977632)
passes 90 Linux and 87 native Windows portable CTest contracts, plus ten
repeated restart handoffs on each platform and the supporting policy suites.
All ten [native tooling jobs](https://github.com/ThereptileII/Work/actions/runs/36929976171)
pass. Fourteen downloaded early artifacts have verified size, SHA-256 and ZIP
CRC records, including 238 actual COM shortcut migration checks on each host
architecture, 22 maintenance reports, native broker/Prepare/Arm checks, and
supplemental AIS tests. See [the exact-candidate early evidence](evidence/scrum-224-37978e-early-gates.json).

The [prototype run](https://github.com/ThereptileII/Work/actions/runs/36929980191)
has passed its supporting checks, but native composition job `110596574723`
failed before curl configure: MSYS Perl received a backslash script path,
and the certificate generator searched the output directory for source inputs.
The producer correctly rejected the empty certificate. The separate object-flow
job failed at the same preflight; its independently verified artifact confirms
the same error. The integrated Windows job also failed at the same preflight; all three
native failure artifacts have verified hashes and ZIP integrity.
See [the verified failure record](evidence/scrum-224-37978-certificate-path-failure.json).
The replacement caller-path repair has passed its focused native gate; its
full qualification remains open as recorded above. No existing generator or
TLS assertions were weakened.
No artifact is approved for boat deployment. The previous b48 Linux job has
completed successfully, including 146 fixture and 146 fixture-free test cases,
the 10,800-second soak, loader checks and mode/navigation/chart fixtures. See
[its bounded evidence review](evidence/scrum-224-b48-linux-complete.json).
This older Linux result is not acceptance of this replacement revision. The boat remains
unchanged and old installations are retained until a replacement is known-good.

## SCRUM-224 — native test-host producer binding verified; product gates remain open

The focused producer binding run at exact commit `fcd5adfca9e4ffdda00eddd40a48b2d85b433587` completed successfully in [run 36929342744](https://github.com/ThereptileII/Work/actions/runs/36929342744), job 110594459325. Its 13,192-byte artifact (SHA-256 `5d17f69c19d7c4f763d4e7277565708318f5b5f54ce77b1dd73b502d8b729db0`) passes all 19 ZIP CRC checks. The native Win32 CMake cache binds curl tests to MSYS2 Perl; wrong-host and tampered-cache cases refuse, and restored facts pass. Local 13+14+9 checks pass. This qualifies the test-host binding diagnostic only, not the complete upstream curl/HTTP/TLS suite or application, UI, installer, or boat acceptance. See [SCRUM-224 native binding evidence](evidence/scrum-224-native-test-host-binding-pass.json).

## SCRUM-224 — native curl test-host proof; frozen application gates remain open

The frozen candidate `b48bf4a8f12f98c459d79aa08805141ac49e0306` still has two
cancelled native prototype jobs in
[run 36901029053](https://github.com/ThereptileII/Work/actions/runs/36901029053);
their UI gates were skipped. Bounded retained logs show Strawberry Perl reaches
a curl test-runner state where Windows cannot resolve `exec`, curl fails to
start its HTTP server, and test 1 remains active.

Four focused native cases on source commit
`622157fed6f2a3a9c5aeb476e502941789790498` use unchanged curl 8.22.0
`runner.pm` and `servers.pm`. Strawberry Perl reproduces readiness and server
launch defects; MSYS2 Perl reports `cygwin`, passes readiness and starts a
disposable marker process. This confirms the test-host choice is material to
these modules. The [pinned Windows workflow](https://github.com/curl/curl/blob/01346829096c61b372692f6dc43ffa778c6caccd/.github/workflows/windows.yml)
selects MSYS2 for curl tests while the product target remains native Win32 MSVC.
See [host-probe run 36926615433](https://github.com/ThereptileII/Work/actions/runs/36926615433).

The follow-up [run 36927000029](https://github.com/ThereptileII/Work/actions/runs/36927000029)
also passes certificate generation and CA/host/key verification using the
existing patched `genserv.pl` under MSYS2, with explicitly selected host OpenSSL
3.6.4. No additional generator patch was needed; this does not qualify the
pinned production OpenSSL 3.5.9. The earlier b48 composition artifact also
shows the pinned OpenSSL 3.5.9 certificate setup generated local test material
and verified `test-localhost.crt` before the runner failure. Neither focused
probe is a full HTTP/TLS or curl-suite pass. Earlier failed selector and
host-label checks remain preserved in the
[SCRUM-224 evidence](evidence/scrum-224-native-test-host-pass.json).

The separate [integrated run 36901026915](https://github.com/ThereptileII/Work/actions/runs/36901026915)
still had Linux integrated and native MSVC XNav/Legacy/Safe jobs in progress at
21:14:07Z. Full application, UI, installer and boat acceptance remain open.

## Current qualification and boat cleanup — 2026-10-01

SCRUM-222 cold-profile preservation tooling at
`034545be03295352a1c509d8979e4a3eb5bf64b6` passes all ten native Windows
tooling jobs in [run 36911321279](https://github.com/ThereptileII/Work/actions/runs/36911321279).
Downloaded artifacts were independently size/hash/ZIP checked. The cold suite
passes 40 native disposable groups, including actual private ACL refusal and
restoration plus normal commissioning/restore; see
[native preservation evidence](evidence/scrum-222-native-cold-baseline.json).
This qualifies the exact tooling, not the frozen application or boat acceptance.

The boat source checkout was updated to that exact tooling commit without
changing the installed application. An earlier SCRUM-222 cold-profile capture
refused before backup publication because two known-hash o-charts decoder
helpers remained from September 29 despite OpenCPN being closed. The current
user INI remained intact; at that point no launch or cleanup occurred. SCRUM-223
tracks separately reviewed normal local shutdown for these exact orphan
processes outside an active commissioning transaction. Existing closed-process
checks remain mandatory.

SCRUM-223's first native maintenance attempt at commit
`363fee0cc37a70b09d41799638b0ab7935041411` is preserved as a failure: 24 of 25
reports passed, while the isolated cold-helper workflow fixture rejected its
CRLF source substitution before native fake transport. The corrected test-only
source at `20765cf374da5a1ab47dff0b3b427cd24f595455` passes all 10 native jobs
and all 26 reports in [run 36919418416](https://github.com/ThereptileII/Work/actions/runs/36919418416);
the 30,731-byte artifact digest and ZIP integrity were verified. The 33-check
workflow substitutes process, ACL, transport and ledger observations. Separate
checks include 45 native pipe-suite checks (10 disposable fake-pipe cases) and
23 native disposable-ledger ACL checks; none invokes the vendor helper or
accesses the boat. See [SCRUM-223
native evidence](evidence/scrum-223-native-cold-helper.json). On the boat, the
reviewed capture covered two helpers, 2,031 profile entries and three plugin
trees with 80, 40 and 80 entries. The two fixed one-shot local requests
completed at 20:25:28Z; independent verification at 20:25:56Z confirmed both
retained process identities and exact pipe peers, then measured exit code 0 for
each. The post-close inventory found zero OpenCPN processes and zero chart
helpers. The profile was unchanged at SHA-256
`d891d88c62657139e1b1c6ff7d9e8acdae726a4e116dbc844126992a39adbdb6`, and the
stock executable remained at its reviewed SHA-256. All three remote services
were running. There was no application launch, profile change, plugin move,
force termination, reboot or physical output. The required post-close SCRUM-222
cold Capture succeeded and its exact record was downloaded and hash-checked.
Independent exact-key review and native Complete now pass: all 19 changes are
preserved, including the current display and saved-position state, without
attributing their origin or treating saved coordinates as live data. The completed
record is hash-bound to the unchanged 21,492-byte profile and predecessor; see
[actual cold preservation evidence](evidence/scrum-222-boat-cold-preservation.json).
Fresh source/plugin review and input-only commissioning remain mandatory before
remote application launch. Installation, retirement of the old copy and boat
acceptance remain pending.

The frozen full candidate is `b48bf4a8f12f98c459d79aa08805141ac49e0306`
(local mapped commit `0053ed507b03c79877fd6bb96c0d761fb9f68ac9`). Its narrow
native tool-facts and version-probe regression passed under both Windows
PowerShell 5.1 and PowerShell 7 in [run 36900544171](https://github.com/ThereptileII/Work/actions/runs/36900544171);
both artifacts were independently verified. The probe covers real x86 tool
facts, stream stability, refusals and restored verification. The same-commit
Linux and Windows contract jobs now pass (90 and 87 CTest cases respectively,
plus the separately recorded Python, native and repeated lifecycle checks).
The [Windows tooling run](https://github.com/ThereptileII/Work/actions/runs/36901026982)
passes all 10 jobs; verified actual COM shortcut reports pass 238 checks on
each Windows host architecture. See the [contract evidence](evidence/scrum-217-b48-full-contracts.json)
and [shortcut evidence](evidence/scrum-215-b48-native-shortcuts.json).
The [integrated](https://github.com/ThereptileII/Work/actions/runs/36901026915)
run remains in progress; two native Windows prototype jobs in [run
36901029053](https://github.com/ThereptileII/Work/actions/runs/36901029053) were
cancelled during build/regressions before downstream UI gates. No full candidate
acceptance is established. This focused pass does not qualify the full dependency
producers, dependency reuse, application, package or boat.
See [native stream-pass evidence](evidence/scrum-217-native-stream-pass.json).

Downloaded same-commit recovery evidence now verifies 21 native maintenance
suite reports, 51 guarded-broker checks and 56 Prepare/Arm/Collect checks.
These use disposable files and marker processes, not the boat or OpenCPN UI;
see [recovery tooling evidence](evidence/scrum-17-b48-recovery-tooling.json).
The pristine upstream Windows build passes 60/60 tests. The pristine Linux
comparison builds but reproduces its two documented upstream failures (64/66);
it is explicitly not an all-tests-pass result. Integrated application tests
still must pass without exceptions. See [upstream baseline evidence](evidence/scrum-217-b48-upstream-baselines.json).

An earlier frozen candidate, `141265ea2fa8796c9570065034f31575292ff49e`
([integrated run](https://github.com/ThereptileII/Work/actions/runs/36892709388),
[prototype run](https://github.com/ThereptileII/Work/actions/runs/36892710519)),
was **not qualified**. Windows contracts failed on nonportable path fixtures;
the dependent integrated Windows application job was skipped. The prototype
composition job separately failed before OpenSSL compilation because its child
PowerShell could not resolve `Get-FileHash`. These remain historical SCRUM-217
integration failures, not acceptance of the certificate repair or product.

The isolated receipt-fixture correction at
`899af50a1a4623a08f71d199d146a1cecd1959c2`
([focused run](https://github.com/ThereptileII/Work/actions/runs/36894286613))
passes the 15 receipt, 9 reuse and 4 staging tests on native Windows. The
13-test evidence suite failed at OpenSSL retained-log summary parsing;
Linux passed all 41 focused tests. A subsequent CRLF parsing repair at
`5a0015a2911a0b643c761df9657e6a3c6ab31e7c`
([focused run](https://github.com/ThereptileII/Work/actions/runs/36895232260))
passes all 42 tests on both native Windows and Linux: receipt 15, evidence 14,
reuse 9 and staging 4. Downloaded artifacts were independently size/hash/ZIP
checked. Original byte hashes and contradictory-failure rejection remain
enforced. The separate PowerShell producer repair progresses past the original
missing-module failure. Its diagnostic run at
`d6b05c8554c671e7504b797d15cc33404e226495`
([native run](https://github.com/ThereptileII/Work/actions/runs/36898370390))
passes Windows PowerShell 5.1 but fails PowerShell 7: only NMAKE's merged
stdout/stderr ordering changes the recorded first line and output hash. The
retained captured/observed records have identical executable identities,
environment and exit codes. Bounded separate-stream capture was subsequently
implemented and passed its real-process regression under both interpreters;
strict identity and output comparisons remain required. See the
[stream-order failure evidence](evidence/scrum-217-native-stream-order-failure.json)
and [subsequent narrow pass](evidence/scrum-217-native-stream-pass.json).
The narrow interpreter gates subsequently passed on the frozen b48 candidate;
its broader native qualification is still in progress as noted above.
No new boat deployment or removal of older installations has occurred.

The published candidate `f1e2cde8fcbf92826d648007b267cc0f5320aa55`
([integrated run](https://github.com/ThereptileII/Work/actions/runs/36875827855))
is not qualified: its Linux user-flow test stopped at the Create waypoint
Name field, which measured 46 pixels against the unchanged 48-pixel touch
minimum. The integrated build, 146 unit tests and preceding mode, navigation,
route-progress, marine-input and object checks passed; later chart, endurance
and fixture-free Linux gates did not run. All three native application jobs also failed at curl test certificate
generation (SCRUM-221); their earlier tooling successes do not qualify the application.

A later bounded Windows certificate probe passed at [run 36891804473](https://github.com/ThereptileII/Work/actions/runs/36891804473),
job `110468992463`: the locked original reproduced the expected filename failure,
the pipe-free helper passed 5/5 focused tests, and CA/host generation took
717 ms. Certificate, key and chain checks also passed with host OpenSSL 3.6.4. The
corresponding-source reproduction passed. This is not pinned OpenSSL 3.5.9,
full curl/application qualification, or boat acceptance; the three application
job failures remain part of the unqualified candidate history. See the
[SCRUM-221 evidence](evidence/scrum-221-native-certificate-pass.json).

SCRUM-220 fixes the shared editor minimum without changing the prototype
Preferences token: single-line editor heights are 48/52/56 DIP at 100/125/150%
interface scale. Local implementation `8a938eccfe8ca05392d6ebe5e404ad1f702a76e5`
has passed a real wxGTK modal component probe and an integrated Linux build,
install and unchanged eight-group waypoint/route interaction smoke. Both Name
fields measure 488×48 and navigation-database persistence checks pass. Exact
replacement full-suite Windows/boat evidence remains required. See the
[regression review](design/reviews/scrum-220-waypoint-touch-regression.md).
The boat still has no new SKAGER replacement installed; old versions and
recovery backups remain intact pending a known-good installation.

The replacement integration worktree retains the reviewed Display preferences,
1920×1080 geometry and native CMake extraction repair (SCRUM-216/213/219), then
adds the touch repair and explicit same-job dependency reuse (SCRUM-220/217).
The reuse receipt, producer evidence, tamper refusal and staging suites pass
41 focused cases locally; Linux and native Windows CI now invoke them. Reuse
is restricted to a successful fixture build in the same job and never replaces
downstream product/security/installer tests. The SCRUM-221 helper-level native
certificate probe now passes; the pinned producer/full curl suite still needs
integration validation.
Complete replacement Linux, native Windows and boat gates remain required.
No website work, boat installation or physical hardware output is included.

The corrected frozen candidate `e9737d6bad9f3eb3db71877bce0816f59c62b0e7`
has not qualified. Its [prototype run](https://github.com/ThereptileII/Work/actions/runs/36850600318)
reached terminal cancellation in both native build jobs. The downloaded,
hash-verified composition artifact shows OpenSSL completed 347 files / 4,283
tests and zlib passed 13 tests, then curl preparation produced no configure
output before the 180-minute job limit. This is a separate failure from the
earlier zlib URL mismatch below. SCRUM-219 investigates the exact preparation
boundary in a disposable native probe; no root cause or replacement acceptance
is claimed. See [retained evidence](evidence/scrum-219-curl-preparation-stall.md).
A short native probe now reproduces a stall in Windows system tar on the exact
locked curl XZ archive; CMake extracts those same bytes in 936 ms. SCRUM-219
has now passed its corrected native extraction probe in 767 ms, with all
4,406 source paths and SHA-256s identical to the locked archive. The focused
repair is validated; full-product acceptance is not inferred from that result.
The earlier e973 full integrated Linux job has passed, including its real
three-hour endurance and fixture-free build. Its native job remains separately
under observation; this is not acceptance of the new revision. No
replacement was installed on the boat and obsolete copies remain intact until
the new installation and recovery modes are known-good.

The zlib source URL correction is published as
`c85db9d308c4a0ae1bd527bb7634b559c0fdb8e6` (local
`5957f14a26675b29950423556a9813a6faa36c10`). Its
[native source-only run](https://github.com/ThereptileII/Work/actions/runs/36843124074),
job `110306680157`, passed. The downloaded artifact was independently checked
against GitHub's artifact digest; its source record confirms the expected and
observed zlib archive are both 1,502,830 bytes with SHA-256
`bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16`.
The [combined native run](https://github.com/ThereptileII/Work/actions/runs/36843282070)
and [prototype run](https://github.com/ThereptileII/Work/actions/runs/36843282641)
are for that same exact commit. The prototype run's native composition/object-flow
jobs stopped at curl's zlib manifest identity guard; the verified failure artifact
is from composition job `110307179535`, not the combined run. Its zlib producer
recorded the reviewed GitHub release URL, while two curl consumers still
required the earlier zlib.net URL. The downloaded native artifact was
hash-checked before review; OpenSSL and zlib
built, and zlib's 13 tests passed before this guard. The subsequent local
consumer correction aligns the URL in the PowerShell builder, Python package
verifier and notice provenance while retaining archive/hash/size/signature
locks. A self-contained producer/consumer guard now runs before the costly
native dependency build, while all later real build checks remain in place.
Its actual producer-manifest, focused PowerShell 7 and Python checks pass; corrected
native application/package qualification has not run. See the
[consumer closure record](evidence/scrum-209-zlib-consumer-closure-20261001.json).
The combined run also reached a terminal native-job failure; that separate
failure is not attributed from the reviewed prototype artifact. These runs do
not establish a qualified fixture-free application/package,
installer, boat or release result. The earlier source failure/pass record is in
[zlib source URL evidence](evidence/scrum-209-zlib-source-url-correction-20261001.json).

The preceding prototype candidate
`7464a00d1071cd019c51821b7c858f5b1f989837` failed its native source guard in
[run 36841633753](https://github.com/ThereptileII/Work/actions/runs/36841633753),
job `110301869566`, before application testing. Its retained source report
records expected size/hash 1,502,830 bytes / `bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16`
and observed size/hash 11,975 bytes /
`d5b43f44bbe15f74bae56414ae917f7b2ed13ab0df81bff0b845a100efda5681`.
The response body was not retained or characterized. No cause beyond the
observed identity mismatch is claimed.

The earlier frozen security/display candidate
`99b0a58b08c5742a7e31341071eabd0124530e4a` (local
`325ccd3596d368a6aaf0787bdf4b4e673b13e881`) also had an incomplete
[full native/Linux run](https://github.com/ThereptileII/Work/actions/runs/36832799208)
and [prototype run](https://github.com/ThereptileII/Work/actions/runs/36832798681).
Its prototype object-flow job failed the zlib archive lock check; that older
artifact did not expose the observed size or hash. Keep this distinct from the
later `7464a00` report above. The current corrected runs remain partial results.
Native portable contracts pass 86/86; the actual Windows PowerShell 5.1
filesystem and historical shortcut suites pass in both bitnesses. The earlier
`322aecdf8cd51fd6ea951ae0caf6b0bde5cefaba` candidate failed because Windows
translated a disposable peer marker to CRLF; the replacement writes canonical
bytes and preserves the strict consumer check.

The user authorized leaving the tested SKAGER build on the boat PC and retiring
obsolete OpenNav X copies and shortcuts (SCRUM-17). A read-only inventory
confirmed the supported stock OpenCPN hash, the accepted Beta 1 installed
generation, older owned generations and duplicate historical Start-menu groups.
No application, profile, chart or recovery file has been removed. Replacement
qualification and boat checks precede retirement; recovery material is retained.
The [fresh cold backup](evidence/skager-boat-cleanup-readiness-20261001.json)
verified 2,123 application and 2,008 profile files against the original bytes;
it is readiness evidence, and no cleanup or software install has occurred.

SCRUM-215 adds versioned SKAGER shortcuts and installer/maintenance captions in
a separate subsequent increment. Both historical layouts remain supported for
rollback. Its subsequent source is published as
`010fe057dcbe4aa85d14f8734d2cd1113646cdd1` (local `6d640e1`), with every
mapped blob/mode/type verified. All nine jobs in the
[short native tooling run](https://github.com/ThereptileII/Work/actions/runs/36837038651)
pass, including 238 actual COM/historical-lifecycle checks in each PowerShell
bitness. The rollback handoff removes verified SKAGER links before exposing
the old maintenance engine, preventing orphan links after an interrupted
rollback. Full application packaging, NSIS lifecycle and boat checks remain
required; SCRUM-215 is Testing. This is not part of the frozen `99b0a58`
candidate and its tests do not qualify that different source.

The subsequent SCRUM-214 Vessel Preferences form is integrated locally at
`2174501` from `d534649`. Its five native fields save the vessel display name,
explicit model assumptions and edited stock S-52 safety depth. Blank chart depth
keeps the current chart value; missing model inputs stay unconfigured. Local
focused results are 124 component checks with 12 captures, seven persistence
checks and the actual OpenCPN integration syntax check. The Night capture was
reviewed and an independent preservation review found no defect. A subsequent
isolated Linux integration configured embedded source commit `ab485998` (while
the worktree advanced to documentation-only `f7f5b9f`). Its pinned-source
application, settings-driver and test targets built 720/720; the remaining
install dependencies built 287/287. The integrated settings tests passed 7/7,
peer-unavailable tests 4/4 and isolated 1280×800 component driver 124 checks.
The installed developer build passed its loader/resource self-test and XNav,
Legacy and Safe startup plus IPC quit. One concurrent Safe capture was killed
under local memory pressure while Legacy launched; its sequential rerun passed.
See the [local integration record](evidence/scrum-214-vessel-linux-integration-20261001.json).
This is development evidence for the embedded source, not a new candidate,
native Windows, boat or release qualification.

The subsequent form-spacing refinement and capture-harness update pass a
separate 130-check Linux component run with 12 captures. Its five input bounds
and Save button match the independent prototype rectangles exactly in that
component environment; screenshot review found the small paired captions
visible and unclipped. The revised Linux capture harness passed four responsive
cases against the earlier installed `ab485998` developer app, including lower
Save visibility and vessel-profile entry. That run validates the harness flow,
not the refined geometry in an integrated app. See the
[refinement record](evidence/scrum-214-vessel-form-refinement-20261001.json).

A fresh read-only boat inspection at 08:49 UTC confirmed no OpenCPN process,
the exact supported stock 5.12.4 x86 executable, and running SSH, Tailscale and
RustDesk services. Windows currently reports 1920×1080. This is readiness
inventory, not product, physical-touch or new-display acceptance. No old
software or user navigation data has been removed.

## Display coverage and current priority — 2026-10-01

The user added **1920×1080** as a supported target alongside the primary
**1280×800** prototype layout (SCRUM-213). The native DPI harness now exercises
the large fullscreen chart, four primary rail values, alert accessibility and
Passage/Traffic/Settings/Alerts panels, then verifies return to 1280×800.
Existing 100/125/150% DPI checks remain. Source syntax checks pass; native
captures and boat-display acceptance are still pending. See the
[qualification record](design/reviews/1920x1080-qualification.md).

Website development waits until the software is complete and tested on the
boat PC, following the user's latest sequencing instruction. Jira remains the
sole backlog. Read-only boat readiness confirmed connectivity and the supported
stock executable; it is not acceptance of the current development candidate.

## Security integration in progress — 2026-09-30

The integrated source was published as `7a7392cc6ef15594db6ccd6c330e615f2db33f5d`
(local `b3ee47884b8455adee11f2915bfe95e85d7848db`). All 1,337 mapped
entries were checked by blob hash, mode and type before publication; eight
unrelated entries and seven unrelated branches were preserved. Native/Linux
[run 36818668852](https://github.com/ThereptileII/Work/actions/runs/36818668852)
and prototype [run 36818667685](https://github.com/ThereptileII/Work/actions/runs/36818667685)
failed their native dependency build before application qualification:
zlib build logging reported a Windows file-sharing violation after MSVC setup.
The exact locked file was not identified by that log. The notice-byte regression
gate passed on native Windows and Linux; application packaging/installer
acceptance is not established. A follow-up removes repeated batch log opens
and records each build stage, without retrying or suppressing failures. SCRUM-208/211 are Testing, not Done. SCRUM-209 is In Progress after
review found that preserved plugin/addition files could restore legacy TLS DLLs
into an otherwise maintained candidate. Candidate-only refusal now preserves
those original files and the previous installation. It also audits normal and
delay PE imports after preserving plugins, including transitive local modules.
Linux PowerShell fixtures pass; native install/update/repair refusal and rollback
tests have been added but require exact-candidate native execution.

SCRUM-212 now implements an explicit unavailable local-peer boundary instead
of inventing a new pairing protocol. Inbound application startup and outbound
UI/model paths are being contained consistently across XNav, Legacy and Safe.
Stock OpenCPN and its isolated server-model tests remain unchanged. This is a
subsequent change requiring separate qualification; it has not been deployed.

At local `f5efdb3`, the reviewed nine-patch Linux integration passes 140/140
tests (22.93 seconds), all five mode-cycle startups and actual CLI peer-key
refusal checks. Later installer/display test increments require their own
candidate qualification; these results do not imply native or boat acceptance.

SCRUM-208/209/211 remain launch blockers. The maintained Windows dependency
chain now has separate source builders for OpenSSL 3.5.9, zlib 1.3.2 and curl
8.22.0, joined only in the disposable integration build. Source hashes, licenses,
manifest consistency, installed DLL hashes and the absence of legacy OpenSSL
runtimes are package gates. zlib's generated wrapper passed 14 upstream Linux
tests; PowerShell manifest interoperability and tamper rejection passed. These
checks do not qualify the native Windows binaries or installer lifecycle.

The actual pinned Downloader accepted an untrusted owned test certificate.
The integration patch now requires verified HTTPS for public catalog/plugin
downloads, rejects insecure redirects, stages path downloads and propagates
network failure before plugin extraction. Its real Linux transport harness
passes 12 cases, including both GET/HEAD rejection and stream exceptions.
The actual WXCURL harness now passes 13 Linux cases, and the native trust gate
covers both download implementations. The peer response buffer and allocation
guards pass exact-source tests. Peer authentication/credential remediation is
tracked separately as SCRUM-212; no complete application TLS remediation is claimed. See the corresponding
architecture and evidence records.

The combined eight-patch integration at `3cb637b` passes its Linux build and
136/136 integrated tests (19.66 seconds). The focused Online AIS pointer flow
passes after the semantic capture wait correction `8bdb445`. These are local
development results, not Windows qualification.

The earlier published f752dae→5652c01 candidate remains retained negative evidence. Its prototype capture
failed looking up the Online AIS Enabled control. Downloaded, hash-verified
evidence shows a stale pre-transition diagnostic snapshot; the actual button
is fully visible. The bounded semantic wait correction retains all assertions.
The full Linux run, including three-hour elapsed trip, passed. The full native
run passed both 128-test suites, OpenSSL upstream tests, DPI/touch and charts,
but recovery packaging failed its exact OpenSSL notice-hash check. Installer
and native endurance were skipped. A disposable Git checkout reproduced the
notice failure with `core.autocrlf=true`. Commit `ba30f9b` preserves third-party
notice bytes using Git attributes; the regression check confirms the original
OpenSSL/curl/zlib license hashes and retains a failing unprotected control.
The package hash requirement remains unchanged. The changes above are a subsequent integration
candidate and require their own exact-revision results. These results do not qualify a boat deployment or public release.

## SCRUM-208 / SCRUM-40 / SCRUM-98 bounded candidate evidence — 2026-09-30

Frozen local revision `f752daeb0fc07e2abb878c0c43a4b64760f22101` has a
published candidate `5652c01aa7dc450cdfc081a981945b0cc6f6905d`. The local
integrated suite passes 136/136, and the Auth0 PostgreSQL CI job
`110049568910` in run [36762862390](https://github.com/ThereptileII/Work/actions/runs/36762862390)
passes all 31 tests, including fresh, reverse and reapplied migrations. The
full native run [36762862383](https://github.com/ThereptileII/Work/actions/runs/36762862383)
and prototype run [36762860209](https://github.com/ThereptileII/Work/actions/runs/36762860209)
are not accepted: prototype composition failed at the Enabled-control lookup,
and the full native run failed its notice-hash packaging gate. The three-hour
Linux run and native prototype object-flow job passed. No release or native
acceptance is inferred.

The source-publication proof maps all 1,289 prepublication entries and
preserves eight unrelated entries (proof SHA-256
`f3fe14b1c93a06d71096a1774bb2a9dd7f3ac21aebb5a1ca6d0fdc84614f897b`). The
PowerShell interop directory is an inert fixture-only check and does not qualify
installed or packaged DLLs. SCRUM-208 remains a Highest security launch blocker.
SCRUM-209's legacy curl/OpenSSL closure remains open, and SCRUM-211's
certificate-validation bypass finding is an additional open security blocker.

The earlier remote run [36742212299](https://github.com/ThereptileII/Work/actions/runs/36742212299)
on `19cf4f70dc4d74789a44b4abe2302241e193d72d` passed its installer step but
failed the retained DPI step; native endurance was skipped. That run's own log
does not print the active DPI scale, so its full-run scale remains unconfirmed
without artifact `11115071251`; the separate retained prototype evidence must
not be transferred to it. Its Linux integrated job subsequently passed, including
the real three-hour elapsed trip; the native failure still blocks acceptance.

## Next combined native candidate — 2026-09-30

The compact rail now fits measured number/unit bounds above status metadata
(`58ce8b2`, `3092417`), addressing the retained 150% overlap. The primary
prototype sizes remain where they fit. Native screenshots are still required;
local syntax validation does not qualify the design.

The OpenSSL source-build boundary is merged (`3cbd7e5`, `bb294f3`). Package
validation (`789cdbf`, `fbc991a`) now checks the actual Windows build manifest,
installed and final packaged DLL hashes/architecture, source archive and
notices. It includes the unchanged verified source tar in corresponding source.
Focused checks pass: 8 OpenSSL package, 11 source archive and 6 AIS probe cases.
The source-build compiler/upstream-test and native regression gates are pending.

SCRUM-209 separately tracks the confirmed legacy curl closure: the current
dependency bundle carries libcurl 7.58.0 importing OpenSSL 1.0.2n. Its maintained
replacement, zlib and trust-policy qualification remain launch blockers.
Updating the major-version-3 libraries alone does not close that issue.

## Account session foundation — 2026-09-30

SCRUM-40's Auth0 login/session foundation is merged at `99f6a71` / `745cc09`.
Exact merged revision `4bd4c36dae43b9b2ab9e5039c687f531b6c605d5` passes
31/31 tests on isolated PostgreSQL 18.6: 11 database-integrity and 20 account
cases. The check includes concurrent one-time callbacks, signature and claim
validation, redirect refusal, bounded responses, session rotation, CSRF logout,
and fresh/reverse/reapplied migrations. The disposable database was stopped
and removed. [Evidence](evidence/scrum-40-auth-local.json) records exact logs;
the [session contract](architecture/auth0-session-foundation.md) records the
boundary. CI now includes both suites, but publication/native and staging
acceptance remain separate. Authentication stays disabled by default; real
tenant, recovery/MFA, abuse controls, privacy/security review and deployment
are not complete.

## Native repairs and security dependency review — 2026-09-30

The pointer-scroll activation repair is merged at `61cdd4c`: it waits for the
actual foreground window within the existing deadline, then rechecks geometry,
window identity and pointer obstruction. Focused delayed/denied activation,
overlay and movement cases pass; native Windows verification remains open.
The horizon row-stretch correction is merged at `fa9a595`. Its isolated Linux
component passes 232 checks, including Return action-identity cancellation.
The strict Linux HTML comparison still fails on platform-specific row height;
the corrected 53-pixel native height matches the retained Windows reference,
but only a fresh Windows comparison can qualify it. See
[retained component evidence](evidence/scrum-100-row-stretch-local.json).

SCRUM-208 is a Highest security launch blocker: retained Windows dependency
DLLs identify as OpenSSL 3.0.5, with no reviewed backport evidence. Official
OpenSSL 3.5.9 source has passed published checksum and isolated signature
verification; a source-built Win32 integration replacement is in progress.
No installed/boat DLLs have been changed. Native TLS, package/import, plugin,
installer and exact-source qualification remain required. The existing stock
OpenCPN compatibility allowlist has not been expanded.

## Active native candidate and database foundation — 2026-09-30

Local `f2152da67f9e6525fd0d5dcc28d127266ccca879` is published as
`19cf4f70dc4d74789a44b4abe2302241e193d72d`. Its [full native/Linux run](https://github.com/ThereptileII/Work/actions/runs/36742212299)
passes the standalone contract suites: 86/86 Windows and 89/89 Linux.
The [prototype run](https://github.com/ThereptileII/Work/actions/runs/36742210448)
passes the native footer component (198 checks), but its strict horizon
comparison fails on 45-pixel event controls versus the HTML grid row's 53 pixels.
The retained 150% DPI suite also fails on immediate foreground activation
observation before scrolling; the pointer hit is inside the intended viewport.
Both verified archives and [exact negative evidence](evidence/scrum-98-native-19cf-negative.json)
are retained. Neither run has overall acceptance. The integrated Linux build previously passed 136/136
upstream tests. Those counts describe different test suites, not a combined
release result. The branch/commit concurrency correction `267c72d` is local and
awaits publication; it does not undo the preceding run's automatic cancellation.

The reviewed SCRUM-94 database schema and CI definition are merged through
`d0105a1`. Its exact local PostgreSQL 18.6 loopback-TCP check passes all 11
integrity cases, migration drift, fresh migration, reverse to 0001, and reapply
through 0003. The disposable cluster was stopped and removed. See
[exact schema evidence](evidence/scrum-94-schema-local.json) and
[database contract](architecture/web-database-foundation.md). CI, backups,
restore and staging gates remain open. SCRUM-40 now owns the bounded Auth0
login/session implementation; no public service has been deployed.

## Next native verification candidate — 2026-09-30

The selected fullscreen test correction is implemented in `882d497`; the
unchanged real pointer checks now get additional scroll-occlusion evidence
(`1634d5d`). Full passage maps only to the supplied Day Passage reference
(`b7e768c`); reprocessing the verified native captures produces all 34 exact
comparison sets, still pending visual acceptance.

The shared custom button now handles Return before wxMSW dialog navigation
without claiming Tab or arrow keys (`c270502`). Its isolated Linux component
passes 164 checks including actual pointer, normal/held Enter, Space, disabled
input and Tab to a real destination. Initial local failures were a Wayland/Xvfb
environment mismatch, retained separately; the corrected diagnostic uses the
same X11 environment as the existing harness. See [keyboard evidence](evidence/scrum-98-footer-keyboard-local.json).
A new exact native Windows run is required. The 150% Preferences obstruction
is instrumented, not yet diagnosed or repaired; no acceptance is inferred.

## SCRUM-98 / SCRUM-100 replacement native evidence — 2026-09-30

Exact local `491c29b` maps to remote `11a97233fa7ad4d1b83e346642c9bc75dd0a30d9`.
The local integrated build/install and 136/136 tests pass. The native integrated
build and 128/128 tests pass, then actual pointer input correctly rejects
Source health: its midpoint hits the Windows taskbar clock. The harness forced
a decorated 1280×800 frame over a 1280×800 desktop's reserved taskbar area.
The selected repair uses OpenCPN's supported fullscreen startup for that exact
test desktop, retaining real input, containment and chart/layout assertions.

Two separate prototype failures remain: the footer component's Enter callback
assertion (after 185 checks), and the retained DPI suite's covered Preferences
scrolling surface at 150%. The latter completed 100/125%; the additional
prototype-only DPI suite passed all three scales, which does not override the
failed suite. Their causes are not inferred from the navigation taskbar result.
The Full passage screenshot also needs its existing HTML Passage state mapped
into comparison. Original HTML and comparison strictness remain unchanged.

[Verified archives and exact negative evidence](evidence/scrum-98-native-11a97233-negative.json)
retain all three failures. Prototype run
[36726644584](https://github.com/ThereptileII/Work/actions/runs/36726644584)
finished with six successful and two failed jobs; full run
[36726657879](https://github.com/ThereptileII/Work/actions/runs/36726657879)
had not completed its elapsed-time endurance gate when the later push for exact
candidate `19cf4f70dc4d74789a44b4abe2302241e193d72d` reused the branch-only
concurrency group and automatically cancelled that run's remaining endurance
step. This workflow now scopes concurrency by branch and exact commit, while
still cancelling duplicate runs of the same revision. Future validation must
observe a separate candidate run through its terminal endurance result; the
original intermittent exit-255 failure remains open. No release or boat
acceptance is claimed.

SCRUM-31's [web architecture decision](architecture/public-beta-web-commerce.md)
is complete at `95d5a12`. SCRUM-94 now owns the next bounded database-schema
increment; authentication, payment services, deployment, backups and staging
remain separate implementation/acceptance work. Public access stays closed.


## SCRUM-98 native negative evidence — 2026-09-30

The exact local revision `8eaaba6a3268f61c5d7cd89bd8825f78464c6156` was published as remote revision `32174cfb716a77c75b03efa0ac368d3e04ce9231`. Full CI run [36615777880](https://github.com/ThereptileII/Work/actions/runs/36615777880) recorded 13 successful jobs, two failures and one skipped job. Linux job 109568284851 failed to compile `TestEarlyStartupTrace.h:100` because the fortified write triggered `warn_unused_result` under `-Werror`.

Windows job 109570349159 built and passed all 128/128 unit tests, then `tools/smoke-navigation.py` failed in `footer_health_action` to find a fully visible `Source health` pointer target. The screenshot shows the target rendered; obstruction, parent geometry, and hit-test rejection are unknown, so the root cause is unproven. Release remained skipped and the original exit-255 failure remains open.

The focused compile fix `84c7c13` covers 11 terminal-only scenarios per policy; no logs were retained and native verification remains pending, so it is not a broad pass. Native artifact evidence, including the 1,175,724-byte ZIP with SHA-256 `2d5e7336b669a243cc2f2dd176495ecc679b43d81dc2fad32a7f35b32b6f9027`, CRC/path verification, and scoped publication proof, is recorded in [SCRUM-98 native evidence](evidence/scrum-98-native-32174-negative.json). No accepted or Done claim is made.

## Current stage: Jira-driven paid public beta

[Navigare / SCRUM](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the sole backlog. The complete [public-beta contract](public-beta-contract.md)
now governs the application, distribution, website, commerce, support and
release gates. The approved public identity is SKAGER / SKAGER App; domain
control and legal clearance remain separate gates in SCRUM-89. Public payment
and download access remain closed pending the final human GO/NO-GO review.

The reviewed Windows marker-reader and horizon-header repairs are committed in
`2345abe`, combined with Linux-only, opt-in startup observation in `4b1f301`.
The actual isolated diagnostic application builds successfully; all three
observation-policy suites pass eleven scenarios each. One marked-file preview
run records 44 captures and seven successful process exits, with complete
initialization/parser/log stages for every process. This verifies the diagnostic
but does **not** close the original intermittent exit-255 failure. The source
revision, precommit embedded build label and binary hash are recorded separately
in [diagnostic evidence](evidence/scrum-98-startup-observation-local.json).
The [trace contract](startup-observation.md) preserves normal product and native
Windows entry behavior. Fresh native Windows reader, component, DPI and
installer gates remain required; SCRUM-98 stays In Progress and SCRUM-100 stays
Testing. Existing CI endurance runs are preserved on their own branches.

The combined portable build caught a further test-reader compilation defect
before publication: GCC 16 defaults to C++20, whose `u8string` element type
cannot concatenate directly with `std::string`. The reader now preserves the
UTF-8 bytes explicitly, with a non-ASCII path regression. Both explicit C++17
and C++20 builds pass 74 marker checks, and all 88 configured portable suites
pass. Native sharing semantics remain pending; no failed candidate ref was
published. See [failure and repair evidence](evidence/scrum-98-marker-utf8-local.json).

Exact replacement `ced99ac379e5622c21e48c83772b43aa7f2e8df6`
(equivalent local `3fb6635`) is **not qualified**. Its Windows portable suite
passes 84 of 85 tests, then fails the concurrent fixture-marker reader; a
separate passing 376-check/24-case process invocation does not waive that
failure. Test-only readers now allow Windows DELETE-handle sharing while
continuing to deny open writers; native execution remains pending. The native horizon screenshot
fixture also fails to compile because its Windows branch lacks the explicit
`wx/dcmemory.h` include; that narrow correction requires a replacement build.
See [retained marker evidence](evidence/scrum-98-marker-sharing-negative.json).

The same local source passes all 136 integrated tests and its first mode cycle,
selected-navigation interactions, recording, both route styles and public ENC
checks ([exact local summary](evidence/scrum-98-integrated-3fb6635.json)).
The portable preview then reproduces Legacy → XNav failure. The new
pre-teardown evidence observes the replacement child exiting 255 before a third
startup/window, after Legacy exits successfully. The cause remains under
early-startup investigation. One bounded process trace passes with all seven
application exits successful; that timing-altered diagnostic does not repair or
supersede the [original failure](evidence/scrum-98-preview-3fb6635-negative.json).
See the [limited diagnostic result](evidence/scrum-98-preview-3fb6635-diagnostic.json).
Follow-on pilot, production and Full passage checks stopped at the first
failure. The concrete failures remain retained; implementation status is
recorded above without transferring acceptance across revisions.

Replacement `701149f` (equivalent local `826a44f`) remains **unqualified**.
The local integrated build passes 135 tests; recording now observes the exact
1280×34 footer. Its mode cycle, selected navigation, both route styles and
public ENC software/OpenGL checks pass. The portable preview subsequently
fails Legacy → XNav after 39 captures: Legacy saves the requested mode and
exits, but no third startup is logged. The original failure remains retained;
the restart child has not yet been conclusively diagnosed. Local OpenGL uses
llvmpipe and does not qualify the boat GPU.

The same exact native candidate passes its fixture-free MSVC build but exposes a separate
Windows commissioning-fixture sharing failure while reading a child readiness
file. The fixture made its final name visible before closing the writer. The
replacement publishes checked, closed markers atomically and adds 70 portable
checks plus native exclusive-handle assertions; production restart guard code
is unchanged. The actual failed OS lock holder was not captured, and Windows
replacement remains mandatory. CI Linux also fails a Chart transition in the
mode harness; that evidence is being investigated separately. See the
[marker development record](evidence/scrum-98-marker-publication-local.json).
These failures are not waived by the passing component or local mode tests.

The terminal `701149f` prototype run has six passing/two failing jobs; its full
run has eleven passing/three failing/two skipped jobs. All 19 archives verify.
Windows integrated suites pass 127 tests; the actual status-only product probe
records zero outgoing bytes. The standalone footer fixture fails on an entirely
blank image, while the actual product footer and coastline render. Both footer
and horizon fixtures now explicitly size their parent host and record native
parent/child/hit geometry before the original assertions; Linux passes
153/208 checks. The retained 150% DPI test also used an 80-DIP rail minimum that
conflicts with the immutable compact layout's 72-DIP rule. Its replacement uses
that exact breakpoint; four-value visibility, clipping, touch and alarm checks
remain. The rest of the 150% suite remains unexecuted. See
[exact terminal evidence](evidence/prototype-native-701149-development.json),
[host-fixture evidence](evidence/scrum-99-component-host-local.json) and the
[compact-layout oracle](evidence/scrum-98-dpi-compact-oracle.json).

One instrumented repeat of the portable preview passes all seven process exits
and 44 captures; the original restart failure remains unresolved. The preview
harness now retains per-launch output and pre-teardown process/window evidence.
No retry is added and no successful repeat is presented as a product repair.

Candidate `88b7bbc` fails qualification: both prototype MSVC jobs stop at the
new footer component's missing console entry point, and Linux recording stops
on a real 1279px footer inside a 1280px frame. Its 135 integrated Linux cases
and 83 Windows portable suites pass; native component captures were not reached.
The narrow entry-point repair follows the other component executables and
passes 98 local checks/ten captures. Independent wxAUI probes reproduce the
fixed-dock trailing spacer; the replacement makes only the footer proportional
and scopes/restores XNav's zero sash metric through theme changes. The recording
gate still requires the full 1280px width. Replacement native evidence remains
mandatory. See [retained failure evidence](evidence/prototype-native-88b7bbc-development.json)
and its [terminal archive record](evidence/prototype-native-88b7bbc-final-development.json).

The same local binary passed XNav → Legacy → XNav and Safe startup, then failed
its final normal startup once. One instrumented repeat passed; the original
failure remains unresolved/intermittent. Mode gates now retain every launch's
stdout/stderr, early exits and failure window/process inventory. No recovery
fix is inferred from the repeat. The retained prior installer also exposed
fixture rejection being preempted by the output-policy rejection. Both guards
remain mandatory; fixture identity is now checked first with strict boolean
typing, and 48 actual PowerShell validator checks pass. Full installed lifecycle
and replacement Linux/Windows gates remain open.

SCRUM-100 implements the prototype horizon's measured fractional columns,
heading, separators, Full passage and guarded contextual actions. Its owned
model passes 81 checks, native Linux component passes 132 checks/14 captures,
and eight related regressions pass. Day/Dusk/Night reference/current/diff
retain visible Linux text differences. Fresh/stale position controls NOW;
route revision and local AIS identity are revalidated at activation. New
integrated pointer checks remain pending with the combined source. Windows,
DPI and boat conformance are open; no Navigation screen is accepted. See the
[horizon review](design/reviews/scrum-100-horizon.md) and
[development evidence](evidence/scrum-100-horizon-local.json).

SCRUM-19 now enforces status-only XNav equipment output at the final transport
boundary, independent of saved permissions. Passive pilot feedback remains
available. The separate non-installable command-test build is restricted to a
verified local TCP peer. Package/install/update checks require an executed
status-only declaration; exact historical rollback/repair remain recovery-only
and explicitly unqualified. Portable denial checks and 27 actual installer
policy checks pass. Linux replacement passes 134 integrated tests, the actual
product probe records zero outgoing bytes despite saved permission and fresh
compatible feedback, and 94 component checks produce nine captures. Visual
review found and corrected a misleading enable-control promise: the product
now visibly says status-only / equipment control unavailable in Day and Night.
See [local policy and presentation evidence](evidence/scrum-19-status-only-local.json).
Native Windows replacement remains mandatory.
Stock OpenCPN/plugin output remains independent and still requires the boat
profile/plugin audit. See the [hardware boundary](st4000-beta-contract.md#public-beta-output-restriction--scrum-19).

Prototype conformance, chart presentation and Online AIS remain in progress
under SCRUM-14/15/16. SCRUM-19 addresses unqualified XNav physical output;
SCRUM-98 tracks the native installer/chart and DPI gate repairs. SCRUM-97 has
committed the Jira workflow and project-goal changes, but its required complete
issue-to-acceptance cycle remains open. No issue is accepted from compilation
or component checks alone.

SCRUM-99 replaces the unrelated navigation footer with the native HTML status
bar: exact three-group composition, owned position/course, navigation quality,
and a source-health action. No cross-track distance is invented; XTE stays
unavailable. Recovery remains visible through Settings → System → Interface
& recovery, plus the existing Ctrl+Shift+S shortcut. Focused Linux validation
passes 44 footer provenance assertions, 49 retained source-health assertions
and 98 component/input checks with ten actual captures. The combined source
passes all 83 portable suites. Exact reference/current/
diff crops retain Linux font-metric differences; Windows and boat conformance
remain pending. See the [footer review](design/reviews/scrum-99-status-footer.md).

The `4b9d134` native replacement passes both 126-case MSVC suites and the
fixture-free product's actual zero-output pilot probe. Its prototype DPI gate
then finds two System captions: the footer action and the page heading. The
retained 100% frame has all four rail values and eight recovery actions visible;
it is not a completed DPI gate. SCRUM-99 now exposes and tests the distinct
footer panel geometry instead of inferring it from that caption. Replacement
native runs must still pass; no layout, chart, touch or persistence check is
waived. See [retained exact native evidence](evidence/prototype-native-4b9d134-development.json).
The current combined candidate also includes the status-only wording
correction. The older local fixture rebuild was deliberately stopped before
integration to avoid qualifying an intermediate revision; it is not counted as
a passed build.

Source Health now uses the prototype disclosure sheet with independent onboard
and Online AIS status, per-measurement quality and protected AISStream key
settings. Incomplete GPS pairs and estimated readings do not appear as healthy
measured connections. See the [Source Health review](design/reviews/prototype-source-health-in-progress.md).

Native `f934f3b` / prototype run `36536723961` finishes seven jobs passed and
one failed. The full run `36536897649` finishes fourteen passed, one failed and
one skipped; publication is withheld. Linux product and fixture suites each
pass 133 tests; their MSVC counterparts each pass 125. Native evidence includes
151 own-key checks/twelve captures, 49 Source Health checks/six captures,
33 primary product captures plus four Online AIS captures, and 19 retained
preview groups/50 captures. Linux completes the actual three-hour endurance
run. Windows endurance was not reached.

The installer passes eighteen checks before applying XNav chart colors to
Legacy. Retained pixels show real coastline in the pinned Standard palette;
the independent exact Standard oracle passes. The replacement checks each
mode against its own independently defined palette and adds the actual
XNav-to-Legacy transition capture. Its portable suite increases to 54 exact
palette checks, including blank, wrong-style and wrong-light rejection, while
retaining eighteen geometry rejection checks. The DPI gate confirms the exact
Preferences scroll endpoint at 100%, then encounters outdated action captions.
The replacement uses the current drawer actions and reopens Settings for each
destination. A further correction requires actual touch scrolling and a fully
visible, exact native hit target for Battery & reserve; all nineteen observation
and touch-helper regressions pass locally. Full native replacement at 100/125/150% and the installer lifecycle
remain mandatory. No failure is waived and no boat deployment is accepted.
Six downloaded archives are hash/size/CRC/path verified. See the
[exact native development evidence](evidence/prototype-native-f934f3b-development.json).

Earlier `60a9e8b` / prototype run `36532671500` finishes seven jobs passed and
one failed; its full run `36532672143` is also terminally failed. The earlier
Preferences observation failure and all retained negative evidence remain in
[native development evidence](evidence/prototype-native-60a9e8b-development.json).
The later `f934f3b` evidence above supersedes the pending-run status, without
transferring acceptance across builds.

The equivalent local `60a9e8b` tree passes 132 product tests, 132 fixture tests,
151 AIS component checks/twelve captures, all route gestures, and eight preview
checks/44 captures without extra X11 focus helpers. The corrected late-resize
guard remains restricted to visible XNav transient surfaces. See
[local corrective evidence](evidence/prototype-transient-raise-local.json).

Native `d8316d8` / run `36530182434` passes the complete 29-state product
capture, AIS own-key checks, software ENC unit/palette checks, and development
100/125/150% probes. Seven jobs pass; the retained fixture test still selects
the duplicate Configure instruments action. All nine archives verify. No
visual conformance or deployment acceptance. See [development evidence](evidence/prototype-native-d8316d8-development.json).

Native `28b5e9c` passes 151 AIS component checks and all twelve captures,
including own-key Day/Night, cancel, replacement, failure and removal. The
separate protected import suite passes seventeen checks. All eight prototype
archives verify; six jobs pass and two fail. The product Off screenshot confirms
the action while its preceding diagnostic still says enabled. Replacement input
checks await the semantic result. The fixture Preferences transition remains
on Sensors; the replacement waits for the destination page and activates only
the exact target-owned native surface before clicking. Neither failure is waived.
The subsequent `7018c7f` full run passes both corrected 78-test portable suites;
its integrated gates finish with the failures below. No deployment qualification or visual
conformance PASS. See [native own-key evidence](evidence/ais-own-key-native-28b5e9c.json).

Full run `36528305413` / `7018c7f` passes 132 Linux and 124 MSVC unit
regressions, but qualification remains negative. Windows chart gestures pass;
the preview chooses the rail's repeated Configure instruments caption, and the
DPI context check counts floating toolbar actions as context actions. Exact
product-control and accessible-identity selectors replace those assumptions.
The local fixture preview passes eight checks/44 captures after owned-surface
focus settling. Pointer evidence traces the Linux Passage library failure to the underlying
timeline during OpenCPN’s one-second deferred resize raise. A narrow transient-
surface guard now passes the complete local route gestures; replacement
qualification is still required. Legacy/Safe retain their original raise.
See [retained negative evidence](evidence/prototype-native-7018c7f-negative.json).

The user-supplied AISStream key workflow now has explicit own-key guidance, a
masked 48px input, disabled empty/invalid Save, and no automatic enablement.
Cancel/Escape never replace a saved key; confirmed removal disables Online AIS.
An Escape defect in modal/drawer dispatch was found and fixed. Linux passes
151 AIS component checks/twelve captures, 132 product and 132 fixture tests, and retained
Pilot/Passage/Settings component regressions. All tests use fake credentials.
Native key-entry checks pass; broader replacement gates remain open. No key is distributed with builds.
See [credential UI contract](online-ais.md) and [corrective evidence](evidence/ais-own-key-ui-local.json).

The first real AISStream service probe on the boat passes: subscription confirmed,
41 peak targets, 54 accepted reports, zero rejected; clean disable/cache clear.
The user-authorized credential is verified in the interactive user's Windows
Credential Manager. A bounded local pipe crosses the SSH/desktop logon boundary
without a plaintext key file, key argument, key log or remote-access change.
Thirteen portable import checks and seventeen native checks pass. The isolated
probe uses qualified `c86c6a7`; it does not launch OpenCPN or qualify the latest
product UI. Live target selection, internet-loss/reconnect and physical display
review remain open. See [live evidence](evidence/boat-ais-live-c86c6a7.json).

Native `e83df7d` passes the nine component suites, including Radar's 64 checks
and Pilot's 77. The product composition check stops because Radar's diagnostic
screen coordinates were compared directly to client-relative HTML coordinates
(8px frame / 31px caption offset). Reviewed native pixels retain the scope.
The replacement normalizes by the measured client origin and retains the exact
one-pixel geometry bound. Its Linux corrective capture passes; replacement
Windows geometry and full qualification are still required. The completed full
run also rejects stale user-flow selectors and a Safe chart check expecting
XNav colors. Downloaded Safe pixels show stock coastline content; replacement
uses the existing pinned stock palette oracle. The fixture Display check
mistook a nested Configure instruments action for the shell action. All 21
archives verify; no failure is waived and deployment remains withheld. See
[exact negative evidence](evidence/prototype-native-e83df7d-in-progress.json).
No conformance PASS.

Radar now has a native prototype focus composition, with a fixed scope and an
independently scrolling control column. It explicitly withholds returns,
heading/range and scanner controls without a validated receive/control source.
Linux corrective captures pass 64 component checks, both integrated suites
pass 132 tests, and the actual product captures all three Radar themes. The
first layout recursion and subsequent scope/gradient differences are retained
and corrected. Review then exposed a blank Night scope during whole-page theme
rebuilds. The persistent component now passes exact fixed-palette interior
checks through the application theme cycle. See [Radar review](design/reviews/prototype-radar-in-progress.md)
and [local evidence](evidence/prototype-radar-local.json). Windows/boat pending.

Native `a1077b0` / prototype run `36520947790` passes 7/8 jobs, including
124 MSVC tests, 53 Alert checks, corrected Pilot modal repaint, software ENC
comparisons and development DPI. Full qualification `36520970749` passes
13 jobs but stops in Linux/Windows loopback pilot tests still addressing the
old full-page controls. A separate Windows preview input assertion also stops
the retained fixture run. All 22 artifacts verify; deployment is withheld.
See [negative qualification evidence](evidence/prototype-native-a1077b0-failed.json).

Updating the loopback test exposed a real clipped safety-confirmation dialog.
The correction sizes it from the application owner and wraps all text to its
available width; 77 component checks cover consent/cancel/mode confirmation.
The local actual OpenCPN TCP pilot loopback again passes all command/feedback,
timeout, stale Standby and reconnect checks, with no physical hardware. Chart,
plugin and endurance harnesses now target prototype entries, retaining their
existing navigation, performance and resource assertions. Replacement native
qualification remains mandatory; no screen or live AISStream gate is accepted.

Alerts now uses the prototype notification drawer, retaining the real chart and
existing episode-specific acknowledgement semantics. No illustrative encounter
is fabricated. Linux passes 132 integrated tests and two 53-check component
runs; the corrective run verifies removal of native grey button corners.
The replacement application pass retains 26 primary and four AIS-settings
captures. Fixture regressions pass all eight check groups and 44 captures,
including readable GPS acknowledgement amid other critical conditions and
mode/persistence restarts. Native Windows and boat conformance remain open. See the
[alert review](design/reviews/prototype-alerts-in-progress.md) and
[contract](alert-presentation-contract.md).

Pilot replacement `0ce6835` / run `36518416053` finishes 7/8 jobs; both MSVC
suites pass 124 tests. Actual native composition, software ENC/units and
100/125/150% development interactions pass. All nine archives verify. The
retained preview incorrectly assumes 800px client height in a captioned window;
replacement checks apply the exact HTML rules to measured client geometry.
Reviewed Pilot component images also expose incomplete repaint after modal
confirmation despite green interaction checks. The replacement invalidates
visible owned children after modal destruction and requires all labels in
captures. No visual or boat PASS. See
[negative native evidence](evidence/prototype-native-0ce6835-failed.json).

Native `403a4e8` / run `36515856835` passes both MSVC 120-test suites and
six component suites, including Anchor's 34 checks. Six of eight jobs pass.
The capture guard rejects the new Anchor owned surface, and retained preview
expects Instruments to cover its intentionally retained timeline. Downloaded
native evidence identifies both assertions; replacements recognize the exact
owned header signature and exact responsive timeline boundary without relaxing
occlusion or geometry. Eight archives and 109 PNG hashes verify. See
[negative evidence](evidence/prototype-native-403a4e8-failed.json).

The native prototype Autopilot drawer now replaces the old full-page controls.
An owned presentation boundary requires measured magnetic pilot feedback and
retains explicit enable/confirmation, pending/timeout and manual-Standby rules.
Linux passes 132 integrated tests and two 61-check/seven-image component runs.
The initial actual-product Day capture is incomplete; a corrected capture gate
requires title and all eight labels and awaits semantic page publication. Its
replacement retains a complete actual drawer in all three themes. Fixture
regressions pass all eight scenarios, 44 captures and mode/persistence restarts.
Exact native Windows replacement and boat review remain pending.
See [pilot review](design/reviews/prototype-pilot-in-progress.md) and
[control presentation](pilot-presentation-contract.md). No boat commands sent.

Anchor's completed Linux fixture run passes 128 tests, five shared-profile mode
phases, selected navigation input, 26 route checks, 26 object captures and all
eight preview scenarios with 44 captures. See
[retained regression evidence](evidence/prototype-anchor-regressions-local.json).

Native `314e0677` / run `36513102807` passes both 115-test MSVC suites,
113 Preferences component checks and the exact 100/125/150% development drawer
probe. Downloaded Windows pixels show complete tab labels. Six of eight jobs
pass overall: composition stops on an old diagnostic publication after Close
(the retained image shows the restored chart), and the broader fixture test
hits Windows' taskbar over System. Replacement harnesses wait for the semantic
close publication and leave desktop room around the unchanged 1280×800 capture.
No assertion is waived. Eight artifact digests/CRCs and 108 capture hashes are
retained in [negative evidence](evidence/prototype-native-314e067-failed.json).

The prototype Anchor drawer is implemented locally using owned OpenCPN watch
observations. New provenance/history and pinned-Mercator projection tests bring
the production Linux suite to 128/128 (31.17s on the corrective pass); the isolated native component
passes 34 checks and five captures, including confirmation cancellation and
stale GPS. Two integrated passes each retain 24 product captures; the second
confirms all three Anchor themes and Close restoring the chart. The fixture
build and full lifecycle/input regressions pass; native
replacement captures remain required. See the
[anchor review](design/reviews/prototype-anchor-in-progress.md) and
[data contract](anchor-presentation-contract.md). The boat is reachable with
OpenCPN closed; no prototype deployment, hardware output or visual acceptance.

The compact Preferences correction passes exact drawer rectangles against the
independent HTML at all four Linux resize checkpoints. Production and fixture
suites pass 123/123 (19.74s and 19.25s); 45 local component/product/layout PNGs
are retained across the text and workspace corrections. Windows tab painting
now uses the same natural DirectWrite metrics as its layout. Native replacement
and boat review remain required. See [correction evidence](evidence/prototype-drawer-correction-local.json).

The wider Linux fixture regression now passes with prototype navigation:
44 screen captures, all eight failure scenarios, alert acknowledgement/recovery,
and repeated XNav/Legacy/Safe restarts preserve seeded navigation/configuration.
The integrated suite passes 123/123 (19.56s). Thirty exact chart-ink checks and
23 status-summary contracts retain fail-closed coverage while replacing obsolete
Beta menu/bottom-toolbar assumptions. Updated native mode, fixture, recovery and
DPI harnesses remain pending execution; no test gate was removed. See
[regression evidence](evidence/prototype-regression-harness-local.json).

Native `e0b6a16` / run `36510429598` passes seven of eight jobs: both integrated
suites 115/115, five component suites, four exact route-ink cases and the new
100/125/150% development interaction probe. All eight artifacts, 88 native PNGs
and 144 reference PNGs verify. The main capture fails because older canonical
metadata lacks the newly checked tab selector. Reviewed pixels also show clipped
tab labels and incorrect compact Preferences bounds. Replacement code paints tab
text with matching DirectWrite metrics and uses measured compact drawer geometry;
independent geometry assertions remain strict. This is not visual acceptance.
See [negative native evidence](evidence/prototype-native-e0b6a16-failed.json).

The lower vessel-profile group now completes the measured sidebar spacing:
Settings matches all three reference positions within one raster pixel, the
32px profile action opens Vessel settings, and the short desktop layout hides
it as the HTML does. Missing vessel identity stays unavailable. Two corrective
Linux passes retain 66 images; fixture-free and fixture-enabled builds each
pass 123/123 after the SDK identifier correction. Capture policy now passes
218 checks and admits only the exact optional profile action. Windows and boat
review remain pending. See [local evidence](evidence/prototype-profile-rail-local.json).

The native replacement `72df728` / run `36509226412` stops during both MSVC
integrated builds: the SDK `small` macro collides with a compact-pilot local
variable after DirectWrite headers are included. Six other jobs pass; all eight
artifacts verify. The correction renames the variable without changing SDK
definitions or UI/data behavior. No runtime/DPI/typography success is inferred.
See [failed native build evidence](evidence/prototype-native-72df728-failed.json).

Preferences text-layout correction is ready for native replacement validation:
Windows uses fractional DirectWrite advance measurements to retain the HTML's
six-tab first row; independent capture checks all eight exact rectangles. Linux
rebuild/install passes 123/123 and the Settings component 102 checks; 12 component
and 21 actual-product images are retained. No Windows font PASS is inferred from
that Linux result. See [local evidence](evidence/prototype-preferences-text-local.json).

Run `36506983264` / `8e73ed0` is retained as a negative development run (7/8
jobs). Its new actual-DPI probe exposes the old rail losing Radar at 150%; the
compact correction follows in `8b6eab7`. The 125% Settings target is also too
small, so the replacement gate uses each measured prototype height rather than
a shared lower bound. A separate Preferences Close capture failure remains
unproven: the harness now verifies/activates the specific native input surface
and retains failed-state evidence without retrying input. All eight artifacts
and fourteen DPI images verify. See [negative evidence](evidence/prototype-native-8e73ed0-failed.json).

The replacement Windows development run `36506092362` / `f13075a` passes all
eight jobs: both integrated suites 115/115, five component suites including
102 Preferences checks, 50 native capture-guard cases, actual product captures,
object workflows and all four unchanged exact active-route ink cases. All nine
artifact hashes/CRCs, 87 native PNGs (including route images) and 120 reference
PNGs verify. This closes the fixture repaint and rail/catalog harness failures;
it does not qualify the complete product or boat. Preferences tab wrapping still
differs visibly from the Windows HTML. See [development evidence](evidence/prototype-native-f13075a.json)
and [preceding negative run](evidence/prototype-native-466d1c6-failed.json).

The compact-shell correction now follows measured HTML desktop breakpoints.
Before correction Settings was compressed at 1024×640 and absent at 853×533.
Both corrected Linux resize passes preserve the chart and all four rail values;
the second also reaches lower Preferences actions by actual scrolling. Returning
to 1280×800 restores primary geometry. There are 20 retained compact captures,
21 canonical product captures and 123/123 passing tests (18.74s).
Actual Windows DPI and boat acceptance remain pending. See
[compact evidence](evidence/prototype-compact-layout-local.json).

Additional responsive prototype references now measure the unchanged HTML at
125/150% equivalents in twelve physical 1280×800 captures. A separate native
Windows development probe will test actual DPI, visible rail controls/readings,
chart content and touch-operated Preferences/theme/Close in the fixture-free
application. The existing full release suite stays mandatory. Linux rebuild
passes 123/123 in 20.17s and all five design-contract tests. Compact-shell
migration is implemented locally; Windows execution and boat review remain pending. See
[responsive review](design/reviews/prototype-responsive-in-progress.md) and
[reference evidence](evidence/prototype-responsive-reference-local.json).

The Preferences migration now preserves the chart behind the prototype's 432px
drawer, with eight sections and shared native sensor links. Three corrective
component passes retain 36 images (100/100/102 checks); the latest integrated
Linux build passes 123/123 in 20.46s. The isolated route fixture now passes its
26 progress and ten exact Day/software ink checks after the repaint request.
Capture policy passes 216 checks; 50 native marker cases await Windows.
The fixture-free product also passes 123/123 in 19.66s and its actual loader
self-test. Twenty-one product captures pass after scoping the rail identity
assertion to the prototype sidebar (Preferences has its own Radar tab). Two new
marker cases now appear in the fixture's explicit catalog; 217 pure guard checks
pass. See [corrective product evidence](evidence/prototype-settings-product-local.json).
Inline forms, remaining section
content, non-default DPI and boat review are still open; this is not Settings
visual acceptance. See [review](design/reviews/prototype-settings-in-progress.md)
and [local evidence](evidence/prototype-settings-local.json).

Windows development run `36503170626` / remote `6091c23` passes seven of eight
jobs. Both integrated suites pass 115/115; AIS/Passage/Instruments/Energy widget
checks pass 71/26/41/44. Actual-product captures and the corrected object workflow
pass. The new active-route ink check fails on its first Day/software image:
the route is not painted despite valid copied upstream projections. The fixture
omitted the explicit canvas repaint used by OpenCPN's normal route manager;
the replacement schedules that same repaint without requesting navigation
processing. Unchanged pixel assertions and replacement Windows results remain
required. All nine artifacts, 62 native PNGs and 120 reference PNGs verify.
See [negative evidence](evidence/prototype-native-6091c23-failed.json).

The Energy screen's next prototype pass corrects primary card proportions,
numeric hierarchy, battery graphic, forecast rows and power tiles. An owned
presentation boundary withholds predictions from mismatched/stale observations
without inventing sensor values. All 123 integrated Linux tests pass after the
correction; two offline Energy passes each pass 44 checks/seven captures, and
the shared-card Instruments regression passes 41 checks/five captures. Fourteen
new presentation assertions cover provenance and failure states. Lower Explore
pace/calibration UI and native/boat qualification remain open; no conformance
PASS is claimed. See [review](design/reviews/prototype-energy-in-progress.md)
and [evidence](evidence/prototype-energy-local.json).

Windows development run `36500844253` / remote `3a90eff` closes the actual-product
capture refusal: twelve primary and four additional guarded captures pass.
Both native integrated suites pass 114/114; AIS/Passage/Instruments components
pass 71/26/41 checks. Seven of eight jobs pass overall. The object-flow test
reads a previous Route detail diagnostic publication immediately after Settings
returns to Navigation; its screenshot shows the correctly restored chart and
North control. The corrected harness waits for a later Navigation publication,
then applies the same strict geometry/chart assertions. Its Linux object flow
passes, including both settings-return cases and persisted navigation objects.
All nine artifact hashes/CRCs, 55 captured PNGs and 120 reference PNGs verify.
Native route-ink tests were not reached; replacement Windows and boat gates
remain pending. See [evidence](evidence/prototype-native-3a90eff-failed.json).

The current active-route presentation increment copies the immutable prototype's
Day/Dusk/Night route ink into OpenCPN's existing software/GL route rendering.
Standard, selected routes, geometry and progress semantics remain upstream-owned.
Eight Linux render cases pass 26 route-progress checks and ten exact projected
stroke checks each; 16 captures are retained. The rebuilt integrated suite passes
122/122 (19.81s), plus five prototype contract tests and 212 capture-guard checks.
OpenGL uses the pinned renderer's exact RGB/256 conversion. Windows and boat
qualification are pending; broader overlay conformance is not claimed. See
[route-ink evidence](evidence/prototype-active-route-ink-local.json).

Latest Windows development run `36498559587` / remote `42255ef` passes seven of
eight jobs: 114 integrated tests, all component suites and independent ENC captures.
The strict actual-product capture refuses the new rail's unexpected structural
caption. Source inspection of the pinned wxWidgets 3.2.8 confirms default panel
names become native captions. The correction explicitly clears labels only on
the decorative rail divider and drawer containers; capture predicates remain
unchanged. All eight negative-run artifacts and 27 manifest PNGs verify. See
[negative evidence](evidence/prototype-native-42255ef-failed.json). Replacement
Windows capture and boat deployment remain pending.

Replacement `299c728` / run `36496486152` passes the corrected Windows
Instruments rendering check (41 checks/five captures), all 114 integrated tests,
48 native marker-window cases and 212 pure guard checks. Seven of eight jobs
pass. The actual-product capture guard refuses Passage after three successful
navigation theme captures; its child stderr was not retained by the original
CI launcher. The replacement retains that refusal explicitly; no guard has
been bypassed. All eight artifacts and 30 manifest-listed PNGs verify, with
six partial navigation PNGs retained separately. See
[negative native evidence](evidence/prototype-native-299c728-failed.json).
No package or boat acceptance is claimed from this run.

The committed rail/scroll correction `da17dab` has a fresh integrated Linux
configure/build/install and 122/122 passing tests (20.59s). Its Windows
qualification remains pending.

The next native Passage increment now follows the prototype drawer instead of
replacing the chart with a full-page route summary. It copies accepted route and
SmartNav values, matches the exact energy input publication and withholds values
on GPS loss, stale data or revision mismatch. Route commands retain their
existing OpenCPN identity/confirmation gates. Three successful component passes
and two full-product passes retain 41 verified PNGs, including the corrective
icon-background fix. All 121 integrated Linux tests pass after the corrections. Offline component
checks: 26; new provenance assertions: 25. Full product checks include Passage Day/Dusk/Night and Close retaining the
1014×566 chart viewport. Windows development run `36491018722` now passes all
seven jobs (113 integrated tests, 26 Passage component checks); all eight
downloaded artifact hashes/CRCs and 48 captured PNG hashes verify. The exact
local source commit passes 121/121 Linux tests in 19.52 seconds. See
[native evidence](evidence/prototype-native-6a75c05.json). Boat and visual
acceptance remain pending. Saved-passage
and waypoint flows still need prototype migration. See
[review](design/reviews/prototype-passage-in-progress.md) and
[local evidence](evidence/prototype-passage-local.json).

The next Instruments migration is in progress: native prototype wind/heading
and tile composition, retained horizon, copied assessed readings and preserved
instrument selection. Directional graphics require real coherent heading/wind;
COG cannot stand in for heading. All **122/122 Linux tests** pass after the
corrections (19.27 seconds), including 26 new presentation assertions. The
non-installed widget passes 36 checks/five captures; two corrective full-product
runs pass the complete theme/Close cycle and existing AIS settings flow. There
are 42 verified PNGs across the retained corrective runs. The first review's
displaced compass text and unavailable-line collision are corrected and tested.
[Design review](design/reviews/prototype-instruments-in-progress.md) and
[evidence](evidence/prototype-instruments-local.json) record the required
navigation-meaning differences, earlier negative runs and pending Windows/boat
gates. The later rail/scroll correction below removes its older Up/Down chrome.

The Instruments Windows increment `1a175e3` / run `36494702427` is **not
accepted**: 114 integrated tests, object interactions and actual-product/ENC
captures pass, but the isolated Instruments widget capture was blank because
its parent panel was not explicitly sized on Windows. An explicit host sizer
and ancestor-containment assertion correct the fixture; unchanged strict pixels
pass on Linux (41 checks/five images). Replacement Windows evidence is pending.
All seven downloaded artifacts were hash/CRC verified; 38 successful captures
and five negative widget images are retained. See
[failed native evidence](evidence/prototype-native-1a175e3-failed.json).

The boat display-review guard is being qualified for the prototype rail and
known owned surfaces. It retains exact installed executable/session/launch
identity and rejects unrelated or changed windows. Linux policy suites pass;
native marker-window and actual-product capture qualification remain pending.
No new prototype product has been launched aboard, and no actuator action was
added. See [guard review](design/reviews/prototype-boat-capture-guard.md).

A further Instruments correction removes its old permanent scroll toolbar and
restores the exact main-rail button widths/group separation. All 122 Linux tests
pass (19.33s); 16 captures pass exact rail measurements and actual pointer-wheel
access to lower readings/back to top. Windows and boat gates remain pending;
the lower vessel-profile group is still an explicit mismatch. See
[corrective evidence](evidence/prototype-rail-scroll-local.json).

Replacement `c86c6a7` / [run 36485693077](https://github.com/ThereptileII/Work/actions/runs/36485693077)
passes all seven development jobs. All eight downloaded artifacts, their ZIP
CRCs, 29 captured PNG hashes and the probe's 17 binaries were independently
verified. The native PowerShell launcher now records the required negative
exit code 4. An isolated read-only boat probe then ran and shut down cleanly:
`credential_missing`, no subscription and zero targets. OpenCPN remained
closed. This is a negative live-service result, not AIS or product acceptance;
the SSH logon has no usable credential. No key or vessel identity was collected.
See [native evidence](evidence/prototype-native-c86c6a7.json) and
[boat probe evidence](evidence/boat-ais-probe-c86c6a7.json).

At local `820cbf9410870554e93df9555296ef7a852e589d`, a fresh integrated
configure/build/install and all **120/120 Linux tests** pass in 19.90 seconds,
including the corrected scale legend backing. Native depth-label run
`36486349316` and scale-placement run `36487339813` both pass all seven jobs.
All eight artifacts per run and 41 recorded product/component PNG hashes per
run verify. Native Feet/Day, Fathoms/Night and scale Day/Night were reviewed;
unit semantics and scale placement work, while broader chart design remains
open. See [depth native evidence](evidence/prototype-native-7b2a6fd.json) and
[scale native evidence](evidence/prototype-native-ebe7010.json). None of these
incremental gates qualifies the unchanged old boat installation as the new
prototype product.

Latest local chart increment replaces only XNav's large depth-unit emboss with
the prototype metadata typography/placement, using OpenCPN's unchanged actual
quilt/single-chart resolver. Twenty real-ENC captures cover software and
llvmpipe Day/Dusk/Night, all three supported depth units and Standard fallback;
all PNG hashes verify and 120 integrated Linux tests pass. Standard retains its
original emboss. These are development results, not native/boat acceptance.
See [depth-unit evidence](evidence/prototype-depth-units-local.json).

The next local scale-bar pass preserves OpenCPN's distance/unit computation and
moves its legend beside Follow Boat. All 120 integrated tests pass before the
backing-only visual correction; twenty software/llvmpipe/coastline captures
retain their hashes. The first pass exposed an ENC sounding behind the legend;
a small neutral backing corrects that ambiguity. The recorded HTML content gap
and actual painted bounds now pass without overlap. This safety-legibility
exception, native Follow Boat width, chart selector, density and full chart
review remain documented; native/boat acceptance is pending. See
[scale evidence](evidence/prototype-chart-scale-local.json).

Current local chart review: the final HTML cascade now supplies all marker
tokens, including eight roles omitted by extraction from its earlier stylesheet.
The immutable HTML is unchanged; 4 design-contract tests pass. A bounded neutral
Dusk/Night sprite derivation passes 388 resource checks and eight real-ENC
software/llvmpipe captures. Review exposed black cached text in the GL path;
renderer parity and visual acceptance remain open. See the
[ink investigation](design/reviews/chart-ink-contrast-investigation.md).

Native development run `36478655922` at remote `335b14e` (local `bf47479`)
passes 112 integrated tests, 71 AIS component checks, transport/provider tests
and twenty product/ENC captures, but **fails overall**. The new actual-object
Windows flow reached the final waypoint edit with an obsolete System menu path;
the probe package separately refused the pinned bundle's old `vccorlib140.dll`
against the explicitly selected licensed CRT. Both negative logs/artifacts are
retained. Replacement fixes use the current Settings → Waypoints entry and
recognize that CRT family without relaxing unknown-library rejection. No probe
package or live boat AIS result is accepted from that failed run.

Replacement `9a5162b` / run `36481625379` closes the Windows object-flow failure
and passes its 112 integrated tests, 71 component checks and product/ENC captures.
Probe packaging verifies 17 x86 binaries and imports, then fails before launch
because a copied Python environment dictionary looks up mixed-case SystemRoot.
The replacement normalizes Windows environment keys and explicitly removes the
development credential; six portable packaging tests cover this boundary.
Run `36483619726` / `1954c19` verifies all 17 binaries, clean-PATH launch and
source/ZIP construction. Its final Windows PowerShell 5 negative probe check
fails because Start-Process loses the fast child's exit code (null). No failed
package is deployed. The replacement owns the .NET process handle through exit,
drains both streams asynchronously, preserves the deadline and rejects unknown
exit status; CI also asserts the recorded exit code explicitly. The boat is
reachable with OpenCPN closed. A new SSH presence check did not list the saved
AIS credential; this differs from the earlier presence check and is not yet a
credential-read or service result.

Live AIS remains unverified. The local rail revision `00ff457` passes 120
integrated Linux cases plus actual object and synthetic navigation input flows.

The main rail's corrective Linux pass now checks the four measured HTML row
bounds and compact pilot-summary bounds at 1280×800. Typography uses the final
48px/−3px numeric rule; observation age and source state remain actual data.
The card cannot infer a pilot mode from missing feedback. Twelve fixture-free
captures retain Day/Dusk/Night and AIS settings behavior. Populated sensor,
native Windows and boat replacement gates remain outstanding.

The user's September 28 instruction supersedes the older visual baseline:
safety/navigation correctness → supplied v8 HTML → extracted prototype spec →
older brief/image → current Beta. The native architecture and all existing
data, control, installer and Legacy/Safe contracts remain in force.

The supplied HTML and all 113 companion files are committed unchanged. HTML
SHA-256: `b04573b920b6bccd16afd22f54a909e48afcfd502ce9cd7c69fdf5b6dd895447`.
Original ZIP/extracted/Git bytes were verified. See
[prototype specification](design/prototype-spec.md),
[component map](design/prototype-component-map.md),
[interactions](design/prototype-interactions.md) and
[chart inspection](design/xnav-chart-style.md).

Sixty canonical renders on each of Linux and native Windows cover twenty
actual states across Day/Dusk/Night using pinned Chromium, offline loading and
a fixed clock. Windows resolves Segoe UI; Linux resolves Liberation Sans.
Both reference artifacts and all Windows PNG hashes were verified. See
[evidence](evidence/prototype-reference-7925b160.json).

Native shell migration now follows the 68/80/186/132/34px composition. Two
Linux capture/review passes exposed and corrected undersized GTK font ems,
selected navigation state and energy-card composition. Eight lossless
reference/current/diff sets retain visible mismatches without masks or broad
similarity tolerances. The actual coastline canvas remains stock presentation.
The integrated build at local `545feca` passed 110/110 tests; the first run
conflicted with the simultaneous capture on upstream's fixed REST port, so the
complete gate and captures were repeated serially. Native Win32 MSVC compilation and 102/102 integrated unit regressions also
pass at remote `8357239` (local `a6a9259`). Its eight captures exposed occluded
chart controls; the capture job failed during secondary command-line quit.
Follow-up z-order/shutdown fixes await replacement native evidence. The next
run at `83569f0` hit a Windows `max` macro collision in the new transport test
driver. The correction builds at `d92e187`; 102/102 integrated native tests and
6/6 portable AIS tests on each platform pass, including protected credential
storage. The new native fragmented-overflow transport test then failed: bounded
reading stalled while OpenSSL retained decrypted bytes. A resume-before-poll
fix now passes 18 hostile-frame scenarios and the actual-provider lifecycle on
Linux and native Windows at remote `ea869a9` (local `2a3c52`). Its native MSVC
build and 102 integrated tests also pass. Downloaded artifact hashes, sizes and ZIP CRC
were verified; see [replacement run evidence](evidence/prototype-native-ea869a9.json).
The native capture job still fails: input/capture timing reads an older light-mode
snapshot. Downloaded ENC images also expose absent floating controls and weak
Night contrast. Neither a successful screenshot command nor `visible=true`
qualifies these views. Replacement state-driven and painted-pixel checks are in progress.
The replacement at remote `e51a7df` (local `37b36dc`) now passes all six
development jobs: native build, 102 integrated tests, AIS contracts on both
platforms, hostile TLS/provider tests and native composition/ENC capture.
Downloaded artifact digests, sizes, CRC and each recorded PNG hash verify.
Controls are visibly painted and real zoom input changes the upstream scale;
observed Day/Dusk/Night/Day retains ENC content and exits cleanly. This closes
the capture/control failure, not visual acceptance. Night hazard/text contrast,
rail composition and primary sheets remain mismatches. See
[replacement evidence](evidence/prototype-native-e51a7df.json).
No screen is accepted; see
[conformance](design/prototype-conformance.md) and the
[review log](design/reviews/prototype-native-shell-in-progress.md).

Online AIS now has owned target/cache/provider contracts, onboard precedence,
aging, bounded viewport subscriptions and reconnect policy. The foundation
passed native Win32 MSVC and Linux tests at `087318c`; downloaded JUnit artifacts
verify ([evidence](evidence/prototype-ais-foundation-087318c.json)). Its pure JSON
codec adds 100 deterministic checks and the session adds 116. The portable
suite now passes 75/75. The actual provider passes a local TLS lifecycle test
covering prompt subscription, viewport replacement, reconnect and resubscription.
The protected Windows credential adapter and Linux environment-only adapter
are implemented; isolated native Credential Manager tests pass at `d92e187`.
The opt-in bounded TLS/compressed WebSocket boundary passes 18 loopback
adversarial scenarios on Linux and native Windows. The worker provider lifecycle
passes on both platforms. XNav-only product/provider wiring and native Traffic
drawer are now in development validation; online chart overlay and live
AIS acceptance remain pending. No service connection or target count is claimed.
The application-thread Online AIS preference/credential boundary and copied
viewport normalization add ten deterministic integrated cases. Linux passes
120/120 with these additions. Opt-in, failed-save disable, replay isolation,
credential separation, worker rejection and pinned LLBBox antimeridian/world
bounds are covered. Fixture-free Linux captures exercise the native settings
path: default OFF, missing-key withholding, OFF, theme changes, Back and Close.
Two failed capture passes exposed native sheet stacking, followed by a correction
and retained paint-latency checks. Windows replacement and populated-target
interaction remain required. See the [AIS drawer review](design/reviews/prototype-ais-drawer-in-progress.md).
The replacement native run at `3db9704` (local `98f3646`) passes all six
development jobs, 112 integrated Windows tests, six AIS contracts per platform,
and twelve product captures including Online AIS settings. Downloaded digests,
CRC and PNG hashes verify. Traffic and Night settings were visually reviewed;
heading mojibake was identified and corrected in the next increment. See
[native evidence](evidence/prototype-native-3db9704.json).

Supplemental online chart marks now have an owned, bounded, independently tested
presentation model and two narrow paint/context-selection hooks into OpenCPN.
Thirty-nine new assertions cover identity/provenance, local precedence, position
validity, directional withholding, retained-state aging/expiry and antimeridian
coordinates. Portable tests pass 76/76 and Linux integrated tests 120/120.
The fixture-free product/settings capture still passes with no synthetic data.
Replacement native [run 36471022893](https://github.com/ThereptileII/Work/actions/runs/36471022893)
at `ad4913f` (complete local `97665f5` mapping) passes all six jobs, 112 integrated
tests and seven AIS suites on each platform. All six downloaded artifacts verify
their API digests, lengths and ZIP CRC. Twenty product/ENC captures verify their
PNG hashes; Traffic Day and settings Night were reviewed, confirming the UTF-8
heading fix. See [native evidence](evidence/prototype-native-ad4913.json).
Populated-target, actual symbol/software/GL and live boat evidence remain
pending; the compiled overlay is not declared accepted.

A separate, non-installed native component executable now exercises populated
AIS list/card interactions without placing fixtures in the product. The first
pass exposed unsupported wxGTK Tab-state querying; input-modality event tracking
replaces that query. Actual painted captures then drove a second pass matching
the prototype's status pills, small units, 16px statistic gutter, callout and
right-aligned detail rows. Fresh OpenCPN estimated CPA/range values are accepted
as estimates and withheld after their own dependency expiry; online CPA/TCPA
remain unavailable. Replacement native Windows and boat review are still gates.
The completed local correction passes 120 integrated tests, 69 component checks
and all three immutable-design checks. Nine painted component captures and
exact drawer reference/current/diff sets are retained as development evidence.
Replacement native `079ec95` now passes all six jobs, 112 integrated tests and
69 populated component checks. Six downloaded artifacts and nine component
image hashes verify; Day/Night were reviewed. Typography/border/shadow mismatches
remain. See [evidence](evidence/prototype-native-079ec95.json).

A separate non-installed read-only AIS commissioning probe links the production
TLS/provider/credential boundary. It never loads OpenCPN, profiles or plugins;
its bounded run reports only aggregate counts and fixed connection-state labels.
Linux integrated build and 30 offline CLI/privacy checks pass, as do nine source
archive tests. Native packaging/closure and live service evidence remain pending.
Only credential-entry presence has been checked aboard; no successful connection
or live target count is claimed yet.
The first probe package attempt at `d79fac6` passed native build, 112 tests,
24 offline probe checks, transport/provider and UI captures, then correctly
failed on conflicting app-local MSVC CRT copies. No probe ZIP was uploaded.
The correction explicitly chooses the licensed toolchain CRT (as the main
packager does), still rejects other ambiguous dependencies, and passes four
portable resolver tests. Added exact OpenSSL dependency notices and a bounded,
hash-verifying boat invocation script. Replacement native packaging is pending.

The actual-model object flow now validates retained AIS summary/report age,
native drawer scrolling and return-to-chart selection. It also exposed and
fixed the timeline being omitted when Advanced Settings rebuilt the chart.
Linux passes all 27 object scenario groups, seven actual pointer actions,
26 captures, both exact settings-return layouts, coastline and navobj storage.
The obsolete compact-card Details hop was replaced by the prototype's direct
target drawer; no underlying route/waypoint/AIS semantic check was removed.
The dedicated widget suite now passes 71 checks and the established integrated
command passes 120 tests. A separate native object-flow development job was
added; the full existing release regression workflow remains mandatory.
See [local evidence](evidence/prototype-ais-navigation-chart-ink-local.json).

The second chart-ink pass preserves Day contrast and improves Dusk/Night text.
Resource checks pass 377; actual public ENC software/llvmpipe captures retain
content through all themes. Baked raster symbols still need contrast work,
and no chart presentation is accepted yet.

XNav-owned S-52 presentation is being implemented as separately packaged,
hash-verified resources derived from the pinned baseline. Its resource tests
preserve all symbols, lookups and conditional navigation semantics while
checking deterministic palettes and depth-role distinctions. The integrated
build, actual ENC palette review, Standard fallback and renderer/mode cycles
remain gates. Linux now passes the integrated build and 110/110 tests, 369
resource checks, 18 adversarial TLS scenarios and the actual provider lifecycle.
Corrected coastline palette captures and public ENC XNav/Standard captures
retain content through Day/Dusk/Night/Day with clean exits. The Linux OpenGL
capture uses Mesa llvmpipe, not a hardware-GPU acceptance result. The review
records remaining chart-text/night-contrast and overlay differences; see
[chart review](design/reviews/prototype-chart-v1-in-progress.md).
AISStream must not
inherit the local Signal K client's disabled TLS validation. The prototype's
newer vendor symbol snapshot cannot replace the pinned resources blindly.

Boat SSH returned on September 28 at 18:36 UTC. The interrupted commissioning
transaction is now closed: normal shutdown of the exact orphan chart decoder,
fresh cold inspection, narrow adoption of reviewed stock-return preferences,
and restoration all passed. All five quarantined plugin DLLs have their original
hashes again; stock executable, navigation database and chart database remain
unchanged. No application/helper process or active commissioning marker remains.
The original connection direction is restored, without launching the application.
Recovery tooling replacement [run 36469956631](https://github.com/ThereptileII/Work/actions/runs/36469956631)
passes all nine jobs, including 178 native adoption checks. See the
[recovery evidence](evidence/boat-prototype-reconnect-20260928.md).
Any next application launch requires a fresh read-only audit/preparation against
the newly adopted baseline; expired launch/restart receipts cannot be reused.
The deployed application still uses the previous Beta UI. No physical actuator
commands are authorized or sent.

## Prior Beta 2 development and deployment evidence

**Beta 2 development is in progress; its current development package is installed
on the boat, but it is not qualified for release.**

The user's Desktop feedback has been read completely and recorded in
[boat Beta 1 feedback](feedback/boat-beta1-feedback.md). The approved design
reference was the Beta 2 baseline before the new HTML design lock. Work separates fixture-enabled CI
executables from the installed product, refines shared visual components and
navigation workflows, and adds repeatable boat deployment and maintenance tools.
The accepted Beta 1 results below remain historical evidence, not Beta 2 gates.

### Boat reconnected and interrupted session recovered — September 28

SSH/Tailscale/RustDesk are available again. Windows booted at 06:35 UTC;
OpenCPN's log records a clean application exit before that restart. The last
unacknowledged Menu/staging requests left no corresponding run records. No
action was blindly replayed. Stock OpenCPN, the installed `7827acb` executable,
navigation database and chart database retain their exact recorded hashes.

The closed-session review found eleven persisted display/position/GL changes
relative to the prepared input-only profile. Pinned `LoadMyConfig` explains
the non-expert texture minimum of 128MB after loading the earlier upgrade's
64MB setting. The narrow adoption extension passed 125 portable cases and the
nine-job native [tool run 36388235873](https://github.com/ThereptileII/Work/actions/runs/36388235873)
at `ad08061eb46d63401a5565ae561bb8f780cfae88`; all nine artifact API/upload/
download hashes, sizes and ZIP CRC agree. A separately staged qualified copy
preserved the interrupted session's source dependencies during restoration.

All five quarantined DLLs and the original connection direction are restored.
Independent byte comparison confirms the adopted INI retains every other
post-session byte. Its SHA-256 is
`9dfc3fb94a5de45047f785aa0d922ea3c990b8d58522a61d9ec5047ed7d328f3`.
The active commissioning marker was removed only after verified completion.
No application or physical command was launched. The source checkout then moved
to qualified tool commit `ad08061e`. A new deployment/commissioning session is
required; the previous four-hour restart session is expired.

Both complete application runs (`7827acb` and `79a95c4`) finished all sixteen jobs
successfully. The latest `79a95c4` candidate has 110 integrated Linux tests,
102 integrated Windows tests, 45 installer lifecycle checks, all three DPI
scales and actual three-hour endurance passes on both platforms. Its final
package, native and Linux evidence match API/upload/download digests and ZIP
integrity. All 999 portable files and 5,101 corresponding-source files verify.
The product is fixture-free and retains the approved Win32 plugin ABI.

Setup SHA-256 `b281362f6bf6d51410aba1491e16049d4c7bf243f6602a2bd7a3d0342ab09c4d`
has been verified on the boat. Update passed: all 1,269 files in the 1,292-entry
cold profile inventory remain byte-identical, including both navigation and chart
databases. The installed executable matches the exact package (`aee52bd2…`),
and stock OpenCPN is unchanged. Boat mode/maintenance/screen acceptance and remaining
Beta 1 retirement are still open; CI success alone is not Beta 2 acceptance.

The exact `79a95c4` product launched with a fresh read-only audit. A Windows
network prompt was inspected and its proxy closed without granting access;
the composited sheet remained visible. The subsequent fixed-size review exposed
a tooling bug: restoring the maximized frame first selected a normal rectangle
partly outside the monitor, and the containment guard stopped before repositioning.
No chart/pilot action was sent. The repair applies the already tested stock-window
recovery policy to the distinct XNav identity, retains strict capture/input guards,
and adds eight disposable native resize cases. Boat deployment awaits native
qualification; the running product and restart-session dependencies are unchanged.

### Reconnection required after the resumed session

At 14:44 UTC, local Tailscale is Running but the boat peer is offline with
last-seen 08:17:38 UTC. A fresh SSH probe times out. The `79a95c4` installation
and pre-launch profile preservation are verified; the present process state
is unknown. The last window action restored the maximized frame and refused
before fixed placement. No action is replayed and remote services are unchanged.

The four-hour session created at 07:11 UTC is expired. On reconnection, inspect
the exact failed request/result, process/start identity, clean-exit log and
active commissioning journal before normal close/restoration. Do not reuse
its launch or mode receipts. Five DLL quarantines and the input-only connection
setting remain part of the unclosed transaction; do not run maintenance first.

Resize tool commit `9c3b7bb7b1518be306bd751788b01912ee1543dd` maps all 716
local source blobs/modes exactly and preserves the pin plus eight unrelated
repository blobs. [Native tool run 36438174776](https://github.com/ThereptileII/Work/actions/runs/36438174776)
passed all nine jobs. All nine artifacts match API/upload/download digests,
sizes and ZIP CRC. Thirty-one actual display cases pass, including eight new
resize cases; fourteen mode cases remain passing. The repair is not deployed
while the boat is offline. Local checks pass: 178 display-policy/compilation checks,
133 restart-window checks and 13 handoff tests. No product rebuild or boat
visual acceptance follows from these tooling tests.

The following September 27 checkpoints are historical; the recovery above
supersedes their offline/active-commissioning state.

### Current boat deployment — 09:36 UTC

The independently downloaded `7827acb` development package was updated on the
boat at 09:09 UTC. Setup SHA-256 is
`7bfe4ba1c31a68304edd04f261ac8bee3f7c93a2612f3bbbb3c1da1b0b076e71`;
installed executable SHA-256 is
`56f435d08904ec5a0b8a840fe13625b9476353e7c97a8d2368f16c100b5672e5`.
All 904 real profile files compare identically before/after the update, and the
approved stock executable is unchanged. Full platform endurance remains open.

A fresh read-only commissioning transaction and guarded-restart session preceded
the 09:14 XNav launch. Its five quarantined plugin copies and input-only serial
change remain active during this review and must be restored after normal close.
Both queued Windows network prompts were cancelled without granting access.
The exact native 1280×800 frame at 150% DPI now shows real chart content, clear
Center, all four rail values and no floating Dashboard. Three zoom-out actions
retain chart content. These observations are scoped visual checks, not complete
boat acceptance. The actual monitor remains 1920×1080; physical 1280×800/touch
acceptance must not be inferred.

The new pan review found the tooling's diagnostic-path mismatch: the installed
product writes under `opennav-logs`, while the helper looked at the profile root.
The correction adds native wrapper cases for a decoy root file, missing/stale
installed diagnostics and wrong commit. Tooling `f44f2b4` passed all nine native
jobs in [run 36310215706](https://github.com/ThereptileII/Work/actions/runs/36310215706),
including 23 actual display cases. All nine downloaded artifacts match
API/upload hashes, sizes and ZIP integrity. It is not yet deployed.
The earlier main-window refusal was traced to a native hover tooltip after
cancelling the network sheet; a bounded pointer-only move cleared it. No failed
zoom/capture request sent input. Product/restart tools were not modified aboard.

### Boat connectivity interruption — 09:50 UTC

Tailscale reports the boat peer offline with last-seen 09:50 UTC; local
Tailscale remains Running/online. SSH and Tailscale probes time out. No remote
access settings were changed. The last confirmed XNav display was Diagnostics;
a subsequent Menu request returned no remote receipt. Do not repeat it or infer
process exit until the actual journal/process is inspected after reconnection.
The commissioning transaction remains active and must be restored only after
normal application close and reviewed profile deltas. Mode/maintenance/old-copy
retirement gates remain open. [Partial display review](design/reviews/beta2-boat-7827-review.md).

The staging command for a separate review-tool copy did not return a receipt;
no archive or staging script was uploaded. A read-only run-directory check is
required before retrying it. A versioned staging helper is being native-tested
so newer qualified display tools can coexist with the frozen restart dependencies.

### Current boat review tools

`07da8e307f174646f4fd2722ec84bda511f4bcfd`,
[run 36303832915](https://github.com/ThereptileII/Work/actions/runs/36303832915),
passed all nine native jobs. Downloaded artifacts match upload/API SHA-256,
sizes and ZIP integrity. Twenty display cases and fourteen mode cases pass;
chart pan records one exact key pair or refuses without input. Source checkout
on the boat passed at 07:51 UTC, with no application change or launch. Actual
boat pan, touch and replacement visual acceptance remain open.

### Native review refinement

The retained native screenshots exposed an ambiguous autopilot subtitle. It now
states “Manual commands require feedback confirmation”; this is a requirement,
not a claim that feedback has arrived. [Twenty-screen review](design/reviews/beta2-native-9f592-review.md).
The fixture-free Linux build/install passes. A separate complete CI candidate
will qualify this refinement together with the bounded chart-pan tooling.
No release or boat visual acceptance is claimed from the old screenshots.

### Current replacement validation

The current full replacement is `79a95c4f39063c20ca4d5a98c3147080d9813077`,
[CI 36311718794](https://github.com/ThereptileII/Work/actions/runs/36311718794).
All 710 mapped blobs/modes match local `3e3c628`; the pinned manifest and eight
unrelated repository blobs are preserved. This includes the direct cleanup
deadline repair and Windows PowerShell assembly load needed for isolated
review-tool staging. Application and installer source still match `7827acb`.
The separate native tool run is
[36311719759](https://github.com/ThereptileII/Work/actions/runs/36311719759).
The tool run completed successfully: nine native jobs, 12 staging cases, 23
display cases and 14 mode-window cases. All nine artifact API/upload/download
hashes, sizes and ZIP integrity agree. [Tool evidence](evidence/beta2-review-staging-tools-79a95c4.json).
The full application run remains unfinished; no newer helper has been deployed
during the boat connectivity interruption.

The earlier `a5b290e52ebeb83bda75c1f7008879c398548dc3` candidate in
[CI 36307149573](https://github.com/ThereptileII/Work/actions/runs/36307149573)
failed Windows installer cleanup and cannot qualify the release.
All **706** mapped blobs/modes match local
`9e0d0c77b5ba2513cc7355da80f38e03ddf631b6`; the pin and eight unrelated blobs
are unchanged. It merges the refined candidate history without rewriting either
branch. Application/installer code is unchanged from `7827acb`; the installer
harness now waits for actual relocated cleanup and preserves failed fixtures.
Six timing/lifetime tests pass locally and both contract jobs pass natively.
The verified earlier failure is documented in the
[maintenance completion review](installer/beta2-maintenance-cancel-and-dpi-readiness.md).
The native installer reached 45 checks, but its final direct-engine uninstall
hit a separate 120-second harness limit. Its prior relocated uninstall completed
in 150.406 seconds. The final direct wait is being aligned with the same bounded
600-second deadline; twelve timing/lifetime cases pass locally. This candidate
is not accepted. Its subsequent native DPI tests passed at 100%, 125% and 150%.
Eighteen verified original screenshots were individually reviewed, including
alert/rail coexistence, System, night surfaces and mode chart content; see the
[scoped DPI review](design/reviews/beta2-native-a5b290-review.md). High-DPI expanded
pages still scroll; physical touch and the complete boat sequence remain open.
Both complete replacement platform gates and boat validation remain pending.

`7827acb7c8b0d708285bd26a4c48d545dd64d139` is running the complete gates in
[CI 36304661282](https://github.com/ThereptileII/Work/actions/runs/36304661282).
All **699** mapped blobs/modes match local
`a883f3fb7c729773e323d9234d95dda1f62fcacf`; the pinned baseline and eight unrelated
repository blobs are preserved. It includes the autopilot wording refinement,
bounded chart-pan tools and the DPI harness correction below. No replacement
product had been deployed at the earlier checkpoint; see the current deployment above.

The boat tooling checkout moved to `7827acb` at 08:36 UTC after its native tooling
gates passed; no application changed or launched. At 08:48 UTC, cold checks still
show the exact `8e780` executable, unchanged stock/INI/navigation database, no
application/helper and no active commissioning. The remaining Beta 1 portable
folder and its three download files match their expected hashes. Retirement is
prepared but awaits the replacement's mode/recovery checks; no files were moved.

`608756` remains historical diagnostic work. Its harness added an input-evidence
dictionary which source review subsequently found shadowed the screenshot Path;
it cannot qualify the release. The corrected function reaches screenshot and
touch-close checks at all scales with a mocked native boundary, but only the
new native run can establish actual Windows behavior. The short-lived `7ac585`
run was superseded by `7827acb` through normal same-branch CI cancellation.

The preceding `9f592` application candidate is **not accepted**: its full Windows
installer matrix passed, but the DPI gate timed out on its first physical chart
context gesture. The bounded harness repair verifies the foreground process and
actual chart HWND geometry before sending one click; no retry, scale or layout
assertion is removed. [Failure and replacement scope](installer/beta2-maintenance-cancel-and-dpi-readiness.md).
The boat remains unchanged while the new exact-commit run proceeds.

### Previous full candidate (Windows DPI failed)

`9f59209914f57ff97af7e184b201752b8f422f0a` ran the native gates in
[CI 36299835767](https://github.com/ThereptileII/Work/actions/runs/36299835767).
All 694 mapped blobs/modes match local `c72061459b8c532c0f6aaad5846c61a52231ea2f`.
It includes the exact upstream manager type, Dashboard presentation, maintenance
Cancel/paint-readiness harness repairs and complete resource-review dependency
closure. The corrected Linux build and all 27 object-workflow groups pass;
replacement native UI and release gates remain open. It is not deployed.

The native integrated application build and both **71/71** contract suites now
pass. Eleven prerequisite artifacts have been independently downloaded and
checked against their API/upload hashes, byte sizes and ZIP integrity. Linux
completed its functional/UI checks and entered the three-hour elapsed-time trip
at 06:45:57 UTC. Windows passed both **102/102** integrated/fixture-free CTest
runs, all **45** installer lifecycle checks, seven production recovery groups,
eight actual user-flow groups and software/OpenGL chart/plugin checks. The DPI
failure withheld the development and release packages; native endurance did not
start. Downloaded full native evidence matches API/upload SHA-256, size and ZIP
CRC. The earlier Linux trip continues as historical evidence, not acceptance.

Before the next boat launch, source inspection found that the restart tooling
rejected separately reviewed stock and bundled DLLs sharing a basename. A
[per-copy shutdown identity](installer/restart-plugin-copy-identity.md) repair
passes 309 portable policy checks and 17 copied-dependency checks. Tooling
candidate `506a70f5c49cf6d9335be31891817bf88ebc120e` is in
[native run 36301213829](https://github.com/ThereptileII/Work/actions/runs/36301213829),
matching all 696 mapped blobs/modes at local `b7c916d90bcb4c0c086516e838696a53e9acf194`.
It does not change the application candidate or cancel its endurance gate.
That tooling passed all nine native jobs and was checked out on the boat without
an application change. The subsequent owned-Legacy-window refinement is also
qualified and is the current source checkout, described below.

The preceding boat tooling was `3552a3ce2c2d78b8c46ec2a94639cd62d060bee6`,
[run 36301703492](https://github.com/ThereptileII/Work/actions/runs/36301703492).
All nine native jobs and downloaded/hash-verified artifacts pass. Fourteen
actual mode-window cases include the preserved floating Legacy instruments;
obscured menus, foreign/unowned windows and unexpected XNav/Safe overlays remain
refused. [Scoped evidence](evidence/beta2-legacy-window-review-3552a3.json).
The boat application is still the closed `8e780` development build. Its current
904-file profile inventory is recorded privately for independent update checks.
The 07:16 UTC cold preflight independently rechecked the stock/application,
INI, navigation database and chart-list hashes: unchanged, no running OpenCPN
or commissioning transaction, and SSH/Tailscale/RustDesk still running.

The earlier cold restoration used qualified tooling `1439b5b2dac669090b5de4ef956eaffa0f42c542`,
[run 36299584290](https://github.com/ThereptileII/Work/actions/runs/36299584290).
All nine jobs and all downloaded/hash-verified artifacts pass, including 163
native baseline-adoption cases and actual broker/Prepare/Arm checks.
[Scoped tooling evidence](evidence/beta2-resource-adoption-1439b5.json).

### Superseded Dashboard type candidate

`ee475f0b5dbef6ea9a5ff5778f3238665f47d944` was tested in
[CI 36298899492](https://github.com/ThereptileII/Work/actions/runs/36298899492).
All 692 mapped blobs/modes match local `bd6d6511da3a6a3d843b9ef1c20d4eb847d0307e`;
unrelated repository files and the pinned OpenCPN revision are unchanged.
It includes the Dashboard presentation refinement and the two scoped harness
repairs below. Its separate branch preserves the preceding Linux elapsed-time
run while testing this exact new product. No replacement boat deployment yet.

The native product link failed on the new Dashboard bridge’s incorrectly declared
layout-manager type. It is corrected to the actual pinned `OCPN_AUIManager`;
[failed evidence and correction](design/reviews/beta2-plugin-workspace.md).
No failed build was deployed.

The first installed boat session has closed normally with a retained native
handle and measured exit **0**. No chart helper remained. The navigation database
is byte-identical. Four final INI changes have been inspected; a narrow
[installed resource adoption](installer/beta2-installed-resource-adoption.md)
check has passed all native tooling gates. The closed transaction is now
restored: all five quarantined DLLs and the original connection byte are back,
the four independently reviewed migrations are preserved, and no application
was launched. [Boat completion record](evidence/beta2-boat-first-xnav-8e780.json).

### Previous full candidate (Windows failed)

`edd8da0a4bd386fbb2dbd249289b9f09eeae8dc3` is the replacement full candidate,
matching all 680 mapped blobs/modes at local
`3e0b17dcbb353793a5c135c18774a1e91a6a3b02`, in
[CI 36295867893](https://github.com/ThereptileII/Work/actions/runs/36295867893).
It downloads both immutable historical installers before clearing the read-only
Actions credential. The previous run is superseded after its native installer
harness failure; its unfinished Linux endurance is not an accepted soak.
The native integrated and fixture-free builds passed, as did chart/plugin checks.
The installer exercised genuine historical update/rollback, then the maintenance
Cancel test incorrectly required exit 0 instead of NSIS's documented user-cancel
exit 1. The DPI harness read Diagnostics before its first paint established the
scroll extent. Both failures and narrowly scoped test repairs are recorded in
[the harness review](installer/beta2-maintenance-cancel-and-dpi-readiness.md).
The complete Windows gate failed; no replacement release artifact is accepted.
The same run's Linux gate completed successfully at 08:32 UTC: both 110/110
CTest suites pass and the trip measured 10,800.19 seconds with 1,080 samples and
540 UI/dropout actions. Its downloaded evidence matches API/upload SHA-256, size
and ZIP integrity. [Historical Linux record](evidence/beta2-historical-linux-edd8da0.json).
This does not qualify the failed Windows candidate or the newer replacement.
The historical product has not been deployed.

The next local increment suppresses only registered bundled Dashboard panes in
XNav, restoring their original workspace in Legacy. This follows the actual
boat observation of large floating Legacy instruments obscuring the chart.
Linux integrated build, **110/110** CTest cases, **27** wx object groups and both
software/OpenGL chart cycles pass. Native replacement and boat review are pending.
[Presentation contract and coverage](design/reviews/beta2-plugin-workspace.md).

### Superseded credential-order candidate

`a896fb5c5fa935b5daa757087ebedc4cd531c1d3` was the previous full candidate,
matching all 664 mapped blobs/modes at local
`5dd114c00cf07ceef8b32e7d6ec978e33d5d9e50`, in
[CI 36292727131](https://github.com/ThereptileII/Work/actions/runs/36292727131).
Native contract, maintenance, transport, broker, display, bilingual stock-tool,
integrated/fixture-free builds, DPI and chart/plugin gates passed. The installer
gate passed initial installation/rollback, then failed because the harness had
already cleared its Actions credential before fetching the second pinned prior
package. The download ordering is corrected locally; genuine early-package
rollback and complete endurance remain replacement-candidate gates.
It includes the explicit historical-layout rollback fix and narrow observed
profile-migration rules.

The superseded `fa27637eb30150b472acd631cd28e6426d43b9fe` ran in
[CI 36291671816](https://github.com/ThereptileII/Work/actions/runs/36291671816).
All 655 mapped blobs/modes match local
`357d4fee21bc6a1c3211646f230b89feadb036a0`. It includes the corrected endurance
palette action and unchanged-value layout caching. It is not accepted and will
not be released: subsequent source review found that an early 0.4 Beta 2 package
still used the historical Start Menu layout. The new explicit layout marker
preserves that package's immutable maintenance engine during rollback. Native
x86/x64 COM tests pass; a complete genuine-package rollback test remains a gate
for the next product candidate.

### Retained development review package (not qualified)

`8e780edc34f68abd693a5d5f6aecdb3ba05a75c4` was exercised in
[CI 36287991989](https://github.com/ThereptileII/Work/actions/runs/36287991989).
All 630 tracked blobs/modes match local `81ec667e1d3740b36b99eaf6a1ed7526b7edd044`;
CI fetches and verifies the unchanged pinned OpenCPN source. This replacement
adds the exact-version installer caution flow, reviewed profile-migration
preservation, complete broker fixtures and component-local dim-mode hover hints.
The local contract suite passes **71/71**, integrated Linux CTest **110/110**,
and actual wx object workflows **25 groups / 25 captures**.
The native functional suite, fixture-free recovery package, installer lifecycle,
100/125/150% DPI/hover checks and chart/plugin gates passed. Their development
review bundle and full native evidence have been downloaded and hash-verified.
The Windows endurance harness then failed before its first sample because it
still selected the removed Light caption. The next source uses the existing
palette action and verifies Day/Dusk/Night. This candidate has no accepted
Windows endurance result. Its Linux elapsed-time run was cancelled with 327
retained samples and has no accepted endurance result. The hash-verified setup has now completed its first boat installation with
exit 0. Independent before/after inventories confirm the complete normal profile
is identical, and stock OpenCPN retains its validated hash. A fresh installed plugin/data audit preceded the read-only launch. The actual
1280×800 frame now shows real licensed chart content, ownship and saved marks.
Live N2K wind, depth, STW, water temperature and tanks are observed; battery and
motor data remain unavailable and dependent energy estimates are suppressed.
A saved floating Dashboard still obscures the left chart, so strict clear-overlay
acceptance remains open. The Windows firewall prompt was cancelled, with visual
confirmation; no Allow action was used. This development bundle is not a
qualified release. [First boat review](evidence/beta2-boat-first-xnav-8e780.json). [Initial installation](evidence/beta2-boat-first-install-8e780.json).

### Previous candidate (not qualified)

`60cd054712c6a930147247970121a959d48e03bf` was exercised in
[CI 36284246369](https://github.com/ThereptileII/Work/actions/runs/36284246369).
All 591 tracked source blobs and modes match local source
`f123bf5c3d33d979409a8f32418653ac56c2d413`; the pinned upstream gitlink is unchanged.
The local portable contract suite passes **70/70**, and the sequential integrated
Linux regression suite passes **110/110**. This candidate includes the
startup-log rotation fix, preserved plugin workspace, tighter instrument spacing,
fresh waypoint selection/permissions, and truthful route Undo eligibility.
The expanded Linux object workflow passes 24 groups with 25 captures, and the
actual-pointer workflow passes eight groups with nine captures. These local
results do not replace native package or boat acceptance.

The same candidate's native fixture-enabled functional suite, fixture-free
recovery-package launch/mode smoke, DPI and chart/plugin checks have passed.
Its installer gate failed when the accepted Beta 1 executable displayed the
expected version-change navigation caution during the real update sequence.
Candidate clean install, chart startup and first-install rollback had passed;
the remaining lifecycle and native endurance steps are not accepted. The
[narrow harness repair](installer/beta2-version-transition-notices.md) retains
the actual captured warning and explicit Agree action, with exact owned-version
checks. The expanded local portable suite passes **71/71**; a replacement native
candidate is now running above. The superseded candidate's Linux three-hour
elapsed-time trip began at 01:26 UTC; it is not an accepted endurance result.

The separate exact-revision commissioning tooling has passed its native marker
transport, full broker and actual Prepare/Arm/Collect gates; downloaded evidence
and upload hashes are verified. See [the scoped qualification](installer/commissioning-restart-qualification.md).
Actual installed-app launch, plugin shutdown, in-app mode transitions and boat
screen review remain outstanding. Later UI-driver tooling is qualified separately;
it is not part of the current candidate's acceptance evidence.

### Boat prerequisite — official 5.12.4 upgrade verified

Read-only inspection found **OpenCPN 5.12.2-0+b69f44c / x86**, executable SHA-256
`2fdcd6a2cdef7f730aa4c094fcd21302ed2a5d531a611ee180c06533f3a2cb48`.
The user authorized a backed-up upgrade. The official visible Upgrade has now
completed, and the installed **5.12.4 x86** executable matches the validated hash
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.
All 539 profile files remained byte-identical. Complete third-party plugin files,
the RTL-SDR registration and its original uninstaller were preserved; no recovery
copy-back was needed. The original cold backup remains verified and retained.
The wizard's Upgrade/reset/summary/Finish screenshots were reviewed; Run and
Show were unchecked, and OpenCPN was not launched. Tailscale, SSH and RustDesk
remain running. [Upgrade evidence](evidence/beta2-boat-stock-5.12.4-upgrade.json).
This closes the stock-version prerequisite; Beta 2 still requires its own gates.

A stock-only commissioning inventory matched all nine reviewed plugin DLLs
and their source evidence. The stock session used a journaled one-byte input-only change and four
temporary DLL quarantines. Both have now been restored after closed-session
inspection, preserving the independently reviewed startup migrations. The exact official stock
executable launched with empty arguments on the interactive desktop, preserving
Swedish locale. Its complete first-start GPL/navigation caution was captured,
downloaded, hash-verified and reviewed. The separately native-qualified warning
tool acknowledged it once after two consecutive complete reviewed-image matches;
modal dismissal and the current-launch startup-finalized marker are confirmed.
[Read-only preparation](evidence/beta2-boat-stock-readonly-preparation.json),
[actual warning review](evidence/beta2-boat-stock-warning-review.json), and
[qualified settling tools](evidence/beta2-warning-settling-4fa62e.json).

The first view showed water at a close 100m scale
and a Windows Update notification. No Windows restart or update action was taken.
The later capture refused the overlapping notification. The fixed-size resize
then exposed a partially offscreen saved normal window and refused before changing
its size. Native-qualified replacement tools have now recovered the boat window
to an exact 1280×800 visible frame at DPI144 without changing display settings.
The exact launch log records an Intel UHD OpenGL4.6 canvas context. This is not
proof of chart rendering or hardware acceleration. No physical command has been
sent. A private diagnostic image now shows the **real detailed nautical chart**,
ownship and saved marks. Its overlapping floating Dashboard pane still fails
the strict clear-overlay capture rule; the diagnostic image is not substituted
for that automated gate. [Native tool qualification](evidence/beta2-stock-chart-5013db.json)
and [actual boat observations](evidence/beta2-boat-stock-first-chart.json).
Both configured chart directories are currently accessible, and the chart
database still matches its pre-upgrade hash. A preliminary live INI comparison
found seven startup changes. After normal close was requested, the log records
complete clean shutdown and OpenCPN is no longer running, but the close checker
did not retain a native process handle and cannot establish its exit code.
The chart plugin also restarted its local helper during the final redraw;
cold restoration correctly refused while that helper remained. Native-qualified
cleanup subsequently verified its exact pipe server/process identity and issued
one source-defined local shutdown packet; its retained handle reported exit 0.
This does not reconstruct the earlier parent application exit code. The post-application-exit navigation database
is byte-identical to the baseline and passes integrity/raw-storage comparison.
The final INI has 13 changed keys, including ordinary resized-window persistence
and an appended source-priority observation. A complete cold inspection and independent exact per-key review now pass.
Restoration reversed only the temporary connection byte, restored all four
quarantined DLLs and published the verified adopted baseline. No application
was automatically launched; the next installed session requires a fresh review.
The configured N2K port was absent from the earlier inventory; at 04:06 UTC the
read-only Windows inventory reports **Actisense NGT (COM8)** present and healthy,
alongside USB Serial COM4. Port presence does not establish sensor freshness or
accuracy. Tailscale, SSH and RustDesk remain running automatically. A
closed-copy navigation-database integrity/content baseline is also
recorded privately using the [byte-preserving audit](installer/navigation-preservation-audit.md).

The actual shared profile's `opencpn.ini` was already zero-filled at first
inspection (21,380 bytes; last written 2026-09-20). A same-sized nonzero temporary
INI from the same write interval is preserved as a potential recovery source.
After the official upgrade, the exact verified temporary copy was restored with
an atomic replacement. The corrupt original and candidate remain backed up; all
other profile contents and owner/group/permissions were verified unchanged.
Connection values were not edited and no application was launched. A separate
30,851,001-byte managed-plugin/metadata backup is verified.
[Profile recovery evidence](evidence/beta2-boat-profile-recovery.json).
The original cold, hash-verified recovery set remains complete: 2,123 application files and
539 profile files (1,318,897,252 bytes). The earlier incomplete attempt remains
separate and is not an accepted backup.
A second cold recovery set now preserves verified stock **5.12.4** plus the
recovered working INI: 2,123 application files, 539 profile files and
1,318,975,173 bytes. Every copy and final source inventory matched; independent
executable/INI hashes also match. The original pre-upgrade set remains retained.
[Working-state backup evidence](evidence/beta2-boat-stock-working-backup.json).
Private chart/profile content remains on the boat PC or in ignored local
inspection evidence, never in the repository.

The normal profile includes an output-capable NMEA 2000 connection and enabled
pilot plugins. The active temporary commissioning transaction makes that
connection input-only and quarantines the four reviewed output-capable or
unqualified plugin DLLs before the stock launch. No physical control command
has been sent. Third-party outputs are addressed independently of OpenNav's
own control switch. Saved user diagnostics show prior live
GPS, heading, wind and depth; this is not current-session hardware acceptance.
Reported desktop mode is 1920×1080 and the saved application DPI is 144;
1280×800 physical-display acceptance remains outstanding.

The old Developer Preview portable folder has been retired by an atomic move
into the boat-local recovery archive, with ownership hash checked before and
after and a durable recovery journal. No profile/chart/user file was deleted.
Beta 1 remains available until a known-good replacement can be installed. Six
obsolete download ZIPs (five identical accepted Beta 1 archives and one accepted
Developer Preview archive, 912,982,707 bytes) have also been moved into versioned
boat-local recovery storage. Every ZIP matched its accepted release SHA-256
before and after the atomic move; durable records retain the original location.
No unrelated Desktop content is included in feedback documentation.

### Beta 2 implementation and validation under way

- Production defaults to `XNAV_ENABLE_TEST_FIXTURES=OFF`; deterministic sources
  remain in separate test code. Package/installer self-tests must reject a
  fixture-enabled executable.
- Alerts share the fixed top status area; the primary rail has four visible
  values without a narrow scrolling viewport. Center and the current palette
  have explicit labels. System uses a page rather than a tall popup.
- Route/energy, instruments, settings and manual pilot layouts are being
  refined against the reference. Native and boat visual review are pending.
- Integration fixes cover chart-layout restoration after actual settings
  reconfiguration, whole-metre anchor labels, safe cleanup of XNav-owned anchor
  marks and copied chart context/Go To actions.
- Late-created input-only loopback GPS and actual AIS decoding pass 17 grouped
  Linux object/integration checks in the earlier iteration. The expanded suite
  now passes 21 groups with 18 screenshots, including copied waypoint context,
  compact chart cards, settings return, active-route arrival/completion/deletion
  and stale modal selections. Earlier UTC+2 testing exposed and fixed AIS observation
  clock conversion that made fresh targets appear two hours stale. Four added
  integrated clock tests pass in UTC and UTC+2. Sensors now distinguishes no AIS
  reports, current reports and stale/lost reports using actual target timestamps;
  decoder existence alone does not imply reception. Boat reception remains pending.
- Latest local portable contract suite passes **67/67**, including AIS reception
  health, native-frame/chart-layout geometry, current route-summary selection
  and pairing diagnostic layout observations with native window bounds. The integrated Linux build
  passes **110/110 individual CTest cases** under the prescribed sequential
  invocation (excluding the duplicate upstream aggregate). The separate
  fixture-free Linux product previously passed 110/110 and five loader/resource
  self-test checks. Seven actual-pointer workflow groups pass with eight
  screenshots: orientation, context dismissal, waypoint create/Go To/stop, route
  point entry/Undo, cancellation and named save with a read-only database audit.
  [Chart workflow and route-lifecycle review](design/reviews/beta2-chart-workflows.md).
  These are local development results, not exact-release qualification. Native
  and boat execution of the expanded workflows remains pending.
- Beta 2 installer wizard, versioned maintenance and boat scripts are implemented
  but their new native lifecycle gates have not yet run.
- First CI candidate `15e5a265` exposed a Windows-only line-ending assumption in
  a source-archive test. The corrected candidate `a0af22c248f8a81bb8068ccf3f64bd921478060f`
  was exercised in [CI 36266809347](https://github.com/ThereptileII/Work/actions/runs/36266809347).
  Linux and Windows contract jobs pass. Native MSVC integration builds and all
  102 integrated CTest cases pass, followed by mode/persistence checks. Its
  navigation UI smoke stopped at an old source-caption assertion after alerts
  moved into that header slot; the replacement asserts the critical-position
  alert and retained stale ages explicitly. Six native frame/mode screenshots
  were reviewed in [the first native review](design/reviews/beta2-native-a0af22c.md).
  Complete Windows, packaging and boat gates remain pending. No Beta 2 release
  acceptance is implied; later candidates below supersede it.
- Follow-up source `ee720380ac72b7459f5ff0538acecb9c2b650180`
  ([CI 36269508823](https://github.com/ThereptileII/Work/actions/runs/36269508823))
  matches all 493 local tracked source blobs and file modes. Its Windows boat-tool
  fixture exposed .NET Framework ZIP backslash entries; the fixture now emits
  the same slash paths as real Python packaging while rejection of unsafe ZIP
  paths remains tested. The correction passes 21 isolated native Windows checks.
  Linux build/CTest and navigation, recording, route, marine, Signal K and pilot
  transport gates pass, but the object workflow stopped at compact waypoint
  Details. This candidate was not qualified. The chart-card layout/focus
  correction and repeated local checks below supersede that failed interaction.
- The authorized official-stock upgrade uses a separate reviewed visible-wizard
  driver, not the OpenNav installer or silent replacement. Native PS5.1 passes
  86 policy/helper checks, 21 filesystem groups and 23 profile-preparation groups.
  The bare OpenCPN registration is explicitly identified as the inspected RTL-SDR
  plugin, with complete value/type and uninstaller preservation. Temporary-file
  recovery tests confirmed Windows adds the DACL AutoInherited metadata bit;
  ownership, protection and every ordered ACE must remain exact. The independent
  real-profile recovery subsequently passed 25 native groups and was applied with
  a separate journal, exact candidate hash and complete preservation checks.
- Candidate `06f3bc32879cff4b1822ff53710388b1146d05a7`
  ([CI 36271371549](https://github.com/ThereptileII/Work/actions/runs/36271371549))
  includes the corrected ZIP fixture, X11 pointer-target observations and native
  boat maintenance tests. All 506 local tracked source blobs/modes match the
  published tree. The object harness passed two additional local 17-group runs;
  this candidate was superseded before full qualification.
- Follow-up `a6c15b22c2344a69437e4ef7d9d738fe3f1aed50`
  ([CI 36272268284](https://github.com/ThereptileII/Work/actions/runs/36272268284))
  exposed a Windows Server file-replacement ACL merge in the disposable profile
  preparation suite. It is not an accepted build. Boat Windows 11 recovery had
  already passed exact permission and content verification; the affected helper
  was subsequently hardened for both environments without loosening permission checks.
  Maintenance suites now run in a separate mandatory native CI job, so MSVC/UI
  validation can proceed concurrently. Final publication still requires every
  maintenance suite, and adds the reversible commissioning transaction tests.
- Boat-local disposable PowerShell 5.1 checks now pass 32 profile-preparation,
  21 commissioning transaction and 11 launch-verification groups, plus the
  existing 21 boat-tool groups. Portable counterparts pass 21, 12 and 11 groups.
  Real commissioning has not yet been applied. Its one-byte input-only change,
  reviewed plugin quarantine, interrupted restoration and complete helper-file
  launch verification are independently tested; no application or physical
  command was launched during these checks.
- Product candidate `62e28e5dfe42e96531f00dd88686634c167b0db8`
  ([CI 36273516935](https://github.com/ThereptileII/Work/actions/runs/36273516935))
  matches all 515 committed local source blobs/modes. Both contract jobs pass;
  native MSVC and 102 integrated CTest cases pass, followed by selected-input,
  Signal K and recording checks. The Windows pilot interaction gate stopped at
  an unavailable course-button caption and is under investigation. Linux reached
  the required elapsed-time stability test. Neither platform is yet accepted.
  The maintenance job exposed a Windows Server security-descriptor difference:
  `Get-Acl -Audit` can omit inherited ACE flags in its DACL view. The corrected
  implementation uses the ordinary owner/group/DACL as the preservation baseline
  and the audit view only to reject unsupported SACL metadata. It never copies
  the transformed audit DACL or relaxes ordered-ACE checks. The corrected split
  passes 34/21/11 boat-local temporary-file groups and the same native Windows
  Server suites. Tooling commit `586df3875a8157e17b27da782b131420d9e6fbd6`
  passes all six maintenance/review suites (266 groups) in
  [CI 36274989439](https://github.com/ThereptileII/Work/actions/runs/36274989439).
  This qualifies that tooling revision only, not the application.
  The native window
  review helper passes 93 policy/compilation groups on Linux and Windows;
  actual UI actions remain unperformed. Both restored chart directories and
  the 442,271-byte Windows chart database are present; rendering is still pending.

- Candidate `c8ff99eaf448147a17c02d99f3a71d430763a618`
  ([CI 36275173742](https://github.com/ThereptileII/Work/actions/runs/36275173742))
  includes explicit UTF-8 conversion for native degree/temperature captions and
  readable alert actions. All 528 committed local blobs/modes match the remote
  tree. Its native maintenance job passes, including 16 new source-checkout
  groups. The exact source was retrieved into the managed boat workspace without
  changing or launching the application.
  [Source-only evidence](evidence/beta2-boat-source-c8ff99ea.json).
  Native MSVC and **102/102 integrated CTest cases** passed; the loopback pilot
  interaction also passed, and two native screenshots confirm correct degree
  glyphs. The object gate then failed before its first chart screenshot because
  the harness had left the application at its default **896×532** while testing
  for a 1280×800 layout. This is not an accepted application build. The harness
  now explicitly sizes the native window before validation and checks chart
  dominance, four visible rail values and control separation against actual
  frame/client geometry. It retains coastline checks and captures failure
  evidence without resizing a pending modal. The replacement passed the local
  21-group object run; its native rerun remains required.
  [Native review](design/reviews/beta2-native-c8ff99ea.md) and
  [partial native evidence](evidence/beta2-windows-c8ff99ea-partial.json).
- Tooling-only commit `6f9ee027518307ac373d1080edf38534b1af1ff7` passes
  **288 checks across seven suites** on native Windows Server 2022 / PowerShell
  5.1 in [CI 36275798350](https://github.com/ThereptileII/Work/actions/runs/36275798350).
  This includes source-checkout and launch guards in addition to profile recovery,
  reversible commissioning and window-review policies. All 533 source blobs
  match the published tree. [Tooling evidence](evidence/beta2-windows-tooling-6f9ee02.json).
  This is maintenance-tool qualification only: no application or hardware command
  was launched, and it does not qualify the current product or boat deployment.

- Candidate `6160d3e4bcd924853462f96a32f1b502a72a2884`
  ([CI 36277024981](https://github.com/ThereptileII/Work/actions/runs/36277024981))
  passes native MSVC, **102/102 integrated CTest cases** and the expanded
  21-group navigation-object suite. Later Windows checks fail and no product
  package is accepted or deployed. Pointer diagnostics stop updating after a
  North-to-Course interaction; the Linux reproduction continues updating.
  Bounded, opt-in tracing is added only to the fixture build to distinguish a
  stalled event loop from diagnostic publication failure on the native rerun.
  No speculative navigation behavior change or timeout relaxation is made.
  Separately, the preview helper expected the old destination caption; the
  plugin/route chart checks expected the old flat Settings and route-save flow.
  Those checks now follow the actual UI while retaining geometry, plugin paint,
  route identity and persistence assertions. The 150% rail failure paired
  pre-resize diagnostics with current HWND bounds; the saved native image and
  subsequent independent bounds show all four values fitting. The new barrier
  synchronizes observation times before the same containment/touch assertions.
  [DPI investigation](design/reviews/beta2-dpi-observation-6160.md).
  [Failed native evidence](evidence/beta2-windows-6160d3e4.json) and
  [four-image review](design/reviews/beta2-native-6160.md) preserve the exact
  downloaded artifact; none of these partial results implies acceptance.
  All Linux functional CI steps passed; its three-hour endurance step was later
  canceled by the superseding candidate and is not accepted. The separate fixture-free Linux product passes
  110/110 integrated cases, five loader checks and seven synthetic-data exclusion
  groups. These results do not replace the failed native or pending boat gates.
- Tooling-only `5b597bff0df7295a0ab1f3edbc3d45217582b56d` passes **340 checks
  across seven native Windows suites** in
  [CI 36277973803](https://github.com/ThereptileII/Work/actions/runs/36277973803).
  This extends fixed, read-only waypoint/AIS selection policy; no physical UI
  action or hardware command was executed. Automatic in-app restart remains
  outside the independently audited cold-launch procedure.
  [Tooling evidence](evidence/beta2-windows-tooling-5b597bf.json).

- Candidate `12100a74ff619b7268a6e20902bd9d0de3b53b39`
  ([CI 36279275114](https://github.com/ThereptileII/Work/actions/runs/36279275114))
  passes 67/67 portable contracts on each platform, native MSVC and 102/102
  integrated CTest cases, 340 maintenance checks, all 100/125/150% DPI checks
  and both chart phases. The OpenGL-requested phase used the verified upstream
  software fallback; hardware OpenGL remains open. Rail/alert bounds and
  Legacy/Safe return coastlines pass at each scale. Actual ENC chart switching,
  route creation/editing and plugin-manager paint pass. Pointer Course-up still
  stops periodic updates after the callback has returned; fixture autopilot
  interaction also fails. Packaging, installer and native endurance were skipped.
  This is a failed development candidate, never deployed.
  [Evidence](evidence/beta2-windows-12100a74.json) and
  [four-screen review](design/reviews/beta2-native-12100.md).

- Replacement `b565553d284e91f9aea552b3993428206de5247d`
  ([CI 36281109419](https://github.com/ThereptileII/Work/actions/runs/36281109419))
  passes both 67-contract jobs, native integration, the Course-up pointer gate
  and the complete fixture UI suite. Its fixture-free native build, 100/125/150%
  DPI/touch checks and chart/plugin gates also pass. Software basemap rendering uses a
  copied viewport instead of invalidating the live quilt during paint. Native
  diagnostic JSON is read explicitly as UTF-8. Extracted portable testing fails
  because its startup observer counts earlier initialization markers after
  upstream log rotation; the actual Safe-to-XNav window has coastline content.
  The corrected observer requires a fresh startup/finalization sequence and
  retains chart/clean-exit checks. Installer and native endurance were skipped;
  the candidate has not been deployed. [Failure evidence and repair](evidence/beta2-portable-b565-log-rotation.json).
  The downloaded full native evidence archive matches the API and upload-log
  SHA-256 `a548bac982aa8406b960584dd88b8a52071e04d546a5949da8ba2e0f78e78cb2`.
- A separate plugin-workspace repair restores the normal upstream perspective
  while preserving only XNav-owned temporary panes. It uses exact manager and
  window identity. Local fixture and production builds pass, along with 110
  integrated cases, 22 object groups and software/OpenGL chart checks including
  actual floating Dashboard captures. The native and boat gates remain open.
  [Workspace review](design/reviews/beta2-plugin-workspace.md).
- The optional commissioning restart boundary passes 886 protocol checks and
  359 native assertions across 24 marker-only process scenarios. This is a
  deliberately armed, one-use recheck of the saved profile, plugin trees and
  original process identity before launching a replacement. Normal unarmed
  mode switching is unchanged. The full broker, actual Prepare/Arm procedure,
  packaged application capability and boat transitions require separate gates;
  these standalone results do not authorize an older helper or application.
  [Native transport evidence](evidence/commissioning-restart-fe397.json).
- Current local contracts pass **70/70**, including packaged guard capability
  and startup-log rotation. Guard integration passes 110/110 in both fixture and
  fixture-free Linux builds; the separate actual loader test preserves the
  normal profile. Instrument whitespace refinement retains value fonts and
  data-quality labels and passes the full Linux UI/scenario smoke.
  [Instrument/energy viewport review](design/reviews/beta2-instrument-density.md).
- Actual full restart-broker native tests now pass six marker-only cases and 48
  assertions, including refused output/plugin changes and uncertain consumed
  permits. The scheduled-task policy still needs a native replacement: observed
  Task Scheduler uses an account name for its principal and null for no triggers.
  Both representations must be verified without weakening exact SID/action
  identity. Actual Prepare/Arm/Collect and real application transitions remain
  pending. The separate stock-after-uninstall review tool has portable coverage
  but has not launched stock OpenCPN on the boat.

## Accepted Beta 1 baseline

**Beta 1 software qualified: all ten CI jobs passed, downloaded release verified, native visual review complete.**

Beta 1 (`0.3.0-beta1`) is the software package for desktop and staged boat
commissioning. It is not approved for navigation or production use. Physical
boat acceptance is separate. No autonomous steering is implemented.

Qualified application/source commit: `a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a`.
[CI run 36137990012](https://github.com/ThereptileII/Work/actions/runs/36137990012).
[Acceptance and download verification](evidence/beta1-a3e6e08-accepted.json).

OpenCPN remains pinned to **5.12.4 /
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`**. Windows retains the supported
**x86 application/plugin ABI on Windows x64**. The `win64` suffix describes the
host. The pristine upstream checkout is unchanged; the reviewed integration
patch is documented in [upstream patches](upstream-patches.md).

## Download and first test

[Download OpenNavX-Beta1-Windows](https://github.com/ThereptileII/Work/actions/runs/36137990012/artifacts/10876531379).
Extract that outer Actions archive to find:

- `OpenNavX-Beta1-Portable-win64.zip`
- `OpenNavX-Beta1-Setup.exe`
- `OpenNavX-Beta1-Test-Guide.md`
- `OpenNavX-Beta1-Boat-Commissioning.md`
- `OpenNavX-Beta1-source.zip`
- `SHA256SUMS.txt`

Extract the complete portable ZIP to a short writable folder, then run
`Run-XNav-Demo.cmd`. Confirm DEMO and real coastline/chart content. Portable
modes use an isolated profile. Before Setup, back up the normal OpenCPN profile
and follow the [desktop guide](beta/OpenNavX-Beta1-Test-Guide.md). Setup uses
the normal shared profile and runs beside the untouched original executable.
Choose Update for an existing Alpha installation. Alpha's historical install
directory/Start-menu folder remain to preserve upgrade continuity.

## Exact-revision qualification

| Gate | Result |
| --- | --- |
| Portable contracts | 60 suites on each platform; ten additional restart repetitions separately |
| Linux integrated | 106 cases; live input, route, object/AIS/anchor, recording, pilot, recovery, UI and chart gates |
| Native Windows MSVC | 98 cases; same functional boundaries, extracted portable launch and three recovery cycles |
| Installer | 29 real lifecycle checks, including accepted Alpha upgrade and failure recovery; 39 filesystem checks in each 32/64-bit PowerShell 5.1 host |
| Display | Real 1280×800 at 96/120/144 DPI; mouse, injected touch tap/pan, fullscreen, Night and mode returns |
| Charts/plugins | Public NOAA ENC plus deterministic coastline checks; Dashboard, GRIB and WMM; Windows software fallback, Linux software/llvmpipe OpenGL |
| Endurance | Three actual hours each; Linux 10,800.175 s / Windows 10,800.109 s; 1,080 samples and 540 page actions each |
| Download/review | Five release hashes, 1,009 portable file hashes and 4,814 source ZIP entries verified; 40 native Windows and three Linux images individually reviewed |

Repeated tests are not counted as additional distinct cases. Native Windows
images are authoritative; injected touch does not establish physical touchscreen
acceptance. Hosted Windows rejected hardware OpenGL, so target GPU testing
remains open. Exact measurements and screenshot hashes are in the evidence. Sustained resident
growth was 184 KiB on Linux and 2.97 MiB on Windows; Windows private growth was
3.73 MiB. Linux descriptors/threads and Windows handles/GDI/USER had zero median
growth. Mean CPU was 1.65% / 1.10% of one core respectively. Maximum page
observation was 1.25 s / 1.73 s, including the 1 Hz diagnostic observation delay;
these are not paint-latency measurements or a universal leak-free claim.

## Implemented Beta product

| Area | Implementation and boundary |
| --- | --- |
| Navigation/shell | Real chart; touch controls, follow/orientation/measure, contextual pages, configurable rail, Day/Dusk/Night, fullscreen, persistent alerts and Legacy/Safe restart. |
| Routes/waypoints | OpenCPN-owned storage/progress, guarded basic creation/edit/activation/reversal, next point and remaining-distance snapshot. Advanced/protected cases remain in Legacy. |
| Marine data | Selected OpenCPN navigation plus NMEA 0183, 15 supported N2K PGNs and own-vessel Signal K; physical NAME, source precedence/age/cadence and explicit invalid/unavailable/uncertain state. |
| Propulsion/battery | Standard marine acquisition plus explicitly bound boat adapter; independent producer-expiry contract; high-voltage 127751, SOC 127506, gear/regen and motor-temperature meaning where configured. No PC Leaf CAN decoder. |
| Energy | Explicit capacity/reserve/sign/auxiliary assumptions, empirical curve import, filtered speed/power calibration export, advisory route energy/range/arrival SOC with quality and blocking reasons. |
| Commissioning | Per-item health/PGN/instance diagnostics, bounded opt-in recording, deterministic REPLAY with historical ages and controls disabled, privacy-limited field diagnostic ZIP. |
| SmartNav/AIS | Turn/timeline/energy advice, OpenCPN CPA/TCPA/alarm context, modern target selection/card/detail and expiring chart highlight. No autonomous command path. |
| Anchor/alerts | Normal OpenCPN anchor watch, distance/history/depth/wind/battery and a persistent actionable alert layer. No invented drag prediction or automatic standby. |
| Manual pilot | ST4000 adapter, exact identity binding, bidirectional TCP Actisense transport, six manual commands, feedback/timeout/rate limits and every-start OFF. Simulator retained; TRACK/WIND unavailable. |
| Hazards/radar | Tested bounded corridor/provider architecture and unavailable/uncertain semantics. No accepted live ENC corridor provider or Pathfinder display/control adapter. |
| Robustness | External JSON/input bounds, malformed/stale/reconnect tests, two-failure startup Safe fallback, serial-discovery resource ownership and installation failure recovery. |
| Distribution | Hash-gated side-by-side per-user Setup; real Alpha update, immutable generations, repair/rollback/uninstall; stock executable/shared profile preserved; isolated portable recovery. |

Contracts: [marine input](marine-input-contract.md), [boat producer](boat-propulsion-contract.md),
[recording/calibration](recording-replay-contract.md), [energy](energy-model.md),
[pilot](st4000-beta-contract.md), [AIS](ais-beta-contract.md),
[alerts](operational-alerts.md), [display](display-beta-contract.md),
[robustness](beta-robustness.md), [installer](installer-transaction-contract.md).

## Stable foundation and feedback

The reported Legacy → XNav blank-chart regression remains closed by `16dbaf7`
and repeated exact-release coastline/ENC/mode-return checks. The active-leg
console overlap was closed in accepted Alpha `08bc92f`. The user's Alpha
acceptance and deliberate deferrals are recorded in [the Beta plan](beta1-plan.md).
The immutable remaining-route contract retains normal upstream active-point
range plus subsequent stored legs; no independent navigation calculation.

Earlier accepted increments and rejected candidates remain in
[development history](status-history-beta-development.md) and [evidence](evidence/).
Failed runs are not release acceptance.

## Post-build visual review and deliberate deferrals

- Diagnostics retains an old **ALPHA 1 / NOT FOR NAVIGATION** warning caption.
  The separate Beta 1 version, commit and compiler fields are correct. This is
  a cosmetic label issue, recorded for the next feedback-driven increment.
- At 150% the transient System popup partly covers the Alerts button. Critical
  alert text remains visible; press Escape or click outside to dismiss the
  popup before opening Alerts. This presentation refinement is deferred.
- With larger DPI or an alert, use Up/Down or pan to reach lower rail/page
  values. The heading card can be partly outside the viewport even at 100%
  with an alert; configure the rail order to prioritize desired instruments.

These acceptance notes follow the build; they do not change the downloaded
binaries or their bundled guides. Of the 78 automated DPI captures, a subset
is included in the 40 individually reviewed Windows images. Fullscreen was
1920×1080; normal window gates were 1280×800.

## Remaining physical/product gates

Follow [boat commissioning](boat-commissioning.md): read-only first, propulsion
comparisons/calibration next, pilot status next, then individual deliberate
commands in a secured safe environment, and supervised underway trials last.
The C6 producer patch compiled but was not flashed. The complete PC → gateway
→ ESP32 → SeaTalk → ST4000 path still needs physical feedback/loss validation.
Physical touch, target GPU/navigation PC, broader plugins, radar hardware, live
ENC corridor integration and at-sea operation remain open. Beta is unsigned;
native/Legacy dialogs can remain bright. [Known limitations](beta/KNOWN_LIMITATIONS.md).

Stop at Beta 1 for user/boat feedback. No production-release work or autonomous
steering is authorized by this acceptance.
