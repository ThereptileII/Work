# SKAGER status — 2026-10-05

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
assets but failed on the final corresponding-source ZIP; delivery is incomplete
until a separately authenticated publication-only continuation succeeds. The
failed upload remains recorded; no rebuild or native retest is required.

The original actual-Lifecycle step also passed all 46 checks,
including a deliberate six-second child startup delay, authenticated after about
7.8 seconds. Suspended-loader healthy/wrong-hash cases also pass. This supersedes
the pending focused corrections below, without turning inert fixture results
into packaged application or boat acceptance. The earlier isolated green probe
did not execute the delayed Lifecycle script; that omission remains recorded.
The qualified 119-file boat tool bundle is staged separately. No boat application,
profile or public update configuration has changed. The exact verified Setup is
staged on the boat but has not run: SCRUM-310 tracks substantive profile changes
inside the active commissioning transaction which the existing restoration tools
cannot yet preserve. The old application remains running; old baseline bytes
have not overwritten the current configuration. A controlled signed channel
and protected signing operations (SCRUM-72/73) are still needed for SCRUM-23's
authenticated no-update/offer/Later/offline boat acceptance; an unconfigured
bootstrap is not a substitute for those scenarios.

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
confirms the…65012 tokens truncated…s earlier initialization markers after
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
