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
[cache correction](evidence/scrum266-selector-cache/README.md) no…40499 tokens truncated…ied
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
