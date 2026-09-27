# OpenNav X status — 2026-09-27

**Beta 2 development is in progress; its first development package is installed
on the boat, but it is not qualified for release.**

The user's Desktop feedback has been read completely and recorded in
[boat Beta 1 feedback](feedback/boat-beta1-feedback.md). The approved design
reference is the Beta 2 baseline. Current work separates fixture-enabled CI
executables from the installed product, refines shared visual components and
navigation workflows, and adds repeatable boat deployment and maintenance tools.
The accepted Beta 1 results below remain historical evidence, not Beta 2 gates.

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

`7827acb7c8b0d708285bd26a4c48d545dd64d139` is running the complete gates in
[CI 36304661282](https://github.com/ThereptileII/Work/actions/runs/36304661282).
All **699** mapped blobs/modes match local
`a883f3fb7c729773e323d9234d95dda1f62fcacf`; the pinned baseline and eight unrelated
repository blobs are preserved. It includes the autopilot wording refinement,
bounded chart-pan tools and the DPI harness correction below. No replacement
product has been deployed.

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
