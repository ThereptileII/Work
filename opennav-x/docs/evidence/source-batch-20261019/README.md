# Recent bug-fix audit — 19 October 2026

Audit window: 15–19 October. The source review starts at installed baseline
`9feae59ce0ef1f54107c3c51f920085297ea7bc8` and includes the integrated corrections
through `0b2bd995` in `source-batch-20261011`. This is an evidence record for the
existing Jira work, not another backlog or a release-acceptance certificate.

Earlier source-only instructions explain the deferred checks in the linked
contracts. On 19 October the user authorized the audit, implementation/build
repairs and one coordinator-owned Staging build and boat installation. This
record reports only completed source review and the bounded checks below. It
does **not** assert that a replacement Staging package was built, installed or
accepted on the boat. No new design review or endurance run was performed.

## Reports, source response and evidence

| User report / Jira | Source response reviewed | Evidence and limit |
|---|---|---|
| Route naming and hard-to-find Finish — [SCRUM-298](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-298) | Shared name editing; naming across native completion paths; prominent Finish/Undo/Cancel; readiness and draft-focus guards (`2f2554cd`, `1984e8b8`, `1764c8d5`). Audit correction `867e060d` makes route/waypoint/track attribute writes atomic and avoids an unchecked duplicate save after successful XNav route preparation. | Native extracted-method SQLite and Finish results below. Full GUI editing, restart persistence and native Windows behavior remain unqualified. |
| Native pilot Track/Wind — [SCRUM-313](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-313) | Exact mode allowlists, measured AUTO prerequisite, coherent native route-output receipts and no inferred waypoint-turn acknowledgement (`4ff89a7b` through `25327428`). Separate controller changes guard measured apparent-wind forwarding and late echoes. | [Source contract](../../pilot-track-wind.md). Application source is not evidence of bridge delivery or physical acceptance. Controller firmware has not been flashed for this work; Track/Wind commissioning remains unperformed. |
| Arrival distance and passage width — [SCRUM-373](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-373) | Optional route-specific radial arrival policy and geographic passage overlay; stock behavior when unset; shared waypoint radii preserved (`862cf195`, `811e666d`, `fb49bdf9`). | [Policy and limits](../../route-passage-settings.md). Authored failure/policy cases and source review do not establish GUI arrival-loop or software/OpenGL runtime correctness. |
| Online AIS names and rapid expiry — [SCRUM-16](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-16) | Merge independent names/static fields by MMSI; fallback label uses MMSI; online aging/stale/lost/expiry at 3/10/20/30 minutes (`a392315d`, `fa6a86cf`, `fb4918d7`). Static updates do not renew position age; local AIS policy is unchanged. | [AIS contract](../../online-ais.md#cache-and-time). Source and revised cases reviewed; live reconnect/aging and replacement chart labels remain unverified here. |
| Small weather-box text — [SCRUM-357](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-357) | Increased helm text size (`6ad9ca44`); later forecast presentation refinements are recorded under SCRUM-378. | Source review only; no new native legibility result. |
| Windows launcher/shortcut identity — [SCRUM-236](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-236) | Approved icons embedded in package intermediates and secure launcher/installed shortcuts branded (`2f50b4e2`, `4c9b152a`). | Package-source changes; Windows installation/Search identity is not qualified by source inspection. |
| Alarm settings hard to find — [SCRUM-374](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-374) | Alarms & warnings remains reachable from an empty bell drawer and Settings → Navigation; owned readback and guarded links expose existing AIS/sound/anchor/energy/freshness settings (`8db2b8e0`, `3ce82695`, `a9356f61`). | [Operational boundary](../../operational-alerts.md#finding-and-adjusting-alarms). No new alarm policy, measured-depth alarm or suppression semantics. Native readback, interaction and replay acceptance remain open. |
| Yellow measure label, unclear chart tools and wrong waypoint placement — [SCRUM-375](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-375) | Measurement-only themed label preserves upstream bearing/distance/unit/placement calculations; persistent hints and Cancel/Done; one-shot real chart tap before the name sheet (`804347bb`, `594950bc`, `8183ae7c`). | [Tool contract](../../chart-tools-and-first-start.md). Source review covers queued generation/weak ownership, cancellation and replay boundaries. Touch/pan/object overlap, actual coordinates, software/OpenGL and Legacy/Safe runtime checks remain open. |
| Stock first-start presentation — [SCRUM-341](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-341) | Early SKAGER title and full native safety notice; original consent/receipt ownership retained; automatic XNav setup uses Boat Setup (`cd930e30`, `ab9c1542`). | [First-start boundary](../../chart-tools-and-first-start.md#first-start). Agree/Cancel fixture source exists; no new consent pass asserted. Startup uses Day palette rather than persisted Dusk/Night. Plugin/recovery dialogs and Windows Search identity are outside this fix. |
| Serial startup crash — [SCRUM-376](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-376) | Shared listener-registry synchronization and replacement-move registration cleanup (`8b109bde`). | Four ordinary and four ASan/UBSan scenarios pass; original source fails both expected negative controls. This proves the corrected synchronization/registration boundary, not the unique cause of the captured crash or boat startup stability. |
| Record/saved tracks and track-layer control — [SCRUM-377](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-377) | Native recording drawer, saved-track library and XNav master display filter; owned snapshots, replay and fresh-position gates; per-object/layer flags preserved (`8810325e` through `0b3189cc`). Daily restart/replacement corrections and atomic attribute saves included. | [Track contract](../../track-recording.md) and SQLite results below. Actual recording, GPS loss/recovery, daily merge, restart, filtering/hit-testing and Windows/boat interaction remain unqualified. Sailing tracks are separate from pilot TRACK. |
| Rail details/presets, scaling, compass, chart and surface refinement — [SCRUM-378](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-378) | Very Light option; immediate 100/125/150%; four quantity selectors and named presets; source/age detail; coherent explicitly labeled apparent-wind fallback; profile/route identity, rounded surfaces and forecast refinements (`29281137`, `c20c9a08`, `d1c8df75`). Compass now cycles North/Course/Head (`608b8375`). | [Item-by-item review](../../design/reviews/scrum378-source-refinement.md) records all 14 requests. Measured depth is retained but the unsafe chart-depth comparison is withheld for incompatible datums. Source review preserves replay resets, original observation age and rail-only saves without source reconfiguration. Native fit, paint and DPI acceptance remain open. |
| “No active route” and other periodic flicker — [SCRUM-378](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-378) | Idempotent roles/themes/settings copy, correct absent-age comparison and top-header-only MSW buffering (`01f876b1`). Audit `0b2bd995` also removes unchanged ChoiceField Enable refreshes. | [Repaint analysis](../../design/reviews/scrum378-source-refinement.md#2026-10-19-ui-flicker-follow-up). No route-text oscillation was found; buffering is a mitigation, not a reproduced cause/fix. Live 250 ms assessment and real freshness/state changes remain intact. |

## Additional audit corrections and executed checks

- **Navigation storage (`867e060d`): 36 SQLite checks and seven native Finish
  checks passed**, as reported by the coordinator from the isolated
  [native persistence fixture](../../../tests/navobj_persistence/README.md).
  It extracts actual patched methods and uses real in-memory SQLite failure
  injection, with inert object/UI boundaries. It covers failed prepare/write/
  commit, rollback, nested transactions, route/point order, track attributes and
  replacement. It is not filesystem durability or a complete application test.
- **Observable (`8b109bde`): four ordinary plus four ASan/UBSan cases passed**
  on Linux/GCC 16.2.1 and retained wxWidgets 3.2.11. Two original-source negative
  controls failed at their expected assertions. The
  [fixture record](../../../tests/observable_concurrency/README.md) identifies
  the patch hash, retained logs and corrected initial fixture failures. No
  production exception was attributed to a plugin merely from log ordering.
- **Preset downgrade preservation (`0b2bd995`, worker `7b6dddc0`):** presets
  now use `/OpenNav/RailPresetsV1`; unchanged v1 `/OpenNav/AlphaSettings` retains
  old-reader-compatible bytes. Earlier inline records migrate on explicit save;
  absent, empty and corrupt separate records have distinct tested handling.
  Save, SaveVessel, SaveBoatSetup and RestoreBackup include rollback of every
  touched key. Portable backups include presets; restoring an older backup
  without presets clears them. No read-time migration write occurs.
- **Focused settings verification:** four isolated targets compiled; all six
  registered settings groups, the backup codec fixture, the backup/store fixture
  and the boat-setup/store fixture passed (the latter two under Xvfb). These
  checks include failed writes, restart readback, backup restoration, source
  preservation and live-energy freshness. A first bare `settings_tests` call
  printed “Choose group”; the corrected run executed all six registered groups.
  Retained worker output is in `.tmp/audit-ui-build` in `scrum378-audit-ui`.
- **125% and repaint (`0b2bd995`):** sheet actions now use the shared scaling
  path at 100/125/150%; ChoiceField refreshes only on an actual Enable change.
  Sheet/Choice fixture source was corrected for the current field dimensions;
  these UI cases were not executed in this worker. The existing ChoiceField
  fixture lacks a maintained CMake/tool registration; no coverage is inferred.

Manual integration review covered UI declarations/call sites, owned snapshots,
queued callback lifetime, mode/replay guards, native model versus GUI link
boundaries, maintained CMake lists and patch/source/dependency-receipt inventories.
That review is not an MSVC compile, whole-application patch/build pass, installed
source-package check or runtime acceptance result.

## Explicit remaining evidence boundaries

The specific northern feature reported as a lighthouse is still unclassified.
Existing LIGHTS and qualified support aliases were reviewed; a generic “Light”
label does not justify replacing all lights with a tower. No correction of that
particular feature is claimed.

The [17 October boat record](../scrum17-9fe-boat-resume-20261017/README.md)
remains failed acceptance for installed `9feae59c` / Beta 2.5. Historical native
Staging and plugin-loading observations do not qualify these later sources.
Startup/crash recovery, the actual no-route flicker, chart rendering, sensor
freshness transitions, profile preservation and supported native Windows
interaction remain candidate-specific checks. No new boat runtime pass, physical
command, controller flashing, Track/Wind commissioning, Production promotion or
public release is established here. The existing endurance skip remains in
effect; no new design acceptance is claimed.

## Coordinator preflight before candidate freeze

- Full maintained 21-patch preparation/reconstruction passed against the pinned
  OpenCPN revision. Earlier track include-context conflicts were corrected.
- Standalone production UI library and Sheet/Choice fixtures compiled. Both
  fixtures passed under Xvfb, including 100/125/150% field/action sizing and
  selector dismissal/lifetime. Choice already had integrated registration;
  its new standalone target is guarded to avoid a duplicate native target.
- Twelve actual integration/upstream translation units compiled with the
  configured Linux production flags: canvas, GL canvas, frame, track painter,
  integration bootstrap, navigation objects/actions, track recording, settings,
  chart presentation, S-52 and navigation database. This is object preflight,
  not a linked product or Windows qualification. The initial Ninja invocation
  pulled unrelated dependencies and was stopped; direct generated compile
  commands then completed the bounded set.
- Preparation fixtures: 2/2. Corresponding curl-source fixture: 1/1 against the
  retained locked archive. Dependency receipt tests: 13/13; bundle tests: 31/31.
  Distribution inputs and workflow YAML parsing passed. Initial fixture calls
  missing the archive argument and stale patch inventory were corrected; they
  are not counted as product failures or passing runs.
- Independent SDK review rehashed both image-locked retained ZIP/manifests and
  compared all 29 declared source payloads to exact producer Git. Only eight
  application-patch inventory entries changed. Exact producer-bound helper
  hash pairs now cover the two locked producers; no dependency recipe, binary,
  ABI, image/toolchain check or native reprobe was waived. Fresh Windows reuse
  still must pass in CI.

Local coordinator evidence is retained under
`.tmp/delivery-audit-20261019/` (not shipped). The versioned candidate and its
CI/boat results will be recorded separately; these preflights do not authorize
Production or establish physical pilot/lighthouse acceptance.

## First native CI pass and bounded corrections

Run `39237673752` at `c053001d` failed early gates before the full MSVC
application job. It is not accepted evidence for delivery. The retained logs
identified these separate issues:

- The corresponding-source fixture lacked the new persistence/observable
  patches and retained an obsolete count. It now checks all 20 source-package
  patch paths and their hashes; 13 fixture cases pass. Five related lightweight
  source/dependency suites also passed (63 cases).
- The route-focus fixture rejected the intentional XNav StartRoute policy.
  It now executes the actual StartRoute and guards both the explicit-finish
  and stock paths: 316 assertions in each of four desktop/macOS, XNav/stock
  variants. Original regression and missing/unconditional policy mutations
  fail their expected controls.
- The pilot serial fixture passed two clock reads as separate arguments.
  Unspecified evaluation order could make its source sample future-dated.
  Sequencing source capture fixes the reproducible fixture failure; the
  production future/stale rejection remains intact. All three native cases
  pass locally; Windows rerun is still required.
- MSVC rejected a nested name-editor weak-reference init capture. The weak
  reference is now constructed before copying into the callback, retaining
  destruction safety. Focused GCC/Clang compilation passes; native MSVC is
  the remaining authority.
- Current installer icon checks passed; the final test invoked the immutable
  historical installer and incorrectly required its shortcuts to use the new
  explicit icon. That historical-only assertion now verifies inherited target
  icons, while all current-generation exact icon checks remain unchanged.
  Fourteen portable policy checks pass; COM execution remains a Windows gate.

These are classified corrections, not a passing replacement candidate. The
boat remained on `9feae59c` / Beta 2.5 throughout this work; no application or
hardware state was changed.

## Replacement gate findings (before full Windows application build)

Run `39239032704` at `5f02e1a8` passed both contract platforms, native
UI/startup/shortcut checks, pilot serial, boat-maintenance checks, and the
actual installer rollback/interruption fixture. It did **not** qualify a
package: the Linux integrated compile and native launcher packaging failed.

- Linux rejected the wider upstream PGN in aggregate initialization of the
  normalized 32-bit frame. `f814885b` copies the canonical ordered PGN only
  after the unchanged exact-equality guard. The actual unit compiles with
  `-Werror=narrowing`; the original source fails that same compiler check.
  This does not broaden accepted PGNs or permit output without the existing
  pilot, route, position and transport conditions.
- Native diagnostic run `39248589437` reproduced the launcher failure on an
  inert Go 1.27.1 Win32 executable. Windows icon insertion moved its intact
  `.symtab` from file offset 1633792 to 2003968 but retained the old COFF
  symbol-table pointer. Consequently `go version -m` rejected the file. Both
  images and detailed PE/Go parser evidence are retained in the diagnostic
  artifact; this is a packaging defect, not an application crash attribution.
- Independent delivery checks `39239611242` passed Linux and Windows after
  correcting the case-alias test fixture. No production inventory rejection
  was weakened.
- Cold-profile helper run `39240558236` passed the native cold baseline
  (188 groups), adoption (131), and session preservation (86, including
  native reconciliation 229). The job then exceeded its 15-minute bound while
  redundantly repeating reconciliation. Run `39248796997` removes that duplicate
  and runs the three independent suites in parallel; its final result is not
  inferred from the earlier partial job.

The boat still runs the preceding installation. Subsequent corrected package
and boat results must be recorded separately with their exact identities.

## Qualified focused repairs and remaining runtime helper mismatch

- Native launcher diagnostic `39249552874` at `6c748c27` passed after preserving
  the relocated Go COFF metadata during icon insertion. The original failure
  remains recorded above. Native cold-preservation run `39248796997` at
  `c8ba7883` passed its three parallel suites without repeating reconciliation.
  These focused results do not qualify an application package.
- Integrated run `39249861160` at `6c748c27` compiled Linux, passed 160 unit
  tests and the peer checks, then rejected an obsolete route-card fill-color
  assertion. Correction `62fe2a0f` changes that fixture to the deliberately
  rounded background surface; actual route/action assertions are unchanged.
  The focused native GTK check passes, and a stale Day-color mutation fails.
- Replacement run `39253310076` at `62fe2a0f` passed the Linux compile, 160
  unit tests, peer checks, all 13 boat-feedback components, listener lifetime,
  atomic navigation saves, Sheet/Choice, mode lifecycle, navigation, recording,
  route progress, instruments, N2K, boat-source mapping and Signal K. It then
  failed `smoke-pilot.py:226`: the helper still equates TRACK/WIND capability
  with manual control. Current TRACK deliberately also requires qualified
  native route output, which the TCP fixture does not provide.
- Both runs were cancelled before the full Windows application compilation,
  preserving the failing evidence rather than producing a known-unqualified
  package. The remaining runtime helpers received a bounded source review;
  no other definite stale expectation was found. The fixture-free tail also
  invokes `smoke-pilot.py --production`, whose all-capabilities-disabled TCP
  refusal must remain intact. Source review is not a runtime pass.

No new boat installation, profile mutation or physical autopilot command is
claimed by these CI or isolated desktop-fixture results.
