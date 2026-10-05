# Boat feedback batch — 2026-10-05

The user authorized all 16 remaining boat bugs and their requested upgrades.
Delivery stays Staging; this record does not authorize Production promotion.
Each issue receives its own code-fix update. Jira remains the authoritative
acceptance/status tracker. Code completion is separate from native acceptance.

## Implemented increments

| Jira | Change | Local commit |
|---|---|---|
| SCRUM-305 | Fresh coherent upstream anchor distance | ebdd271 |
| SCRUM-295, SCRUM-307 | Passive pilot status on existing data connection; no duplicate status-only setup | f1a6c60 |
| SCRUM-296 | Prototype scale-legend ordering with actual projected scale | ab23609 |
| SCRUM-302 | AIS wheel/touch scroll, release capture, preserve list position | 8f2bc76 |
| SCRUM-300 | Confirm anchor/route transition while preserving saved marks | caeacd8 |
| SCRUM-291 | Consistent inactive/selected default-route style | 1b0bc00 |
| SCRUM-299 | Actual completed-pass OpenCPN XTE in the footer | 9776f99 |
| SCRUM-309 | Complete chart-object information in a native drawer | caaa20f |
| SCRUM-294 | Dimension-scaled ownship prototype glyph; unavailable direction remains unoriented | 8f053c0 |
| SCRUM-306 | Persistent chart-centered 1–200 NM AIS radius; missing-target investigation remains open | f77cb02 |
| SCRUM-298 | Inline waypoint/route names and truthful chart-based defaults | 5abef0b |
| SCRUM-297 | Contextual route card from upstream chart selection/hover | 4d125a4 |
| SCRUM-303 | Theme-aware light-hover sector inks; unchanged geometry | afa010f |
| SCRUM-308 | Classified light-support tower palette, preserved navigation shapes | 36011cc |
| SCRUM-304 | Anchor-watch prototype mark/ring, alarm priority and custom-icon preservation | 842da56 |

## Focused verification

[Eleven shared component suites](components-linux.json) passed, linked against
production component libraries. Their receipt deliberately records the
configured commit and **dirty worktree**; it is not clean release evidence.
The subsequent [twelfth anchor renderer target](anchor-registration-linux.json)
also passed, without rerunning the other eleven. Its compiled painter is the
production source, with isolated observation stubs and the actual prepared
upstream ring method. Three widget test mains were corrected to propagate a
failed assertion through `OnRun`, rather than trusting an ignored `OnExit`
return. Early failed GTK/display attempts remain in local build evidence.

Targeted checks additionally exercise pinned Mercator anchor geometry,
route-progress/XTE validity, ownship/route presentation, actual S-52
loader/lookup/render boundaries and source-locked atlas derivation. Their
individual issue records give scope and counts. Pixel/assertion counts are not
presented as thousands of independent user scenarios. Actual changed application
source units compile locally. All nine patches reproduce against the pinned
5.12.4 source at 37fd0cddb7334fe489e9f18aa163977a9c5c84f7.

The shared offline suite is registered once for Linux and native Windows after
fixture application compilation. Runtime binaries and the configured commit are
identified in its receipt. This does not require another application build.
No SDK producer recipe changed; the authenticated dependency bundle is reused.

## Still pending

One coordinated integrated Linux/native Windows Staging candidate is required,
then the relevant installed boat checks. No integrated Windows application,
OpenGL context, installer, physical chart or boat acceptance is claimed here.

SCRUM-306 is still In Progress. The older verified isolated read-only probe
received 31 real targets/45 accepted reports in a public test region, while the
installed product remained connected with zero counted reports outside that
region. The actual sent area is absent from its diagnostics; geography is a
hypothesis, not an established root cause. See the
[redacted read-only evidence](../scrum306-boat-readonly-20261005.json).
The new radius and commissioning diagnostics need current-application validation.
No API key, private chart data or vessel coordinates are included.

No physical autopilot, propulsion or radar command, remote-access change,
endurance run or Production publication is part of this batch.

## First native preflight correction

[Staging run 37266663445](https://github.com/ThereptileII/Work/actions/runs/37266663445)
failed before full Windows compilation on an obsolete helper assertion which
required PrivateOCharts to be the last build-script parameter. The approved
delivery/dependency options made that assumption invalid. The production
opt-in, maintained-dependency validation and installed TLS-before-deferral
checks were still present.

The corrected AST/order test passes in the focused native
[run 37267103145](https://github.com/ThereptileII/Work/actions/runs/37267103145).
Its downloaded artifact matches the authenticated GitHub digest; see the
[receipt](loader-helper-native.json). Existing rejection cases and eight
installed-TLS/delivery contexts pass. No SDK producer input changed and no full
Windows application was built for this helper repair. Packaged release notes
and the short feedback test guide accompany the next Staging candidate.

## Callback-lifetime review hold

The coordinated candidate `ff7ae5d84884b8369acdd5eba9666152869ad497`
([run 37267805917](https://github.com/ThereptileII/Work/actions/runs/37267805917))
passes 97 Linux and 94 native Windows contract cases and the separate native
commissioning/restart/private-loader prerequisite jobs. Integrated compilation
was still running when parallel source review found SCRUM-300 retaining route
and waypoint pointers across synchronous plugin callbacks. A handler can delete
or replace those objects. The issue returns to In Progress for actual boundary
revalidation and a mutation regression. This candidate must not be installed
with the known defect; no pass or qualification is inferred from compilation.

The correction resolves owned route identity/revision, current navigation,
selected position and watches after persistence, deactivation and each
synchronous notification. It selects the best point only immediately before
native activation. The fixture extracts the actual production function; 33
mutation cases plus normal/failure paths pass. The prior source fails with
`Stale raw Route pointer dereferenced after callback`. The changed integration
unit passes the actual application syntax check, and its CMake target builds
and runs locally. It becomes the thirteenth shared native component without
rerunning the other twelve locally. Replacement integrated qualification is
still required; the old candidate is not relabelled as fixed.

Completed logs from the superseded run also expose four fixture-only MSVC
`LNK2019 _main` failures: pinned wxWidgets 3.2.8 supplies `WinMain` for their
`wxIMPLEMENT_APP` entry points. The shared and standalone CMake targets now
select the Windows GUI subsystem only for those four fixtures; explicit-main
fixtures remain console applications. This is not an application linker defect.

The bounded review also found the equivalent inherited watch-set notification
lifetime issue. The anchor now copies owned identity before notifying plugins
and re-observes the watch afterward. Seven added cases cover removal,
replacement (including retained GUID/revision), modification, unavailable state,
an added watch and unchanged success. The actual integration syntax and focused
CMake target pass. Native qualification remains pending for the replacement.

## Native component diagnosis and retention correction

Replacement `30a0bb7853c1fa879d682cdc70b2d72204eb5977` in
[run 37270689330](https://github.com/ThereptileII/Work/actions/runs/37270689330)
successfully compiles the Windows application and passes 144/144 integrated
CTests. Twelve shared component executables pass, including all four previously
misconfigured entry points and the callback-lifetime fixture. AIS scrolling
fails after 22 successful assertions at its outside-owner input count.

The focused native reproduction confirms one pre-existing native owner motion,
followed by exactly one delivery of each explicitly dispatched outside event.
The test now separates incidental owner motion from explicitly dispatched and
child-origin input. Both original lifetime non-leak assertions remain, and three
new immediate assertions require exactly one outside delivery. The corrected
fixture passes 47 checks on Linux and native Windows, including its OS pointer
drag. [The downloaded evidence](ais-scroll-native.json) identifies both the
failed diagnostic and successful correction; no application code changed.

The initial shared-suite placement mistakenly preceded compiled-input retention.
It now runs in downstream qualification after authenticated restore, preserving
the original producer manifest and all thirteen executable hashes. Fixed names
and paths are verified before rebasing, producer/harness identities stay separate,
and any failure still prevents delivery. Twenty focused retention/refusal tests,
two attempt-selection tests and three workflow-policy checks pass locally. Older
archives without the required components are refused. This corrects scheduling;
it does not turn the failed application's run into installer acceptance.

The boat development checkout is at `30a0bb7`; all 118 boat helper files match
the reviewed source. The installed application remains `0da2c64`. The development
checkout update did not change or launch the installed application. A fresh cold
backup, profile-preserving commissioning transition and exact accepted installer
are still required before replacement.

The separate delivery-policy run then exposed test portability assumptions:
published workflows live above `opennav-x`, and Windows TEMP can use a short
path spelling. Correcting only those test expectations passes 199 checks in
13 suites on each platform. [Downloaded receipts](delivery-helper-native.json)
bind the helper revision independently of application candidate `982a2b54`.
The running application build was not cancelled or restarted for this correction.
