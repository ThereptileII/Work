# Focused native chart compile preflight — SCRUM-247

`tools/test-windows-changed-units.py --chart-units-only` is an isolated opt-in
Win32/MSVC production-object preflight. The existing focused workflow accepts
`chart_units=true` or `[chart-units]` on its eligible changed-unit branch.
It refuses combination with component/legacy proof modes, inherits the existing
15-minute job bound and limits the object build to ten minutes. Root freezes and
publishes the combined source before its single focused dispatch.

The inventory is fourteen complete source files, not extracted functions:

- Local: `ChartPresentation`, `ChartRouteWaypoint`, `ChartRouteUnderlay`,
  `ChartRouteUnderlayGeometry`, `OnboardAisPresentation`, `OnlineAisOverlay`.
- Patched OpenCPN: `chcanv`, `route_gui`, `route_point_gui`, `waypointman_gui`,
  `ais`, `piano`, `s52plib`, `chartsymbols`.

The pinned upstream verifier and all nine reviewed integration patches run
before preparation and again after compilation. Source and transitive header
hashes include the actual geographic-font resolver/spacing headers, S-52
headers, production CMake files, chart resource definition/source lock and
original prototype artwork inputs. The production generator creates and
verifies the real `XNavChartResources.h`, manifest and chart resources; no stub
resource or alternate font implementation is supplied. Both software and GL
code in these production units compile with the installed GL-enabled feature
profile and test fixtures/pilot loopback disabled.

The narrow CMake configuration follows the pinned application's Windows
C++17/MD/EHa definitions, real wx SDK and application/S52/transitive include
roots. It is a reviewed compile profile, not a substitute for the full
application's feature detection. It rejects missing include roots, hidden
NOMINMAX/test macros, forced includes and precompiled-header substitution.
It uses the existing locked wx 3.2.8 and GLEW inputs; maintained curl is downloaded
for headers only. ShapeFileCpp is copied from pinned upstream. Shapelib 1.6.1
and RapidJSON 1.1.0 are the upstream-selected archives, now SHA-256/length locked;
all three use the exact ordered upstream patch series and actual GNU-patch
CMake runner. No dependency library, curl/OpenSSL producer, full UI library or
application is built or linked. No GUI or vessel input is started.

Before reporting success, every expected source must yield its own nonempty
x86 COFF/bigobj object. Source/header/resource/workflow identities are checked
again. Source manifests, actual local and patched translation units, generated
resources, CMake projects and command/compile logs remain in the focused
artifact even on failure. Unbuilt SDK libraries and downloaded archives stay
outside that artifact; locked identities and all consumed header hashes remain
in `chart-inputs.json` and the final summary. A successful summary explicitly
sets `nativeProductAcceptance=false`.

## Preparation checks and limits

Seven offline guard tests pass for archive traversal/link/case-duplicate
refusal, actual object inventory/architecture refusal and exclusive CLI mode
selection. The twelve existing component-runtime/layout guard tests still pass.
Actual locked dependency archives and all original upstream patches staged
locally without building libraries. The production resource generator passed;
a private Linux configure-only harness checked all CMake include paths and all
fourteen target declarations. This syntax/path check did not compile chart
objects and is not a native Windows pass. Python syntax, workflow YAML and diff
whitespace checks passed.

An initial local header-preparation attempt used git-apply, whose strict EOF
handling rejected an existing bundled ShapeFileCpp test-file patch. The helper
now uses upstream's actual GNU-patch runner and the complete original patch
series; that preparation passes. No production source or upstream patch was
changed to accommodate the harness, and no native attempt was consumed.

Native execution remains pending root's exact combined-source run. This gate
cannot establish application/dependency linking, renderer behavior, font/DPI
appearance, resource installation, installer/update/rollback, or boat safety.
It does not compile the full `glChartCanvas`/`ocpn_frame` dependency trees or
prove separately compiled GL-disabled builds. Full corrected-candidate
qualification under SCRUM-224 and visual/real-chart acceptance under SCRUM-15
remain required, regardless of this preflight's outcome.
