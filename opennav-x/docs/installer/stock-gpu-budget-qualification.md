# Same-version texture budget: evidence before policy

The migration policy currently admits the observed official upgrade reset from
128 to64 only. This fixture does not change that policy or approve the reverse
change for a boat session.

Pinned OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7` reads
`Settings/OpenGLExpert` with default false (`gui/src/navutil.cpp:767`), then clamps
the loaded texture budget to at least128 whenever expert mode is false
(`navutil.cpp:555-560`). This clamp is compiled with GL support; it is not
conditioned on the active backend. `UpdateSettings` saves the resulting budget
at `navutil.cpp:2168`.

The separate GL-capable first-run/upgrade path assigns64
(`gui/src/OCPNPlatform.cpp:634-646`). On a later ordinary start, an existing notice
state avoids first-run (`ocpn_app.cpp:1382-1384`) and an exact unchanged version
marker avoids upgrade (`ocpn_app.cpp:1444-1445`). This is why the next start may
persist64→128, without a user changing its preference.

`tools/test-stock-gpu-budget-windows.ps1` is native disposable CI only. It extracts
the exact official setup `e949f55...b8aa` and requires stock executable
`7c654756...ae0c`, with full hashes in source/report. Three separate portable
profiles contain the exact existing official version/notice state, configured
`OpenGL=1`, budget64, empty connections and all four bundled plugins disabled:

1. OpenGLExpert absent: source predicts128.
2. OpenGLExpert explicitly0: source predicts128.
3. OpenGLExpert explicitly1: control predicts64.

Each process must finalize actual startup without an acknowledgement action,
close normally with measured zero exit, and preserve version, notice, configured
OpenGL/expert state, disabled plugins and empty connections. The fixture retains
exact before/after INI files/hashes and isolated startup logs. No GPU DLL shim,
`--no_opengl`, profile rewrite after launch, unknown-modal click, force-kill or
real helper is used. If the native runner disables OpenGL due to capability,
the requested unchanged-OpenGL condition fails explicitly; no backend is assumed.

Run this in its own fresh Windows job, separately from the urgent chart-helper
cleanup gate. Require 7-Zip and invoke Windows PowerShell with `-Evidence` pointing
to a new directory; upload that directory on success or failure. The script does
not alter desktop resolution and a successful run proves configuration
normalization only, not hardware acceleration, chart rendering or performance.

The first isolated native run, tooling `ca3a0c7`, failed this environment gate:
expert-absent startup and measured normal exit0 passed, and64→128 was observed,
but the official capability utility disabled OpenGL. The expert-false/true
controls were not executed after that failure. The artifact is hash/size/CRC
verified; see [the scoped failed evidence](../evidence/beta2-stock-gpu-budget-ca3a0c.json).
No reverse-budget policy or hardware-rendering acceptance follows from it.
The nine previously qualified maintenance jobs remain in the standard tools
workflow. This experimental preserved-OpenGL job is not a release gate and its
failure remains outstanding; its fixture is retained for an appropriate GPU
environment. No accepted functional test was removed. The boat's actual next
closed-profile diff still requires inspection.
