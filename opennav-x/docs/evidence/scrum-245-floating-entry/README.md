# SCRUM-245: floating-surface console entry point

Frozen candidate `4c597955f96647a9c7b837139aa2732f2c3a3fa5` failed native
[run 37063131823 / job 111026476044](https://github.com/ThereptileII/Work/actions/runs/37063131823/job/111026476044)
at 2026-10-02 22:07:54 UTC. The retained original receipt records both linker
errors and artifact 11254867923 identity. `floating_surface_test` was a console
CMake target, but `wxIMPLEMENT_APP(TestApp)` supplies the Windows GUI entry point.
The console CRT therefore could not resolve `_main` (LNK2019/LNK1120).

The correction changes only the final macro to `wxIMPLEMENT_APP_NO_MAIN` and
adds `main` returning `wxEntry(argc, argv)`, following the other existing
component fixtures. All lifecycle methods, timer steps and assertions are
byte-identical to the starting revision `2f3a33c`. `wxEntry` still runs OnInit,
OnRun/the native event loop, OnExit and wx cleanup; the fixture's failure return
code remains authoritative. The target stays a console executable.

The exact fixed fixture and production FloatingSurface.cpp linked locally with
wx base/core and GTK and exited zero after all **17 Linux checks**, including
both actual native origins at (980,549). See `linux-fixture.json`. A preliminary
local command unnecessarily linked wx HTML and encountered an unavailable
transitive mspack library; the focused base/core link uses precisely the needed
components. That local environment failure is not the native `_main` defect.
Twelve runtime/negative-control/layout guard tests pass; Python syntax and whitespace
checks pass. The original native error receipt is accepted by the negative
control checker, while unrelated link/compiler errors are rejected.

## Prepared native proof

Dispatch `.github/workflows/skager-windows-changed-units.yml` on the published
exact source with `floating_surface=true`, or push an eligible focused branch
with `[floating-surface]` in its commit message. The matching CLI is
`tools/test-windows-changed-units.py --floating-surface-only`.

This mode skips OpenCPN source preparation, curl, unrelated UI components and
layout captures. It downloads only the existing hash-locked wx SDK and uses a
small MSVC Win32 CMake target containing the **existing** fixture and production
FloatingSurface.cpp. It first restores the old macro in an evidence-only copy
and requires exactly the observed `_main` LNK2019/LNK1120 failure. It then links
the fixed source, stages the locked wx DLLs plus the installed x86 compiler
runtime using the existing staging helper, and executes every unchanged native
assertion. Success requires exit zero and exactly **12 Windows checks**; the
Linux count cannot satisfy this gate. Source/header, executable, runtime, command,
SDK lock and negative-control logs are retained. Source/runtime drift fails.

Native proof has **not** been run by this subtask. Root owns publication and the
single focused dispatch. Full product, package, installer, DPI and boat gates
remain open; a focused fixture link does not qualify the frozen candidate.

Prepublication review caught a workflow-path assumption before dispatch: local
product checkouts contain `.github`, while the published product lives at
`repo/opennav-x` and the workflow at `repo/.github`. The proof now checks the
actual Git top-level and accepts only these two exact layouts. It records the
resolved file hash under its product-relative key and repeats the resolution and
hash check afterward. A missing workflow, unknown nested layout, redirected path
or ambiguous product-local shadow fails closed. Focused tests exercise both real
Git layouts, changed workflow bytes, ambiguity and missing/unsupported paths.
No native retry was consumed for this correction.
