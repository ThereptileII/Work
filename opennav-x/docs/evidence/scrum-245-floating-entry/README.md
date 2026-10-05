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

## Focused native proof

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

Root published the exact minimal correction and dispatched the focused proof.
It passed as recorded below. Full product, package, installer, DPI and boat gates
remain open; a focused fixture link does not qualify the full candidate.

Prepublication review caught a workflow-path assumption before dispatch: local
product checkouts contain `.github`, while the published product lives at
`repo/opennav-x` and the workflow at `repo/.github`. The proof now checks the
actual Git top-level and accepts only these two exact layouts. It records the
resolved file hash under its product-relative key and repeats the resolution and
hash check afterward. A missing workflow, unknown nested layout, redirected path
or ambiguous product-local shadow fails closed. Focused tests exercise both real
Git layouts, changed workflow bytes, ambiguity and missing/unsupported paths.
No native retry was consumed for this correction.


## Downloaded native result verified

[Run 37073219984 / job 111057239525](https://github.com/ThereptileII/Work/actions/runs/37073219984/job/111057239525)
completed successfully at 2026-10-02 22:35:30 UTC for published commit
`e67f70e7dafc31b6da9d34d5eab8690a9eab09c8`, corresponding to local minimal
qualification commit `8ef2baca8de5820ae91968b9ce67065c0ce43baa`. Run/job status
and completion time come from the coordinator's receipt; the downloaded
artifact's candidate, executable, source and results were independently checked.

Artifact **11256121540** is 4,357,411 bytes, SHA-256
`17191734b5bdb4438a84c344b175186d4914e26809f0332ddfd6fc3cb4d9e030`.
ZIP CRC verification passed and all 150 extracted files matched the archive.
All 130 recorded source inputs matched the exact local qualification Git tree:
129 differed solely by Git LF-to-CRLF checkout conversion, while the explicitly
LF-pinned `SkagerBrandAsset.h` matched byte for byte. The guarded monorepo
workflow path resolved to the same qualification workflow. No other source
normalization or later application changes were accepted.

The artifact contains the actual MSVC 19.44.35229.0 Win32 console fixture,
69,120 bytes, SHA-256
`8bfca5c6b4f7063a22dad39d5dd60918b7f0b4dbc1d0fce807a67b09754b50a1`.
It exited zero with exactly **12 floating-surface lifecycle checks passed**.
The executable and all 11 app-local DLL hashes match the recorded runtime;
PE headers independently confirm x86/PE32, with console subsystem for the test.
The evidence-only old-entry-point source differs solely by restoring the old
macro and fails with exactly LNK2019 `_main` and LNK1120. It produces no executable.

[Verification receipt](native-37073219984/verification.json),
[native summary and source/runtime hashes](native-37073219984/summary.json),
[fixed link log](native-37073219984/floating-link.log),
[actual run output](native-37073219984/floating-surface.log), and
[negative-control log](native-37073219984/legacy-link.log) are retained here.
Retained text is normalized to LF; the receipt records both original artifact
bytes/hashes and retained text hashes. The full downloaded archive remains in
private evidence. This review did not execute an artifact, start fresh tests or
CI, access credentials/hardware, or change root working files.

This evidence satisfies SCRUM-245's isolated entry-point repair and native proof.
SCRUM-224 and full corrected product, installer, DPI and boat acceptance stay open.
