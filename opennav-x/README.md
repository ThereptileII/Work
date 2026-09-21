# OpenNav X

OpenNav X is being developed as a touch-first marine navigation interface on
OpenCPN. **This repository is a development build, not a navigation release.**
The authoritative scope and acceptance requirements are in
[the project specification](OpenNavX_Codex_Project_Specification.md).

Developer Preview 0.1 provides custom XNav Navigation, Route, Energy and System
views around OpenCPN's real chart canvas, plus Legacy/Safe startup and controlled
mode restarts. Explicit deterministic Demo scenarios exercise complete vessel
data and advisory range/arrival-SOC predictions. Live selected position/SOG/COG
and the accepted [remaining-route snapshot](docs/route-progress-contract.md)
retain OpenCPN ownership, provenance and freshness. Missing/stale inputs withhold
predictions; live battery acquisition and calibrated capacity/reserve remain
future work. No OpenNav steering or radar commands exist.

The [preview contract](docs/developer-preview-contract.md) documents ownership,
Demo separation, energy assumptions, portable isolation and test gates. The
preview does not patch an installed OpenCPN or use its normal profile. Hardware
adapters and the production Windows installer remain later milestones.

The application preserves OpenCPN 5.12.4's supported **32-bit application/plugin
ABI on 64-bit Windows**, as explicitly approved by the user. Native Windows
MSVC validation remains mandatory. Windows screenshots govern UI acceptance.

Current accepted increments, tested revisions and remaining release work are in
[the development status](docs/status.md).

## Build and evidence

- Exact upstream revision and ABI: [upstream.lock.json](upstream.lock.json).
- Toolchains, commands and limitations: [docs/baseline.md](docs/baseline.md).
- Narrow source hooks: [docs/upstream-patches.md](docs/upstream-patches.md).
- Decisions: [docs/ARCHITECTURE_DECISIONS.md](docs/ARCHITECTURE_DECISIONS.md).
- Approved visual board: [design reference](docs/design/OpenNavX_Design_Reference.png).
- CI: [isolated opennav-x branch](https://github.com/ThereptileII/Work/tree/opennav-x/opennav-x).

Portable contract tests:

```sh
cmake -S . -B build/contracts -DCMAKE_BUILD_TYPE=Release
cmake --build build/contracts --config Release
ctest --test-dir build/contracts -C Release --output-on-failure
```

Linux OpenCPN baseline: `bash tools/build-pristine-linux.sh`.
Linux integration: `bash tools/build-integration-linux.sh`.
Windows: `./tools/build-pristine-windows.ps1 -Architecture Win32`; add
`-Integration` for the guarded XNav source build.

Integration patches apply only to a disposable pinned worktree under `build/`.
They do not modify an installed OpenCPN. Always use a disposable `--configdir`
for development; the capture scripts create one automatically. No supported
installer compatibility entries are published until their Windows gates pass.

## Windows Developer Preview 0.1

The accepted preview packages explicit Demo scenarios, Route/Energy/System pages
and the real OpenCPN chart canvas.
[Download OpenNavX-DeveloperPreview-win64](https://github.com/ThereptileII/Work/actions/runs/35663416666/artifacts/10668776591)
from the [successful CI run](https://github.com/ThereptileII/Work/actions/runs/35663416666).
Extract the inner `OpenNavX-DeveloperPreview-win64.zip` into a short writable
folder, then run `Run-XNav-Demo.cmd`. Its private profile and bundled runtime do
not require modifying an installed OpenCPN. Exact revision, test counts and
native screenshot evidence are tracked in [status](docs/status.md). Read the
[Windows test guide](docs/preview/TEST_ME_FIRST.md) and
[limitations](docs/preview/KNOWN_LIMITATIONS.md). This is not approved for navigation.
