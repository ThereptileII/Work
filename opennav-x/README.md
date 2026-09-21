# OpenNav X

OpenNav X is being developed as a touch-first marine navigation interface on
OpenCPN. **This repository is a development build, not a navigation release.**
The authoritative scope and acceptance requirements are in
[the project specification](OpenNavX_Codex_Project_Specification.md).

Current work is the first vertical slice: custom XNav chrome around OpenCPN's
existing chart canvas, Legacy/Safe startup, controlled mode restart and an
explicitly labelled vessel-data simulator. A second slice subscribes read-only
to OpenCPN-selected position, SOG and COG; its native validation is tracked in
[the data bridge notes](docs/navigation-data-bridge.md). Missing inputs display unavailable;
stopped samples age into stale data. No OpenNav steering or radar commands exist.
SmartNav, real vessel adapters and the Windows installer are later slices.

The application preserves OpenCPN 5.12.4's supported **32-bit application/plugin
ABI on 64-bit Windows**, as explicitly approved by the user. Native Windows
MSVC validation remains mandatory. Windows screenshots govern UI acceptance.

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
