# OpenNav X

OpenNav X is a touch-first marine navigation interface on OpenCPN. **Alpha 1 is
under qualification; it is not approved for navigation or production use.**
The [project specification](OpenNavX_Codex_Project_Specification.md),
[approved design](docs/design/OpenNavX_Design_Reference.png) and
[current status/evidence](docs/status.md) define scope and acceptance.

The Alpha integrates the real OpenCPN chart canvas with XNav navigation,
route/waypoint workflows, AIS cards, configurable instruments, energy prediction,
SmartNav advisories, anchor watch, settings and diagnostics. OpenCPN owns charts,
objects, selected navigation and AIS calculations. OpenNav consumers retain owned
snapshots with source, observation time and explicit validity/freshness.

Live marine inputs reuse OpenCPN NMEA 0183, supported NMEA 2000 and own-vessel
Signal K infrastructure. Battery/propulsion configuration and optional mappings
are explicit. Missing or stale inputs suppress dependent predictions. Demo is
clearly marked and supplies deterministic desktop scenarios. The manual autopilot
simulator, unavailable live radar adapter and chart-corridor abstraction preserve
separate hardware/physical-validation gates. SmartNav has no steering path.

Legacy and Safe Mode preserve the shared OpenCPN profile. The portable ZIP has
its own isolated profile. The exact-hash-gated Alpha installer stages a per-user
integration beside the supported stock installation; it leaves the original
program intact. Native install/repair/update/rollback/uninstall qualification
must pass before a stock hash enters the public compatibility allowlist.

The application preserves OpenCPN 5.12.4's **x86/Win32 application and plugin ABI
on Windows x64**, as approved by the user. `win64` distribution names describe
the host. Native Windows MSVC, rendering, DPI, DLL/plugin and installer gates
remain authoritative. Linux supplies fast build, model and integration feedback.

## Architecture and testing

- [Pinned upstream and ABI](upstream.lock.json)
- [Architecture decisions](docs/ARCHITECTURE_DECISIONS.md)
- [OpenCPN integration patches](docs/upstream-patches.md)
- [Vessel Data and marine sources](docs/navigation-data-bridge.md)
- [Read-only remaining-route contract](docs/route-progress-contract.md)
- [Alpha settings](docs/alpha-settings-contract.md)
- [Installer transaction contract](docs/installer-transaction-contract.md)
- [Chart/plugin/DPI evidence](docs/chart-plugin-performance-validation.md)
- [Physical validation procedures](docs/physical-validation.md)
- [CI branch](https://github.com/ThereptileII/Work/tree/opennav-x/opennav-x)

```sh
cmake -S . -B build/contracts -DCMAKE_BUILD_TYPE=Release
cmake --build build/contracts --config Release
ctest --test-dir build/contracts -C Release --output-on-failure
```

Linux baseline: `bash tools/build-pristine-linux.sh`.
Linux integration: `bash tools/build-integration-linux.sh`.
Windows: `./tools/build-pristine-windows.ps1 -Architecture Win32`; add
`-Integration` for XNav. See [toolchain setup](docs/baseline.md).
Integration patches apply only to a disposable pinned worktree under `build/`.
Use disposable `--configdir` profiles for development; the harnesses create them.

## Windows packages

Alpha download names are `OpenNavX-Alpha1-Portable-win64.zip` and
`OpenNavX-Alpha1-Setup.exe`, accompanied by hashes, corresponding source and the
[test guide](docs/alpha/OpenNavX-Alpha1-Test-Guide.md). The
`OpenNavX-Alpha1-Windows` artifact is published only after both platform gates
and accepted compatibility qualification. See [status](docs/status.md) for the
exact accepted run; a candidate build or successful compile is not acceptance.
Read [known limitations](docs/alpha/KNOWN_LIMITATIONS.md) before testing.

The preceding repaired Developer Preview foundation remains recorded in
[run 36047288188](https://github.com/ThereptileII/Work/actions/runs/36047288188),
with its [verified portable artifact](https://github.com/ThereptileII/Work/actions/runs/36047288188/artifacts/10830135079).
