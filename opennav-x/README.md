# SKAGER App

SKAGER is a touch-first marine navigation interface for a separately installed,
supported OpenCPN. **The paid public beta is being implemented and qualified;
this repository does not establish release or navigation acceptance.** Beta 2
is the current package generation. Public downloads and payments remain closed
until the complete readiness review and explicit product-owner approval.

[SCRUM](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the sole backlog. The [current objective](PROJECT_GOAL.md),
[public-beta contract](docs/public-beta-contract.md),
[project specification](OpenNavX_Codex_Project_Specification.md) and unchanged
[HTML visual reference](docs/design/prototype/index.html) define the product.
[Technical status and evidence](docs/status.md) record results and open gates;
qualification is specific to an exact revision and its artifacts.

The later [delivery policy](docs/delivery-workflow.md) governs channel selection
and design-validation scheduling where older source documents differ.
Development defaults to **STAGING**. Both Staging and Production are delivered as
versioned GitHub Releases; releases remain **draft** while public access is closed.
Production readiness/promotion requires an explicit user instruction and reuses
the exact selected Staging package. Design review requires an explicit request;
promotion alone does not trigger it or authorize public publication.

The native application integrates the real OpenCPN chart canvas with passage
and waypoint workflows, AIS views, configurable instruments, energy prediction,
SmartNav advisories, anchor watch, settings and diagnostics. OpenCPN owns charts,
navigation objects and underlying navigation calculations. SKAGER uses copied
observations with explicit source, time, validity and freshness.

Live marine inputs reuse OpenCPN NMEA 0183, supported NMEA 2000 and own-vessel
Signal K infrastructure. Battery and propulsion mappings are explicit. Missing
or stale data suppress dependent predictions. Installed product builds exclude
synthetic runtime input and keep equipment output status-only; SmartNav never
steers. Developer fixtures and loopback tests do not qualify physical control.
Recording, replay and private diagnostics retain their commissioning limits.

Legacy and Safe Mode preserve the shared OpenCPN profile. The portable recovery
ZIP has its own isolated profile. The exact-hash-gated installer stages a
per-user integration beside the supported stock installation and preserves the
original program. Installation, repair, update, rollback and uninstall still
require qualification for the exact integrated revision.

The application preserves OpenCPN 5.12.4's **x86/Win32 application and plugin ABI
on a Windows x64 host**. Native Windows MSVC, rendering, DPI, plugin loading,
installer and actual boat functional evidence remain authoritative within their
applicable qualification scope. Design-only rendering/DPI review is scheduled
only on explicit request under the delivery policy. Linux builds and component
checks support development and do not replace native functional gates.
Internal OpenNav/XNav identifiers remain where needed for compatibility;
customer-facing product identity is SKAGER / SKAGER App.

## Architecture and development checks

- [Pinned upstream and ABI](upstream.lock.json)
- [Architecture decisions](docs/ARCHITECTURE_DECISIONS.md)
- [OpenCPN integration patches](docs/upstream-patches.md)
- [Vessel Data and marine sources](docs/navigation-data-bridge.md)
- [Read-only remaining-route contract](docs/route-progress-contract.md)
- [Installer transaction contract](docs/installer-transaction-contract.md)
- [Physical validation procedures](docs/physical-validation.md)

```sh
cmake -S . -B build/contracts -DCMAKE_BUILD_TYPE=Release
cmake --build build/contracts --config Release
ctest --test-dir build/contracts -C Release --output-on-failure
```

Linux baseline: `bash tools/build-pristine-linux.sh`.
Linux integration: `bash tools/build-integration-linux.sh`.
Windows: `./tools/build-pristine-windows.ps1 -Architecture Win32`; add
`-Integration` for SKAGER. See [toolchain setup](docs/baseline.md).
Integration patches apply only to a disposable pinned worktree under `build/`.
Use disposable `--configdir` profiles for development; the harnesses create them.

## Windows candidate packages

Current package outputs are `SKAGER-Beta2-Setup.exe`,
`SKAGER-Beta2-Portable-Recovery.zip` and `SKAGER-Beta2-source.zip`, accompanied by
hashes and the same-revision [installation guide](docs/beta2/SKAGER-Beta2-Install-Guide.md),
[test guide](docs/beta2/SKAGER-Beta2-Test-Guide.md) and
[release notes](docs/beta2/SKAGER-Beta2-Release-Notes.md).
Read [known limitations](docs/beta2/KNOWN_LIMITATIONS.md) before testing.
Use the selected versioned GitHub Release and verify its package identity and
hashes. `opennav-baseline.yml` serves Staging delivery; manual
`skager-production.yml` serves exact-package promotion; `opennav-prototype.yml`
is for explicitly requested design review. A release channel is separate from
the immutable package version. CI artifacts provide build evidence and
intermediate outputs. A generated package or passing build does not establish
public-beta acceptance.
Earlier releases and their exact artifacts remain recorded in historical
[evidence](docs/status.md); their acceptance cannot be transferred to a new build.
