---
name: skager-build
description: Select and run SKAGER application builds or verified Windows dependency reuse from actual changed inputs. Use for build planning, build failures and retained-output recovery; use skager-test for focused test selection. Does not authorize delivery, Production promotion or boat changes.
---

# SKAGER build

Work from the selected SKAGER checkout, not a remembered worktree. Resolve paths
below from its repository root. Read `AGENTS.md` and `opennav-x/AGENTS.md`, then
`opennav-x/docs/build-efficiency.md` and
`opennav-x/docs/delivery-workflow.md`. These maintained
documents control scheduling; historical evidence is not current acceptance.

## Select the work

- Compare actual resolved base/HEAD inputs using `opennav-x/tools/ci_changes.py`
  and inspect its reasons, including renames/deletions. Unknown paths or missing
  history take the conservative build path. Ordinary docs/evidence, helper-only
  repairs and channel changes do not require application compilation. Packaged
  notices/licenses/resources, installer/build recipes and `tests/**` can be
  product inputs; filename extensions do not decide this.
  This is the default efficiency policy, not a veto on an explicit user request
  for a fresh candidate. Honor that request through the coordinator's supported
  manual Staging selection, with its preflight and required gates.
- The coordinator integrates a coherent batch and owns the full application
  build. Delegated tasks return focused results without starting duplicate builds.
  Read current entrypoints/options in `opennav-x/README.md` and the selected
  workflow; use existing pristine/integration drivers rather than inventing a
  parallel recipe. Windows uses native MSVC **Win32/x86**, preserving the OpenCPN
  application/plugin ABI even on 64-bit Windows. Linux/Wine cannot qualify it.
- Default to Staging. Production requires explicit user instruction for an exact
  candidate and reuses qualified package bytes; changing channels does not change
  the embedded version. Both channels remain draft while public access is closed.

Before requesting a candidate build, run the cheap distribution preflight from
the selected checkout: `python opennav-x/tools/verify-distribution-inputs.py`.
In particular, `src/application/Version.h` and the installer's
`compatibility.json` `openNavVersion` must agree. Fix inconsistent inputs before
compilation; do not change supported stock hashes to resolve a version mismatch.
A preflight pass is not build or release acceptance.

## Verify dependencies and preserve completed work

- For maintained Windows OpenSSL/zlib/curl, follow the current dependency section
  of the build contract and `opennav-x/tools/windows-dependency-bundle.lock.json`.
  Authenticate the exact successful producer run/attempt/job/artifact, complete
  inventory/hashes, source/recipe/options/ABI and fresh native tool/environment
  reprobes. A cache key, matching name or same-job fixture receipt cannot replace
  cross-run SDK provenance. Missing, expired, redirected or mismatched inputs stop
  reuse; do not silently use stock or partial libraries.
- Check the complete native dependency closure. Maintained curl's stock-cache
  marker can hide missing LibArchive support; use the authenticated
  `prepare-windows-stock-deps.py` path before the unchanged build driver. For
  consumer-only fingerprint drift, inspect the exact producer diff and existing
  compatibility policy; never normalize hashes or exempt toolchains/recipes.
  Preserve manifest-bound installed DLL/import checks and legacy TLS refusal;
  authenticating an SDK alone does not prove the application's runtime closure.
  Read `opennav-x/docs/architecture/windows-native-dependency-integration.md`
  when changing this boundary.
- Retain compiled outputs and identity before downstream packaging/tests, including
  the contract's separate unqualified compiled-recovery artifact. Repair helpers
  against authenticated retained bytes; record producer commit/run/attempt/hashes
  separately from helper commit. Reuse the clean same-run native updater artifact
  rather than rebuilding it in the integrated checkout. Failed SDK recovery
  material stays unqualified and cannot be selected as a verified producer.

For a candidate build, downstream failure or interrupted delivery, use the
[candidate-cycle decisions](references/candidate-cycle.md). Resume the failed
stage against authenticated unchanged inputs; preserve successful job attempts
and distinguish compiled recovery, sealed test inputs and a qualified package.

Report exact build/input identity, artifact hashes, checks and remaining gates.
Compilation or SDK reuse does not establish package, native desktop or boat
acceptance. Preserve navigation, safety/security, data preservation, installer/
updater/recovery, exact-source/license and release gates. Batch full checks at
integration/release milestones. Design review and long-running suites require
explicit user instruction; endurance remains skipped until the user changes it.
