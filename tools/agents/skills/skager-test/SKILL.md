---
name: skager-test
description: Select and run focused SKAGER functional or helper checks, diagnose native fixture failures, and retest authenticated retained candidate bytes. Use for regression evidence and test repair; use skager-build when product inputs require compilation. Does not authorize boat operations or release publication.
---

# SKAGER test

Work from the selected SKAGER checkout. Resolve paths below from its repository
root and read `AGENTS.md`, `opennav-x/AGENTS.md`, the current
[build contract](../../../../opennav-x/docs/build-efficiency.md) and
[delivery policy](../../../../opennav-x/docs/delivery-workflow.md). Read the selected
Jira acceptance criteria and relevant component contract before choosing checks.

## Choose evidence for the changed behavior

- Inspect actual changed inputs and the workflow/helper that exercises them.
  Ordinary docs and helper changes need focused checks, without a fresh application
  build. `tests/**` is conservatively a product input because fixture support can
  be compiled into executables; use the current selector rather than assuming all
  tests are helper-only. Escalate product-input changes to the build coordinator.
- For small reversible changes, run meaningful focused functional checks and an
  applicable smoke check. Batch full suites at integration/release milestones.
  Do not add tests that merely repeat implementation or weaken existing failure
  assertions. Native Windows remains authoritative for MSVC/Win32 ABI, platform
  behavior and installer/updater qualification; portable mocks cannot grant it.
- Preserve navigation, safety/security, data-preservation, authentication/payment,
  installer/updater/recovery, exact-revision/source/license, boat and release gates.
  Design/prototype comparisons and design-only DPI sweeps require explicit user
  instruction. Check a concrete usability/rendering defect at its relevant
  resolution without turning it into a general appearance review. Long-running
  suites also require explicit instruction and separate background execution;
  endurance is currently skipped, never passed, until the user changes that choice.

## Diagnose a failed fixture before blaming the product

Find the first causal failure in original logs and identify product, fixture,
environment or transport scope. Preserve the failed evidence. Reproduce with the
smallest relevant native check before requesting another full build.

For Windows file/lock fixtures, inspect the actual PowerShell/.NET behavior and
handle lifetimes. The profile-lock fixture needs an explicit `File.Replace`
backup path and checks original backup bytes. Unlink a junction itself with the
native directory API, verify its target/profile bytes survive, and prevent cleanup
from masking the original exception. Publish child readiness by writing a private
pending file, closing its writer, then renaming it to the polled name. Model
Windows deletion disposition until handles drain; POSIX unlink of an open file
is not equivalent. Retain replacement-file identity and safety assertions.
Use the maintained sources `opennav-x/tools/boat/test-commissioning-profile-lock.ps1`
and `opennav-x/tests/ais_connection_collector_tests.py` when this behavior applies.

## Retest and report exact bytes

Use authenticated retained application/package/component outputs when only the
test helper changed. Follow the build contract's retained-check scope and record
original producer commit/run/attempt/hashes alongside the new harness commit.
Missing or changed artifacts stop reuse. A helper rerun neither creates a new
executable nor relabels its source/version. Keep failed original runs intact;
composed Staging acceptance needs its explicit authenticated evidence record.

Report each relevant result as passed, failed, skipped, pending or not requested,
with exact revision, platform, command/scope and retained evidence. Separate
fixture/component passes from full application, installed-product and boat
acceptance. A green helper or dependency reprobe cannot close unrelated gates.
