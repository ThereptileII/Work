# Lessons from SKAGER development

Paths below are relative to the selected repository, not this installed skill.
Read linked evidence only for the applicable failure class. Historical runs and
counts illustrate incidents; they never establish current candidate status.

| Observed problem | Better next action | Source evidence |
| --- | --- | --- |
| Rebuilding after a downstream helper failed discarded useful work. | Retain compiled outputs before qualification; authenticate and retest exact bytes with a separately identified helper. Rebuild for changed product inputs. | `opennav-x/docs/delivery-workflow.md`, `opennav-x/docs/build-efficiency.md` |
| Cold dependency work repeated for unchanged libraries. | Reuse a verified producer with exact run/attempt/artifact, inventories and fresh native tool reprobes. A cache name or failed producer is not authority. | `opennav-x/docs/evidence/2026-10-04-delivery-efficiency.md` |
| Unchanged OpenSSL reported different CPUINFO on another runner. | Separate live CPU observations from immutable build fields; compare other fields strictly. Never fake CPU state. | Same delivery-efficiency evidence and current build-efficiency contract |
| Caller-relative paths pointed children at the wrong SDK directory. | Resolve against the caller before changing directory; preserve link/reparse refusal and byte/provenance checks. | `opennav-x/docs/evidence/2026-10-05-boat-feedback/dependency-reprobe-path-defect.md` |
| A Linux setup helper treated an offscreen optional field as an app failure. | Observe the actual bounded interaction and fresh closure; repair the fixture without weakening action/identity checks. Native acceptance remains separate. | `opennav-x/docs/evidence/2026-10-11-staging-linux-setup-harness.md` |
| Native fixtures assumed POSIX unlink, null backup paths or PowerShell junction cleanup. | Reproduce on real Windows with inert data. Model deletion after handle drain; preserve original profile bytes and replacement-identity assertions. | `opennav-x/docs/evidence/scrum311-native-prerequisites-20261013/focused-results.json`; history of `opennav-x/tools/boat/test-commissioning-profile-lock.ps1` |
| A readiness file existed while its writer was open. | Close the private writer, then atomically publish the owned marker in the same directory; retain bounded waits and actual mutex checks. Existence alone is not readiness. | Commit `f738c4627b0abff7d97b59e47426897ac229df28`, integrated as `967617249ef07c27b7ba77499339970ff2735190`; focused run `38386439515` |
| Creating/sending an operation was mistaken for confirmation. | Distinguish fresh service state from completed stopped-zero-exit operations; pilot commands need fresh device feedback. Keep ACL and command-safety contracts strict. | Historical SCM evidence in `opennav-x/docs/status.md`; `opennav-x/docs/boat/manual-pilot-commissioning.md` |
| Source inspection/unit passes were overstated as loader/product acceptance. | Name the observed boundary and untested loader paths; qualify native behavior and the product separately. | `opennav-x/docs/evidence/2026-10-11-scrum209-local-import-closure.md` |
| A candidate version bump omitted the installer manifest; the distribution gate failed before Windows compilation. | Run the cheap distribution preflight before submitting the batch; keep application and installer versions consistent without changing accepted stock hashes. | `opennav-x/tools/verify-distribution-inputs.py`; the 16 October replacement-candidate section in `opennav-x/docs/status.md` |
| An unchanged inert suite exceeded a tight deadline; one evidence-selected failed-job replay passed. | Preserve original failure and successful prerequisites, compare inputs/environment/timing, and retry only the affected jobs once when justified. A second failure needs diagnosis, not repeated retries. | `opennav-x/docs/evidence/scrum209-plugin-probe-budget-20261016/README.md`; [candidate-cycle decisions](../../skager-build/references/candidate-cycle.md) |
| A dependency guard counted failed path probes as admitted PE files and refused valid plugins. | Reproduce the actual bounded graph; account for lookup count, admitted files and bytes separately while preserving strict resource/security bounds. Policy-fixture success does not prove plugin Init. | Same 16 October probe-budget evidence; `opennav-x/src/integration/Win32PluginTlsImports.h` |
| The boat rebooted after a recovery file move but before its final receipt. | Inspect actual file hashes, running processes and durable intent; continue the existing transaction only through its supported path. A missing logged-in desktop remains a stop condition even when SSH works. Do not infer a reboot cause from event 41 alone. | `opennav-x/docs/evidence/scrum17-9fe-install-20261016/README.md` |

Before retrying, name the failed stage, frozen inputs, supporting observation and
smallest discriminating check. Do not turn a workaround into a universal
exception or lower a safety assertion for a green run. If a focused pass later
exposes a race, preserve both results and fix synchronization rather than adding
arbitrary sleeps or rerunning until lucky.
