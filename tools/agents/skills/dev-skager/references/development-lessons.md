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

Before retrying, name the failed stage, frozen inputs, supporting observation and
smallest discriminating check. Do not turn a workaround into a universal
exception or lower a safety assertion for a green run. If a focused pass later
exposes a race, preserve both results and fix synchronization rather than adding
arbitrary sleeps or rerunning until lucky.
