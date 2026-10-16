# Continue a candidate without unnecessary rebuilds

Use with the selected checkout's `opennav-x/docs/build-efficiency.md` and
`opennav-x/docs/delivery-workflow.md`. This is an operating guide, not permission
to build, deploy or promote. Historical success never selects a future candidate
or dependency bundle.

## Decide from the failed stage

| Actual situation | Next action |
| --- | --- |
| Version or distribution preflight fails before compilation | Correct the inconsistent product inputs and rerun the cheap preflight before requesting the candidate build. There are no completed application outputs to recover from that failed stage. |
| Product code, packaged resources, installer or dependency inputs change | Use the change selector and build the affected candidate. A previous executable does not acquire the new commit's identity. |
| Unchanged inert test suite hits its whole-suite deadline with evidence of timing variation | Use `skager-test` to inspect the original failure. A justified single failed-job replay may be appropriate after the run is terminal; do not rerun all jobs or repeatedly retry until green. |
| A downstream test helper is corrected, product/package inputs unchanged | Use the maintained retained-check workflow and its allowed scope against authenticated original inputs. Record the new harness commit separately; use explicit composed qualification when required. |
| Qualified package assembled, draft asset upload interrupted | Use retained publication to authenticate existing bytes and resume missing assets. Do not rebuild or reassemble to repair transport. |
| Boat SSH drops or an installer wait expires | Inspect the existing process, report and transaction journal before any retry. An ended SSH connection or timeout does not prove the remote operation stopped. Boat recovery and startup authorization remain separate. |

If the failing prerequisite prevented application compilation, no retained
application/package exists. Qualify a prerequisite helper correction at its own
boundary, then inspect the supported workflow continuation. Post-build retained
checks cannot substitute for arbitrary prerequisites. If no explicit supported
reuse/composition path exists, the coordinator selects the corrected candidate
run; do not fabricate mixed-revision acceptance to avoid that run.

## Freeze identity and retain useful work

The coordinator freezes the candidate SHA/version, selected workflow/options and
dependency inputs. Other agents can investigate failures or prepare future
changes in parallel; they must not mutate the frozen candidate or start another
full build. Keep the existing CI run/job and remote-operation handles.

Use the current workflow's separation of responsibilities:

1. Qualify the dependency producer and fresh consumer environment before reuse.
2. Build the native updater once in its clean prerequisite job; authenticate its
   output in the application job rather than recompiling in the integrated tree.
3. Retain compiled recovery before packaging. It is unqualified recovery material,
   not a deliverable or a replacement for sealed qualification inputs.
4. Retain sealed application/package inputs before downstream native qualification.
   A test-helper failure need not discard that completed compilation.
5. Assemble the draft Staging release only after required qualification succeeds.

Record each job's actual run, attempt, commit and result. A successful prerequisite
from attempt 1 may feed a downstream job in attempt 2 when the maintained workflow
authenticates it; never relabel the first job as rerun. Changed helper evidence
and same-source job replay are different paths. Preserve the original failure.
For timing comparisons, retain the measured stage and elapsed time, runner/tool
versions and reuse status. Do not infer total cycle duration or promise a speedup
from one successful stage or its completion timestamp.

## Verify the delivered bytes once

Use the maintained `opennav-x/tools/github_release_delivery.py fetch-staging`
entrypoint (inspect its current arguments) for the exact authorized draft tag.
Require complete producer/qualification evidence and verify actual downloaded
Setup, recovery, source, notices and release inventories. Artifact metadata, an
upload message or a compiler success alone does not authenticate those payloads.
Retain the fetch receipt and reuse the verified local copies. Avoid downloading
large equivalent intermediate archives unless investigating their specific stage.

Compare the installer hash again on the destination before execution. Report
native CI, package verification, installation and ordinary startup separately.
Cold DLL hashes do not prove plugin initialization. Do not broaden a requested
install/start check into design review or endurance; those remain explicit opt-ins.

## Source-backed example, not a fixed recipe

Frozen `9feae59c`, Staging run `38911440527`, passed after one failed-job replay:
the unchanged 14-case inert suite completed in 48.163 seconds under its original
60-second cap; original independent Linux/restart results remained attempt 1.
Windows built once after the prerequisite passed, then qualified retained bytes.
The later 180-second harness allowance was **not** used to qualify that candidate.

Read `opennav-x/docs/evidence/scrum209-plugin-probe-budget-20261016/README.md`
for the evidence and limits. The separate
`opennav-x/docs/evidence/scrum17-9fe-install-20261016/README.md` records an
installation followed by an interrupted recovery check and pending boat startup.
Neither historical record claims completed boat or public-beta acceptance.
