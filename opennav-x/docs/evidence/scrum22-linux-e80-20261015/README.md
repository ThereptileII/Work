# SCRUM-22 / SCRUM-209: exact-candidate Linux evidence

The original Linux integrated job and its fixture-free successor passed for
`e80cb962e1a32d98bb75b964dca6eb516f7a5e23` in
[run 38744236920, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38744236920).
This directory records that tested candidate. Its later documentation commit
does not identify a new application build or qualify another product revision.

[receipt.json](receipt.json) retains the exact job identities, step timestamps,
counts, original-log hashes, selected artifact-member hashes and source hashes.
The original logs and complete archive remain private. Evidence collection did
not rerun tests, dispatch a build, modify the product or operate a boat.

| Execution | Result |
| --- | --- |
| Linux integrated CTests | 154/154 passed |
| Linux fixture-free CTests | 154/154 passed |
| Linux offline component executables | 13/13 passed |
| GTK floating-surface / frame-recapture checks | 17 / 41 passed |
| Fixture-free loader/resource checks | 5/5 passed |
| Ubuntu contracts | 118/118 CTests; 10/10 additional restart repeats |
| Windows contracts | 115/115 CTests; 10/10 additional restart repeats |
| Pilot serial / native protected-mode CTests | 3/3 / 3/3 passed |

These are separate executions, not a total of unique tests. All eight jobs in
this receipt succeeded; five additional updater jobs were skipped by the selected
workflow scope. The receipt preserves their names and the separate Go, Python
and native check counts. The offline component report records matching candidate
and harness commits but also `source_dirty=true` and `harness_dirty=true`; those
emitted flags are preserved, not represented as a clean-tree attestation.

That flag includes untracked paths. The original log records the upstream clone
under `opennav-x/upstream/OpenCPN`, which is untracked and not excluded by the
repository ignore rules: an expected preparation source of dirtiness. The raw
status/diff was not retained, so the complete cause is unknown. Preparation
independently verifies the pinned upstream tracked tree and its reviewed patch
result. The component manifest checks commit and target identities, and the
result hashes each executed binary; it does not retain a closed inventory of
every consumed source file. No source mismatch was observed in this bounded
review.

The [Linux job](https://github.com/ThereptileII/Work/actions/runs/38744236920/job/116453779566)
passed the original chart/mode step at `2026-10-15T13:58:54Z`. Both software and
OpenGL phases completed Legacy → XNav, with Dashboard visible in Legacy and
absent as a native window after return. The returned pane state was `2098127`
and the final saved preference remained `2098125` in both phases.

The corrected fixture waits for a diagnostic newer than the pre-restart
publication, from the returned PID, with Dashboard presentation enabled outside
plugin layout. It then checks the first eligible publication once for loaded
ENC, pane state and geometry, the originally hidden pane and visible rail. A
separate one-shot native-window assertion remains required. Missing or malformed
readiness and timeout still fail; the helper never polls for desired pane state.
The fixture-only observer reads an existing presentation without creating or
mutating it. On the UI thread, all Hide flags precede `manager.Update`; readiness
does not attest native repaint completion.

The archive's final saved diagnostic records PID `25035`, readiness `true`, UI
tick `4` and publication `1293393`. It is a final profile snapshot, not a
separately retained copy of the exact first-ready assertion sample. Successful
execution of that assertion is established by the unchanged candidate helper
and successful original chart step. The
[original failure diagnosis](../scrum22-linux-chart-readiness-20261015/diagnosis.json)
and [focused readiness proof](../scrum22-linux-workspace-readiness-20261015/verification.json)
remain preserved.

The fixture-free loader reports the exact candidate, `test_fixtures=false`,
`build_purpose="INSTALLED PRODUCT"`, no profile initialization and all required
resources present. Its executable SHA-256 is
`8454c5c5628ba4716a24b4bd72a29ece2da8039bba8bed4d146d692b0da0bbae`.
The original log records profile preservation and overwrite refusal. This
artifact retains the test evidence, not the application executable.

Linux artifact `11979468874` is 14,637,306 bytes, with SHA-256
`b8c28c02788b0fc727918cd4f76334f891492f310e77bc10883498c150960390`.
Its digest matches GitHub's original artifact metadata; all ZIP CRCs passed for
3,984 file members. The original Linux job log SHA-256 is
`9a1e42463d45c70e34c3b4cea52664831db8ebf52d98129f0972775fe428be15`.

Windows contracts also passed the maintained PE bounds, large-export, PE32+,
helper ABI and closure cases under PowerShell 7 and native PowerShell 5.1 x64/x86,
including actual system-directory and signed AMD64 Microsoft VC14 checks. This
component evidence does not accept a third-party TLS version or an installed
product generation.

Endurance was explicitly skipped under the existing user direction, with
`release_duration=false`; it did not pass. Opt-in design/prototype review was
not requested or performed. Functional chart checks and captured screenshots
are not design acceptance. Native Windows product/UI/GPU and fresh installed
boat acceptance remain separate; this receipt makes no release or physical
output claim.
