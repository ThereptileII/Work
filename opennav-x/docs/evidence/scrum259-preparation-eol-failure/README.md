# SCRUM-259: inherited Git EOL policy blocked preparation

[Full run 37097634494, job 111130711901](https://github.com/ThereptileII/Work/actions/runs/37097634494/job/111130711901)
failed at remote `c9ff4ae2c110d234807a326bc30c25084eb42a5d` (local
`dfa7b721ef6eca3f084f77e95dd8e2adc20bde4b`) before the native loader wrapper ran.
The first failure was the exact-byte assertion in
`test_patch_isolated_from_parent_git_and_crlf`: expected `b'after\n'`, received
`b'after\r\n'`. Thirteen of fourteen preparation checks passed. The later upload
error was secondary: the loader had not created its evidence directory.
`job.log.gz` retains the complete connector-provided job log, normalized only to
LF for storage. There was no uploaded loader artifact and no native guard result
from this failed job. The earlier standalone 38-group pass remains separate.

The production preparer normalized the derived input text and patch to LF but
let its isolated `git apply` inherit the runner's `core.autocrlf` policy. The
identical failure was reproduced locally using `core.autocrlf=true`; the original
failure is retained in `reproduction-before.log`. The correction sets
`core.autocrlf=false` and `core.eol=lf` only on the private apply/check commands.
Original pinned blobs and binary import library remain untouched. The existing
test retains its exact LF assertion and now exercises inherited `true`, `input`
and `false` policies with `core.eol=crlf`.

All fourteen focused preparation checks pass after the fix. Independently
verified all 219 pinned source/import blobs, applied both actual patches, and
compared all 219 derived files under default and inherited Windows EOL policies;
the results were byte-identical. No native compile is claimed by this check.

Both existing workflow loader jobs now run the same preparation/build-wiring
step before the unchanged native loader command. Each creates a separate
`ocharts-loader-contracts` log directory first. Native command exit codes are
checked explicitly, including the child PowerShell wiring check; the existing
always-upload step includes those logs. This avoids precreating the loader
wrapper's required fresh evidence directory. No `needs` dependency, assertion,
loader source, native test or trigger branch was removed or changed.

The exact shared shell block executed locally under PowerShell: fourteen
preparation tests and the existing wiring/order/reuse plus sixteen refusal
checks passed. A controlled preparation exit 7 retained its failure log and
prevented the later wiring command. This tests orchestration, not Windows native
loading. No CI dispatch, rerun, application build or boat action was performed.
The existing fast native loader branch must qualify the corrected source before
another full candidate can use it.
