# SCRUM-255: native prerequisite proof passed

[Run 37087435049 / job 111100530739](https://github.com/ThereptileII/Work/actions/runs/37087435049/job/111100530739)
passed at remote `4330cae78c53707310e94ded4d1b933d0fb519a8`, mapped to exact local
`36ab8e7b2e59e94dd35fc0486c5bc400315779e9`.

Artifact **11261311795**: **10,885 bytes**, SHA-256
`850564d42deeeab8861f933be77444d234328e6585cc40c7ff12909f8cdec131`.
Independent download matches GitHub's digest; all **30 ZIP entries** pass CRC.
All original entries are retained under `raw/`, with hashes in `verification.json`.

All **25 native contracts passed with no skips**, including actual Windows short
alias/same-file proof and junction-ancestor refusal. The retained diagnostics
confirm `RUNNER~1` expands to `runneradmin` with normal file/directory attributes.
Poedit **3.9.1 acquired successfully on the first attempt** through the existing
pinned Chocolatey provider. Both exact known-path Gettext tools report version
0.26, and their live identity reprobes pass. The real `msgmerge` and `msgfmt`
operations succeed; this audit independently reads the native-produced MO file
and confirms the exact UTF-8 translation `SKAGER översättning`.

Four source inputs match local 36ab8e7 with only Git CRLF conversion. 8 native
stdout/stderr pairs, receipt, merged catalog and compiled catalog match their
recorded hashes. Tool executable hashes are recorded native observations; the
binaries were not uploaded for an independent byte rehash. Earlier failures
remain retained separately.

This closes the short prerequisite proof, **not full product qualification**.
No app/dependency build, native chart/GUI/DPI, installer or boat result is implied;
`nativeProductAcceptance` remains false. No rerun, full workflow, push or boat
action was performed by this audit. Root controls the next integrated candidate.
