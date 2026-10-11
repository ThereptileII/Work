# SCRUM-311 focused native owner qualification

[Run 38168539396](https://github.com/ThereptileII/Work/actions/runs/38168539396)
passed at exact `b08f9a1cf3360ff10e075b8b0134e5c39ac15f73`, attempt 1.
Both selected jobs succeeded; unrelated application/UI/build jobs were skipped.

Windows PowerShell 5.1 ran as both 64-bit and 32-bit processes, with separate
passed receipts. Each owner harness passed **79 assertions** and each transaction
harness emitted **60 passing summaries**, including nine emitted-application ABI
checks. The actual image owner exercised creation after SUBST remap, selected-name
refusal, authenticated health, normal close, uncertain resume and interrupted
cleanup. Transaction checks retained spoof/malformed/timeout refusal, one-shot
startup, rollback, live-child preservation and failed-persistence handling.

The Go Win32 test executable passed **10 top-level groups and two subtests**,
including all nine required named fixtures, with no skipped or failed tests.
The Go toolchain itself runs as windows/amd64; the test binary uses GOARCH=386.

[audit.json](audit.json) verifies both artifact API digests/sizes, all eight ZIP
member CRCs, three exact-commit passed receipts and **25 unique native source
hashes** against Git with the actual Windows CRLF checkout normalization.
Twelve original API/job/artifact records are retained under `raw/` without text
conversion. Archive identities are recorded in the audit.

The earlier failed runs remain retained. The final corrections changed test
window visibility, x86 script invocation and the transaction fixture's emitted
ABI/error reporting. Production launch/cleanup code was unchanged by those
fixture repairs. This confirms the selected held-image launch mechanics; general
DLL/resource/environment custody, the signed commissioning broker, integrated
product qualification and boat acceptance remain separate gates. No full
application build, boat operation or release was performed by this run.
