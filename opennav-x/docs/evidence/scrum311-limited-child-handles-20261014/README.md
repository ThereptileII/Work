# Original child standard-handle observation

Exact diagnostic helper `fe21e95add675a0bd1118086d4b8e3d51ad4bcbe` in
[run 38634616586 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38634616586)
refuses before resume with `original-probe-standard-handles-refused`.
The original suspended child's stdin, stdout and stderr values each return
`ERROR_INVALID_HANDLE` (6) from `DuplicateHandle`, with the original child as
source and observing parent as destination. No comparison or resume was reached.

This proves those intended standard-handle values do not designate valid objects
in that exact child's handle table. It does not prove the cause of the earlier
60-second timeout. Child progress, successful exit, usable desktop and customer
UAC acceptance remain unobserved. Original identity checks passed. Cleanup
remains deferred to destruction of the disposable job; no product was installed.

The helper preserves existing token, account, logon, session and one-resume
requirements. It adds no child handle, privilege, desktop ACL or timeout change.
Portable tests: **45 probe + 34 customer-launch cases pass**. Native qualification
remains blocked; the next investigation concerns a correct standard-handle
launch, not repeating this unchanged diagnostic.

Original run, job, source, artifact digest/length and ZIP CRC were independently
checked. [Receipt](receipt.json) omits disposable account identifiers; complete
original artifacts remain private. No Windows application build was requested
for this diagnostic-only revision.
