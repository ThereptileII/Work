# SCRUM-311 live installed-journey composition

`tools/installed_updater_coordinator.py::run_live` composes the maintained
approval HTTP/transport/binding modules with one `run_joined` invocation. It is
an executable Python API for the pending qualification job, not a publisher,
approval producer, host preparer or customer product entrypoint. The caller must
already authenticate the original candidate/operator-kit producer artifacts,
prepare both maintained declarations and supply the independently authorized
approval producer `ApprovalSource` and exact pending context. It must retain the
same original Windows job, account and files; a newly started job is not a
continuation of approval for an earlier job.

The coordinator retains the existing native account/profile observer and original
consumer executable deny-write owner through approval wait and actual journey.
The separately approved consumer hash must identify the harness-only native Go
test executable, not a shipped provisioner. The exact private invocation is:

```
<approved harness executable> --consume-installed-b0-challenge=<protected qualification parent>
```

Before approval, that same original helper authenticates and retains protected
deny-delete ancestry, then flushes one READY JSON line with exactly schema=1,
state=`ready` and parent (the full native identity below). It remains alive with
stdin open throughout approval. The final parent identity must equal READY. EOF
before a complete record drains that child with no consumption effect.

After approval, stdin is at most 4096 canonical JSON bytes, with exactly schema=1,
purpose=`installed-updater-b0-consumption`, repository, headSha, runId,
runAttempt, jobId, jobName, challenge, requestSha256 and approvalSha256. Native
code authenticates protected ancestry and exclusively creates the fixed child
`.installed-b0-consumed-<challenge>.json`; it flushes, reopens, checks full native
identity and exact bytes, and emits success only after checked cleanup. Existing,
partial, foreign or uncertain records permanently refuse; no deletion/reset or
retry is available. Product code-upgrade reservations are not reused.

The second and final bounded stdout line has exactly schema=1, state=`consumed`,
recordSha256, parent and record. Each identity has volumeSerial and creationTime
as nonzero canonical uint64 decimal strings, fileID as nonzero lowercase 32-hex,
and descriptorSha256 as lowercase 64-hex. Parent and record have the same volume
and distinct file IDs. The receipt is retained in a separately selected absent
evidence file outside the protected parent and outside the joined journey's
fresh result directory. Python retains its original output handle and obtains an
independent read cursor through Windows ReOpenFile on that handle, with
GENERIC_READ and FILE_SHARE_READ|FILE_SHARE_WRITE, never a pathname reopen. The
reader closes only after the original child drains. Each frame is at most 4096
bytes, total 8192; a third frame refuses. READY and completion each have a
sixty-second observation/execution deadline. Native code provides the directory-security authority;
Python path/identity checks do not replace it.

Approval expires and original input/account/profile checks repeat immediately
before and after consumption. A failure after consumption cannot re-enter the
journey. A sixty-second consumer deadline refuses late success; if the original
child is still live, the coordinator waits for its actual exit while retaining
executable custody, without kill/relaunch or later installation. This drain may
outlive the execution deadline. Cleanup failure suppresses success.

`run_joined` retains its existing exact online/offline publication handoff and
acceptance boundaries. The coordinator does not supply signed artifacts,
postproduction approval, private-network authorization, stock/profile fixtures,
real interactive UAC capability, Repair or automatic-fallback qualification.
Portable tests use inert API/ZIP and process fixtures; native consumption and the
actual installed customer journey require separate execution evidence.

Windows qualification must execute the real same-object ReOpenFile streaming
adapter against the native helper, including READY, EOF-without-effect, exact
consumption and permanent replay refusal. Linux fd-based inert fixtures prove
composition only and are not that native adapter evidence.
