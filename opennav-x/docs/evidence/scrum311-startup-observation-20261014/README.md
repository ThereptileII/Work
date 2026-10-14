# Focused Windows startup observation and disconnected profile

Helper revision `1aa6275486ff6f981d3d46562c85705a3622b169`,
[run 38643305138 attempt 1](https://github.com/ThereptileII/Work/actions/runs/38643305138),
job `116123937795`. Portable checks pass 60 probe and 17 profile-adapter cases.
This is a diagnostic helper execution, not another application build.

The create-only profile step **passes** on native Windows. The actual ProgramData
known-folder lookup, retained canonical directory checks, relative `NtCreateFile`
creation, file-handle conversion and maintained `.ini`/navigation observer all
execute. The controlled route/waypoint fixture and disconnected settings hashes
are retained in the [receipt](receipt.json). No existing profile was adopted or
changed. This does not prove application profile loading/migration or an installed
customer journey; no application was installed or launched by this step.

The original LIMITED-primary account probe still reaches its 60-second timeout,
so the overall job remains **failed**. Original process/thread identity verifies
before resume and at timeout; the single ResumeThread call returns 1. The bounded
observation reports:

- Before resume: module enumeration is unavailable with Windows error 299.
- At timeout: enumeration succeeds; only `ntdll.dll` is observed among the seven
  explicitly retained module names. Kernel32, KernelBase, Python312, UCRTBase,
  User32 and `_ctypes` are not observed by this query.
- Original primary-thread accounted kernel/user CPU is 0/0 in both samples;
  pending I/O is false and the bounded thread-window list is empty.
- No child progress bytes, entry marker, final report or exit are observed.
  Caller desktop and UAC policy remain unchanged (1/0/1).

These observations distinguish an early startup boundary but do not prove zero
instructions, inactivity of all threads, a complete mapped-image inventory,
permanent suspension, an exact loader wait, console cause or SKAGER crash.
No launch-policy/privilege change or equivalent retry is selected from these
observations. Cleanup remains correctly deferred to disposable job retirement.

Both original archives were authenticated against original run/job/upload
metadata, size, provider SHA-256, sole result member and CRC. Root independently
rechecked archive hashes, lengths, CRCs and member equality. The receipt excludes
account SID and process identifiers. Its account helper hash identifies the
unchanged `installed_customer_launch.py` child under Windows CRLF checkout,
not the new parent observer. Private original evidence stays outside Git.

Application candidate `89854110b41286cab0c6bf9748f37882a595a366` remains unchanged.
Native installed/UAC, joined update/recovery and boat acceptance remain pending.
