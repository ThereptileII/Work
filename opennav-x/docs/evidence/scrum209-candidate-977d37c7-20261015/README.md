# Original candidate and focused installer evidence — 2026-10-15

SCRUM-209 / SCRUM-22 / SCRUM-212. These records preserve original CI results;
they do not accept a package, boat operation, or an entire issue. No checks were
rerun to prepare this evidence. Private paths, configuration contents, fixture
credentials and raw logs are excluded. Original artifact/report/log hashes are
retained; the sanitized records have different bytes from the original reports.

| Scope | Exact source and original run | Result |
| --- | --- | --- |
| Cold observer, filesystem and protected-broker guards, native PS5.1 x86/x64 | `22f9108427009871807f47e1b7abf3e66c88f5f3`; [38700086122, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38700086122/attempts/1) | Pass, focused scope only |
| Earlier cold regression | `5c5719c85f3fbc399c46974343560c4890aedf97`; [38699617052, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38699617052/attempts/1) | Failed negative-fixture classifier; preserved |
| Windows CLI, modes and concurrent ordinary/portable applications | `977d37c79f652fd9b9e28e55f26ec259c35eea8e`; [38686209123, attempt 2](https://github.com/ThereptileII/Work/actions/runs/38686209123/attempts/2) | Peer containment/preservation checks passed; later Setup Preflight failed |
| Linux CLI, modes and concurrent applications | Same `977d37c7` source; [38686209123, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38686209123/attempts/1) | Passed development checks; no Windows/boat acceptance |

## Focused installer result

[installer-cold-receipt.json](installer-cold-receipt.json) binds both native jobs,
downloaded artifacts, result members and log hashes. Each host ran 44 filesystem
checks plus the protected-broker guard. Fresh PS5.1 child processes compiled the
actual observer with a real native `System.dll` shadow, then exercised a real
compiler TEMP-path failure. Both distinct caller directories were restored in
both cases. The failure was specifically `Win32Exception`, native code **3**;
the observer type remained absent. Present/unknown broker refusal and durable
state preservation passed.

The earlier run already passed positive cold compilation and broker guards,
but failed because its negative fixture recognized only managed I/O exceptions.
Its red result is not replaced or relabeled by the corrected run.

## Original peer runtime result

[peer-runtime-receipt.json](peer-runtime-receipt.json) binds the original run,
jobs, API artifact digests, downloaded bytes, report hashes and source hashes.
Native CLI success belongs to compilation job `116274722975`, attempt 2; Linux
CLI success belongs to job `116261621665`, attempt 1. The native receipt binds
the executed CLI and three source files, allowing only LF/CRLF checkout changes.

Both key commands refused without stdout/key echo or changed configuration.
Both concurrent-instance reports passed: profile/navigation fixtures and
certificate/key bytes survived, and observed process-owned TCP listener lists
were empty. Mode-cycle reports recorded five corresponding peer observations
per platform and passed persistence steps. Retained concurrent-instance reports
and streams were checked for fixture credential disclosure. These observations
are not packet capture or proof of every outbound path. Mode reports' historical
visual-review wording is retained; no design acceptance is claimed here.

## Remaining package gates

Original native runtime job `116290535162` and run `38686209123` remain failed.
The installer report stopped at **Preflight**; it does not establish successful
Setup or downstream installer/chart acceptance. The observer implementation
changes an installer product input, so the focused `22f91084` pass cannot repair
or qualify the immutable `977d37c7` package.

A new exact packaged candidate still needs its applicable native Setup,
install/update/repair/rollback/uninstall and profile-preservation gates, runtime
dependency/TLS closure, downstream chart checks, and source/license integrity.
Boat acceptance, restored peer sharing/trust and credential migration remain
separate. No new build, release or boat acceptance follows from these records.

The pre-existing [linux-receipt.json](linux-receipt.json) records the original
Linux regression result independently and is unchanged by this evidence update.

## Retained native plugin qualification

The [retained native run 38704113599, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38704113599)
passed using helper `b096faa20828c5c85b81e4418d074e157cde1457` against the
original producer `977d37c79f652fd9b9e28e55f26ec259c35eea8e` from run
`38686209123`, attempt 2. The application was not rebuilt.

The [sanitized receipt](plugin-retained-native-receipt.json) records the
verified result artifact `11957991338`, its SHA-256, the independently reviewed
retained inventory and 99 consumed-source bindings. Only the standalone probe
changed. Native results passed 13 plugin download cases, the ignored-failure
negative mutant, verified cleanup, 58 loader checks, 17,625 deterministic
forwarder checks and 282 plugin TLS byte-policy checks.

Original logs, metadata, downloaded artifact and the independently checked
[verification receipt](/home/standard/Work/skager-plugin-retained-38704113599/verification.json)
remain private. This qualifies the native fixtures; it does not grant installed
product or boat acceptance, or resolve the earlier installer qualification
failure recorded in [native-receipt.json](native-receipt.json).
