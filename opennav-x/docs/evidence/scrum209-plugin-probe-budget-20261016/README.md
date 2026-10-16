# SCRUM-209: plugin import probe budget

Frozen candidate `854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc` refused all four
rebuilt builtin plugins with `Plugin import closure node budget exceeded` during
ordinary installed startup. The counter charged every unique directory/basename
candidate, including missing files. Repeated imports and directory aliases were
already deduplicated; this was not an unbounded cycle.

Fix `0648d4ed43bdbc7f8f002fcf434d9a2b0dcc6bf4` (integrated as `896ad520`)
separates **1,024 candidate probes**, including misses, from **256 admitted
dependency PE nodes** and the unchanged **256 MiB aggregate byte limit**.
The root-count preflight, inspection of every supplied root, name/path rules,
normal/delay/forwarder parsing, known-obsolete-byte checks, read failures and
native file custody remain unchanged.

## Retained-byte measurement

The authenticated recovery archive and two previously retained CRT files,
whose hashes match the recorded managed-plugin inventory, supplied inert input
bytes. No DLL was loaded or executed. The compiled C++ graph and SHA-256 byte
policy reproduced the original refusal and admitted all four graphs after the
correction. All 90 staged input-file hashes remained unchanged.

The modeled roots are plugin directory, host directory and managed-plugin
directory, plus the repeated plugin directory supplied by upstream `Libdirs`.
The recorded separate plain directories use synthetic stable identities here;
the original application log does not record their native volume/file IDs.

| Policy / topology | Probes | Found dependency files | Missing candidates | Outcome |
| --- | ---: | ---: | ---: | --- |
| Original, package-only two roots | 178 | 59 | 119 | Structural walk completes |
| Original, three roots including managed CRTs | 256 | 58 | 198 | All four refused before probe 257 |
| Corrected, same three-root bytes | 270 | 62 | 208 | All four graphs admitted |

The completed three-root graph has 90 unique module basenames. The trace places
probe 257 at the host candidate `api-ms-win-core-memory-l1-1-0.dll`, declared by
packaged `ucrtbase.dll`. Even an empty third existing directory requires 267
probes, so removing the managed CRTs would not fix the budget defect.

| Root plugin | Completed aggregate bytes, including root |
| --- | ---: |
| `chartdldr_pi.dll` | 40,724,816 |
| `dashboard_pi.dll` | 40,971,600 |
| `grib_pi.dll` | 41,448,272 |
| `wmm_pi.dll` | 44,680,016 |

## Focused checks and limits

`win32_pe_imports_test.cpp` passed **9,930 portable C++ checks**, compiled with
`g++ -std=c++17 -Wall -Wextra -Werror -pedantic`. Coverage includes the
270-probe/62-file topology; exact 270 versus 269 probe limits; the default
1,024-probe refusal; exact 256-found admission and 257-found refusal; shared
byte bounds; duplicate roots/cycles; and normal-import, delay-import and
byte-policy refusals after probe 256. Two independent read-only reviews found
no blocker. No tests were repeated to create this documentation.

The 256-node limit bounds admitted/queued dependency PEs, not every transient
read or held handle. The callback must read a found candidate before the walker
can detect a node/byte overflow; the native per-file read remains bounded at
128 MiB. The root PE is additional to the dependency-node count and included in
the aggregate byte count.

## Focused native Windows result

[Run 38910455080, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38910455080)
([job 117004670051](https://github.com/ThereptileII/Work/actions/runs/38910455080/job/117004670051))
passed **9,930 checks** at exact source
`0615e79ba122a32d5adb6193ab5c5b9bfad30ce9`, using native **Win32 MSVC
19.44.35229.0** on image `20261012.333.1` (Windows SDK `10.0.26100.0`).
The four reported source/workflow hashes match that commit's Git blobs with
normal Windows CRLF checkout bytes. Original artifact `12049428405` is 2,237
bytes; its API SHA-256 and all four member CRCs/hashes match the retained
archive. The machine-readable receipt records those exact hashes.

This accepts the compiled declarative policy fixture on native Windows. The
fixture uses synthetic PE graphs; it does not load plugin DLLs or qualify native
file custody, actual loader topology, builtin `Init`, or the installed 854
application. No check was rerun for this documentation.

Application build, native product qualification, native file custody/topology
and installed boat acceptance are **not established** by this focused fixture.
Full candidate acceptance remains separate; original 854 failures remain
failures. This record makes no assertion about a full Staging run's status.

[receipt.json](receipt.json) records exact source/input/result hashes. Large
original logs, graph traces and executable inputs remain private; this directory
contains no vendor binaries, personal paths, configuration or credentials.

## Replacement candidate prerequisite timeout

Candidate `9feae59ce0ef1f54107c3c51f920085297ea7bc8` in
[run 38911440527, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38911440527)
passes both platform contract builds and native boat commissioning/recovery.
Updater job `117008198606` fails before application compilation: the existing
wrapper kills `test-provisioner-package.py` at its 60-second whole-suite limit.
Thirteen successful test dots precede the timeout; there is no final summary,
assertion failure or recorded child stack. Original log SHA-256:
`6190ad0c7d975f5061444a8539b75bb924f7d658f104458bc28ba052556d03f2`.

The same 14-test suite passed in 56.119 seconds in predecessor job
`116527478215`, run `38765418126/1`, with the same Windows image
`20261012.333.1` and Python `3.12.10`. Eleven relevant workflow, test, fixture,
producer and verifier files have identical Git blobs/modes. Three substantial
preceding suites now take 6.5–7.4% longer; a 7% increase puts 56.119 seconds at
60.047 seconds. Static inspection finds 1,078 archive rewrites across the
source-omission and registration-tamper cases, plus fresh per-test fixtures.
No real compiler, application or network is invoked by that suite.

This supports a tight whole-suite deadline explanation, without establishing
a specific host I/O, antivirus or process cause. The coordinator selects at
most one retry of the failed job and downstream dependencies after the current
run completes, retaining the candidate, assertions and time limit unchanged.
A repeated failure requires a focused harness correction, not further retries.
The original failure remains evidence; neither this analysis nor a prior pass
qualifies the new product. Private comparison receipt SHA-256:
`43b46a14cc82a549d24241117a81eca0ae3cf501cf8dd7840d8bf6980b28dcc0`.

The original Linux job `117008195582` completed successfully at 14:36:58 UTC:
154 distinct CTest names passed in both fixture-enabled and fixture-free builds
(308 executions), alongside chart/data/lifecycle and loader checks. The preview
design sweep and endurance were skipped. Original Linux log: 995,530 bytes,
SHA-256 `bddecad8060a66758e5aa830d39d844a8340e278a8d86cc01f9dcb59ddf79c84`.

After attempt 1 completed, the single selected failed-job replay created attempt
2, retaining the successful Linux and other independent jobs. Windows updater
job `117028247900` completed successfully at 14:55:45 UTC. All eight source-package
suites passed (18, 16, 33, 12, 10, 14, 7 and 16 cases: 126 total). The previously
timed-out provisioner suite passed its 14 cases in **48.163 seconds**, with the
same source, assertions, Windows image, Python version and 60-second deadline.
Original replay log: 5,766,950 bytes, SHA-256
`b919a1f8fdf8a1a9cb7cc5508b8e0afc68dd05ebaf1b1b977a62e51083d7374b`.
This result does not erase the original failure or establish its specific host
cause. Native application producer `117036304896` started subsequently; its
build/package/boat results are separate and not accepted by this record.
