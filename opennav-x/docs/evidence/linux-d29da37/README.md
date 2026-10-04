# Exact d29da37 Linux integration audit

[Linux job 111383445948](https://github.com/ThereptileII/Work/actions/runs/37184477492/job/111383445948),
run **37184477492**, attempt **1**, completed successfully at remote candidate
`d29da372af86c01cb2817f932891fd9408d882fe`. All 29 reported job steps completed
successfully, including the fixture-free build and artifact upload. Root supplied
the previously verified publication mapping from frozen local
`615118f351abdea7b79a042db63e58fe0a635e0a` to tree
`06beb1e9a06af7d7ee30abf32556a08602a165b6` (6,781 entries); this audit did not
reconstruct that whole mapping again.

Original [artifact 11296938668](https://github.com/ThereptileII/Work/actions/runs/37184477492/artifacts/11296938668),
`linux-integration-d29da372af86c01cb2817f932891fd9408d882fe`, is **19,751,200 bytes**,
SHA256 `90476e6898f36ee7b28d794945894805953a7ef1bd788abe9759fd9e5482776f`.
The local original independently matches the authenticated artifact metadata.
All **4,251 entries** passed CRC and safe-path checks, including case-fold
uniqueness, with no traversal, drive paths, backslashes or symlinks. The original
ZIP remains privately in the main worktree's `.local/linux-integration-d29da37/`;
this audit's extracted copy remains under the same relative directory in its
worktree. No duplicate full archive is committed.

| Original evidence | Audited result |
| --- | --- |
| Fixture-enabled application XML/log | **147 unique cases**, no failures/errors/disabled/skipped; CTest 13.55 s. |
| Fixture-free production XML/log | **147 unique cases**, no failures/errors/disabled/skipped; CTest 12.99 s. Same names: **294 build executions**, not 294 distinct tests. |
| Production loader | Exact remote commit; 5 checks passed, `test_fixtures:false`, `INSTALLED PRODUCT`, hardware policy `status-only`, required resources present, no profile initialization. |
| Production pilot | Five loopback checks passed; saved permission cannot enable commands; zero received bytes, empty sent/wire-output/transport-failure lists. |
| Actual patched Downloader | **12 cases**, expected 3 accepts and 9 refusals. Covers HTTPS/redirect, downgrade/file redirects, partial transfer, wrong/untrusted/expired certificates, stream failure, unrelated working directory, initial HTTP and missing trust. |
| Actual patched core wxCurl | **13 cases**, expected 7 accepts and 6 refusals. FTP/telnet entries are configuration-only checks, not successful network transfers. |
| Additional retained gates | Peer buffer: 1 CTest case; floating surface: 17 checks; GTK frame recapture: 34 checks, exit 0. Fifteen application result reports retain passing outcomes and their original screenshot-review limitations. |
| Endurance | **Skipped by explicit user direction**, bound to this exact commit/run/attempt; `release_duration:false`. Never counted as a pass. |

[audit.json](audit.json) records the counts, exact outcomes, hashes and source
bindings. [selected-original-reports.zip](selected-original-reports.zip) retains
27 unmodified original reports/logs/XML files (63,090 bytes); each extracted
entry's hash is recorded. [endurance-skipped.json](endurance-skipped.json) is the
unmodified skip record. No screenshots were newly reviewed in this audit.

Frozen-source review confirms that both probes include `TrustProbeConsole.h`
only under `_WIN32`; the private-adapter preparation input closure now includes
that header. Its frozen bytes match the [previous native console proof](../scrum211-native-console/README.md)
after LF-to-CRLF checkout conversion. These Linux trust runs use their explicit
test-CA macros and therefore do **not** exercise the Windows native CA store,
the console helper, or the private o-charts wxCurl copy.

The artifact does not contain executable bytes: the fixture hash
`f2dd2f32d4aec2d313363eee589c6ceb28b78c8ff91333853c7aef4d5f34c355`
and production hash
`3592ff800473027c9a87c33be0dd51f68b6aba7125862ff4e641ce2946eaf481`
remain runner observations. Linux success does not qualify native Windows,
installer/package, private-renderer, physical GPU, boat or public-release gates.

Root subsequently viewed the original `chart-software-01-loaded-linux.png` and
`chart-opengl-01-loaded-linux.png` from that same archive. Both visibly retain
Seattle coastline, land, depths and chart objects, with the neutral palette and
compact SKAGER header. The wide view does not establish individual symbol
recognition, prototype-perfect typography, private-chart or boat GPU acceptance.
