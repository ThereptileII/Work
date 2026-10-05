# Exact 4ddf1f3 contract jobs

Both completed contract jobs in [run 37155858878](https://github.com/ThereptileII/Work/actions/runs/37155858878) report successful CTest suites. Each terminal log was fetched once through the GitHub connector; no live logs, reruns, tests or application actions were performed for this audit.

| Platform / job | Release CTest suite | Additional restart lifecycle repeat |
|---|---|---|
| ubuntu-24.04 / 111298976067 | 94 passed, 0 failed; 5.86 seconds | One existing test repeated 10 times, all passed; 19.50 seconds |
| windows-2022 / 111298976070 | 91 passed, 0 failed; 6.54 seconds | One existing test repeated 10 times, all passed; 20.01 seconds |

These are separate platform executions, not 185 unique tests. The 91 Windows test names are a subset of the Linux suite. The three additional Linux cases are `early_startup_fixtures_isolation`, `early_startup_product_isolation` and `early_startup_msw_excluded_isolation`, registered only on Linux by `CMakeLists.txt:233–250`. They were not skipped Windows tests. Neither log reports skipped/not-run CTest cases or an error annotation. Both reach post-job cleanup after the repeat succeeds. This does not assert the status of other jobs or unexecuted application gates.

Both original checkout excerpts bind remote `4ddf1f383e495150551946dd36103cd77cea85eb`. The parent-supplied independently verified mapping is local `48c2f8b8d5d4103bcbaacc45a18c4e6a238d4c52`. Frozen `.github/workflows/opennav-baseline.yml:82–160` defines this matrix, the Windows Win32 configuration, suite and repeated lifecycle command. The exact workflow hash is in `receipt.json`.

The two small `.log` excerpts retain original timestamped bytes, including Windows CRLF. `receipt.json` records their source line numbers and SHA256 values plus full decoded-log hashes. Complete logs remain ignored in this evidence worktree's `.local/contracts-logs/`; they were not added to Git. These hashes identify the connector-returned decoded UTF-8 logs, not an original GitHub ZIP archive. No authentication values are retained in the excerpts.

Scope: contract qualification only. These results do not establish integrated application, AIS/TLS, private renderer, installer/package, endurance, native visual or boat acceptance for this candidate.
