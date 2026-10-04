# Windows ecf7e0c — partial passes, private module access violation

**The native job failed; no recovery package or Setup is eligible.** In [job 111405087286](https://github.com/ThereptileII/Work/actions/runs/37191400051/job/111405087286), the fixture-free product build passed at 11:07:29 UTC, then the real-host private chart module check failed at 11:07:33. Public Downloader's separate gate, peer-buffer gate, recovery packaging and installer transactions were skipped. Independent DPI and public ENC checks subsequently passed. This audit preserves those boundaries; it does not authorize a build, retry, publication or boat action.

Exact candidate `ecf7e0c46609cf4cb29141964d7c4bde98b002b7`, run `37191400051`, attempt `1`; frozen local source `1988df7a8a0ae8ddc6365ca46026d32fccfa0bdc`. [api.json](api.json) retains the authenticated terminal job and artifact metadata.

Original artifact **11301641242**, `windows-integration-ecf7e0c46609cf4cb29141964d7c4bde98b002b7`: **77,267,696 bytes**, SHA256 **`ce868e58356da4e74ba469f7f918bf2500d45e3f44cc2e3e9936c4aa5645fe17`**. The downloaded archive matches the API size/digest; all **19,800 entries** pass CRC and safe relative-path, case-fold uniqueness, no-encryption and no-symlink/special-file checks.

Original archive: `/home/standard/Projects/X-nav-worktrees/bccdbb1-contracts-restart/.local/windows-integration-ecf7e0c/original.zip`. Selected original reports are extracted under its sibling `extracted/` directory, preserving their `evidence/local/` and `build/` paths. [selected-original-reports.zip](selected-original-reports.zip) retains 137 unmodified documentary member payloads; it is a new subset, not the original artifact. [audit.json](audit.json) records each retained member's size/hash and the independent checks. No binaries, charts, certificates or profiles are committed here.

## Failure boundary

`evidence/local/ocharts-real-host-module/summary.json` records:

| Case | Actual / expected process exit |
|---|---|
| Option without self-test | 2 / 2 |
| Default loader self-test | 0 / 0 |
| Module check | **3221225477 (`0xC0000005`, access violation) / 0** |

`module.json` is absent; `module.stdout.txt` and `module.stderr.txt` are empty. The summary reports `profilesUnchanged=true`. Thus the process failure is established, but its internal crash site is not. No loader-stage or root-cause conclusion is inferred. All 21 recorded harness/application source identities match the frozen local Git blobs with their actual Git or Windows CRLF representation.

The failed module-check summary records production `opencpn.exe` SHA256 `9580a069cf37b5ce20df082e7e5ddf08140c21f6f1cdfd816d283ef1625e3583` and adapter SHA256 `604b46a91a13a38196ee6b6933194e872e2c00b6c0cde5dc538e10d6e8a67c09`. These are original report identities, not independently rehashed binaries: this archive does not contain `opencpn.exe`.

## Completed evidence

| Gate | Recorded result and limits |
|---|---|
| Fixture / production CTest | **139 + 139** executions; identical **139 unique** case identities. Zero failures/errors/disabled/skipped in both XML files. Production cache has route scenarios, fixtures and pilot loopback OFF; fixture cache has them ON. |
| Production default loader | Exact candidate; passed; `INSTALLED PRODUCT`, fixtures false, `status-only`, plugins/profile initialization false, four resources present. |
| Production output policy | Five native simulated-loopback checks passed, **zero** received bytes, sent commands and wire-output entries, no transport failures. No physical pilot qualification. |
| Production actual trust probes | **12 core Downloader cases** (3 accepted, 9 refused) and **9 private o-charts wxCurl cases** (3 accepted, 6 refused) passed; owned trust cleanup verified. All 21 process receipts have expected exit 0/1, no timeout, and console logger deletion/wx cleanup completion markers. The first valid Downloader completed in 0.1428917 seconds; wxCurl in 0.1312163 seconds. This is the completed production trust matrix, distinct from the skipped later public-Downloader workflow step. |
| Native AIS | Actual logs: **178 session checks**, **18 TLS/transport PASS rows**, **6 provider/server lifecycle pairs**. Exact candidate/run/attempt; all 15 retained runtime binaries and same-job receipt independently rehashed. Product, WFP and boat acceptance remain false in the original report. |
| Preview and recovery | Preview records 19 checks passed. Three distinct recovery reports each record five passed checks (15 executions of the same five-check scenario), including crash guard, Safe Mode and retry/persistence; screenshot review remains required. These are disposable fixture runs, not installer transactions. |
| DPI | Native GetDpiForWindow/wxDpi both 96, 120 and 144 at requested 100%, 125% and 150%; interactions passed and display restored to 100%. Original report explicitly requires native visual review. |
| Public ENC | Software rendering passed. Requested OpenGL was **rejected by the host**; the report verifies upstream software fallback and leaves hardware GL open. Disposable NOAA fixtures, adjacent-cell traversal and mode return do not qualify private charts or the boat GPU. |

Other retained functional reports preserve their original numeric/stale/screenshot caveats. This audit claims no screenshot acceptance and performs no new captures or tests. The native endurance policy step was skipped after the failure and no `soak/results.json` receipt exists in this archive; it must not be confused with the separate Linux skip receipt. Recovery-package and Setup hashes are unavailable because those steps did not run. Full Windows, package, installer, boat and release acceptance remain blocked.
