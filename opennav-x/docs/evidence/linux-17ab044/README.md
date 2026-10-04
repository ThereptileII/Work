# Linux 17ab044 retained-result audit

Independent read-only audit of the existing Linux artifact for run `37164360050`,
candidate `17ab044a1e5222dc71791ac8118454219efe8734` (frozen local `0a52a6c`).
Original artifact `11292744507` is **20,065,317 bytes**, SHA256
`1532200addc68a47b618aeae777e09ad5ad8743c6da09487026f2a55feeae351`.
This audit rehashed those bytes; root previously checked all 4,259 ZIP CRCs.
The original remains under `.local/linux-integration-17ab044/original.zip`.

| Evidence | Independently checked result |
| --- | --- |
| Fixture-enabled XML | 147 unique test cases, all `run`; zero failures, errors, disabled or skipped cases. |
| Production XML | 147 unique test cases, all `run`; zero failures, errors, disabled or skipped cases. |
| Actual Linux endurance | Passed; requested 10,800 seconds, reported elapsed **10,800.118381349**; 1,080 samples and 540 actions, 90 visits per page slot. |
| Exact source/build | Harness and running build report the exact remote candidate; binary/harness match is true. Reported harness SHA256 matches frozen local `0a52a6c` byte-for-byte. Executable SHA256 remains a runner observation, not an independent executable-byte rehash. |
| UI continuity | Sample elapsed times and UI ticks strictly increase; ticks 25→43,958. Maximum page observation 1.253390109 seconds, below the unchanged 8-second limit. |
| Dropout/recovery | 45 stale and 45 unavailable scenarios, 90 recoveries; 180 samples suppress energy availability. AIS context appears in 990 samples. Day/Dusk/Night each appear in 360 samples. |
| Route progress | Reconstructed sample progression, respecting slot-0/5 resets, gives exactly 765 decreases, matching the report. |
| Resource stability | Recomputed median deltas after 300-second warm-up, using 210 samples at each end of 1,050 stable samples: resident memory −223,232 bytes, handles 0, threads 0. All are below the original limits. Recomputed average CPU is 3.353056139% of one core, matching the report. |
| Exit/profile | Original application log records clean frame/application exit at 03:42:56 UTC. The hash-matched harness requires exit code zero and unchanged navigation/profile snapshot before setting `passed`. No separate before/after profile receipt is present; these are harness assertions, not independently reconstructed profile hashes. |

## Chart observation limits

The 270 retained chart observations contain 90 zoom-in, 90 zoom-out and 90
palette inputs. Zoom-in shows scale ×2 in 89 observations and unchanged scale in
one (zero-based index 204), accompanied by a changed chart centre. All 90 zoom-out
observations show ×0.5. All 90 palette observations show the intended next light
mode. Therefore this is **not proof that every opposite zoom pair restored the
viewport**. The unchanged zoom's selected and fresh advertised control bounds
agree; its cause is not established here.

Only 150 inputs have a recorded activation trace; the existing trace is capped at
2,048 records. Later absence is not proof of failed activation, and fresh ticks
alone are not acknowledgement. The soak uses software chart rendering.
Root separately reviewed start/end and returned software/GL images with coastlines
present. The GL image uses llvmpipe, not the boat GPU. Native Windows and boat
typography, symbols, rendering and final acceptance remain pending.

`audit.json` records derived values and exact member identities.
`selected-original-reports.zip` is a **new documentary subset**, containing six
unchanged files: both XML reports, soak results/metrics, and launch/application
logs. It contains no charts, configuration profiles or private data files.
No new tests, builds, downloads, screenshots, CI polling or boat actions occurred.
No issue transition or commit was made by this audit.
