# Historical Linux integrated baseline, cf44e938

[Run 37057756273 / job 111006548201](https://github.com/ThereptileII/Work/actions/runs/37057756273/job/111006548201)
completed successfully for exact remote source
`cf44e938a3539af6e97a03346542f04f3e7895a4`. The downloaded artifact
11257297491 is 19,492,650 bytes; its SHA-256 matches GitHub metadata and the
job upload log, and all 4,248 ZIP entries pass CRC verification. Exact hashes
and selected original log lines are retained beside this note.

The endurance result passes 10,800.197237 seconds against 10,800 requested
(three actual elapsed hours): 1,080 monotonically ordered samples, 540 UI
actions, 270 viewport observations, 45 stale and 45 unavailable dropouts,
and 90 recoveries. It exercised the actual application with **DEMO**
vessel/route/AIS inputs and software chart rendering, without hardware
commands. This is Linux development evidence, not physical navigation testing.
The artifact reports 765 route-progress samples, 2.124% CPU on one core,
resident-memory growth -884,736 bytes, and zero handle/thread growth under
its reported growth calculation.

Both the integrated development and fixture-free production test reports
contain exactly 147 executed cases, zero failures and zero skips. Original
logs report 12.65 and 12.08 seconds respectively. The production build/loader
step also completed successfully. The binary hash in the endurance report
is harness-recorded; no executable was supplied in this evidence artifact
for an independent binary rehash.

At 2026-10-02 23:43:52 UTC, newer run 37063131823 Linux job 111024324351 was
still naturally running its endurance step; production build and artifact
upload were pending. No job was restarted or cancelled, and no tests were
run during this verification.

This result belongs only to the historical commit above. It does **not**
qualify the current SKAGER branding/chart changes, make the complete older
workflow successful (separate contract/native failures remain historical
evidence), or close SCRUM-224's exact-source native Windows/installer/boat
requirements. SCRUM-224 remains open.
