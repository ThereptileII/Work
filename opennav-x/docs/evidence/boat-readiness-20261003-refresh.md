# Boat readiness refresh — 3 October 2026, 02:18 UTC

Authorized read-only inspection through `ssh boat` ran from 02:16:46 to
02:18:39 UTC. [Sanitized measurements](boat-readiness-20261003-refresh.json)
match the [preceding inventory](boat-readiness-20261003.md). This is preparation
under SCRUM-17 / SCRUM-215, not candidate or launch acceptance.

SSH, Tailscale and RustDesk remain Running with Automatic startup. One active
interactive session has an Explorer owner matching the SSH account. No
OpenCPN/chart helper, commissioning helper, product scheduled task or active
commissioning marker was found. Intel UHD Graphics reports 1920×1080 with driver
32.0.101.7088; stored AppliedDPI remains 144 (150%). Actual window DPI, GL rendering
and physical touch were not measured.

The stock OpenCPN 5.12.4 executable remains SHA-256
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`.
The normal INI remains 21,492 bytes, SHA-256
`d891d88c62657139e1b1c6ff7d9e8acdae726a4e116dbc844126992a39adbdb6`.
Completed cold-baseline record `543f63c…`, its capture/review/predecessor and saved
INI identities verify. All 2,031 recorded live and 2,031 saved profile entries
match, and the live tree still contains exactly 2,031 entries. The retained
recovery set's 2,123 application and 2,008 profile entries also match. No recorded
entry mismatch or redirect was found. The cold record grants no launch permission.

Installation state, all four generation executable/ownership hashes, eight
historical Start Menu links, four accepted old release files, and the retained
portable manifest/marker/executable remain unchanged. Current installed commit
is still `a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a`. Qualified boat tooling remains
at clean tracked revision `20765cf374da5a1ab47dff0b3b427cd24f595455`; it was not
updated. Full payload, ACL, output/plugin review and external-shortcut ownership
checks were not rerun.

Existing inspection scripts ran in memory; none was installed remotely. The
read-only Git query disabled optional locks. No application was launched or
closed, no profile/configuration or remote-access setting changed, no package
was transferred, and nothing was retired. Personal paths, content and credentials
remain private. Required hash comparisons read boat-local bytes only.

The parent-reported candidate `61a0a7838b56ad841bb458af6fc62651464bdafe`, run
37088759582, was still unqualified when this inspection was requested. These
measurements do not qualify it. Exact artifact gates, guarded update, fresh
installed-build/profile/plugin/output audit, input-only commissioning and actual
SKAGER/Legacy/Safe chart/recovery verification still precede replacement acceptance
and retirement of identified old launch surfaces.
