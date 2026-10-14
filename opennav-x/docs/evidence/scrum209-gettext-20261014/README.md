# Poedit prerequisite available again — 2026-10-14

[Original run 38581902769](https://github.com/ThereptileII/Work/actions/runs/38581902769),
attempt 1, source `edd04b37b4a257f0f4307f5e240342fe3529a0f7`,
Windows job `115913689356`, **success**. The existing standalone prerequisite
workflow was dispatched once after the earlier full build's HTTP 503 failure.
No application/dependency build or changed acquisition policy.

Original artifact `11904036367`, 10,888 bytes, SHA-256
`2de4973d6e2eb675ae98d0a1117c5712cb21e0af2667d7dbb81dffbc4a0ac1fb`.
Authenticated GitHub metadata and downloaded bytes agree; all 30 member CRCs pass.
The retained summary's four input hashes match the exact Git source after its
Windows CRLF checkout transformation. Direct comparison to Linux LF files first
refused, as it should; all four original lengths/hashes then matched with only
LF-to-CRLF conversion. `input-checkout-mapping.json` records both identities.
`summary.json` and `gettext.json` retain the original receipt bytes.

Twenty-five prerequisite contract tests pass. Poedit 3.9.1 installation from
the original Chocolatey provider succeeds on its first attempt. Both actual
gettext 0.26 tools retain the same identities before/after the UTF-8 translation
operation; msgmerge and msgfmt succeed and the resulting catalog returns the
expected Swedish translation. Whole job: 14:35:43–14:36:59 UTC (76 seconds).

This proves acquisition and translation worked on this disposable runner at this
time. It neither guarantees future provider availability nor qualifies an
application, installer, security change or boat installation. The next actual
application job still runs its same-job SDK and prerequisite checks.
