# Retained SDK refused on an older hosted image

Frozen application source: `89854110b41286cab0c6bf9748f37882a595a366`.
[Staging run 38638855800, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38638855800).

The native updater job `116109494705` succeeded. Dependent application job
`116117298706` ran from 21:34:47Z to 21:35:20Z and failed before native setup or
application compilation. The original job log identifies hosted Windows image
`20261004.326.1`; the authenticated retained SDK requires `20261012.333.1`.
The immutable-input preflight correctly refused this mismatch. This is an
environment allocation failure, not evidence of a program crash or test failure.

The successful updater job's original test stream records **1,060 top-level Go
passes and 14 skips** (4,697 pass events including subtests), plus **133 Python
packaging checks** across nine suites. Native armed-barrier, actual-Lifecycle
fixture and explicit fresh SCM enrollment checks also pass. These are component
results; real certificate/TSA signing and the complete installed customer journey
are not established by them.

Its protected operator kit has consistent GitHub API/upload-log metadata:
artifact `11930226852`, 160,913,325 outer bytes, SHA-256
`7bdcff553c1f2e6eb3b26304ae5734414c4c6461aac95b46a8be0ac6c3a8b4e7`.
The producer records the inner ZIP as 160,886,913 bytes with SHA-256
`474d0faf4b66a6eff3e3a0dd16ef7f564363327ca51e47c4622291346f91dc72`.
The archive was subsequently downloaded once and authenticated: outer/inner CRC,
exact nine-payload inventory, all hashes/bounds, 30,862 corresponding-source
members and embedded PE32/Go build identities pass. See the
[sanitized receipt](operator-kit-receipt.json). Its paired application launcher
prerequisite remains unverified; these component bytes alone do not qualify B0.

The original log SHA-256 is
`134a6ddf4a1019ca8d0f569f38443778a6eac5b67369d08fe0a06d18287247a9`.
Original job metadata and annotations were retained with it. The missing local
evidence upload is secondary to the early stop. Plugin input capture was skipped;
there is no application or plugin-capture artifact to qualify from this job.

The selected continuation is a failed-Windows-job-only rerun of the same frozen
candidate after the ongoing Linux job finishes. Retain the failed attempt and
authenticate the actual producing attempt for every later artifact. Do not
rebuild the SDK, relax the image/toolchain check, cancel successful independent
work, or create a new candidate for this allocation mismatch. A scheduled rerun
is not evidence that it obtained the required image or passed its gates.
