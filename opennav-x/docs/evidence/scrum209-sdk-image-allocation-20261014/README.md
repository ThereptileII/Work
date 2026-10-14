# Retained SDK refused on an older hosted image

Frozen application source: `89854110b41286cab0c6bf9748f37882a595a366`.
[Staging run 38638855800, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38638855800).

The native updater job `116109494705` succeeded. Dependent application job
`116117298706` ran from 21:34:47Z to 21:35:20Z and failed before native setup or
application compilation. The original job log identifies hosted Windows image
`20261004.326.1`; the authenticated retained SDK requires `20261012.333.1`.
The immutable-input preflight correctly refused this mismatch. This is an
environment allocation failure, not evidence of a program crash or test failure.

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
