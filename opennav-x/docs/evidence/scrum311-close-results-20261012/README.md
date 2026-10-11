# SCRUM-311 broker cleanup results — 2026-10-12

Source commit: `4fc0704ea8c912fade6849de6af85c16c1dac118`.

Two defects were reproduced against the preceding implementation:

* A repeated candidate close returned `nil`, losing its original file/backend
  cleanup failures. Subsequent store shutdown also lost those child failures.
* A second store close returned `nil` while the first close was still blocked
  inside backend cleanup.

Both new tests failed before the production correction. Their observed failure
messages were `repeated candidate close changed original result: <nil>` and
`second close returned before cleanup finished: <nil>`. These are transcribed
tool observations, not separately captured raw log files.

The store now serializes its complete close operation separately from ordinary
operations, marks itself unavailable before draining children and retains the
original joined cleanup result. The candidate retains its result under its
existing mutex. No native cleanup operation is retried. The existing reservation
test now explicitly requires the retained error rather than an erroneous clean
result on a repeated close; its other fault and ID-reuse assertions remain.

The following **nine top-level tests**, including **thirteen fault subcases**, pass
with pinned Go 1.27.1, `-race -count=1`, on Linux:

* `TestCloseRetainsCleanupFailuresWithoutRetry`
* `TestConcurrentStoreCloseWaitsForOriginalCleanup`
* `TestPersistentReservationAndClose`
* `TestStoreAPIValidation`
* `TestCancellationDuringHeldRehash`
* `TestCancellationDuringStreaming`
* `TestStagingOperationOrdersCleanupBeforeReceipt`
* `TestStagingOperationFaultsWithholdSuccess`
* `TestStagingOperationRejectsConcurrentAndCancelledWork`

The changed production/test bytes were tested before committing; only source
comments changed afterward. The final source identities are in
`source-receipt.json`. This receipt records observed tool results rather than
claiming a retained raw transcript. A bounded independent Codex review inspected
lock ordering, error publication and cleanup attempts and found no concrete
defect. The requested expanded Claude review has **not run**: automatic approval
rejected private-source transfer and exact-scope consent remains pending.

This does not establish crash/orphan reconciliation, native service behavior,
installation custody, installed Update Now or release acceptance. No application
build, CI dispatch, public push, boat change, service activation or physical
command occurred. SCRUM-311 remains In Progress.
