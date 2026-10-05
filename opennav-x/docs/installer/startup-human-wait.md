# Navigation warning during supervised startup

SCRUM-312 fixes the observed updater failure while OpenCPN waited for its
standard version-change navigation warning. This does not replace the warning
or consent on the user's behalf.

## Source boundary

Pinned OpenCPN `MyApp::OnInit` calls `ShowNavWarning` before its deferred
initialization timer and `opennav::Attach`. The narrow XNav patch reports entry
and the actual returned decision at this call site. Original warning text,
controls, version condition, Cancel return and profile persistence are unchanged.
Legacy/Safe retain the original path without phase notifications.

## Authenticated phases

The current-user-only local pipe binds expected live PID, creation time, image
path/hash, generation, compiled commit and random challenge. Every phase carries
that identity. It accepts historical READY plus EOF, or exactly WAIT → CONTINUE
→ READY plus EOF. WAIT → CANCEL cannot become healthy. At most three records of
at most 256 bytes are accepted. Repeated, unknown, out-of-order, truncated and
trailing data fail closed.

Receiver-owned monotonic deadlines are 90 seconds to initial startup or the first
authenticated WAIT; 300 seconds for the human decision; then a fresh 90 seconds
after CONTINUE for initialization/health. Repeated messages never extend these
bounds. The launcher allows 11 minutes to cover all phases plus existing
10-second graceful close and 120-second recovery bounds. It never force-kills a
live transaction.

Both modal edges clear readiness and the recovery-checkpoint flag. Nested modal
timer events cannot acknowledge readiness. Agree alone is not success: a fresh
30-second continuously ready XNav shell plus its successful durable checkpoint
are still mandatory. Cancel permanently consumes the sender's request. One
bounded FIFO/worker preserves fast-Agree ordering, retains only owned bytes and
never carries a UI pointer across threads. Request environment is cleared before
plugins. Delivery failures are neither retried nor accepted.

Phase/failure diagnostics contain no challenge. A cancelled app that exits before
final verification can report a process-exit failure; it still cannot write a
known-good receipt. Timeout/cancellation retains existing conservative recovery:
close normally before rollback; preserve pending evidence when closure cannot be
established. Production code never auto-accepts or force-dismisses a warning.

## Qualification

Focused policy, receiver, native sender and exact pinned dialog fixtures exercise
this boundary without touching the boat. Accelerated fixture clocks prove state
transitions, not installed 30-second health. An integrated application/package
and native installed changed-version warning/accept/cancel/recovery exercise
remain required before deployment or issue closure. The boat remains on its
previously installed candidate.
