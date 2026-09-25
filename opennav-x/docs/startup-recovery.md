# XNav startup recovery

Pinned OpenCPN already writes `startcheck.dat` and offers its Safe Restart dialog
if the previous run did not shut down cleanly. It defaults to a normal restart
after its countdown. Alpha retains that behavior and adds a bounded XNav-specific
startup guard; it never resets navigation data to recover the interface.

## Lifecycle

One early hook in `ocpn_app.cpp` runs after the normal single-instance check and
before `safe_mode::check_last_start()`, platform/plugin/OpenGL initialization and
configuration loading. It reads the profile's `opennav-startup.state`. A previous
pending XNav startup increments a saturating failure count once. Two unfinished
XNav startups select OpenCPN Safe Mode before dangerous modules can initialize.
An explicitly requested `--legacy` remains an escape path. `--safe-mode` always
wins. An unknown/corrupt/unreadable guard record also fails closed to recovery.

After normal mode selection, XNav records a pending startup before creating its
shell/services. Failure to persist this record selects Safe Mode. Thirty seconds
of normal application processing after upstream deferred initialization marks
startup healthy. A clean XNav close also clears the guard; this prevents a short
intentional session from being counted as a crash. Later runtime crashes still
receive OpenCPN's existing Safe Restart offer.

Safe Mode displays a concise recovery notice, retains logs and does not create
OpenNav Vessel Data, SmartNav or hardware services. It does not persist Safe as
the user's interface preference. A human choosing **Switch to XNav** explicitly
resets a blocked guard, saving the previous record as `.retry-*` evidence before
starting another process. Automatic relaunch never bypasses the guard. An
unwritable/unsafe record cannot be bypassed by pressing retry.

The small versioned record is validated before use and atomically replaced with
wxTempFile. Existing navigation/configuration/plugin files are not rewritten by
the guard. A record symlink or oversized/non-regular file is refused. No raw
sensor stream is added to logs. Normal installed XNav also creates a dedicated
`opennav-logs` folder inside OpenCPN's existing private-data directory; portable
and explicit test profiles retain their established diagnostic paths.

## Tests and limits

Portable tests cover first/repeated failures, saturation, one-time accounting,
healthy/reset records and malformed schema. Real wxFileConfig-adjacent storage
tests cover crash counting, atomic record replacement, 30-second/deferred
requirements, retry evidence and corruption without modifying navigation data.

`tools/smoke-recovery.py` creates a disconnected marked profile, starts and kills
only its two owned XNav processes before healthy startup, observes automatic
Safe Mode on the third launch, then uses the real menu to retry XNav. It verifies
actual database integrity and unchanged synthetic route/track/waypoint,
connection, AIS alarm and Dashboard preferences after every phase. Safe and
returned XNav chart captures must retain land and water; water-only is a failure.
The test is included in Linux and native Windows gates.

This is startup crash-loop protection, not a promise of crash-free operation,
transactional recovery for arbitrary upstream defects, or an at-sea approval.
Failures before OpenCPN's platform/single-instance initialization cannot be
handled by an in-process guard; the standalone Safe/Legacy launchers and
installer repair remain necessary recovery paths.

An intermittent native Safe-to-XNav return timeout at `54029a3` is retained in
[failure evidence](evidence/windows-recovery-54029a3-failure.json). The guard
record reset and coastline checks passed, but process exit did not; the precise
cause was not established from that run. Mode-request/close logging and failed
window captures now improve diagnosis. Final native qualification includes three
separate actual crash/recovery cycles; no automatic test retry hides a failure.

The repeated native failure at `c26e456` now has reviewed failure images: the
initial recovery notice remains open while the old harness posts a menu command
to the disabled Safe parent. This identifies an invalid test interaction, not
a proven product close failure. The replacement clicks the actual visible
button once, requires the modal to disappear, and rejects menu commands to
a disabled parent. All three independent native cycles must still pass.
[Failure review](evidence/windows-recovery-c26e456-modal-failure.json).

The stricter actual-click gate also fails at `bc892a7`. The notice was scheduled
from Attach while upstream deferred startup was still moving focus, raising the
frame and finalizing canvases. It now waits for a narrow end-of-OnInitTimer hook
and queues once after that work returns. Safe selection still happens early;
no plugin/control services are enabled to display the notice. The harness also
asserts that the notice log follows canvas finalization. Native replacement
acceptance is pending. [Ordering evidence](evidence/windows-recovery-bc892a7-startup-order.json).
