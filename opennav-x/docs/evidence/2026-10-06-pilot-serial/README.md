# SCRUM-313 guarded serial commissioning software

This follow-up preserves the [earlier serial bounds evidence](../2026-10-05-pilot-integration/README.md)
and the [historical boat observation](../../pilot-boat-integration.md).
No boat, serial port, CAN connection, profile or plugin was operated here.

The software now declares `manual-commissioning` contract1. Saved permission and
an explicit current session are distinct; both start OFF for a new profile,
and the session never survives restart, reconnect, stale feedback or isolation
changes. The final sink accepts only the exact six manual commands through the
existing enabled bidirectional OpenCPN Actisense serial driver. An explicit
rate-limited identity request is possible before a NAME is known and cannot
turn on steering. No hard-coded destination or inferred acknowledgement is used.

[Machine-readable evidence](evidence.json) records code hashes and these checks:

- Fourteen focused portable contracts pass, including the pinned actual bridge
  parser, identity/feedback, command/timeout behavior, discovery, traffic,
  product/loopback output policy and settings. The changed live-session
  cancellation assertions also pass in the three applicable adapter tests.
- Two offline actual-source harnesses pass under ASan/UBSan: writer/serializer,
  receive handler, serial-write wrapper, state/queue/framer, and the actual
  OpenCPNPilot final command/session methods with the real ST4000 adapter.
  Fake ports/registry/listeners are explicit. No hardware was opened.
- Six complete changed Linux translation units compile with normal product
  defines and retained generated/dependency headers: serial worker,
  OpenCPNPilot, OpenCPNIntegration, PilotSettings, InstallerSelfTest and
  PreviewDiagnostics. This is neither an application link nor Windows evidence.

The serial harness verifies that a pending AUTO is atomically replaced by exact
STANDBY, disable removes unsent pilot output, reconnect purges output and changes
epoch, expired items never write, partial/throwing writes invalidate the
connection without retry, and worker receive provenance survives event delivery.
It also verifies that the real write wrapper no longer calls `flushOutput`,
which discards pending bytes in the pinned Windows and Linux serial library.

One new test initially used two `Clock::now()` calls in one function invocation.
The compiler evaluated the observation time after the comparison time; the
production future-time rejection correctly ignored that fixture. The test now
uses one captured time. This was a test-construction failure, not a relaxed
production check. An initial local runner lacked CMake on PATH; the retained
pinned tool environment was then used. Neither is counted as a passing check.

Required open gates: root review/integration of the exact guarded policy with
packaging, native MSVC harness and changed-component compilation, the coherent
Staging candidate, then actual observed identity, physical mode/heading, six
manual responses and failure/reconnect behavior on the authorized boat setup.
The full application event loop and actual serial hardware are not qualified by
these fake-port tests. The installed boat candidate has not acquired this change.

## Follow-up: exact write identity and native portability

Root review identified a race in the initial completion counter: an earlier
AUTO could complete after the counter snapshot but before STANDBY enqueue. The
follow-up replaces that counter comparison with an exact monotonic ticket
allocated inside the enqueue lock. Worker completion records that same ticket;
`OpenCPNPilot::GetState` requires equality with the currently awaited command.
Overflow disables the connection instead of wrapping. The regression executes
the actual `GetState` body with post-AUTO/pre-STANDBY feedback and proves it
cannot open STANDBY confirmation; a sample preceding STANDBY's own write also
remains older than the controller's time boundary.

The production state helper now handles Win32 `min`/`max` macro exposure; the
harness deliberately defines those function-like macros while including it.
Both actual-source tests pass under ASan/UBSan and the two affected complete
Linux translation units compile. [Follow-up hashes/results](ticket-followup.json)
identify this later revision separately from the original evidence above.
Native MSVC and actual hardware remain root-owned gates. The focused workflow
should explicitly include the pinned serial and NavMsg headers in its sparse
checkout; the two newly added helper headers are supplied by the patch itself.

## Follow-up: serial worker ownership

Review found an existing detached-thread lifetime defect: immediate close could
skip stop/wait before `Entry` set the active flag, or abandon a live worker after
ten seconds. The parent could then be freed while the worker still referenced
it. The worker is now joinable and close always stops and joins before deletion.
Thread creation/start errors are checked; a created but unstarted thread is
cancelled and joined. Both shared flags are atomic. Retry waits check stop every
50ms and ordinary serial reads/writes use250ms timeouts. Shutdown does not pump
GUI callbacks or forcibly terminate/abandon a thread. OS port open/close has no
application hard deadline, so a broken OS/device driver may still delay shutdown.

The cleanup sequence follows inspected wxWidgets3.2.6 implementations:
[Windows](https://raw.githubusercontent.com/wxWidgets/wxWidgets/v3.2.6/src/msw/thread.cpp)
and [POSIX](https://raw.githubusercontent.com/wxWidgets/wxWidgets/v3.2.6/src/unix/threadpsx.cpp).
In particular POSIX cancellation of a created thread still needs `Wait` to join.
The serial concrete class is internal; the public message/base classes and
plugin API header were not changed by this follow-up.

All three offline actual-source harnesses pass under ASan/UBSan. The new harness
extracts the actual `Open`/`Close` bodies and uses a deterministic wxThread double
to cover entry delayed until close, active/repeated close, creation failure and
start failure. This verifies ownership/call ordering, not the real wx scheduler.
The complete changed serial translation unit also compiles with retained Linux
product flags and dependencies. [Hashes/results](lifecycle-followup.json) record
this revision. Native MSVC, actual framework lifecycle and hardware checks remain
root-owned gates; no application build or hardware access occurred here.
