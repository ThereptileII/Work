# SCRUM-301: focused Online AIS send-lock repair

Scope: one provider concurrency defect. No chart/UI redesign, real credentials,
physical hardware commands or application packaging.

## Finding and implementation

The original provider held its state mutex across `IXWebSocket::sendText`.
OpenCPN 5.12.4, pinned at `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`,
contains a synchronous write-failure Close path through
`IXWebSocketTransport::sendOnSocket` / `setReadyState` and the WebSocket Close
callback. The provider callback reacquires the same mutex. Normal UI provider
reads then wait behind it. This is a reachable deadlock, not proof that this
exact write failure caused the user's boat report.

Reserve the current subscription under lock; release the lock for the actual
send. The reservation stays unconfirmed until the service reply. A chart pan
during send remains pending. Ignore a failed old-generation completion after
shutdown, disable/re-enable or credential replacement. Socket ownership and
shutdown/join stay with the worker. No upstream patches change.

## Focused Linux results

- Actual pinned/reviewed IX client build: PASS, GCC 16.2.1, OpenSSL 3.6.4.
- Existing six provider TLS scenarios: PASS (including IPv6, credential changes,
  reconnect and delayed Open callbacks).
- New `send-viewport`: PASS. A synchronous send callback reenters Read; while
  send completion is paused, service confirmation/report and a UI-side chart
  pan proceed. The server verifies the exact subsequent viewport payload.
- New `send-disable-enable`: PASS. Both user intents/readbacks proceed during
  the held send; the superseded connection closes and the new one subscribes.
- Negative control: the original provider from local parent `355ec0c` compiled
  against the identical test/IX objects hits the expected 8-second timeout
  (8.3 seconds wall time). The working source was not replaced for this check.
- Existing AIS runtime-gate helper tests: 9/9 PASS.
- Python syntax/YAML structure/whitespace checks: PASS. `actionlint` unavailable.

The initial local attempt discovered a stale generated integration tree lacking
the existing connection-observation hook. It was preserved separately and
recreated from the exact pin plus all reviewed patches; source verification
then passed. No production source change was made to bypass that mismatch.

Local detailed logs are in `evidence/local/scrum301/`. Tests use loopback TLS and
inert test credentials only. The tracker proves reentrant-send responsiveness;
it does not directly inject the stale failed-send return branch.

## Native/acceptance boundary

The new `SKAGER native AIS loopback` workflow runs the existing three-client
MSVC Win32 harness and all eight provider scenarios against the authenticated
immutable SDK. It never builds the complete application or promotes a release.
Native evidence will be linked after the run. Native application/boat acceptance
remains required before SCRUM-301 can be Done. The installed boat version and
public release state are unchanged.
