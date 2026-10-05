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
First focused attempt:
[37236808641](https://github.com/ThereptileII/Work/actions/runs/37236808641),
source `32f6840fec9c3c20d648f4993e6f6967c12e0ce4`. It stopped before AIS
compilation during the strict SDK reprobe. The only native-fact difference was
the parent interpreter path spelling (`pwsh.EXE` versus `pwsh.exe`); its hash,
size, versions, PATH hash and all compiler/tool records matched. The downloaded
failure artifact SHA-256 is
`af56b2849ee99cecacce55f209c19f70c29c4ec6e72c70f2df079315214b1496`.
The launcher correction selects the exact authenticated captured interpreter
path and checks its current bytes before execution. Strict comparisons and the
SDK producer stay unchanged. Documentation-only follow-ups do not retrigger
the focused workflow. All 11 launcher/runtime helper tests pass, including
refusal of changed interpreter bytes or changed captured facts.

Replacement native result: **PASS** at
`e4273b68d4024ed1d38a3fcc3f5550221135a11a`,
[run 37237278099, attempt 1](https://github.com/ThereptileII/Work/actions/runs/37237278099).
The provider implementation and concurrency fixtures are unchanged from the
Linux-tested increment. Native MSVC Win32 builds all three bounded clients;
8 provider TLS lifecycles, 18 adversarial transport scenarios, 178 session
checks and 11 helper tests pass. No dependency or full application rebuild.

Downloaded and reviewed artifact:
`ais-native-runtime-e4273b68d4024ed1d38a3fcc3f5550221135a11a-run37237278099-attempt1`,
ID `11315648301`, SHA-256
`e102566af392460ba5fa6fb4f4a37d94d980d9b7072101de0f0dab7ff04e35fd`.
The archive hash, report commit and retained compiled provider/test-client
sources match. Actual provider, transport and session logs were reviewed.
Dependency authority remains producer `37230581131` /
`1b25542aea3f9ab62c83c5d7a3ecdac9652f7d3e`, using OpenSSL 3.5.9 and zlib 1.3.2;
the runner retains exact native tool, import and runtime inventories.

Native application/boat acceptance remains required before SCRUM-301 can be
Done. The increment stays on `skager-ais-freeze`, ready for the next coherent
Staging candidate. The installed boat version and public release state are
unchanged.
