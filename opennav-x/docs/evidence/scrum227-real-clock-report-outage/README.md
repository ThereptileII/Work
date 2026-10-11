# SCRUM-227: real-clock report silence over responsive TLS

This opt-in fixture runs the actual `AisStreamProvider`, pinned patched IX TLS
transport, subscription codec and target cache against disposable loopback TLS.
It sends one positioned target and then no AIS reports for more than 120 actual
steady-clock seconds. Both sides continue WebSocket ping/pong traffic. The
provider must retain the same connection generation, endpoint tuple, capture
time and position receipt while its target becomes stale and then lost.

The server then closes the actual TCP connection. The provider must clear its
current observation, reconnect with bounded backoff, publish a new generation
and capture time, and resubscribe to the same viewport. Subscription confirmation
alone must leave the retained target lost. Only a fresh position report restores
active status and updates its receipt. The server independently compares both
observed endpoint tuples with its accepted sockets. Retained snapshots remain
unchanged after provider destruction.

The test client is `EXCLUDE_FROM_ALL`; this fixture is not registered with CTest
or added to ordinary native/runtime suites. To explicitly select a run after
building the focused target:

```sh
cmake --build <ais-transport-build> --target ais_outage_test_client
python3 tests/ais_transport/outage_tests.py --client <ais-transport-build>/ais_outage_test_client
```

`--ipv6` selects IPv6 instead of IPv4 and takes a separate real-time run. A
155-second subprocess watchdog, socket deadlines, provider deadline, and cleanup
paths bound failure. Only generated certificates and the fixed fake test key
are used. No external AIS service or real credentials are contacted.

This is a **report-silence and reconnect fixture**, not network packet-loss
qualification. It does not exercise WFP, longer dynamic-filter leases,
helper-death cleanup, real-product process/module binding, changing DNS,
nonzero IPv6 scope, remote-access continuity, onboard AIS continuity or the boat.
Those SCRUM-227/274 native/product/boat gates remain open. This helper revision
does not build or qualify a new application package.

Local results and exact input/binary identities are recorded in adjacent
`local-proof.json` and runtime logs. CMake is unavailable on this host; local
execution uses a retained direct compiler recipe derived from the focused
target and pinned IX CMake source list. It verifies prepared IX bytes against
the retained SCRUM-274 manifest, compiles actual source with OpenSSL/zlib, and
does not claim CMake configuration or native Win32 acceptance.

The 2026-10-11 local runs passed on both loopback families. IPv4 became stale at
61.009 seconds and lost at 121.035 seconds; IPv6 at 61.045 and 121.027 seconds.
Each server withheld reports for 121.24 seconds, answered four client pings and
received thirteen timely replies to its own pings. Reconnect completed after
2.207/2.407 seconds, with generations 2→4. Both runs confirmed the retained lost
target before the fresh report restored it, then passed provider destruction
and retained-copy checks. A separate immediate-client-exit negative control
refused acceptance and returned within its bounded invocation. The interrupted
attempt's missing-CMake diagnostic is retained separately from these passes.
