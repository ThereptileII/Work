# Online AIS implementation and security

AISStream is supplemental traffic information; OpenCPN onboard AIS remains
the navigation authority. Provider snapshots own their values and retain
provenance. No socket, decoder or credential object crosses into Vessel Data.
# Supplemental Online AIS — contract and implementation status

Online AIS is an optional Internet input for chart/list presentation. It never
becomes vessel position, an autopilot input or a source of invented CPA/TCPA or
alarms. OpenCPN local AIS decoding and calculations stay intact.

## Owned provider and aggregation boundary

`ais/IAisProvider` returns an owned `ProviderSnapshot`: targets plus transport
health. `AisFeeds` retains **onboard**, **online**, and **display** separately.
Receiver health and upstream safety calculations consume onboard, never infer
receiver connectivity from the combined display list. No socket or OpenCPN
target pointer crosses this interface.

For duplicate MMSI retain the complete onboard target, including its position
timestamp, stale/lost state, range/CPA and alarm. A newer online report cannot
hide an onboard dropout. Online-only display becomes eligible after OpenCPN
removes that MMSI. Optional static enrichment is absent in this first slice.
Ambiguous online identities are withheld; ambiguous local identities remain
duplicates so the existing selection contract rejects them. Display is bounded
to 2,000 targets.

Online provenance is `AISSTREAM_ONLINE` and typed `AisOrigin`. Supplemental
targets have no upstream alarm or range/bearing/CPA/TCPA even if an untrusted
provider tries to supply them. Missing fields remain empty.

## Cache and time

Position and static reports have independent monotonic observation clocks.
Static updates never refresh position. Out-of-order/duplicate positions and
future/missing timestamps are rejected. Reading does not refresh observations;
owned snapshots survive cache edits/removal. Online position thresholds:

| Age | State |
|---|---|
| <15 s | LIVE |
| 15–60 s | AGING |
| 60–120 s | STALE |
| 120–600 s | LOST, not current/selectable navigation |
| ≥600 s | Expired, removed from display |

Age denotes local receipt unless a validated service `MetaData.time_utc`
timestamp is available. `AisTimeBasis` distinguishes OpenCPN report, online
service and online receipt clocks. It does not establish shore/satellite latency or guarantee current
physical position. Details must identify Internet provenance and this uncertainty.
Cached metadata coordinates cannot become a new position report. Cache capacity
is 2,000; expired entries are reclaimed before allocation, otherwise new
identities are refused. Numeric bounds, AIS sentinels, strings, extreme clocks
and retained-copy behavior have deterministic coverage.

## Inspected transport boundaries — 2026-09-28

[Official documentation](https://aisstream.io/documentation) specifies
`wss://stream.aisstream.io/v0/stream`, a complete subscription within three
seconds, replacement subscriptions with confirmation, binary UTF-8 JSON frames,
and at most one subscription update per second. Use focused geographic filters
and bounded exponential reconnect with jitter. Confirmation establishes a
subscription, not receipt of a vessel report.

Pinned OpenCPN's Signal K IXWebSocket client disables hostname/certificate
validation. Do not inherit those local-network settings for AISStream. The
vendored transport also accepts frames up to `1ULL << 63`; rejecting JSON only
after receipt would not bound transport allocation. The Internet transport needs
verified TLS, bounded frames/decompression, cancellation and generic errors.

Useful reports: Class A position, standard/extended Class B, ship static/voyage,
Class B static. Envelope/payload identities must agree. Strict UTF-8 JSON,
frame/string/depth bounds and nonfinite/sentinel rejection are mandatory.
Confirmation is a separate envelope. CI uses sanitized fixtures without a key.

## Credentials and outstanding integration

Development uses `AISSTREAM_API_KEY`; installed Windows uses per-user Credential
Manager, separate from plaintext settings/backups. Never export keys,
subscriptions, server echoes or raw errors. Update/repair preserve credentials;
explicit uninstall removal policy is still to be implemented.

Implemented: owned models/cache, age/validation, onboard-precedence aggregator,
antimeridian viewport filtering with 15% margin, coalesced five-second
subscription changes with a single confirmation in flight, bounded exponential
reconnect/jitter and cooldown policy; 2,074 deterministic checks.
The bounded codec has 100 additional deterministic checks. The earlier
foundation passed three native Win32 MSVC tests and three Linux tests at
`087318c`, with verified downloaded JUnit artifacts; the newer codec native
codec, session and protected credential gates pass at `d92e187`; six portable
tests pass on each platform and the native integrated suite passes 102 tests.
The buffered TLS receive correction now passes all 18 adversarial transport
scenarios and the actual provider lifecycle on Linux and native Windows at
`ea869a9`. See [downloaded replacement evidence](evidence/prototype-native-ea869a9.json).
Pending: settings, chart/card wiring,
full native product regressions and live boat gates. **The product does not yet connect to AISStream.**
No configured key, live target count or network acceptance is claimed.

The current [service documentation](https://aisstream.io/documentation) was
rechecked on September 28: binary UTF-8 JSON, prompt full subscription,
compression confirmation and replacement-subscription acknowledgement agree
with the tested client boundary. Desktop native code does not expose a browser
connection or embed the key in HTML.

## JSON decoder and subscription serialization

The pure codec uses the same RapidJSON 1.1.0 dependency as pinned OpenCPN.
Integrated builds reuse `ocpn::rapidjson`; standalone contracts verify the
upstream archive with SHA-256
`bf7ced29704a1e696fbccf2a2b4ea068e7774fa37f6d7dd4039d0787f8bed98e`.
The decoder bounds JSON to 64 KiB, nesting to 16, objects to 64 members and
arrays to 256 entries. Duplicate keys, invalid UTF-8, nonfinite numbers,
identity mismatch, invalid coordinates and oversized/control-character text
are refused. It does not parse or expose arbitrary server error text.

Class A, standard/extended Class B, voyage/static and both Class B static
parts are normalized. AIS unavailable sentinels remain empty. Zero/partial
hull dimensions remain unavailable. Only typed position coordinates are used;
cached metadata coordinates cannot refresh position. Service UTC timestamps
accept strict ISO UTC and Go's UTC representation; stale/future/malformed
timestamps are refused. Missing service time is explicitly receipt-based.

Complete subscription serialization includes required key/boxes and five useful
message types, plus optional unique nine-digit MMSIs (maximum 200). The output
contains a credential and must be sent/erased without logging. This codec is
not a substitute for bounding the transport before allocation/decompression.

## Credential boundary (implementation under qualification)

The Windows adapter uses a per-user Generic Credential Manager entry named
`OpenNavX/AISStream/v1`, persisted on this computer only. Readback verifies writes;
removal verifies absence. Credential errors are fixed enum states, never Windows
or server text containing request payloads. A move-only bounded secret clears its
owned storage when moved, replaced or destroyed. Linux development reads only
`AISSTREAM_API_KEY`; it does not save a plaintext fallback.

Update/repair must leave this per-user entry untouched. Uninstall preserves it
for reinstallation; an explicit Remove Key action will be the user-facing removal
path. That settings action and the live transport are still pending: the current
credential adapter is not yet reachable from the product UI. Diagnostic exports
must not enumerate credentials, environment variables or subscription payloads.
Do not collect raw process memory in ordinary diagnostic bundles.

Native automated credential tests address only a unique `OpenNavX/Tests/AISStream/`
entry, verify absence before creating it, and remove their own entry. The test
factory is excluded from the production library. No real service key is needed.

## Runtime session and transport

`AisStreamSession` is an independently tested state machine. A valid viewport,
explicit enable and readable credential are prerequisites. A socket opening is
not reported as Connected: a complete subscription must be sent and confirmed.
One replacement may be in flight; panning is coalesced at five seconds. Failure,
missing confirmation or stalled connection enters bounded exponential backoff
with jitter and a cooldown. Reconnection always sends the complete latest area.
A successful confirmation resets failure backoff. Static reports and reads never
refresh a position. Disable removes online targets; network loss retains their
original observations so they age out. Old callbacks cannot revive a disabled
or superseded connection.

`AisStreamProvider` owns a controller thread and the bundled socket's receive
thread. Its public methods copy values under a mutex; no window pointer or
OpenCPN model pointer enters either worker. Socket stop/join occurs outside the
state mutex. The production endpoint is fixed WSS, certificates and hostname
are verified, compression is negotiated, redirects/downgrades are refused and
wire/inflated buffers are bounded before JSON parsing. Library peer errors are
reduced to state changes, never logged verbatim. The small subscription buffer
is erased after sending. Optional MMSI filters exist at the codec boundary;
normal viewport operation has no MMSI filter.

CI uses a separate loopback-only factory compiled exclusively into a test
executable. It verifies the actual TLS/provider/subscription/parser/cache path,
initial subscription within three seconds, five-second viewport replacement,
connection loss, bounded reconnect, full resubscription, retained snapshot
lifetime and explicit disable. It never connects to AISStream or reads a real
credential. Deterministic session tests separately exercise absent viewport/key,
unconfirmed reports, malformed messages, stale/lost/expired positions,
out-of-order callbacks, acknowledgement timeouts and repeated failures.

Product settings, chart rendering, compact target/detail views and live boat
acceptance remain pending; compiled provider tests alone do not establish those
features as complete.
