# Online AIS implementation and security

AISStream is supplemental traffic information; OpenCPN onboard AIS remains
the navigation authority. Provider snapshots own their values and retain
provenance. No socket, decoder or credential object crosses into Vessel Data.

The onboard target summary uses the retained upstream report observation epoch,
matching the copied position fields. The enclosing container's copy timestamp
is not report freshness. The integrated actual-model scenario verifies 64
repeated reads and a 70-second-old report without refreshing either timestamp.
The prototype target drawer opens directly, scrolls to Show on chart, and closes
only after a successful current-position action. Separate local and supplemental
chart actions retain their existing ownership boundaries.
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
uninstall preserves the entry for reinstall. Explicit Remove key deletes it.

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
The fixture-free XNav product now constructs the client only behind the XNav
startup gate. It copies the already-computed primary viewport on the normal
application tick; opening a panel does not update target observation times.
Legacy and Safe do not construct it. Replay/fixture operation stops it. Normal
close destroys the panel, then stops/joins the worker before releasing settings.
The native Traffic drawer presents the owned combined display, while SmartNav,
alarms and onboard receiver health continue to consume onboard-only state.
Online target positions are not inserted into OpenCPN's decoder or CPA model.

Traffic → Online AIS settings provides explicit OFF/Enabled and, on Windows,
masked key entry/removal. Only the enabled preference enters wxFileConfig;
the key goes directly to the bounded credential adapter. Runtime diagnostics
contain status enums, counters, confirmation and a presence flag, never secret
text. Update/repair leave the per-user credential entry intact. Uninstall
preserves it for reinstall; Remove key explicitly deletes it and stops online
traffic. Linux development remains environment-only.

Each user supplies **their own AISStream API key** through **Traffic → Online AIS
settings → Set AISStream key**. No shared service key is distributed. The input
is password-masked, touch-sized and always starts empty, including replacement.
Save writes only through the protected credential adapter; it does not enable
Online AIS. Cancel/Escape leave the existing credential untouched. Empty or
invalid input cannot be saved. A storage failure remains visible without
claiming success. Explicit, confirmed Remove key stops Online AIS and removes
the credential. The testing credential on the boat is local to that Windows
account and is absent from source, installers and portable packages.

The offline native component test now exercises this exact UI with an explicitly
fake storage callback: missing key, masked input, Cancel, invalid/queued Save,
successful Save without enablement, empty replacement input, failed storage,
Escape, cancelled removal and confirmed removal. It never loads a real key or
connects to the service. Native Windows credential adapter tests separately use
only a unique test namespace. The initial Linux UI run caught Escape navigating
behind a modal; drawers now defer to an open modal. The final corrected Linux run passes 151
checks with twelve captures; exact native Windows validation remains pending.

Chart overlay/hit testing is implemented below; full native product regressions
and live chart/boat UI gates remain open. On September 29 the user's authorized
key was placed in the interactive user's Credential Manager and the isolated
native probe confirmed a real subscription: 41 peak targets, 54 accepted reports,
zero rejected, then disabled/cleared. This is service-path evidence, not product
chart acceptance. See [live probe evidence](evidence/boat-ais-live-c86c6a7.json).

The application-thread settings boundary now has ten integrated tests with fake
credentials and a controllable provider. Construction/default settings do not
connect. Saved opt-in still requires a valid copied current viewport and live
(not replay/fixture) operation. Invalid areas cannot retain an old subscription.
OFF stops networking even if saving the preference fails; the user receives an
explicit persistence error. Credentials never enter wxFileConfig. Failed key
removal leaves networking disabled; all preference/credential mutations reject
worker-thread access. The existing marine/integration suite plus these tests
passes 120/120 on Linux in development; native replacement evidence is pending.

Pinned `ViewPort::SetBoxes` produces ordered LLBBox longitudes which can be
unwrapped across ±180°. The bridge normalization retains that span (including
the full-world case), then hands copied bounds to the existing antimeridian
subscription policy. Tests use the pinned LLBBox implementation and reject
invalid, nonfinite and overflow geometry. This is geographic filtering only;
no route, bearing, range or collision calculation is introduced.

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

Native Windows development run `36466500140` passes the fixture-free product
build and 112 integrated tests. The downloaded, digest-verified captures cover
Traffic and Online AIS settings, default OFF, missing-key withholding, disable,
all themes and Back/Close. See `evidence/prototype-native-3db9704.json`. Populated
target interaction, exact visual conformance and live boat acceptance are open.

### Owned chart overlay (development)

`ais/ChartTargets` derives bounded presentation marks from the aggregated owned
display state. It rejects ambiguous identity, mismatched coordinate timestamps,
invalid provenance/numbers and fixture data. A stale onboard identity still
suppresses the matching online mark. No online record enters OpenCPN's decoder,
local AIS health, CPA/TCPA calculations, alarms or SmartNav.

`integration/OnlineAisOverlay` retains only these owned marks. The normal shell
observation copies the upstream AIS model and online feed on the application
thread at most once a second (selection/disable can update immediately).
Only a changed mark set invalidates the canvas. Painting/hit testing rechecks
the original timestamp, so a stopped producer cannot leave retained marks live.
Both upstream renderer paths call the same `AISDraw` hook/`ocpnDC` implementation.
The existing canvas projects coordinates, including raster georeferencing.
Symbol orientation uses pinned OpenCPN `ll_gc_ll` for a short bearing solely
for display; it creates no navigation position, route distance or CPA.

The glyph path follows `.ais-ship` and its inherited palette. Missing valid
heading/motion uses an unoriented ring, never an invented northbound vessel.
Aging is dashed, stale is crossed, lost is double-crossed and ten-minute-old
marks disappear. Online provenance uses a small dot; detail views identify
AISStream and distinguish service/receipt age from transponder observation.
There is no speculative course vector or risk classification.

Right-click/long-press reaches online selection only after upstream local AIS,
waypoint and route hit tests have declined the position, and outside route edit
or measure modes. Equal-distance ambiguous marks select neither. Selection
opens the native owned drawer; Show on chart revalidates current position.
Direct tap, populated rendering, label decluttering, exact fractional stroke,
real service traffic, software/GL and physical boat review remain required.

The native drawer distinguishes measured vessel motion from upstream estimated
CPA/TCPA/range/bearing. Fresh upstream estimates remain displayable and sortable;
each estimate's own freshness (including selected own-position dependency) is
honoured. A stale relative estimate cannot become current merely because target
position is still fresh. The card labels CPA/TCPA as OpenCPN estimates. Online
targets have no such values. Unavailable/fixture containers and duplicate target
identities cannot enable a retained chart action.

The non-installed `ais_drawer_test` process exercises real widgets against owned
cache/aggregator fixtures. It cannot access the OpenCPN profile, marine equipment,
network client or credential store. Its prominently labelled component captures
are not product, live-traffic, chart-symbol or boat-acceptance evidence.

## Authorized remote credential commissioning

`tools/boat/import-ais-desktop-credential.ps1` accepts only a bounded binary stdin
frame (four-byte little-endian length, 1–512 printable ASCII bytes, EOF). It
never accepts a key argument or writes a plaintext key file. SSH can have a
different credential logon context from the interactive desktop. A local named
pipe hands off to a temporary limited task in the one existing same-user
desktop, without altering remote access. Its ACL denies network logons and
admits only the current user/SYSTEM. The sender verifies the receiving process's
desktop session and executable; the receiver checks the exact live sender PID.
Only then does `CredWriteW` use the product's `OpenNavX/AISStream/v1` target with
per-user local-machine persistence. Readback must match. A different existing
key is preserved and the import refuses replacement. Managed/unmanaged transfer
buffers are cleared. No key hash or byte sequence enters evidence.

`-TestOnly` uses and removes an isolated random test credential, including native
same-value/different-value tests. Thirteen portable framing/parser checks and
seventeen native checks pass on the actual desktop; native CI retains its own
disposable test gate. Initial SSH and cross-logon inspection failures are kept
as negative evidence. `probe-online-ais-desktop.ps1` runs the existing pinned,
dependency-verified, internet-only probe in that credential context. It does
not launch OpenCPN, access its profile, or operate vessel hardware. Its 45-second
result contains aggregate counters only. Temporary tasks are removed.
