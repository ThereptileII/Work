# Local peer sharing unavailable in the public beta

SCRUM-212 chooses containment while authenticated peer identity, pairing and
credential migration remain unqualified. This does not repair upstream's trust
protocol or claim compatibility with an authenticated replacement.

The integrated pinned source has one non-configurable policy:
`ocpn::PeerSharingAvailable()` in `model/peer_policy.h` returns false.
It applies to XNav, Legacy and Safe modes. Pristine OpenCPN is unchanged.

Application startup gates the entire certificate-generation, REST-server start
and peer mDNS-advertisement block. No peer certificate or credential needs to be
read or rewritten. `RestServer::StartServer` remains callable by explicit
isolated upstream model tests; it has one audited production application caller,
which is gated. This preserves those baseline tests without a runtime override.

Outgoing chart menus omit Send-to-Peer. Route Manager keeps its three peer
buttons disabled with an explanation, including after selection refresh.
Handlers reject forced events before cache validation/discovery. The peer
dialog disables Send/Rescan, explains unavailability, and rejects scan/send/timer
events. Model `CheckNavObjects` and `SendNavobjects` return false even for empty
input, before credentials or object serialization. `GetApiVersion` clears a
retained version to 0.0 instead of manufacturing a legacy compatibility result.
The externally linked `CheckKey` also rejects before constructing a credential
URL. The shipped `opencpn-cmd` generate/store-key commands exit unsuccessfully
with the unavailable explanation before reading or changing peer credentials.

Normal chart navigation, local route/track/waypoint storage, file export/import
and Send-to-GPS are unchanged. Existing stored peer credentials and identity
files are preserved for a future reviewed migration. Older peers cannot discover
or transfer objects to this integrated application. Remote peer REST operations
are also unavailable.

Actual model tests exercise empty and forced nonempty transfers and retained
version rejection. Integrated mode-cycle checks must prove credential/file
preservation and no process-owned peer TCP listener after startup/restart.
The disposable monitor inspects both IPv4 and IPv6 TCP listener ownership
without connecting to a peer. It fails on missing process/table evidence, and
retains fake saved-key and certificate sentinels across the existing mode cycle.
Its focused tests include actual loopback socket ownership, another process,
terminated process rejection and profile tampering. These tests do not exercise
or qualify a replacement peer authentication protocol.

Native Windows UI, complete regression and installer/boat qualification remain
required. A future authentication implementation needs separate design review;
there is no first-use trust, automatic re-pinning or insecure fallback here.
