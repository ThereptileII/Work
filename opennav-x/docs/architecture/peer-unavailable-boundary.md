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

The Windows integration job runs the installed CLI refusal test immediately
after installation and dependency verification, before CTest or GUI smoke tests
(`build-pristine-windows.ps1 -Integration -VerifyPeerCli`). The pinned CLI sets
the wx app name to `opencpn`; wxMSW 3.2.8 resolves `GetConfigDir()` through
`CSIDL_COMMON_APPDATA`, so its config is `ProgramData/opencpn/opencpn.ini`.
Redirecting environment home paths or passing the GUI's `--configdir` does not
isolate that CLI config. In particular, the GUI's `InitializeLogFile()` creates
the normal `GetHomeDir()` even with a separate GUI profile. Running the CLI
check after the mode tests therefore collided with their directory in native
run 36984898997, job 110769523919. The reordered test keeps the original refusal
of any pre-existing common-data directory; it never deletes or adopts an
unknown profile. It still checks both commands, nonzero exit, exact refusal,
no key output, and byte-for-byte config preservation.
After successful execution, `peer-cli-receipt.py` writes a positive receipt
bound to the committed harness/build sources, exact CLI hash, candidate commit,
run and attempt. The original named workflow prerequisite now verifies that
receipt without rerunning against the GUI-created home. Missing, failed, stale
or changed-source receipts fail; `windows-peer-cli-receipt.json` is retained in
the existing native integration evidence artifact.

`python tools/test-peer-cli-isolation.py -v` exercises the filesystem guards
with a disposable substitute for the Windows known folder. On a fresh hosted
Windows runner, `python tools/test-peer-cli-precondition-windows.py --evidence
evidence/local/peer-cli-precondition` reuses the locked wx SDK and compiles only
the pinned home-creation excerpt. It verifies the real wx path and the guard
before/after that side effect, retaining source, build logs and cleanup evidence.
This inexpensive prerequisite proof does not execute or qualify the installed
peer CLI; its full refusal check remains part of the integrated native build.

Native Windows UI, complete regression and installer/boat qualification remain
required. A future authentication implementation needs separate design review;
there is no first-use trust, automatic re-pinning or insecure fallback here.
