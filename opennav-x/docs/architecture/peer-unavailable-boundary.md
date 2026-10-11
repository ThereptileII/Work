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

`tools/smoke-peer-two-instances.py` adds the concurrent gate alongside those
sequential mode checks. It creates two new disconnected profiles with synthetic
navigation objects and fake saved-key/certificate sentinels, requires both
applications to remain alive, verifies each running build commit matches the
test checkout, and observes each process's peer-listener boundary. Both profiles
must preserve their navigation objects and peer state during overlap and after
clean exit. The current first-start sheet is dismissed using its process-owned
Later action. Native execution is restricted to disposable GitHub-hosted CI;
Linux uses a private X server. No real peer, boat or existing profile is used.

On Windows the first profile starts as an ordinary `--configdir` application
and reaches ready state before the second starts with `--portable --configdir`.
OpenCPN's pinned Windows `WxInstanceCheck` uses the fixed
`_OpenCPN_SILock` mutex name. wxWidgets MSW ignores the supplied profile
directory when creating that mutex, so two ordinary processes in the same
logon session cannot be the independent-profile test claimed by the earlier
harness. Portable mode bypasses that upstream single-instance branch while
`--configdir` still isolates navigation files and peer credentials. Both live
processes use their own profile for data; the portable process also starts
with that profile as its working directory. In pinned OpenCPN 5.12.4,
`AbstractPlatform::NormalizePath` makes portable resource paths relative to
`PrivateDataDir`; launching from the CI checkout instead left bundled tide
data unresolved and blocked startup on OpenCPN's missing-harmonics modal.
The native gate now preserves the upstream portable path convention rather
than dismissing that warning.
Both live
processes must pass the unchanged listener, persistence, secret-output and
exact-build assertions; the portable peer closes first. The report records
each launch mode and argument list. This is concurrent ordinary-plus-portable
containment, **not** evidence that two ordinary installed instances can run.
The ordinary XNav/Legacy/Safe mode-cycle gate separately checks the installed
startup path. Portable resource, plugin and relative-path behavior differs and
requires its own package qualification. On any Windows startup failure the
harness captures bounded process-owned window captions and diagnostic identity
before cleanup, without clicking controls or reading editable text.

The gate rejects fixture secrets in application logs, stdout and stderr, then
keeps raw child stdout/stderr in disposable temporary storage outside the
artifact directory and exports only redacted copies of all three application
streams (plus display-server output). Process cleanup, certificate inspection,
and individual stream read/write failures are recorded independently, so one
failure cannot skip sanitization of the remaining streams or replace the
original gate exception. Report strings are redacted before JSON encoding.
A finalization failure also prevents a PASS result; unavailable evidence remains
missing rather than being replaced with raw output. Its focused contracts
also reject stale/missing binary identity, non-hosted Windows execution, malformed
startup JSON and an exited process. This is process/listener and supported-log
evidence, not a packet capture or acceptance of a replacement trust protocol.
Porting the historical concurrent harness from `eb92c9d8` does not transfer that
commit's runtime evidence: the integrated Linux and native Windows jobs must run
this gate for the exact new candidate.
For helper-only retained retests, the qualifier passes the original product
commit separately from the current test-source commit. A different product
identity requires the authenticated restore receipt bound to the current
harness and workspace; its retained manifest must also match the exact tested
executable. Missing or mismatched receipts, manifests and binaries fail before
application launch. Ordinary runs still require the current checkout identity.
Linux runs it after the integrated install and CTest in
`tools/build-integration-linux.sh`. Native Windows runs it against retained
exact-build inputs in `tools/qualify-staging-windows.ps1`, after mode startup.

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
