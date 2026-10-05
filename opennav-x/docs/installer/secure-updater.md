# Secure startup updater

Implementation contract for SCRUM-23, SCRUM-25 and SCRUM-26. Jira remains the
backlog. This document describes the implementation, not release acceptance.

## Startup and consent

The installed SKAGER shortcut starts `skager-start.exe`. Legacy and Safe
shortcuts continue to start the application directly. Portable recovery remains
independent of installed update state and starts its isolated profile directly.

The launcher verifies its owned generation and measured supported stock OpenCPN
identity before checking for updates. The check precedes application, plugin and
hardware-adapter startup. Missing configuration, offline service, rejected
metadata or a five-second check deadline continues the installed application.
An eligible signed offer opens a separate native **Update Now / Later** dialog.
Escape, close, timeout and Later continue startup; no second prompt appears in
that session. The first choice is latched, including queued repeated clicks.

During acquisition a separate native progress window provides Cancel. Cancelling
this phase aborts verification/download, discards the prepared artifact and
continues the current application. The progress process must confirm completion
before the installer starts. Cancellation never kills an executing installer.

Update Now binds the entire parsed release policy, version and commit. A second
authenticated refresh must return that same policy before acquisition. A changed
offer requires fresh consent on a later startup. Release versions must increase:
the existing policy rejects a different commit with equal version precedence.
Staging/Production delivery and beta/stable metadata channels are separate
concepts; this implementation does not promote or publish a release.

## Trust and retained state

`tools/update-verifier` uses the pinned go-tuf implementation and dependency
locks. HTTPS validation, a fixed origin, per-role byte limits, a total response
budget, bounded requests and strict release-policy parsing apply before any
installer is selected. The stock executable's measured SHA-256 and PE32 ABI
must agree with both the independent installed allowlist and signed policy.

An optional package-owned `app/update-trust.json` contains schema 1, a public
`bootstrapRoot` object, `metadataUrl`, `artifactOrigin`, `channel` (`beta` or
`stable`) and `localCompatibilityAllowlist`. It contains no signing secrets.
There is deliberately no built-in test root or invented public endpoint.

The installer explicitly invokes `--initialize-trust`. Ordinary startup never
initializes trust. Protected retained state lives under
`update-trust/<channel>`; artifacts use `update-artifacts`. A separate durable
`update-trust-initialized-<channel>` marker precedes initial provisioning and
binds the configuration hash. Missing, changed or interrupted retained state
requires explicit recovery; repair/update cannot silently reset rollback floors.
Existing valid state is preserved, including newer authenticated metadata after
a later refresh fails. Root rotation follows TUF; switching bootstrap trust or
channel requires an independently authorized migration, not a server instruction.

Windows directories have a protected ACL and explicit current-user owner.
Reparse paths, broadened permissions, unknown retained files and concurrent
state writers are refused. SHA-256 binds the selected state generation and each
metadata file. These controls do not claim protection against a compromised
Windows account or administrator who can replace the whole application.

## Installer custody and transaction

The installer download must match the signed positive byte length and SHA-256.
The verified Windows handle denies write/delete and remains held through the
exact fixed-argument NSIS invocation. No URL, command line or executable path is
accepted from the dialog. A fifteen-minute installer wait expiry does not kill
the installer, release custody while it runs, retry, or accept the candidate.

The existing versioned side-by-side installer owns backup, profile preservation,
repair and rollback. `/SUPERVISED=1` adds a pending update record before current
generation publication. It requires a previously authenticated known-good
generation. Navigation files and stock OpenCPN are never part of the updater's
replacement set. Historical installations remain manually installable and
repairable; they first need a new healthy generation before automatic updating.

## Startup proof and recovery

The supervisor creates a unique, current-user-only local named pipe, records one
launch attempt durably, then starts the exact candidate. Acceptance binds its
PID, process creation time, executable path/hash, generation, compiled commit and
random challenge. Writing a file or successfully creating a process is not proof
of successful startup.

The application's existing main-thread health path must observe a continuously
ready XNav shell for thirty seconds and persist its recovery checkpoint. Only
then does an independent worker send a bounded one-shot receipt. Its environment
challenge is cleared before plugins or child processes can inherit it. Legacy,
Safe, an early exit and an unready shell cannot acknowledge successful XNav
startup. The supervisor stores a DPAPI current-user protected known-good receipt
only after authenticating the live process.

The actual OpenCPN navigation warning reports an authenticated, bounded
[human-wait phase](startup-human-wait.md). Waiting/Agree are not healthy receipts;
the thirty-second readiness and durable recovery checkpoint remain mandatory.

A failed or interrupted update restores the exact prior verified generation
through the existing locked lifecycle engine. A still-running candidate is
asked to close gracefully; inability to close preserves the pending state and
does not force-kill the application or roll files underneath it. The next startup
recovers pending state before loading the candidate or contacting the network.
Corrupt candidate helpers do not veto recovery using the verified previous
engine. Recovery never repeatedly launches a failing candidate.

## Qualification and distribution

The focused updater workflow checks Linux contracts, native Windows/386 file
custody and ACLs, actual named-pipe/PID receipts, interruption recovery and native
popup interactions. Staging requires these checks before expensive Windows
application compilation. The integrated installer and actual application still
need their exact-commit Windows/boat acceptance; fixture tests are not that proof.

The installed-package smoke additionally bootstraps the actual launcher and
application, applies the exact Setup with supervised mode, verifies its live
startup receipt, then tests guarded fallback after an explicit invalid-PE fault
in a disposable installed candidate. Stock/profile preservation is checked.
This same-package sequence proves transaction mechanics, not selection of a
new signed release. Historical retained packages without the health contract
report this check as not applicable; current health-capable packages must pass.

Native packaging includes the launcher, native prompt and exact corresponding
source: main module, all resolved dependency sources/notices, Go standard-library
source and build identity. The source ZIP and launcher hashes are checked again
when assembling the recovery/installer package.

Public activation still requires an independently provisioned release root,
protected signing/repository operations, a real signed release set, dependency
license review and full installed update/rollback qualification. No signing key,
fixture trust root, automatic production promotion or public endpoint is supplied
by this implementation. An unconfigured installation remains fully usable and
can be updated with a separately obtained, verified installer.

The [Windows signing interface](code-signing.md) is prepared separately. Its
default operation checks prerequisites and creates an isolated unsigned copy;
signing requires an explicitly selected existing certificate and separate Sign
operation. No certificate/key is supplied and no release workflow silently signs
or changes previously accepted artifacts. A new automatic release also requires
an increased embedded product version matching its signed policy, rather than
relabeling another commit with the same version.
