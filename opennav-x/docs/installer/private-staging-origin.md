# Loopback origin for private Staging

`tools/staging-origin.py` is the operator-only Linux HTTP origin for SCRUM-311.
It serves the `public` child of one explicitly selected repository produced by
[the offline repository tool](staging-update-repository.md). It does not publish
a release, change a service, configure Tailscale, read keys, or grant access.
Only temporary local fixtures are used by its tests.

After the coordinator separately approves the repository and private HTTPS
hostname, an operator can run the helper in the foreground:

```sh
python3 tools/staging-origin.py \
  --repository /private/operator/repository \
  --host APPROVED-HOST.ts.net --port 8765
```

Replace the uppercase placeholder with the exact lowercase approved hostname.
The helper always binds `127.0.0.1`; it has no all-interface binding option. A
separately configured Tailscale Serve HTTPS reverse proxy can forward that
private origin root to `http://127.0.0.1:8765`. Preserve the original `Host` header
and paths. The origin accepts only the selected hostname, optionally followed
by `:443`. It does not trust forwarded-host headers and does not accept the
loopback hostname as a substitute. Test the actual proxy's Host behavior before
making this the selected update endpoint; a mismatch is a refusal, not a reason
to broaden the allowlist. Stop the foreground process with Ctrl-C.

Tailscale account login, device identity, trusted HTTPS certificates, ACLs,
private-only Serve configuration and key custody are the coordinator's separate
work. Do not enable public Funnel. This helper provides no HTTP authentication
or TLS itself and is not public hosting software. Loopback is a transport
boundary, not authorization against other processes already on this machine.
The selected repository remains an operator-owned trusted publication directory.

## Persistent operator user service

For the selected private origin on this operator machine, install the
[version-controlled user unit](../../tools/systemd/skager-staging-origin.service)
only after its pinned script and repository have been checked. The unit runs as
the `standard` user, never as root. It invokes `/usr/bin/python3` on
a hash-pinned, versioned copy of `tools/staging-origin.py`, selects
`/home/standard/.local/share/skager-staging/repository` and requires the exact
`omarchy.tail5878fc.ts.net` Host on loopback port 8765. It does not start or
configure Tailscale Serve. The script's fixed allowlist, absence of directory
listings and refusal of private repository files remain the HTTP boundary.

From the integrated checkout at
`/home/standard/Projects/X-nav-worktrees/source-batch-20261011`, run as
`standard` (the only repository file read is the public root; never inspect or
copy the repository's private key material):

```sh
set -euo pipefail
source=/home/standard/Projects/X-nav-worktrees/source-batch-20261011/opennav-x/tools/staging-origin.py
unit=/home/standard/Projects/X-nav-worktrees/source-batch-20261011/opennav-x/tools/systemd/skager-staging-origin.service
repository=/home/standard/.local/share/skager-staging/repository
sha=198242e4f7635ace9ca831ae394becd1838e8428704a50b511506604e72fa8ad
printf '%s  %s\n' "$sha" "$source" | sha256sum --check --status
printf '%s  %s\n' a041ec78891a5e3d8896385ed939b691a32ed0ec11e291fb25de773754515d57 "$repository/public/1.root.json" | sha256sum --check --status
test -d "$repository/public"
copydir=/home/standard/.local/lib/skager-staging-origin/$sha
install -d -m 0700 "$copydir" /home/standard/.config/systemd/user
if test -e "$copydir/staging-origin.py"; then
  cmp -s "$source" "$copydir/staging-origin.py"
else
  install -m 0400 "$source" "$copydir/staging-origin.py"
fi
printf '%s  %s\n' "$sha" "$copydir/staging-origin.py" | sha256sum --check --status
install -m 0600 "$unit" /home/standard/.config/systemd/user/skager-staging-origin.service
systemd-analyze --user verify /home/standard/.config/systemd/user/skager-staging-origin.service
systemctl --user daemon-reload
systemctl --user enable --now skager-staging-origin.service
```

`ProtectSystem=strict` and `ProtectHome=read-only` leave the selected public
repository and copied script readable while denying service writes; Python
bytecode writes are disabled. The host must support these user-service mount
restrictions. If it does not, leave the service stopped and review the unit on
that host; do not silently drop the read-only boundary. The unit retries a
failed process after five seconds, at most three starts in sixty seconds. It
does not restart a clean operator stop. For service continuity after logout or
reboot, an administrator should verify `loginctl show-user standard -p Linger`
and run `loginctl enable-linger standard` if it reports `Linger=no`.

Check the running user unit and the loopback response before relying on the
existing private HTTPS proxy:

```sh
systemctl --user is-active skager-staging-origin.service
set -o pipefail
curl --fail --silent --show-error --header 'Host: omarchy.tail5878fc.ts.net' \
  http://127.0.0.1:8765/1.root.json | sha256sum
```

The root digest must be
`a041ec78891a5e3d8896385ed939b691a32ed0ec11e291fb25de773754515d57`.
From the intended tailnet client, verify the already configured HTTPS endpoint
and compare the same root digest:

```sh
set -o pipefail
curl --proto '=https' --fail --silent --show-error \
  https://omarchy.tail5878fc.ts.net/1.root.json | sha256sum
```

A local response alone does not prove tailnet access or installed-updater
acceptance.

On failure, inspect `systemctl --user status skager-staging-origin.service`
and `journalctl --user -u skager-staging-origin.service`; the helper emits no
request log or secrets. Check only the selected paths, pinned hashes and port
availability. After correcting the cause, reset the unit's failed state and
restart it, then repeat both response checks:

```sh
systemctl --user reset-failed skager-staging-origin.service
systemctl --user restart skager-staging-origin.service
```

If the source script changes, review and pin the new bytes in a new versioned
copy and unit; do not redirect this unit to a mutable checkout. Running
`systemctl --user stop skager-staging-origin.service` stops the origin without
changing Tailscale Serve.

## Published-file boundary

The fixed path allowlist matches repository output exactly:

- `timestamp.json`, served without caching;
- numbered `N.root.json`, `N.targets.json`, `N.snapshot.json`;
- `targets/releases/<64-lowercase-hex-sha256>.beta.json`;
- `targets/bootstrap/<64-lowercase-hex-sha256>.broker-beta.json`;
- `artifacts/<40-lowercase-hex-commit>/<fixed-artifact-name>` for the five names
  in the repository tool's reviewed application selection contract, plus
  `skager-update-broker.exe` and `skager-update-provisioner.exe`.

The bootstrap policy uses the application's existing `SKAGER-Beta2-source.zip`
and `SOURCE_AND_LICENSES.md` corresponding-source and notice artifacts. Other
source or notice filenames are not served, even if a policy declares them.
Unhashed bootstrap aliases, other channels and additional executables remain
outside this allowlist. Bootstrap publication does not enroll or activate a
broker, approve operator pins, or establish installed application qualification.

The numbered metadata, hash-named policy and commit-named artifacts are immutable
publication inputs; the helper does not modify or re-sign them. The publisher
must continue using its atomic immutable-file and timestamp publication rules.
GET streams exact bytes in 64 KiB chunks. HEAD returns the same content length,
type and cache policy without a body. Files are opened once per request, so an
atomic timestamp replacement does not mix versions within a response. Clients
still perform the existing TUF signature, version, expiry, hash and length
checks. Serving a file is not proof of qualification or signature validity.

There are no listings, redirects, uploads, dynamic handlers or arbitrary-file
routes. Percent encodings are rejected, as are queries, fragments, backslashes,
dot components, duplicate slashes and absolute request targets. `state.json`,
trust configuration, operator locks, keys and `.pending-*` names are excluded
even if accidentally placed under `public`. Every directory component is opened
relative to a pinned descriptor with no symlink following; final files must be
regular, nonempty, and have one hard link. Symlinks, hard links and special files
are refused. Windows/reparse-point hosting is unsupported and fails closed.
The root's ancestors may not be symlinks either.

Only GET and HEAD are supported. Range and conditional requests are served as
ordinary full 200 responses; no range-resume behavior is claimed. Unsupported
methods, bodies, transfer encoding, Expect, Upgrade and ambiguous Host/content
length headers are rejected. Every connection closes after one request.
Errors have fixed empty bodies. The helper emits no access logs and never echoes
request paths, query strings, header values, local paths or tracebacks.

## Resource bounds and checks

Defaults are four request workers and a listen backlog of sixteen. Excess
accepted connections receive 503 without spawning another worker. Request
headers have a five-second absolute deadline, 16 KiB aggregate limit, forty
header lines, 4 KiB line limit and 2 KiB request target limit. Streaming has a
30-second write timeout and ten-minute absolute transfer deadline. Metadata and
policy files are limited to 1 MiB; artifacts to the repository's 4 GiB limit.
A timeout/disconnect closes the connection; clients must reject incomplete
bodies using Content-Length and their signed metadata.

Run `python3 tools/test-staging-origin.py -v`. The focused tests create real
loopback clients against temporary repositories and cover GET/HEAD, binary
integrity, atomic timestamp replacement, retained old metadata, forbidden
paths/hosts/methods/framing, no error reflection or logs, symlinks/hardlinks/FIFOs,
ancestor substitution, size limits, slow-header deadlines, bounded concurrency
stalled download/write deadlines, bind failure, and recovery after timeout.
Bootstrap routes also cover exact binary/source responses and refusal of
aliases, extra files, linked files and oversized metadata. Tests do not deploy
a service or contact Tailscale.
Actual tailnet HTTPS reachability and the installed updater sequence remain
separate integration gates.
