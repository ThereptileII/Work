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

## Published-file boundary

The fixed path allowlist matches repository output exactly:

- `timestamp.json`, served without caching;
- numbered `N.root.json`, `N.targets.json`, `N.snapshot.json`;
- `targets/releases/<64-lowercase-hex-sha256>.beta.json`;
- `artifacts/<40-lowercase-hex-commit>/<fixed-artifact-name>` for the five names
  in the repository tool's reviewed selection contract.

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
stalled download/write deadlines, bind failure, and recovery after timeout. They do not deploy a service or contact Tailscale.
Actual tailnet HTTPS reachability and the installed updater sequence remain
separate integration gates.
