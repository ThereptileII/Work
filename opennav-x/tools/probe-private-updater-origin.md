# Private updater origin transport preflight

`probe-private-updater-origin.py` makes one read-only, direct HTTPS request for
`/1.root.json` at the exact `metadataUrl` in an independently approved public
trust configuration. It is an operator preflight on an already connected test
machine, not a Tailscale login, TUF validator, publication, installer, or product
acceptance test. It neither changes network configuration nor reads signing keys.

Supply the original `public/1.root.json` bytes from the retained operator
repository and two **independently approved** SHA-256 pins. The original root
file is distinct from the `bootstrapRoot` JSON nested in the config: the latter
may be formatted differently. The helper requires the parsed objects to agree,
and requires the HTTPS response bytes to equal the original file exactly.

```powershell
python tools/probe-private-updater-origin.py `
  --config C:\approved\update-trust.json `
  --config-sha256 <approved-exact-config-sha256> `
  --original-root C:\approved\1.root.json `
  --original-root-sha256 <approved-exact-original-root-sha256> `
  --evidence C:\acceptance\private-origin-preflight
```

Run only after the operator separately connects the disposable host to the
approved private tailnet. The helper offers no enrollment or fallback to public
hosting. The selected URL must be canonical `https://<lowercase-host>.ts.net`,
with no port, path, credentials, query, fragment, or alternate artifact origin.
It uses the system default CA and hostname verification, SNI and the identical
HTTP Host value. It opens a socket directly to port 443, without an HTTP proxy,
redirect, cookie, token, or alternate-host retry. It requests only the fixed
root path, and requires one HTTP/1.1 200 response with an exact Content-Length,
body size, SHA-256 and byte match. An HTTP rejection, redirect, TLS/hostname
failure, malformed response or unavailable origin refuses.

Both input files must be absolute, ordinary, single-link files under plain
ancestors. The config is limited to 576 KiB, the original root and response to
1 MiB, headers to 8 KiB/32 fields/2 KiB per line. Duplicate JSON keys and
duplicate HTTP headers refuse. DNS resolution has a five-second wait (a timed-out
resolver cannot initiate an HTTP request); all candidate addresses share a
five-second connect/TLS deadline; response reads share a ten-second deadline
after the request. Input
handles, bytes and path identities are checked again after the request, including
on request refusal. The evidence directory must be fresh with an existing plain
parent; `result.json` is exclusively created and never overwritten. Failures
print one fixed message and record only a refusal, without URL, response bytes,
paths, private trust contents, or exceptions.

A passed result says only that this host reached the selected private HTTPS
endpoint and received the independently pinned original root bytes at that
moment. It does not prove tailnet device authorization policy, current signed
timestamp/offer, TUF signature/expiry/floor acceptance, B0/B1 provenance,
protected enrollment, installed startup, SCM behavior, or boat reachability.
The real repository validator and updater retain those separate checks.

The inert deterministic test is `python tools/test-probe-private-updater-origin.py
-v`; it makes no network connection.
