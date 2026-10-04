# SCRUM-288: unattended certificate prompt reproduced

Run [37177201888](https://github.com/ThereptileII/Work/actions/runs/37177201888),
remote `66592083443e6bad0a13acd83dfe88416a1b6695`, local
`396ead91446e1eb0191bf93474749a1805ad7687`, attempt 1.
Original artifact **11293828012**, 9809 bytes, SHA-256
`7e319c487d8f30f10ced055d2956cc98f822f6245700aeee051e1edad018ecb1`.
Root verified digest, length, all ZIP CRCs and member paths before extraction.
The original archive is retained unchanged; it contains public certificates,
never their private keys.

Both original `Import-Certificate` to CurrentUser/Root and the alternative
`certutil -user -f` timed out at 30 seconds. Their exact owned process trees
contained a **visible Security Warning** window. Durable events show entry into
the import with no return. Neither certificate was imported; exact-owned
cleanup verified absence. This reproduces an unattended test-harness prompt;
it is not an application crash. The earlier 17ab044 log stopped immediately
before this same original import boundary.

The first diagnostic run [37177101089](https://github.com/ThereptileII/Work/actions/runs/37177101089)
failed before certificate generation because executable discovery returned
multiple OpenSSL paths. The correction selects the first resolved application;
the original failure remains retained in CI. No TLS acceptance was claimed.

The correction will use a fresh, exactly tracked LocalMachine/Root test
certificate only inside the disposable GitHub-hosted Windows runner, with a
bounded import and verified removal. Actual Downloader/wxCurl TLS validation
remains unchanged and is still required in the next candidate. No boat trust
store, application TLS policy or production settings are modified.

## Corrected native proof passed

Run [37177584391](https://github.com/ThereptileII/Work/actions/runs/37177584391),
remote `e65eb4a411f6ef63f2f6233756390599b40d302b`, local `22d4280`, attempt 1.
Original artifact **11293469162**, 6224 bytes, SHA-256
`15f1d63c1e008c7209825a8bd96a4f17b4a3f5aeb7b6813d792002b757dd5e11`.
Root verified original bytes, CRCs, safe member paths, actual extracted functions
and both script hashes against the integrated source with native CRLF endings.

All four focused cases pass: exact owned-root import, normal exit, preserved
nonzero exit and one-second timeout with the child absent afterward. The import
completed in **0.3442643 seconds**. Windows chain evaluation rejects the fresh
root before import and accepts the exact root afterward. Exact cleanup restores
the original machine-root inventory. No test certificate or trust policy is
installed on the boat. The full actual TLS matrix and package gates remain
required in the replacement candidate; this proof does not qualify the app.
