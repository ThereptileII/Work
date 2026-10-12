# SCRUM-311 retained private Staging trust selection — 2026-10-12

The existing repository's public config and original public root match
`docs/evidence/2026-10-06-staging-batch/private-origin.json` exactly. No root was
invented, key regenerated, signing material read or repository state modified.
Selected config source: `110e61f78a1b39d4461eeef31d923b610c7f3bc5`.

The discovered `/tmp/skager-repository-check` executable did **not** match the
retained operator executable hash. It was not run. Its measured SHA-256 is
`9915860a44f4f133f5b22d04efd34d807e957b15b02fd5c09259d715c5f72519`,
whereas the old receipt specifies
`f0e372cdef7908e5fba43172af561d38464cc4593a8e0fc2a7ca26f885e61827`.
This is a provenance refusal, not proof of malicious modification: the temporary
filename alone did not authenticate that binary.

One small Linux operator/validator component was compiled offline from clean
`5c6e7d766eb12e1ede4e312d5a689541c96db5e3`, with pinned Go 1.27.1,
CGO disabled, `-trimpath -buildvcs=true`, `GOPROXY=off` and `GOSUMDB=off`.
Its embedded VCS identity matches that source and reports `vcs.modified=false`.
Its SHA-256 is
`ff92904c909a0fd9b3c7ec97de82bef12f9f2c4fd2c3261e2d97d12b93221ff4`.
This is not an application build or a native Windows validator qualification.

The authenticated component ran `export-public-config`, `export-public-root`
and `validate-trust`, all without key-input flags. Export rechecks retained state
against the public configuration, validates the signed bootstrap root and current
expiry, four distinct Ed25519 roles, beta channel and exact qualified stock tuple.
The complete exported configuration was committed unchanged at the package
selector's fixed `installer/windows/staging-update-trust.json` path.

| Public bytes | SHA-256 |
| --- | --- |
| Original and exported full config (2,726 bytes) | `4cd61f55731732a06c4e303506f29bc5f46d8bde9c296ae76680d2dee4d27826` |
| Original retained `public/1.root.json` (1,577 bytes) | `a041ec78891a5e3d8896385ed939b691a32ed0ec11e291fb25de773754515d57` |
| Root re-exported from the nested config | `f07d7527759953cc5c1be06a0770e85d63344d55a7c8e3460b73b90397d3bed0` |

The two root byte streams differ because nested JSON indentation changes the
framing. Their parsed signed metadata and signatures are equal; neither hash is
substituted for the other. Exact full-config equality is the selection boundary.
Root expiry is `2027-10-06T04:53:04.423001063Z`.

This corrects an earlier incomplete investigation: no committed package trust
file had been found, but actual retained public trust already existed. Historical
source/package evidence is preserved. Current selection does not establish HTTPS
availability, signed offer publication, protected broker provisioning, installed
Update Now, native Windows/boat acceptance or Production signing custody. The
earlier custody record explicitly says the root is not offline-isolated.

No full application build, CI dispatch, public push, service activation,
application/profile change or physical command occurred.
