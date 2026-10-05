# Private Staging update repository

SCRUM-311 supplies an offline operator tool, `tools/update-verifier/cmd/skager-repository`,
using the locked go-tuf v2.4.2 dependency. It does not create keys, qualify releases,
contact GitHub, upload files, configure hosting, launch an installer, or activate
public access. Staging GitHub Releases remain draft. The TUF channel is `beta`;
that channel name does not mean Production promotion.

## Authority and custody

The coordinating operator must first authenticate the selected retained Staging
release and qualification provenance using the established release-delivery
checks. The repository tool then binds that approved selection to actual local
bytes. A caller-supplied `--selection-sha256` identifies the reviewed selection;
a locally self-described `passed` record is not independent release authority.

Use four distinct Ed25519 keys, one per `root`, `targets`, `snapshot`, and
`timestamp`, each with threshold one. This is the bounded private test-channel
policy; stronger thresholds and root rotation require separate implementation
and review. Keep the root role offline between initialization and future planned
rotation. Root metadata expires after 365 days; targets/snapshot after seven days;
timestamps after 24 hours. Refresh before expiry using the same exact policy and
selection, or publish an approved newer version. Expiry is enforced by clients.

The tool never generates or writes private keys. Supply a decrypted envelope
from the external secret manager directly through a private pipe to stdin with
`--keys-stdin`. The envelope is JSON with exact fields `schema: 1` and `keys`.
Each entry in `keys` maps a role name to a standard-base64 32-byte Ed25519 seed.
Initialization requires exactly all four roles; publication requires exactly the
three online roles. Root material is rejected by `publish`. Alternatively,
`--keys-file` accepts a regular local file without symlink components, accessible
only to its owner. Neither option accepts keys in argument strings. Secret
manager integration, encrypted backups, unlock and operator authentication remain
external. Do not enable shell tracing, use command substitution for secrets, or
write decrypted envelopes into a checkout, package, evidence directory or log.
Errors use a fixed message and never print raw key or input values. In-process
buffer clearing is best effort; it is not a claim of complete runtime memory erasure.

## Initialization and public export

Build the command from the selected clean source with the pinned Go toolchain.
Mutating operator commands run on Linux. `validate-trust` and public export build
for Windows as well; the packaging producer can compile the validator from its
same exact source revision without distributing the operator tool.

With a separately configured secret-manager pipe, invoke:

```
skager-repository initialize --repository /private/operator/repository \
  --metadata-url https://APPROVED-PRIVATE-HOST \
  --artifact-origin https://APPROVED-PRIVATE-HOST --keys-stdin
skager-repository export-public-root --repository /private/operator/repository
skager-repository export-public-config --repository /private/operator/repository
skager-repository validate-trust --config /reviewed/public/update-trust.json
```

The host tokens above are placeholders, not selected deployment endpoints.
The repository directory must be fresh; the parent must exist. Initialization
creates a private directory, `state.json`, `update-trust.json`, and
`public/1.root.json`. An interrupted initialization is not silently reset.
Export commands emit only public bytes on stdout. Configuration includes the
signed bootstrap root, selected HTTPS URLs, beta channel and the single existing
qualified OpenCPN 5.12.4 x86 hash/upstream tuple. It contains no signing secrets.
The read-only validator emits exactly:

```
{"schema":1,"status":"valid","configSha256":"<exact input SHA256>","channel":"beta"}
```

Package that reviewed configuration through the explicit package trust selector,
never by modifying installed ownership records. Keep its exact bytes unchanged
across the bootstrap and successor: installer provisioning retains a configuration
hash and refuses silent trust migration. Never package test fixture roots.

## Reviewed selection and policy

Supply a local directory containing the unchanged retained `RELEASE.json`,
`QUALIFICATION.json`, and these five artifact files:

1. `SKAGER-Beta2-Release-Notes.md`
2. `SKAGER-Beta2-Setup.exe`
3. `SKAGER-Beta2-Portable-Recovery.zip`
4. `SKAGER-Beta2-source.zip`
5. `SOURCE_AND_LICENSES.md`

The first four artifacts must match the retained manifest's exact size and hash.
The notices must be the exact
`SKAGER-Beta2-Portable-Recovery/docs/SOURCE_AND_LICENSES.md` member from the
retained recovery archive, not freshly rewritten text. The input policy is the
strict existing `releasepolicy.Policy` schema, with product `skager`, channel
`beta`, qualified version/commit, the five role declarations above and the pinned
stock compatibility tuple. Every artifact path is
`artifacts/<exact product commit>/<fixed filename>` and its URL is exactly the
selected artifact origin followed by that path. No credentials, queries, fragments
or redirects are used. The signed TUF target is `releases/beta.json`; its custom
identity repeats the exact channel/version/commit from the policy.

The reviewed selection has this exact schema (files follow the five-role order
above, with exactly five entries):

```
{
  "schema": 1,
  "version": "<embedded SemVer>",
  "commit": "<40 lowercase hexadecimal characters>",
  "releaseManifestSha256": "<SHA256 of retained RELEASE.json>",
  "qualificationSha256": "<SHA256 of retained QUALIFICATION.json>",
  "policySha256": "<SHA256 of exact policy JSON bytes>",
  "files": [
    {"path":"artifacts/<commit>/<filename>","sha256":"<SHA256>","bytes":123}
  ]
}
```

The abbreviated array illustrates one record, not an accepted one-file selection.
Hashes, sizes, role paths, version, commit, supported stock and local qualification
gates are checked before signing. The operator-authenticated selection pin is a
required argument:

```
skager-repository publish --repository /private/operator/repository \
  --assets /verified/retained-release --policy /reviewed/beta-policy.json \
  --selection /reviewed/selection.json --selection-sha256 <reviewed SHA256> \
  --keys-stdin
```

## Publication and recovery

Only one writer may hold `operator.lock`. The private directory must retain its
owner-only permissions. A durable state update reserves the next metadata version
and product floor before copying/signing. Artifact copies are bounded and rehashed;
existing artifacts cannot be overwritten with different bytes. Metadata uses
consistent snapshots: immutable numbered targets/snapshot files and a hash-prefixed
policy target. Files and containing directories are synced. The sole publication
switch is atomic replacement of `public/timestamp.json`, performed last.
Readers holding the previous timestamp can still obtain all previous metadata
and policy bytes.

An interrupted publication consumes its reserved metadata version. Rerun with
the identical reviewed selection/policy, or an approved higher product version;
the next publication uses a greater metadata version. A lower product version,
or different commit/policy/selection at equal version precedence, is refused.
Never remove `state.json`, restore only an old state backup, delete immutable
metadata, or reset the repository to work around a refusal. Back up the complete
operator directory consistently and retain its monotonic state; ambiguous state
requires explicit recovery review. This protects ordinary interruption/retry,
not a compromised owner able to replace the entire directory and keys.

Serve only `public/`, not the operator directory, with direct HTTPS 200 responses
on the independently approved private network. Preserve all immutable files.
For the simplest deployment, metadata and artifact origins are the same origin
root and map to `public/`. Other URL path mappings must preserve both metadata
and artifact paths exactly. Disable directory listing and access to `.pending-*`
temporary names. The launcher has no bearer-token/cookie authentication and
rejects artifact redirects, so private draft GitHub asset links cannot be used
directly. Tailnet ACLs and a trusted HTTPS certificate are separate prerequisites;
this command never modifies remote access or host services.

## Focused checks

Run `go test ./repository ./cmd/skager-repository` from `tools/update-verifier`.
The Linux operator tests use temporary repositories, ephemeral in-memory keys and
an isolated TLS fixture; they exercise actual go-tuf metadata/target verification,
tampering, wrong keys, interrupted publication with old readers, monotonic floors,
input refusal, symlinks and concurrent writers. A Windows cross-build proves only
portability; native package validation and the real installed Now/Later/no-update/
offline sequence remain separate required acceptance. No application, installer,
boat session or real signing material is used by these tests.
