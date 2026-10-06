# Private Staging signing-key custody

`tools/staging-keyring.py` is a Linux operator helper for the existing
[`skager-repository` command](staging-update-repository.md). It has only two
operations: explicit first initialization and publication. It is not installed
in the application, a key export tool, a release qualifier or an automatic
publisher. The coordinator must review the helper and select the exact local
repository executable and SHA-256 before actual use.

This is bounded **private Staging** custody. The root seed remains at rest in
the same desktop Secret Service collection as the three online seeds; it is
loaded only for initialization. It is **not offline-isolated root custody** and
does not establish Production key protection, independent role holders,
threshold signing or recovery assurance. Anyone already authorized to access
this user's unlocked keyring may be able to access these items. The helper's
role restriction does not change the keyring's security boundary.

## Before first use

The selected Secret Service must already be running, with an existing unlocked
default collection. The helper checks service ownership and collection state;
it does not create a collection or intentionally activate a missing service.
Searches request all matching items without unlocking or loading secrets.
Initialization uses libsecret's negotiated secret encoding and a bounded D-Bus
CreateItem request with replacement disabled. A returned prompt is refused
without invoking it. Locked, unavailable or incomplete stores fail closed.
Unlock and authenticate through the desktop separately.

The reviewed local tool must be a regular executable ELF file, owned by the
operator or root, without group/world write permissions, symlink components
or additional hard links. Its exact SHA-256 is required for every invocation.
The helper opens the file without following links, checks its hash, and executes
that same descriptor through `/proc/self/fd`; it never runs a caller-provided
shell command. Review/build provenance of the selected tool remains external.
A hash argument supplied by the operator is not independent provenance.

Use an absolute canonical repository path. The initialization destination must
not exist, and its parent must exist without symlink components. Paths, origins
and command flags are constrained; arbitrary delegated arguments, `--keys-file`,
production scope and a root-key export operation are unavailable. This helper
uses one canonical HTTPS origin root for both metadata and artifacts.

## Explicit initialization

The following is a template, not an instruction to create keys during review:

```sh
python3 tools/staging-keyring.py \
  --tool /reviewed/bin/skager-repository --tool-sha256 REVIEWED_BINARY_SHA256 \
  initialize --repository /private/operator/repository \
  --origin https://approved-private-host.ts.net
```

Replace the hash placeholder with the reviewed 64-character lowercase digest.
Initialization requires an empty fixed key scope. It generates four distinct
32-byte Ed25519 seeds from the operating system's cryptographic random source
and creates four separate records. Their fixed schema is
`app.skager.PrivateStagingSigning.v1`, scope is `private-staging`, and roles are
`root`, `targets`, `snapshot`, `timestamp`. Each record also binds the canonical
repository path through its SHA-256 attribute. No seed is an attribute or label.
The helper re-inventories the stored record identities, reads back the four
roles, validates canonical base64/length/distinctness, and pipes exactly the
repository tool's schema-1 key envelope to its standard input.

A fixed owner-only lock in `/run/user/<uid>/skager-private-staging-custody.lock`
serializes this helper's initialization/publication on this machine. The empty
lock is non-secret. Items in another collection, duplicates, unknown attributes,
wrong repository binding, missing roles and locked records are refused.
Existing items are never overwritten, deleted or repaired automatically.

## Publication

After authenticating the retained Staging selection using the established
release checks:

```sh
python3 tools/staging-keyring.py \
  --tool /reviewed/bin/skager-repository --tool-sha256 REVIEWED_BINARY_SHA256 \
  publish --repository /private/operator/repository \
  --assets /verified/retained-release --policy /reviewed/beta-policy.json \
  --selection /reviewed/selection.json --selection-sha256 REVIEWED_SELECTION_SHA256
```

Publication validates the metadata/attributes of the full four-role key set,
then retrieves only `targets`, `snapshot` and `timestamp` secret values. The
root item's secret is never requested or included in the publication envelope.
The existing repository command remains responsible for matching online keys
to the signed root, authenticating selected local bytes, validating policy and
qualification records, monotonic versions and atomic publication. The helper
does not bypass those checks or deploy any hosting service.

Use the existing repository tool directly for public-root/config export and
`validate-trust`; those read-only operations need no keyring access.

## Failure and confidentiality boundaries

Key creation and repository initialization are not one cross-service atomic
transaction. If creation, process execution, storage or publication fails, retain
both keyring and repository state. A partial set blocks further operations; a
complete set blocks another initialization even if the repository command failed.
There is no automatic cleanup or regeneration. Recovery requires coordinator
review; do not delete keys or reset monotonic repository state to bypass a
refusal. Publication can be retried with its existing complete key set and the
repository tool's documented exact-selection recovery rules.

The envelope exists only in process memory and a private subprocess stdin pipe.
It is never printed, put in argv/environment or written to a file. Child output
is discarded; errors and parser failures use one fixed redacted message.
GLib/D-Bus debug environment switches are removed before loading the binding,
and core dumps are disabled for the helper and inherited child. The child gets
a minimal environment and a 120-second execution timeout. Do not use external
process tracing or memory dumps. Python, GI and Go can retain immutable copies;
complete memory zeroization is not claimed. Keyring durability, encrypted
backups and independent disaster recovery still require separate verification.

## Focused verification

Run `python3 tools/test-staging-keyring.py -v`. Tests replace the custody and
process boundaries with deterministic fakes; they never connect to Secret
Service, create real signing keys, run a repository subprocess or alter services.
They cover exact role envelopes, root exclusion during publication, fixed scope,
canonical seeds, distinctness, partial/duplicate/locked/wrong-repository state,
no replacement or prompt invocation, pinned-file refusal, subprocess timeout,
redaction, state retention and argument/environment isolation. The GI parameter
shape can be checked locally without connecting to a store. Actual keyring
backend behavior and the coordinator's first real initialization remain
explicit operational gates.

API behavior reference: libsecret's
[search options](https://gnome.pages.gitlab.gnome.org/libsecret/method.Service.search_sync.html)
distinguish metadata searches from unlock and secret-load flags. The installed
Secret-1 GIR also documents that high-level item creation may prompt; this
helper deliberately uses its no-prompt D-Bus creation boundary instead.
