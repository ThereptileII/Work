# Signed-update artifact content inspection (SCRUM-311)

`tools/boat/signed_update_artifacts.py` is a standalone, read-only Python
library. It is not called by the product, launcher, supervisor, broker,
commissioning scripts, journal store or CI. It never starts a process, changes
a profile, signs a receipt or returns launch permission.

This increment addresses the gap between plausible hash strings in the inert
signed-session model and the contents of actual supplied artifact files. It
uses the existing formats emitted by `commission-read-only.ps1`,
`initialize-target.ps1` and `Lifecycle.ps1`; it does not replace their live
verification or change their schemas.

## Input and result

Call `validate(expectation_json_bytes, now_unix_integer)` with a caller-retained
expectation and explicit inspection time. All errors refuse without a partial
success report. The expectation has exactly these fields:

- `schema`: integer 1.
- `session`: nonzero lowercase 64-hex session identity.
- `identity`: exact `generation` (32 hex), `commit` (40 hex), `packageSha256`
  and `executableSha256` (64 hex).
- `createdUnix`, `expiresUnix`, `reviewedUnix`: integer times. The review must
  belong to the session, not be in the future, and the session must remain
  unexpired with a maximum four-hour lifetime.
- `objectHashAlgorithm`: `sha256-json-sortkeys-utf8-v1`.
- `contextContentSha256`, `treesContentSha256`, `environmentContentSha256`:
  expected digests of the corresponding parsed objects, calculated by
  `object_digest`. This format sorts keys, preserves Unicode, omits whitespace
  and encodes UTF-8. **These are not interchangeable with PowerShell receipt,
  context or tree hashes.** No receipt is created by this library.
- `artifacts`: exactly the twelve descriptors below. Each descriptor has only
  `path` (absolute local snapshot path), `sourcePath` (original canonical local
  Windows path) and `sha256` (the expected raw-file digest).

The eight JSON records are `prepared`, `applied`, `inventory`, `plan`, `target`,
`state`, `ownership` and `active`. The four raw artifacts are `baseline`,
`inputProfile`, `profile` and `executable`. Explicit copies can therefore be
inspected on Linux while preserving their original Windows location bindings.
No path discovered inside a record is followed. The test fixture is an
executable example of the expectation format, not boat acceptance evidence.

The records are checked for actual cross-document bindings: selected
generation, owned executable entry and bytes, commit/package identity,
state/ownership hashes, transaction directory, preparation/application chain,
active marker, baseline/input/current-profile hashes, independent target audit,
review times, complete named loader-root set, context and launch environment.
Re-pinning the raw hash of a changed record does not bypass those comparisons.
The audit's three read-only assertions must be actual JSON booleans, but their
truth is not established by this content checker.

Success returns only `status=content-verified-only`, `launchAuthorized=false`,
`nativeCustodyQualified=false`, the session identity and the artifact count.
The caller must not reinterpret this report as an allow token or approval.

## Bounds and file observations

Each JSON record and raw profile is limited to 4 MiB; the inert executable-byte
inspection is limited to 128 MiB. Exactly twelve artifacts bound total input
file size to 172 MiB. The expectation itself is limited to 4 MiB. JSON allows
at most 24 levels and 100,000 value nodes, signed 64-bit integers only, strict
UTF-8 and no duplicate or case-folded duplicate keys. Non-finite numbers,
fractions, invalid surrogate strings and non-object roots refuse.

Original Windows source paths are at most 240 characters, absolute local-drive
paths without relative/device/stream aliases. Snapshot paths are canonical
absolute host paths of at most 4,096 characters. The evidence set rejects
duplicate source paths and files sharing an inode. Only nonempty regular files
with one link are accepted; symlink/reparse ancestors and files refuse.

Read-only file descriptors remain open until all comparisons finish. The
reader compares path/descriptor device, inode, mode, link count, size and
nanosecond modification/change times, plus ancestor identities, and re-reads
every digest before returning. Same-byte replacement and in-place mutation
observed during inspection refuse. Descriptor cleanup also runs on errors.

## Deliberate limits

The expectation and its time are supplied by the caller, not authenticated by
this library. A forged but internally consistent evidence set is not made
trustworthy by passing. The inspector does not prove fresh physical review,
source-review text or signatures, profile input-only semantics, baseline
transformation, actual loader-tree file contents, process closure, absence of
restoration intents, Windows ACL/share custody or whole-disk replay freshness.
Those existing commissioning checks and the future authenticated broker remain
required. Source review and quarantine semantics are not replaced by a tree
manifest digest.

The twelve reads are not an atomic filesystem snapshot. There is no protection
against a coordinated adversary restoring old contents/metadata between
observations, or against changes after a file's final check. Descriptors close
before the report returns. Native retained handles, shared mutation exclusion,
all spawn hooks and actual prelaunch verification remain unimplemented here.
No Windows/native qualification or production/boat acceptance is claimed.

## Focused verification

From `opennav-x`, run:

```
python3 tools/boat/test-signed-update-artifacts.py
```

Twelve test methods pass on Linux using only disposable temporary files. They
cover all twelve changed/missing artifact cases, re-pinned candidate/profile/
chain mismatches, stale/future reviews, unsupported hashes and schemas,
duplicate/invalid/oversized JSON, unsafe source paths, linked ancestors,
hardlinks, file aliases, empty/oversized/nonregular files, same-byte replacement
and in-place mutation. The positive control also verifies that all artifact
bytes remain unchanged and that success explicitly denies launch authority.
PowerShell and native Windows checks remain pending; no runtime was downloaded.

The two symlink tests are separate: Windows error 1314 (missing symbolic-link
privilege) reports an explicit skipped fixture, not a passed refusal. Alias,
hardlink, missing-field and remaining file checks still execute. Other fixture
creation errors fail. If Windows rejects replacement/write with sharing or
lock violation 32/33 while the read descriptor is open, the mutation test
requires the original bytes and identity to remain unchanged and its recheck
to succeed. It reports prevention rather than an observed mutation/refusal.
Every successful mutation must still be rejected. No other error is accepted
as prevention; native symlink qualification remains open when those cases skip.
