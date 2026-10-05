# Release policy contract (SCRUM-24)

The policy/parser semantics below remain authoritative. Their original
verify-only integration description is superseded by the
[secure startup updater](secure-updater.md), which adds protected state,
native consent, artifact custody and startup-supervised transactions. Production
trust provisioning and exact installed qualification remain separate gates.

`tools/update-verifier/releasepolicy` defines a backend-independent, schema 1 JSON declaration for a candidate SKAGER update. Parsing and evaluation are pure checks. They do not fetch a release, authorize an install, alter OpenCPN, or run code.

The declaration names `schema: 1`, `product: "skager"`, `channel: "beta"` or `"stable"`, a strict [SemVer 2.0](https://semver.org/) `version`, and a 40-character lowercase hexadecimal `commit`. `0.4.0-beta2` is a valid beta version. Stable policy rejects a prerelease version. `releaseNotes`, `installer`, `recovery`, `correspondingSource`, and `notices` each require `path`, `url`, `sha256`, and `bytes`. The URL must be HTTPS and its path must match the safe relative artifact path. Byte counts must be positive and at most 4 GiB. The JSON declaration is at most 32 KiB, versions are at most 128 characters, and there are at most 16 supported OpenCPN identities. Field spelling is exact, including case. Unknown or duplicate keys, malformed UTF-8, nesting beyond 16 levels, and trailing JSON fail parsing. Artifact paths reject Windows reserved device basenames, traversal, alternate separators and trailing dots.

Each `supportedOpenCpn` entry uses the existing `installer/windows/compatibility.json` field names: `version`, `arch`, `executableSha256`, and `upstreamCommit`. The current supported ABI is `x86`. Evaluation requires the observed OpenCPN identity to match both a policy entry and an independently trusted local allowlist entry, including the exact executable hash and upstream commit. A version string alone never establishes compatibility.

`Evaluate` classifies a candidate as `upgrade`, `current`, `downgrade`, `incompatible`, `channelmismatch`, or `versionconflict`. It compares SemVer numeric identifiers as decimal strings, so a large valid number cannot overflow an integer; build metadata does not change precedence. Equal precedence with a different commit is a version conflict, including versions that differ only in build metadata. A lower version is a downgrade classification, never automatic permission to install. Channel selection must match the signed policy. Beta may carry a normal release; stable cannot carry a prerelease.

Before a future installer bridge may use this result, it must verify the **exact policy bytes** through signed TUF target metadata, bind the signed target identity to the parsed policy, read the installed version and commit from trusted local state, measure the stock OpenCPN executable and compare it with the local allowlist, verify artifact bytes, and apply the existing atomic install/recovery gates. The parser alone cannot establish any of those prerequisites. Root provisioning, metadata-cache protection, repository key policy, native Windows validation, and a separate anti-downgrade rule for installed application versions remain acceptance gates. This contract is a probe; it does not change the current installer or release qualification.

`verifier.VerifyRelease` is the isolated read-only composition: it verifies the
TUF target first, parses those exact bytes, and requires exact agreement of
channel, version and commit between signed target metadata and the release body.
Its tests use real ephemeral signatures and do not contact a release server or
execute a package. A successful return still requires compatibility/version
evaluation and verified transactional installation; it is not a startup updater.
