# Signed-update trust review

SCRUM-25, 2026-10-02. This reviews the isolated verifier at
`24bd013f4891638f0e59411836c67f271a66a645`, using go-tuf v2.4.2
(`f5edbde31e5507f46db2069402dc38903fe6d9d4`). It does not qualify an installed
updater. Native feasibility evidence is in
[the verifier record](../evidence/scrum-25-verifier-feasibility.json).

## Root trust and rotation

The caller supplies trusted root bytes; a server response cannot select the
initial trust anchor. In the pinned library, `metadata/config/config.go` sets
`LocalTrustedRoot`; `metadata/updater/updater.go` constructs trusted metadata
from it before refreshing. Root rotation downloads successive versions and
verifies them through the existing trust chain.

The library does not adopt cached `root.json` as the next invocation's bootstrap
root. Reusing an old embedded root therefore requires retrieving its rotation
chain again. The probe limits a refresh to 32 root rotations. Production must
define how a verified newer root is durably retained, how interrupted writes
recover, and how a client beyond that bound receives an authenticated recovery
package. An arbitrary cached root must never become trusted merely because it
exists on disk.

## Rollback state

Cached timestamp metadata provides a version baseline. Reverting or removing
that cache can reset the baseline and permit replay of older, still-unexpired,
correctly signed metadata. This does not bypass signature verification.
Snapshot and targets metadata are also verified; a writable cache does not
permit an attacker to sign a new release.

`EnsurePathsExist` creates missing directories with mode 0700. It does not
validate existing directory ownership or Windows ACLs. Production must establish
the cache's permitted owner, reject unsafe locations/reparse substitutions,
serialize concurrent writers, and retain verified state atomically. Cache loss
or restoration must have an explicit recovery policy instead of silently
claiming full rollback protection.

The present installer is per-user (`RequestExecutionLevel user`) and its
lifecycle script intentionally performs no elevation. This review does not
introduce an administrator service or claim protection against arbitrary code
running as the same user. If a privileged helper is introduced later, it must
independently verify its inputs and cannot treat user-writable cache contents
as installation authority.

## Release policy and handoff

TUF metadata-version rollback checks do not establish application-version
downgrade protection. SCRUM-24 separately defines strict release identities,
Beta/Stable selection and exact OpenCPN compatibility. Signed target identity
and parsed release identity must agree before any installation decision.

Neither parsing a manifest nor a successful metadata refresh authorizes package
execution. The eventual handoff must verify the exact package bytes again,
bind them to the selected release and compatibility identity, and use the
existing recoverable generation lifecycle. Startup-success acceptance and
automatic/manual rollback remain SCRUM-26 work.

Production signing-key custody, threshold/rotation policy, bootstrap-root
provisioning, dependency notices and Windows package signing are still separate
release gates. No production endpoint or signing key is configured by the probe.

## Runtime dependency inventory

The non-test verifier imports 12 third-party modules on both Linux/amd64 and
Windows/386, compared with 83 modules in the full selected module graph. The
module-root license texts observed are two MIT, six Apache-2.0 and four
BSD-3-Clause, with a separate TUF NOTICE. Sigstore is a runtime dependency,
not merely a test helper dependency. The exact module versions and 13 file
hashes were independently reproduced; see the
[runtime inventory](../evidence/scrum-25-runtime-license-inventory.json).
This module-root inventory does not establish package-level notice completeness
or legal approval for the final distribution.
