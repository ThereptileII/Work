# SCRUM-212 separate exact-candidate two-instance probe

This is test-only preparation for separate artifact qualification; no new native
result is claimed. Publication alone is not dispatch or acceptance. The frozen
product is not rebuilt or edited.
Jira SCRUM-212 comment 10412 records the selected separate qualification.

The manual `peer-candidate-probe.yml` workflow takes an integrated baseline run
in `ThereptileII/Work`, its full candidate SHA, the artifact ID, and its reviewed
SHA-256. The final `beta-candidate-<SHA>` path requires completed successful CI.
The `beta2-boat-review-pending-endurance-<SHA>` path may run while endurance is
still active, after all 19 named native functional, security, installer, DPI,
chart, restart and artifact preparation/upload steps succeed. It downloads and
verifies the restart qualification receipt's artifact length/digest/CRC and exact
commit/run/attempt, rather than relying only on a green step. Failed/cancelled
upstream runs or jobs, stale attempts and incomplete prerequisites are rejected.
These checks run again at probe completion. Pending development qualification
remains pending even if the two-instance probe passes; it is never release
acceptance. This permits useful independent work during the three-hour soak.

Two invocation paths are prepared. Manual workflow_dispatch remains available
when GitHub has registered the workflow; registration/API availability is not
assumed and no default-branch change is required by this preparation. The
alternative push trigger matches only branch `skager-candidate-security-probes`
and path `opennav-x/tools/peer-candidate-request.json` together. Publishing only
the workflow or unrelated changes does not match that path filter.

No request file is supplied. The parent creates and reviews it only after an
exact artifact exists and its native prerequisites pass. Its strict JSON schema
has exactly four string fields: `run` (positive ASCII decimal run ID), `commit`
(full lowercase 40-character SHA), `artifact` (positive ASCII decimal artifact
ID), and `digest` (lowercase 64-character artifact SHA-256). Duplicate keys,
unknown/case-aliased fields, non-string/nested values, invalid UTF-8, trailing
JSON and requests over 2048 bytes are rejected. The request parser emits only
validated workflow outputs; the existing probe still independently checks API,
receipt, prerequisite, archive and runtime identities. No placeholder IDs are
committed, and adding the request file is an explicit CI trigger action.

Both paths retain separate test-source and candidate-binary identities. The
push SHA identifies the test orchestration/request revision; the request commit
identifies the already-built binary and must match both running diagnostics.
Publication and invocation remain with the parent review; this preparation
neither publishes a branch nor triggers a run. Inert request-schema checks cover
valid input and malformed/ambiguous inputs without invoking the existing probe.

`tools/probe-peer-candidate.py` binds the inputs to GitHub's run and artifact
metadata, declared size and SHA-256, downloaded ZIP length/digest/CRC, internal
SHA256SUMS, embedded package commit/payload hash, and every package file hash.
Archive names, duplicate/case-alias paths, symlinks and expanded size are checked.
Only NSIS `$PLUGINSDIR/package.json` and `$PLUGINSDIR/payload.zip` are streamed
through 7-Zip. Setup, Lifecycle.ps1 and Maintain.exe are never executed. The
extraction creates no installation, shortcuts or registry registration.

The payload contains the complete app/docs generation. Its producer,
`tools/package-alpha-installer.py`, already excludes OPENNAV_PORTABLE_PREVIEW.
The adapter rejects a payload containing that marker; it does not remove one.
Every payload file must match its counterpart in the same bound recovery ZIP,
whose marker remains untouched. Product metadata must declare the requested
commit, an installed product without test fixtures, and status-only hardware
output. Executable identity and packaged plugin presence are required.

The exact recovery ZIP retains `profile/opencpn.conf`, written by
`tools/package-preview.py` from the original generated build/include/config.h.
The adapter reads its single ConfigVersionString and writes a separate two-line
profile-adapter/include/config.h containing VERSION_FULL and VERSION_DATE. This
is input for the unchanged profile creator, not a build or product modification.
The retained config and adapter hashes are recorded. No version/date is guessed
from source or PRODUCT_BUILD.json, which does not retain those two values.

The unchanged `tools/smoke-peer-two-instances.py` is pinned to SHA-256
`0fce57bf823bf98a4a1e385f8b9ddbd5978588baf1ab649e77ad037fd7544091`
from local test commit `bdf5b6881ed60cb7d05914e012e4c33c02001423`
(remote test increment `eb92c9d81d95300df9c90074d0def8cf02d62628`). Existing
profile, boundary, diagnostic and Windows UI helpers and the GPX fixture are
byte-identical to that test worktree. Evidence records all dependency hashes and
both identities separately: test_source_commit identifies the orchestration;
candidate_commit and each live process build_commit identify the frozen product.
Both live diagnostics must equal the requested full candidate SHA. The complete packaged file hash map is retained in candidate-binding.json. All
packaged runtime hashes are checked again after the test. API tokens are removed from the
test process environment.

The evidence scope is two concurrent XNav instances with independent newly
created disconnected profiles: no process-owned TCP listeners on 8443/8444,
credential/certificate preservation, navigation preservation, no seeded secrets
in observed application output, and clean exits. This does not replace the
existing all-mode/CLI/source contracts, packet capture, installer lifecycle,
boat validation, or release acceptance. Linux is a separate development route:
the harness can use an exact installed Linux runtime plus its generated config,
but the current workflow does not retain such a Linux runtime artifact. This
Windows workflow makes no Linux availability or qualification claim.

Preparation validation: six inert unittest methods cover the complete runtime
and version adapter, candidate/payload/runtime mismatches, marker rejection,
ambiguous metadata, path traversal/Windows special paths and case aliases.
Eligibility checks cover both permitted paths and 12 rejected failure, missing
prerequisite, stale receipt/attempt and identity scenarios.
A static extraction check against retained historical candidate
`79a95c4f39063c20ca4d5a98c3147080d9813077` verified all 950 package file hashes
and recovery counterparts without running an executable. Its Setup SHA-256 was
`b281362f6bf6d51410aba1491e16049d4c7bf243f6602a2bd7a3d0342ab09c4d`,
embedded package SHA-256
`c6c1c229c474631e108ce3fb28d34764a2c77e72222d31cffa25c92d02ac8ffe`,
and payload SHA-256
`aa8cc41bc06cde46dcdc828daa8f9fd56a17d9c103ec4bc25a43070a7e75bc30`.
This demonstrates the archive layout only; it is not evidence for frozen
candidate 0e9ec666. Native 7-Zip extraction and the actual current two-instance
run remain pending.
