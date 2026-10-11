# Repository secret scanning (SCRUM-73)

`python opennav-x/tools/secret_scanning.py --repo .` is the explicit checkout
qualification gate. Exit 0 means no findings in the documented scope; exit 1
means findings require review; exit 2 means scanning could not complete. No
failure is reclassified as a clean result. `--report-only` is an explicit audit
mode and does not qualify the checkout.

The scanner is [Gitleaks 8.30.1](https://github.com/gitleaks/gitleaks/releases/tag/v8.30.1).
`tools/secret-scanner.lock.json` pins the Linux and Windows x64 release archives
by exact size and SHA-256. Both digests were compared with the official release
asset metadata and checksum file. The runner verifies bytes before extracting
only the scanner executable, verifies its version, and runs inert PAT/private-key
self-checks before inspecting the repository. An offline archive may be supplied
with `--scanner-archive`; the same pin is mandatory.

Every tracked checkout file is copied into a private, temporary snapshot under
the ignored product `.local` directory and scanned under an opaque filename.
This prevents source filename/path allowlists, `.gitignore`, `.gitleaksignore`,
source-provided scanner configuration and inline `gitleaks:allow` markers from
hiding a credential. Symlink text is scanned without following its target.
The pinned scanner's complete built-in rule set and value heuristics remain;
there are no repository exclusions or accepted-finding baseline. Its default
path exclusions cannot match the opaque `.scan` filenames.

Only original tracked paths and rule IDs are retained per finding. The scanner
report template contains no match, secret, source line, author or fingerprint.
Its stdout/stderr are captured and discarded, even on failure; the runner emits
fixed safe errors. Temporary reports, scanner and snapshot are removed on exit.
The runner makes no credential-validity requests, does not contact signing
services, and does not generate or change a real key.

## Current CI bridge and qualification boundary

The existing Staging `ci_changes.py --github-output ...` entrypoint runs the
self-checks and a **report-only** audit before selecting product/helper checks.
Its single JSON result includes `secretScan.status`, `qualificationPassed`, and
all path/rule findings. Scanner availability, checksum, self-check, snapshot or
report failures stop selection without writing dependency outputs. Historical
findings do not silently become accepted exclusions: they are reported as
`unreviewed-findings`, with `qualificationPassed: false`.

The initial scan of `113d2175` produced 228 generic-api-key matches before path
normalization. Digest-only structural review found checksum values under public
screenshot/source/prototype-token property names, including `docs/evidence/**`
and chart-symbol provenance records. This explains observed false positives;
it is not a blanket clearance of these directories or of future findings. No
whole evidence/test folder or generic rule has been disabled. Until individual
findings are curated, the required clean gate remains the explicit command
above; the report-only Staging bridge must not be cited as a passed release gate.

The current Git credential cannot push `.github/workflows` changes. This slice
therefore changes no workflow and does not add PR, arbitrary-branch, scheduled
or history scanning. Existing Staging trigger/path filters still govern whether
the bridge runs. Required-check enforcement needs a later authorized workflow
change and review of the historical matches. SCRUM-73 stays open.

## Scope and follow-up controls

The audit covers current tracked bytes, including modified tracked files. It
does not inspect untracked files, Git history/deleted secrets, CI protected
storage, runner logs, remote services, gitlink contents or credentials already
on the boat. It does not unpack nested archives. Detectors are heuristic; a
clean checkout is not proof that no secret exists. Submodule/gitlink paths are
listed separately in `gitlinksNotScanned`.

Before enabling a required CI check, investigate each finding privately and
record its path, rule, disposition and reviewer without copying its value into
Jira, logs or evidence. Confirmed secrets require authorized revocation/rotation,
access review and incident auditing; deleting their current checkout occurrence
is insufficient. A future fixture/public-root exclusion must identify exact
reviewed evidence and continue detecting a different credential in that file.
The external secret-store inventory, access owners, rotation schedule and audit
retention are not configured by this software-only slice and remain SCRUM-73
acceptance work. No web/payment or real signing configuration was changed.

Focused verification: `python opennav-x/tools/test-secret-scanning.py` exercises
the actual pinned scanner and failure/redaction/CI behavior;
`python opennav-x/tools/test-ci-changes.py` retains selector output contracts.
The test suite downloads the pinned scanner once (about 8.2 MB on Linux).
The CI bridge likewise downloads it once per selector invocation; its scan
self-check is bounded and no application compilation is started. The optional
offline archive avoids the download but does not bypass verification.

Local measurement on the isolated `113d2175` checkout: 7,323 tracked files
(244.8 MB), 30.76 seconds including download/self-check, and 179 unique
path/rule findings after opaque-name scanning. All used `generic-api-key`.
This is a local measurement, not a hosted-runner timing guarantee.
