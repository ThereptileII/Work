# SCRUM-311 atomic history and original commissioning custody

Integrated source: `8ee7e6338001384d9b1c037e93003d6b67271286`.
History `1b193639` (isolated `b8dd1912`), reader `7ad64ff3`
(isolated `30d99bd7`), producer `86c39897` (isolated `78cf19f4`).

The actual native broker adapter now calls `MaterializeCandidateReadOnlyFresh`.
Publication, distinct read-only file-object transfer and exact live sibling
adoption occur under the Store lease. Existing retained owners cannot observe
an intermediate namespace; unknown entries refuse and uncertain publication
poisons affected owners while preserving original/partial evidence.

Recovery retains the exact twelve `installer/commissioning-v1` source originals,
independently of the existing exact three engine helpers. Current producer copies
and pins them before NSIS and checks them afterward and before publication.
Readers/importers validate the complete reserved namespace and original bytes.
Historical zero closure remains inert compatibility data; partial/unknown/alias
sets do not become acceptable. No NSIS recipe/embedded engine change was needed.
Fixed corresponding-source requirements include all production readonly,
baseline and history modules.

## Results

- Joined Linux race: **20 top-level + 123 nested passed**, no failure/skip.
- Isolated reader race: **59 top-level + 272 nested passed**, no final failure/skip.
  An initial fixture-only integer compile error and corrected verifier rerun are
  retained; the successful leaf/store suites were not repeated needlessly.
- Producer: **35 Python tests passed**. Fixed source package: **12 tests passed**.
- Exact joined Windows/x86 test component compiled once.
- Native boat Windows 10.0.22631 / PowerShell 5.1: **6 top-level + 19 nested
  passed**, no failure/skip. Discovery, exact binary/archive/runner hashes,
  bounded execution, retrieval hashes and raw output are retained.
- Test PE SHA-256:
  `c99060dc341530323c7bef15e5f7a7c4133324d8f2aa55152c0f5ec3988c0e08`.
- Installed OpenCPN PID1852 stayed responsive; no fixture process remained.

## Limits

These checks used only fresh temporary native filesystem fixtures. No installed
product/profile, service, application selection, network session, physical output
or boat configuration changed. The retained commissioning closure does **not**
execute commissioning or generalize the existing COM8/profile-specific policy.
Actual protected selected/pending execution, authenticated interactive launch,
one-use spawn, healthy commit and previous restoration remain incomplete.
This is not full MSVC/application, installer lifecycle or signed update acceptance.
No full application build, design review or endurance run was requested.

`SHA256SUMS.txt` binds original retained logs/receipts/collector; binaries and
transfer/result archives remain local and are deliberately not committed.
