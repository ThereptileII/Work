# Hand off the exact tested Beta 2 package

The application qualification run creates immutable candidate payloads. Boat
review necessarily follows that build. Rebuilding merely to change a release
label would create a different executable and invalidate its boat hash evidence.

`opennav-beta2-handoff.yml` therefore provides a separate final handoff. It
downloads and republishes the **unchanged** complete `beta-candidate-<commit>`
artifact only after a committed, sanitized `release/beta2-acceptance.json`
records actual boat acceptance. The record is deliberately absent while review
is unfinished. This is a Beta test distribution, not production approval.

The collector checks all sixteen required jobs from the exact successful run
attempt, its product commit, artifact name/ID/API digest/size, downloaded ZIP
integrity, exact six-file checksum manifest, fixture-free product identity,
executable hash and corresponding-source revision. An early development bundle,
skipped/failed/unfinished gate, changed payload, duplicate or unexpected archive
entry, missing boat gate or changed evidence document is refused. It executes
no installer, application, source archive or boat command. The GitHub token is
sent only to the API; it is not forwarded to artifact storage.

The acceptance record binds separate public evidence for feedback, primary
screens, real charts/data, mode lifecycle, maintenance, remote recovery and old
version retirement. It explicitly lists limitations and zero physical actuator
commands. A passed field records the scope actually reviewed, never unavailable
hardware or an unperformed physical touch test. Required unresolved defects
prevent acceptance; conditional hardware observations remain clearly identified.

The published `SKAGER-Beta2-Windows` artifact retains the original six payloads,
`SHA256SUMS.txt` and build-time `QUALIFICATION.txt`. It adds the later acceptance
record and a short explanation. The original application and source commit stay
unchanged; the handoff workflow has its own separately reported commit/run.
This avoids claiming that a later documentation commit is the tested executable.

Eighteen offline test groups exercise accepted bytes and refusals for missing
reviews, evidence traversal/change, wrong stage/control/identity, incomplete
runs, missing/duplicate/skipped gates, different attempts, early/expired/replaced
artifacts, corruption, archive traversal/duplicates, checksum mismatches,
fixture-enabled products, incorrect corresponding source, and missing or changed
release notes. The notes must match the outer reviewed copy, recovery copy and
corresponding-source copy, including both inner hash inventories. These tests
qualify the collector only. Real CI and boat acceptance remain mandatory.


`SKAGER-Beta2-Release-Notes.md` is the sixth payload. It describes SKAGER Beta 2
as an evaluation candidate with acceptance pending. Recovery packaging copies it
from the same committed `docs/beta2` source and records it in FILE_SHA256.json;
the installer includes those recovery docs in its hashed payload automatically.
The corresponding-source archive retains the committed document and its source
inventory hash. Both candidate and pending-endurance artifact sets include it
in SHA256SUMS. Adding this deliverable does not enable a named release or satisfy
any CI, boat, source-compliance or public-access gate.

SCRUM-236 changes new product filenames to `SKAGER-Beta2`. The collector still
accepts one complete historical `OpenNavX-Beta2` bundle without changing its
filenames or bytes; mixed old/new payload names are refused. Archive roots and
release-note paths follow the recorded complete bundle, with the same exact
hash, source, CI and boat-review checks in both cases.
