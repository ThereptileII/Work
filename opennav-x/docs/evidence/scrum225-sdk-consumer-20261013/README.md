# Exact SDK consumer correction — SCRUM-225

Scope: repair the pre-compilation SDK refusal in candidate `967617249ef07c27b7ba77499339970ff2735190`,
[run 38386718400 / Windows job 115266541449](https://github.com/ThereptileII/Work/actions/runs/38386718400/job/115266541449).
Its original failed evidence remains unchanged. This correction is based on
checkout `8c593d136ec9c25a5e936677af4f6d7db85cea82`; it does not build libraries
or an application, grant native reuse acceptance, or close application/boat gates.

## Authenticated original SDK

Live GitHub API reads establish successful producer `49f9f07407cbf640f6186757a3fe3582c114b895`,
[run 37824634218](https://github.com/ThereptileII/Work/actions/runs/37824634218),
attempt 1, successful `windows-dependencies` job `113474187733`, and unexpired
[artifact 11574361322](https://github.com/ThereptileII/Work/actions/runs/37824634218/artifacts/11574361322).
The committed selection remains unchanged. Downloaded ZIP SHA-256 matches the
authenticated digest `4cb9996c170f067850ffda07e5c9eb3906e73a21bd4c462893b19fc32775680d`.

The [original manifest](original-sdk-bundle.json) is retained byte-for-byte:
SHA-256 `1f80abd2a2877037c34fae4d0aa31a715beb867420392cb9cfaf63ee351e857a`.
All 295 declared payload files (117,866,926 bytes) match their individual sizes
and hashes. ZIP membership has no duplicates, case collisions, linked/special
members, unsafe paths or undeclared files. All 58 derived payload roots match.
The original input fingerprint and toolchain digest recompute exactly.

The [audit](authenticated-sdk-audit.json) compares **every** declared source
input, plus the producer workflow, with authenticated payload bytes and the exact
producer Git tree. Explicit source checkout encodings explain LF/CRLF only;
payload, manifest and binary bytes are never rewritten. Of these 30 inputs,
only two pre-correction source deltas exist:

- `build-pristine-windows.ps1` adds installed wx DLL provenance after application
  compilation/install. Its dependency-only path seals and returns beforehand.
- `windows_dependency_reuse.py` adds route-focus and route-hit-dpi application
  patch entries to the same-job receipt inventory. The dependency-only producer
  skips application preparation; the complete same-job inventory remains intact.

Library source locks, compiler recipes/options, producer workflow, notices,
native tool-fact helpers, ABI and all other declared inputs are unchanged.

## Bounded correction and local checks

The verifier retains a closed two-producer exception set. Each compatibility
entry must match its own producer and every original/current helper SHA-256;
another approved producer's hashes cannot substitute. There are separate exact
LF/CRLF source pairs. Historical orchestration/receipt-helper hash pairs remain
unchanged; only their current verifier hash is updated for the reviewed verifier
change. No source, recipe, native-tool, inventory or runner checks are exempted.

Against the actual authenticated manifest, the exact current CRLF helper pair
passes the offline compatibility function. Its complete corrected fingerprint
equals the immutable original `30cd19cf526789cc1833c28313d595f41a30d0d8d1ca04e3ded1be6f362e865f`.
Changing any of the three helpers, selecting an unknown producer or mixing
source-checkout encodings refuses. The original ZIP remains unchanged.

[Bundle contracts](bundle-tests.log): 30 Linux tests pass, including original
and selected-producer restoration, retained original manifest/payload bytes,
changed original/current hashes, helper tampering, cross-producer exceptions,
unknown/malformed producer entries, ambiguous duplicate entries and unchanged
recipe/toolchain/inventory refusal tests.
[Same-job receipt checks](same-job-reuse-tests.log): all 13 Linux tests pass.

Native verification remains **pending**. Offline package validators correctly
refuse Linux LF helper bytes against recorded Windows CRLF provenance; running
the original source bytes instead reaches Windows absolute-tool path checks,
which Linux cannot qualify. These refusals are recorded in the audit; no
validator, manifest, path or hash was changed to obtain an offline pass.
The coordinator must review and run dependency-only native restore/reprobes
against this selected SDK before any coordinated application build. Application,
installed runtime/import closure, TLS, installer/updater, corresponding-source,
release and boat acceptance remain separate required gates. Design review is
not requested and endurance remains skipped.
