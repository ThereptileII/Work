# SCRUM-290 — Staging delivery / explicit Production

2026-10-04. This record qualifies delivery tooling only; it does not qualify a
new SKAGER application, installed boat candidate or public release.

## Focused local checks

| Suite | Cases | Result |
|---|---:|---|
| Immutable release inventory/source/package identity | 15 | PASS |
| GitHub draft transport, permission boundaries and attempt provenance | 16 | PASS |
| Staging/Production qualification and workflow policy | 15 | PASS |
| Retained installer inputs and functional chart visibility | 15 | PASS |
| Existing installer completion behavior | 12 | PASS |
| Total | 73 | PASS |

All are offline deterministic helper checks with inert fixtures. Edited workflow
files pass actionlint 1.7.12; all workflow YAML passes duplicate-key rejection.
Changed Linux shell scripts pass syntax checking. Existing application/installer
functional assertions remain, while exact prototype/layout/color review is
explicitly scheduled. Functional chart checks still reject absent coastline and
interrupted route strokes. Those altered native interaction paths require their
next native candidate execution; local fixture tests do not imply native GUI
acceptance.

`skager-delivery-checks.yml` runs the five focused suites on Linux and native
Windows and parses changed PowerShell scripts. It does not compile OpenCPN,
launch a UI, contact the boat, or create a product release.

## Preserved boundaries

- Default Staging; no automatic Production qualification/promotion.
- Versioned GitHub draft Releases in both channels; no public-opening authority.
- Named immutable source, package bytes, hashes and run-attempt identity.
- Production's full installer/recovery/functional work consumes retained bytes.
- Design review is not requested by default and never represented as passed.
- Endurance remains skipped by prior explicit user direction.
- No installer payload, application code, chart resources or boat configuration
  changed in this process increment.

The first real new-format Staging delivery and a user-authorized Production
promotion remain execution gates. Permission/default-branch setup and expired
90-day retest support fail closed. Cross-run dependency reuse remains SCRUM-225;
this work does not claim that native compilation itself is now fast.
