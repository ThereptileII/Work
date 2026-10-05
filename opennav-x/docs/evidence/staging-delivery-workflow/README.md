# SCRUM-290 — Staging delivery / explicit Production

2026-10-04. This record qualifies delivery tooling only; it does not qualify a
new SKAGER application, installed boat candidate or public release.

## Focused local checks

| Suite | Cases | Result |
|---|---:|---|
| Immutable release inventory/source/package identity | 16 | PASS |
| GitHub draft transport, permission boundaries and attempt provenance | 16 | PASS |
| Staging/Production qualification and workflow policy | 16 | PASS |
| Retained installer inputs and functional chart visibility | 15 | PASS |
| Existing installer completion behavior | 12 | PASS |
| Total | 75 | PASS |

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
promotion remain execution gates. Missing release permissions and expired
90-day retest support fail closed. Cross-run dependency reuse remains SCRUM-225;
this work does not claim that native compilation itself is now fast.

## Native verification history

The first focused run, [37221707154](https://github.com/ThereptileII/Work/actions/runs/37221707154), failed. The local and published monorepo workflow paths differed; Windows also normalizes ZIP member separators and disallows reserved filesystem names. The correction preserves raw ZIP names, rejects normalized names before extraction, and fixes test fixtures without dropping malicious-input cases. The replacement is `311ddabafde937ab7764ba1c396730ac2d3e4648`, [run 37221989304](https://github.com/ThereptileII/Work/actions/runs/37221989304). No application rebuild was performed for either focused run.

The replacement passes **75 cases on Linux and 75 on native Windows**, plus
PowerShell syntax checks. [Native CI receipt](native-ci.json) records the exact
jobs and commit. This does not qualify the full installed lifecycle or a new
application package. Subsequent documentation/registration commits retain these
identical tested helper/workflow bytes; no replacement application was built.

Default-branch registration is `f05ded1741a49c96338142dc658232fde3ae7344`:
29 identical software workflow files were added so manual dispatch is available.
Every pre-existing main file's Git blob/mode was compared and preserved. The
registration used `[skip ci]`, requests no release or application execution, and
keeps firmware/main separate from software development on `staging`.
