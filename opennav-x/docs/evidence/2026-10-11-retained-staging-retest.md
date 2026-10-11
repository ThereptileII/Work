# Retained native Staging retest — 2026-10-11

- [Windows retest run 38140837201](https://github.com/ThereptileII/Work/actions/runs/38140837201): completed successfully.
- Producer: `ThereptileII/Work`, product commit `5dd795be16081b4b0e521315bfc45f23a9af8251`, Windows-2022 Win32 build job in [run 38127571851](https://github.com/ThereptileII/Work/actions/runs/38127571851), attempt 1.
- Retest harness commit: `efec7a4e19d8e36298f4d23db175540b7b2ede10`. The authenticated producer receipt and archive restore were checked before execution; no compiler or new product download was used by the selected checks.
- Retained executable SHA-256: `7c8f918f6290727c76dcfebed4e87982d969ca98bc0fafde154d51fb6e575e82`.
- Evidence artifact ID `11703675521`: `staging-retest-evidence-38127571851-attempt1-harnessefec7a4e19d8e36298f4d23db175540b7b2ede10-run38140837201-attempt1`.
- `staging-retest.json` reports `status=passed`, `scope=all`, `checks=[all:passed]`. `windows-production-tests.xml` and `windows-xnav-tests.xml` each report 144 tests, zero failures and zero skips. The concurrent ordinary/portable `peer-two-instances-results.json` reports `PASS`. The retained installer, packaged-updater and extracted recovery summaries report passed checks. The chart summary reports deterministic native rendering checks passed; physical GPU and screenshot acceptance remain separate.

This retest resolves the harness interruptions from runs 38139852479 and
38140234560 for the **same compiled product bytes**. It does not turn the
original failed workflow green, publish a Staging release, validate later source
changes, or qualify legal/compliance and boat acceptance. The boat installation
was not changed by this retest.
