# SCRUM-235/236 final visible-copy audit

Base: `d4ee8d61e005b6e9912af8a78c226e279079eed3`.
Selection and scope: SCRUM-236 comment 10684. All nine SCRUM-89 comments and
attachment 10000 metadata were re-read; the final SKAGER/APP wordmark approval
and compatibility-preservation instruction remain authoritative.

The audit traced quoted production UI strings and integration patch additions,
current `docs/beta2` package guides, recovery launchers, installer/maintenance
captions and branding resources. No remaining normal product UI, current guide,
launcher or current-generation installer caption used the old public name.

Fifteen definite peripheral phrases were corrected:

- `tools/package-ais-live-probe.py`: packaged diagnostic README's CI-recipe name.
- `tools/boat/ReviewWindowNative.cs`: three review/resize refusal messages.
- `tools/boat/RestartWindowNative.cs`: Legacy/Safe return-action refusal text.
- `tools/boat/Preparation.ps1`: process-close instruction.
- `tools/boat/StockReview.ps1`: four integration/registration/restart refusal messages.
- `tools/boat/smoke-test.ps1`: unresponsive-window message.
- `tools/boat/start-official-upgrade.ps1`, `upgrade-official-opencpn.ps1` and
  `upgrade-stock.ps1`: close/running instructions.
- `tools/boat/retire-portable.ps1`: generic portable-release refusal, without
  misnaming older accepted releases as newly branded ones.

Intentional retained matches:

- `BrandedSurfaceTitle` translates old stable drawer/floating-window selectors
  to SKAGER captions. SetName/layout/automation selectors remain stable.
- `Lifecycle.ps1` can restore historical-generation captions and shortcuts on
  rollback. Its new-generation branch uses SKAGER. Registry/storage ownership
  keys, credential target, protocol/config values and persisted chart-style
  value remain unchanged.
- `RetirementPolicy.ps1` explicitly describes its historical OpenNav ZIP
  filename allowlist. That policy still refuses unrelated/new filenames.
- Historical anchor descriptions remain read-compatible; new descriptions use
  SKAGER. Original OpenCPN/legal notices, source-package repository paths and
  immutable prototype/evidence files are unchanged.

Focused checks: all ten edited files were compared to the base, proving bytes
outside the intended 15 quoted copy phrases unchanged; PowerShell syntax parsed
for seven scripts; Python packaging source compiled; approved source/wordmark/
ICO verification passed; diff whitespace check passed. No application rebuild,
broad tests, CI dispatch, native execution or boat operation ran.

Existing branding checks validate approved asset bytes and native PE resources,
not arbitrary README/operator error prose. The runtime tests retain internal
selectors, so they would not automatically detect these copy remnants. This
read-only trace plus bounded literal comparison covers that gap for this
revision; it does not claim future strings cannot regress.

Changed operator scripts have new source hashes. Existing qualified/staged boat
tool receipts remain evidence for their exact older bytes; this audit does not
replace the required qualification/staging closure for a later deployment.
