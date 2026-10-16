# SCRUM-22: retained 854 installer and charts suffix passes

[Run 38882281891 / attempt 1](https://github.com/ThereptileII/Work/actions/runs/38882281891/attempts/1),
job `116911623959`, passed against unchanged product
`854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`. The helper is
`3fe8866fa191a72eb47290368311cbed11463664`; it corrects the installer Finish
checkbox handling. The application was not rebuilt. The original producer
`38765418126/1` and its failed qualification job `116562831922` remain unchanged.

The actual wizard recorded **Open SKAGER: checked before, unchecked after**.
All nine installer checks passed, including unsupported-input preservation,
unmanaged legacy TLS refusal, conventional wizard completion, exact candidate
clean install and first-install rollback. This establishes the corrected helper's
explicit launch choice; it does not retroactively identify the earlier lock holder.

All four real packaged-updater checks passed:

- Installed launcher bootstrap produced an exact authenticated health receipt.
- The same-version Setup published a pending candidate; real startup authenticated
  and finalized one attempt.
- An invalid-PE candidate was refused before process creation; guarded rollback
  restored the known-good generation without profile changes.
- The restored generation launched and closed cleanly with a retained
  authenticated receipt.

The original selected legacy trust identity was retained. This is same-version
installation/startup mechanics evidence. Protected B0/B1 and signed-offer
acceptance remain pending. The separate disposable charts fixture passed its
software and requested-OpenGL phases; the host rejected OpenGL and the upstream
software fallback was verified. Physical GPU acceptance remains open. These
fixture results are not relabeled as final fixture-free application evidence.

Original artifact `12039410527` is 15,078,572 bytes, SHA-256
`24453d32db0c72f1d6c18c9c7e0e27d964faebb626f2aed591d1b719c6c1b824`.
All 1,997 member CRCs passed. [receipt.json](receipt.json) binds the original API,
log and five reports, exact helper hashes, original Setup/source/recovery bytes,
final application identity, and fault/restoration results. Each original was
collected once; no screenshots, raw logs, credentials or private paths are added.

The [original custody](../candidate-854078a5-custody-20261016/README.md),
[preserved wizard diagnosis](../scrum22-wizard-launch-20261016/README.md) and
[retained PluginHandler pass](../scrum211-retained-854-20261016/README.md) remain
separate evidence. This suffix result permits reviewed composition with the
original prefix; it does not itself grant overall release or boat acceptance.
