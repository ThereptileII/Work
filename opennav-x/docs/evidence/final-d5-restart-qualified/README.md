# Final candidate early restart receipt audit

The exact same-run restart gate **passes this bounded independent artifact audit**.
This supports development-only package review; it is not full release or boat acceptance.

- Local source: `d5d71356d806ea8c3518644d10728a24f1334d1d`.
- Remote candidate: `b8cfbf809450208f723095ffb4e00d7800b619a5`.
- [Run 37114216075](https://github.com/ThereptileII/Work/actions/runs/37114216075), attempt 1.
- [Artifact 11271282491](https://github.com/ThereptileII/Work/actions/runs/37114216075/artifacts/11271282491): `commissioning-restart-qualified-b8cfbf809450208f723095ffb4e00d7800b619a5`, 335 bytes.
- Downloaded ZIP SHA256: `f3b01f48c1293aab3bfb49c8c398f09e8d4f91540ff3a07d4e33875ffd8a43aa`, independently matches GitHub's artifact digest.
- The ZIP has exactly one member, `qualified.json`; CRC passes. Original ZIP and receipt bytes are retained.

The receipt has schema 1, owner `OpenNavX.CI.RestartGates.1`, the exact candidate/run/attempt, four success values, and Boolean `actualBoat:false`.
The matching API results independently show completed/success for native transport job 111177754640, maintenance job 111177754837 and broker/Prepare-Arm job 111178161834, including every step. Run metadata independently confirms attempt 1. Only those three job records are retained in `api-evidence.json`.

The frozen workflow at `.github/workflows/opennav-baseline.yml:298` creates this receipt only after checking transport/maintenance job results, broker/Prepare-Arm step conclusions and the preceding evidence upload. Its later product gate rechecks exact receipt identity. No associated broad integration bundle was needed for this bounded decision.

`verification.json` records the audit, workflow hash, collection interruptions and limits. The signed download URL and unrelated API data are omitted. The artifact contains no native executable, so this audit makes no native binary rehash claim. Product hashes, final native UI, endurance and physical boat acceptance remain separate. No source changes, test reruns, builds, application launches, deployment or boat actions occurred.
