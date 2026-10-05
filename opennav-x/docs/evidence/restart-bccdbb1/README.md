# Exact bccdbb1 restart qualification prerequisite

The unchanged original [artifact 11293278829](https://github.com/ThereptileII/Work/actions/runs/37177738716/artifacts/11293278829), `commissioning-restart-qualified-bccdbb11cef827d3b63731fe1e874bc72bf47d2d`, is retained in `original-artifact.zip`: **336 bytes**, SHA-256 **dfb40182835244d9fedd4e7cfd5b93740e79a1885bbedc72b3209cee75b5d8f2**. Its authenticated artifact digest, producer upload-log digest/ID/size, local archive digest, single-entry inventory and CRC all agree. `qualified.json` preserves the original receipt bytes, including CRLF.

The authenticated run, artifact and jobs records bind remote commit **bccdbb11cef827d3b63731fe1e874bc72bf47d2d**, [run 37177738716](https://github.com/ThereptileII/Work/actions/runs/37177738716), attempt **1**. These native jobs completed successfully:

- Transport/process boundary: **111363797693**.
- Disposable maintenance/navigation-copy contracts: **111363797680**.
- Actual broker and Prepare/Arm/Collect against marker-only processes: **111364202746**; broker step, Prepare/Arm step, evidence upload, attestation and receipt upload all succeeded.

The original broker log confirms exact checkout, all four success inputs, and the qualified upload at **2026-10-04 04:47:00 UTC**. Selected original lines are in `broker-excerpts.log`; selected authenticated records and full decoded-log identity are in `api-and-audit.json`. This audit verifies the same-run receipt and completed prerequisite jobs, not individual runtime case counts hidden in the separate broker/maintenance archives.

Receipt schema/owner, exact commit/run/attempt, transport/maintenance/broker/prepareArm values and **`actualBoat: false`** all passed independent inspection. The frozen workflow emits this receipt only after those four successes and checks its exact identity before later product delivery. Remote workflow bytes match local frozen candidate `1a6733a1cbcc817aa0f13fa5acc41aac62a54146`; SHA-256 `f52c3e7a7a350cd09a1cc67b1b47ae36bbcd6b7be59c14d864b515d91d35898e`.

This supplies the required same-run restart prerequisite for later package review. **It does not qualify an application/installer payload or a real boat restart, authorize boat launch, or establish overall candidate/release acceptance.** The complete run was still in progress at collection. Eventual package identity must match this exact producer run/attempt. No tests were rerun and no boat operation occurred.
