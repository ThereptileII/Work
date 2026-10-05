# Exact d29da37 restart qualification prerequisite

The unchanged original [artifact 11296187896](https://github.com/ThereptileII/Work/actions/runs/37184477492/artifacts/11296187896), `commissioning-restart-qualified-d29da372af86c01cb2817f932891fd9408d882fe`, is retained in `original-artifact.zip`: **334 bytes**, SHA-256 **c0a38f6f154071bf3ee59cd7bc54a01360e8abd0d10ead0e8afe22181390f47c**. Its authenticated API digest, producer upload-log digest/ID/size and local bytes agree. ZIP inventory is exactly one regular, unencrypted `qualified.json`, without unsafe paths or duplicates; its CRC passes. The extracted **301-byte** receipt is preserved unchanged, including CRLF; SHA-256 `43b22af2c1f7c11347c4657841ff7f6e5c48af38a69de4af5bfff6168a8743fc`.

Authenticated run, attempt-specific jobs and artifact records bind candidate **d29da372af86c01cb2817f932891fd9408d882fe**, [run 37184477492](https://github.com/ThereptileII/Work/actions/runs/37184477492), attempt **1**. Completed prerequisite jobs:

- Native Win32 transport/process boundary: **111383445862**, success.
- Native disposable maintenance/navigation-copy contracts: **111383445822**, success.
- Actual broker and Prepare/Arm/Collect with marker-only processes: **111383887013**, success. Broker, Prepare/Arm, evidence upload, attestation and qualification upload steps all succeeded.

The original broker log independently confirms exact checkout, all four success inputs, and qualification upload at **2026-10-04 07:06:05 UTC**. Numbered selected lines are in `broker-excerpts.log`. Its complete connector-decoded original remains in this worktree's ignored `.local/d29da37-audit/broker-original.log`: **26,468 bytes**, SHA-256 `62c6d34fd175019fca0e53c6ccea05a1e0aa2a67d5b2fff17be40d4d69e12328`.

Receipt schema **1**, owner **OpenNavX.CI.RestartGates.1**, exact commit/run/attempt, `transport/maintenance/broker/prepareArm: success`, and **`actualBoat: false`** all passed independent inspection. The frozen workflow emits the receipt after these successes and requires its exact identity before later product delivery. Its remotely retrieved bytes equal local frozen **615118f351abdea7b79a042db63e58fe0a635e0a** (workflow SHA-256 `f52c3e7a7a350cd09a1cc67b1b47ae36bbcd6b7be59c14d864b515d91d35898e`). Selected API records and measured archive identities are in `api-and-audit.json`.

This is the same-run prerequisite for later package eligibility review. **It does not qualify application or installer bytes, real boat restart, boat launch, or overall release acceptance.** The run was still in progress at collection. Individual broker/maintenance archive case counts are not claimed; those larger archives were not fetched for this bounded receipt audit. No tests, builds, reruns, or boat actions occurred.
