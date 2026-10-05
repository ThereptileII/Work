# Exact ecf7e0c restart qualification prerequisite

The unchanged original [artifact 11298888538](https://github.com/ThereptileII/Work/actions/runs/37191400051/artifacts/11298888538), `commissioning-restart-qualified-ecf7e0c46609cf4cb29141964d7c4bde98b002b7`, is retained in `original-artifact.zip`: **333 bytes**, SHA-256 **de93c30393b4516b31ca1056ba6b65c22d4eb88516704871969d5b8e55ba4c95**. Its authenticated API digest, producer upload-log digest/ID/size and local bytes agree. ZIP inventory is exactly one regular, unencrypted `qualified.json`, without unsafe paths or duplicates; its CRC passes. The extracted **301-byte** receipt is preserved unchanged, including CRLF; SHA-256 `f9409b59dae01ff4129dafafa6539d43018badcb8d2bfe88553e43131706b228`.

Authenticated run, attempt-specific jobs and artifact records bind candidate **ecf7e0c46609cf4cb29141964d7c4bde98b002b7**, [run 37191400051](https://github.com/ThereptileII/Work/actions/runs/37191400051), attempt **1**. Completed prerequisite jobs:

- Native Win32 transport/process boundary: **111404167754**, success.
- Native disposable maintenance/navigation-copy contracts: **111404167493**, success.
- Actual broker and Prepare/Arm/Collect with marker-only processes: **111404638940**, success. Broker, Prepare/Arm, evidence upload, attestation and qualification upload steps all succeeded.

The original broker log independently confirms exact checkout, all four success inputs, and qualification upload at **2026-10-04 09:18:06 UTC**. Numbered selected lines are in `broker-excerpts.log`. Its complete connector-decoded original remains in this worktree's ignored `.local/ecf7e0c-audit/broker-original.log`: **26,530 bytes**, SHA-256 `ac81714b0008c296eb71acc6aeaeff16842d53c7f3ac60a5fa7ce089424993ae`.

Receipt schema **1**, owner **OpenNavX.CI.RestartGates.1**, exact commit/run/attempt, `transport/maintenance/broker/prepareArm: success`, and **`actualBoat: false`** all passed independent inspection. The frozen workflow emits the receipt after these successes and requires its exact identity before later product delivery. Its remotely retrieved bytes equal local frozen **1988df7a8a0ae8ddc6365ca46026d32fccfa0bdc** (workflow SHA-256 `f52c3e7a7a350cd09a1cc67b1b47ae36bbcd6b7be59c14d864b515d91d35898e`). Selected API records and measured archive identities are in `api-and-audit.json`.

This is the same-run prerequisite for later package eligibility review. **It does not qualify application or installer bytes, real boat restart, boat launch, or overall release acceptance.** The run was still in progress at collection. Individual broker/maintenance archive case counts are not claimed; those larger archives were not fetched for this bounded receipt audit. No tests, builds, reruns, or boat actions occurred.
