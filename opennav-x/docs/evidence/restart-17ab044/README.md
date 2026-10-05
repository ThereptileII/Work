# Exact 17ab044 restart qualification prerequisite

The original receipt artifact for [run 37164360050](https://github.com/ThereptileII/Work/actions/runs/37164360050),
attempt 1, is retained unchanged in `original-artifact.zip`:
`commissioning-restart-qualified-17ab044a1e5222dc71791ac8118454219efe8734`,
artifact **11289061547**, **337 bytes**, SHA-256
`f839dd1d762a84349cc786445a3827c32fd5456b962e0b403b57378f059c849a`.
Its authenticated digest, single entry and CRC were independently verified.
`qualified.json` preserves the original receipt bytes.

The authenticated run and artifact records bind the exact commit, push workflow
and attempt. The three completed native jobs are transport **111324138088**,
maintenance **111324138098**, and broker/Prepare-Arm **111324546100**. The latter
job's actual broker and Prepare-Arm steps, evidence upload and attestation all
passed. The frozen workflow emits this receipt only after all four success
conditions. Receipt schema, owner, commit, run, attempt, all four success values
and `actualBoat: false` were checked; selected original API fields and findings
are in `api-and-audit.json`.

This closes acquisition of a required same-run prerequisite for later package
review. It does not bind or qualify the still-unavailable application/installer
payload, prove a real boat restart, authorize a boat launch, or establish chart,
TLS, endurance or release acceptance. The receipt must still match the eventual
package's exact producer run/attempt. No test was rerun and no boat action occurred.

The first local transfer using Python urllib received HTTP403/1010; ordinary
curl downloaded the tool-provided original archive successfully. No alternate
artifact, receipt reconstruction or expected-hash change was used.
