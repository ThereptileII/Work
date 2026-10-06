# Exact prepackage recovery

`skager-compiled-recovery.yml` handles only product commit
`0db45cb92509c29ef4d1fc33d0c76dc7cf511b97`, original baseline run
`37424595899`, attempt 1. The original Windows compile/security/unit gates passed;
the job failed after NSIS produced Setup when the PE verifier rejected NSIS's
fixed-version structure convention. Native desktop qualification never ran. The
failed job and the prepackage archive remain unqualified historical evidence.

The separate authenticated restore helper checks the fixed original artifact
identities, successful prerequisite steps and exact failure boundary. It compares
every retained tracked source entry against a clean original product checkout and
the original reviewed patches applied to pinned OpenCPN. It restores compiled
bytes, original successful security evidence, the same-product qualified updater,
and verified dependency source archives. It does not compile an application.

New packaging uses the original clean source checkout and explicit product commit.
`PRODUCT_BUILD.json` identifies the original `compiled_ci_run`, the actual new
`packaging_ci_run`, and `packaging_helper_commit`. Build information labels both
runs separately; corresponding source still names the original product commit.
The original installer recipe is unchanged. The repaired helper checks the
resulting application's and Setup's exact version resources.

`recovered_staging.py` creates `RECOVERED_STAGING_INPUTS.zip`, never an original
`STAGING_BUILD_INPUTS.zip`. Its distinct kind is
`skager-recovered-prepackage-staging-inputs`. The manifest and receipt bind the
original compilation `producer`, actual new `packaging` origin, authenticated
restoration receipt digest, and every packaged/compiled input hash. Packaging
cannot change any restored original input. The compiled-feedback and installer
consumers explicitly validate this separate contract; historical schema-1 staging
success requirements are unchanged.

The archive is retained before the **complete** existing native Staging qualifier.
The fixture and product checks include pilot TCP refusal, portable recovery,
installer/updater behavior and charts. Credentials are absent during execution.
The input hashes are checked again afterwards; any failed check or changed input
leaves qualification failed. The final report binds the original product and
compiled origin separately from the new helper/run and archive hashes.

This workflow uploads evidence and retained inputs only. It performs no release
assembly, publication, or boat operation. A passed report does not establish
physical autopilot behavior, visual approval, Linux acceptance, restart-gate
acceptance, or release qualification. Those remain separately authenticated gates
for a later delivery composition; this recovery archive must not be passed through
the old successful-producer staging download path.
