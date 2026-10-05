# Exact missing-dependency installer gate

Run [37199379614](https://github.com/ThereptileII/Work/actions/runs/37199379614),
product `55ef51e4944e8570f6a391dad20b0db8447a7443`, failed the installer
lifecycle assertion after 34 completed checks. The original compact failure
artifact `11304975518` is 571,655 bytes, SHA-256
`10b2b26391bab2ce33631f8891f29ecee087c6531a0486dcdd650a258fcf8b6c`.
It contains 121 safe, CRC-verified entries. The final Windows evidence artifact
is `11305580491`, 84,042,991 bytes, SHA-256
`747e38e8d88c4e49fca6b6efb181101db10824ac87cf55f6e0be80011e8bec7a`.

The harness deliberately removed a wxbase DLL and rehashed its fixture ZIP and
manifest. Production `AssertCandidateTlsRuntime` runs before `SelfTest`, so it
correctly rejected the absent `wxbase32u_net_vc14x.dll` imported by
`opencpn-cmd.exe`. The harness accepted only the later
`Staged executable self-test failed:` error. This is not evidence of a missing
DLL in the original clean package. Preservation/modal assertions following the
failed assertion and remaining lifecycle operations were not reached.

The test-only correction removes the deterministic required
`wxbase32u_vc14x.dll`, requires failed Update with that exact missing import and
an importer inside the newly created unpublished generation, and retains the
unchanged installation/profile/stock and no-modal checks. It independently
invokes the unchanged production `SelfTest` in a separate disposable Windows
PowerShell process against that failed stage. This check requires the actual
`STATUS_DLL_NOT_FOUND` exit, restored error mode, no loader report, and no
publication. It never bypasses a guard in the production installer.

Focused preflight: all 12 existing installer-completion tests pass; changed
Python/workflow syntax and whitespace checks pass. Windows PowerShell
5.1.22621.5624 parsed the helper through a read-only SSH invocation on the boat;
the helper and installer were not executed there. Actual native execution of
the corrected lifecycle remains required.

The failed run retained evidence, but no complete application/Setup/recovery
payload, and has no cross-job build cache. It therefore cannot support an
unchanged-binary lifecycle rerun. The workflow now preserves failure-only
diagnostic inputs, their per-file hashes and exact source/run/attempt identity
in a separately named **unqualified** artifact. This never passes a failed gate
or creates an eligible delivery. It allows a later test-harness correction to
reuse authenticated bytes instead of requiring another full application build.

Application source, production installer, chart resources, fonts, branding and
TLS policy are unchanged. No endurance, boat installation, equipment output or
public release is qualified by this correction.
