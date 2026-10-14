# Original Repair native gate — 2026-10-14

Source: `db71720599cae54b34a9a7c805ac971502900385`.
[Run 38586308295](https://github.com/ThereptileII/Work/actions/runs/38586308295),
attempt 1, **failed**. Existing `skager-startup-restoration.yml`; no application
build, installed customer execution, boat operation or release.

Original artifact metadata, downloaded ZIP sizes/SHA-256 and all CRCs verify.
Counts below exclude subcases. Windows did not produce its required-contract
receipt or reach the source-closure step because the test command failed.

| Platform | Original job | Artifact | ZIP bytes | SHA-256 | Pass / fail / skip |
| --- | --- | --- | ---: | --- | --- |
| Windows 2022 / Go 386 | 115929132294 | 11905492003 | 60035 | `44f782de54bc5b00acc515fc2cd88dcab8548081b5039466de577181651704eb` | 315 / 2 / 5 |
| Linux Ubuntu 24.04 / amd64 | 115929132812 | 11905043195 | 59127 | `72e612a24ec971b20db0f1689423ae18ce8ca43ee61825860736344da4eeaa39` | 214 / 0 / 0 |

Original test-stream hashes: Windows
`15aeef04353b3e2adede43c00f3ee923508523ed0fbd9de68668c3d880d566b4`;
Linux `d1ad95d597548c48296f9352816d3fd4e83e6976dff882ff205bdb9d6684a6f3`.
Windows ran 15:05:53–15:08:10 UTC; Linux 15:05:52–15:06:27 UTC.

## Failure and narrow correction

Both failures are in `brokerstore/generation_repair_ntfs_windows_test.go`:

- `TestNativeRepairNTFSRestoresDamagedPublicPayloadFromOriginalPrivateSource`:
  signed-candidate fixture construction reports a sharing violation.
- `TestNativeRepairNTFSRefusesTamperedOriginalPrivateSource`:
  both signed-candidate fixture constructions report the same sharing violation;
  baseline same-bytes replacement reports unexpected DACL entries opening its
  original private source parent.

Reviewed correction `1036a938` changes only that test factory. Candidate sealing
now closes its retained previous baseline, matching the established native stage
seal fixture. Replacement files use the original client SID for both baseline
and candidate private-source descriptors, matching
`prepareNativeInstalledBaseline`'s `tree.securityFile`. Production descriptors,
sharing modes, source identity checks and negative assertions remain unchanged.
Win386 test compilation and `git diff --check` pass. Native confirmation is still
required; source inspection and cross-compilation cannot establish that the
sharing violation is resolved.

The five Windows skips remain explicit: the opt-in disposable-host SUBST remap
case plus four non-Windows public boundary cases. They are not coverage.
No Repair, installed lifecycle, earlier-version compatibility or boat acceptance
is granted by this failed attempt or by the Linux result.
