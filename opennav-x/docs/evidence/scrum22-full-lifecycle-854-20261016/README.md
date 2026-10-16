# SCRUM-22: retained 854 full installer lifecycle prefix

[Run 38889555934, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38889555934/job/116935466059)
failed in the separate missing-DLL SelfTest fixture. It tested original product
`854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc` from producer
`38765418126/a1` with qualification helper
`3bec26c322ef017aed6e6890b5cbba07989ebebd`. The product was not rebuilt.
The Windows 2022 image was `20261012.333.1`; the maintained installer path uses
Windows PowerShell 5.1. This is a passing prefix of a failed lifecycle job.

The authenticated report retains **35 completed lifecycle checks and 42 operation
records**, plus **all four packaged-updater checks**, including restoration of the
injected fault. The 42 records comprise 15 positive `passed` results, 25 expected
negative `failed` results, and two diagnostics reports without a status field;
they are not 42 successful installations. [receipt.json](receipt.json) lists every
completed check and binds the original report hashes.

The passing prefix covers clean installation/rollback, both pinned historical
installer fixtures, Beta 1 migration and recovery, historical early-Beta2
uninstall, normal XNav/Legacy/Safe startup and mode changes, Repair with preserved
custom additions, optional shortcuts, Update preservation, rollback refusals,
and all eight stock/active legacy-TLS Update/Repair refusals. Strict rejection of
the deliberately missing wx import also passed before loader or commit.
The native Finish checkbox was observed as `Open SKAGER`, checked, then explicitly
unchecked before Finish.

The subsequent isolated SelfTest process failed with `CommandNotFoundException`:
`Assert-LifecycleMutation` was not defined. The frozen qualification helper loads
only `Hash`, `PlainPath`, `PeArchitecture` and `SelfTest`; the exact frozen
`SelfTest` calls the missing guard first. Its receipt binds original Lifecycle
SHA-256 `e601683ee45b9f34fb6a5be20b5f72755be574e8796d1fffd9933ce31666b3e7`
and final application SHA-256
`8ad205a452683316330549bb93a5d0c039b2a2acb4d59eaa6aa4c5d3b85bc511`.
`initialErrorMode=0` and `restoredErrorMode=null` do not establish the intended
loader/error-mode proof. This is a fixture dependency-closure defect, not evidence
that the product's missing-DLL loader handling failed. The production guard must
remain intact.

The missing-DLL proof and later suffix remain unaccepted: fixture-enabled-product
refusal, locked-state/precommit/postcommit recovery, final diagnostics,
current-product conventional uninstall, stock startup/resource preservation,
reinstall, same-version update and final direct uninstall. The receipt names the
unexecuted check labels. Historical uninstall evidence is not relabeled as that
unexecuted final path.

Original artifact `12042839712` is 12,383,751 bytes, SHA-256
`59ab4a28df037718b4d2e50ab41d2f8dbf4c6036f25fdb9781384bf4ecf992be`;
all 1,668 member CRCs passed. The original log is 96,271 bytes, SHA-256
`1214489c98227f0618913df93692b80cef70c293110cd06a7ddfe44e9a9c46db`.
The earlier independent cold-hold fixture failure in this same workflow remains
separate. No retry, application build, Production promotion, boat operation or
protected-host/UAC acceptance is recorded here. Raw logs and private paths remain
outside the repository.
