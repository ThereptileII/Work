# Retained e80 full qualifier: passing prefix and installer failure — SCRUM-22

[Run 38757982655 / attempt 1](https://github.com/ThereptileII/Work/actions/runs/38757982655/attempts/1),
job `116501411584`, failed during fresh installation. It used helper
`842ded95bc226c681bdb36e9ebd177acd1924c04` against unchanged frozen product
`e80cb962e1a32d98bb75b964dca6eb516f7a5e23`, from producer run `38744236920/1`.
The retained run was `scope=all`; its recorded result remains **failed**.

[failure-receipt.json](failure-receipt.json) authenticates the original result
artifact `11985442640`: **25,070,763 bytes**, SHA-256
`01184262a101963594b27543e08be7866172bb26ef4bd769b99762805b088905`,
all **5,516 member CRCs** verified. The original job log SHA-256 is
`cc22d28382b1326818ee11391c8764d6032045e3ac6417b95dc9362b694eb48f`.
Both originals were downloaded once and remain private.

The restored artifact ID, outer digest, inner archive, manifest, 4,578-file count,
source ZIP and Setup hashes match the independently authenticated
[original custody](../scrum211-retained-e80-20261015/custody.json).
The installer report binds the frozen Lifecycle source bytes, later test helper
bytes and original Setup. No application or package was rebuilt or relabeled.

## Passing prefix retained separately

[passed-prefix.json](passed-prefix.json) records each original report's digest
and its original result wording, including any screenshot-review limitation.
The fail-fast qualifier reached the installer only after the preceding commands
returned successfully. Notable retained results are:

| Native disposable-Windows evidence | Result |
| --- | --- |
| Retained compiled feedback components | 13/13 passed |
| Integrated objects fixture | 28 checks passed, including actual modal deferral and stale-selection rejection |
| Objects UI/transport lifecycle | Three reported checks; themed property edit saved/refreshed and delete cancellation preserved the mark |
| Route progress, XNav and standard fixtures | 26 checks each |
| Recovery / user flows | Five / eight reported checks passed |
| Exact portable fixture-free product | Seven reported checks passed |
| Signed-update contracts, each PS 5.1 x86/x64 host | 61 session, 129 operation, 245 store, 42 inert barrier and 106 previous-restoration checks |

These results describe synthetic/disposable CI fixtures, not a boat operation.
The objects report preserves its native property-sheet success. The earlier
baseline's failed dialog observation remains a separate result.
No design/visual acceptance is inferred from screenshot presence.

## First installer failure and source boundary

Seven installer prerequisite/refusal checks passed before the actual fresh
installation. The retained engine log then records the loader/resource self-test
passing, followed by:

```text
FAILED: Update trust provisioning did not finish; current installed app remains recoverable.
```

The error comes from frozen product `installer/windows/Lifecycle.ps1:1253`.
Lifecycle opens `transaction.lock` with `FileShare.None` at line 1145, launches
`skager-start.exe --initialize-trust` while holding it, and retains the lock until
the outer `finally` at line 1312. The child requests the same lock in
`launcher/provision.go:24`. Its native `CreateFile` call returns an error on a
sharing conflict; it contains no waiting loop. The command's error path then
calls a synchronous `MessageBoxW` before exiting.

The frozen source therefore establishes an incompatible parent/child lock
sequence. The blocking error path is consistent with the parent's 15-second exit timeout;
the exact child error text was **not retained** because its stderr was not
redirected. The available failure screenshots show Setup's failure notice, and
job cleanup records a remaining `skager-start` process. These observations do not
justify increasing the timeout or treating the failure as a test-helper issue.
Exact source Git blobs and byte hashes are recorded in the receipt.

Fresh installation did not pass, later installer lifecycle checks remain
unexecuted, and the following chart gate was not reached. The passing prefix does
not qualify the full run, final package, updater, release or boat. The separate
captured-runtime boundary remains: four TLS DLLs match the final package exactly,
while three MSVC DLLs differ. Later repairs and retained qualifications need
their own exact-source and exact-package receipts.
