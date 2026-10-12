# SCRUM-311 store lifetime lock — native Windows, 2026-10-12

This is focused native component evidence, not a native MSVC application build,
authenticated service qualification, installed updater acceptance or release.
Only disposable test directories and inert test-executable child processes ran
on the boat PC. The installed application, profile, services and physical
controls were unchanged.

## Production source and retained results

Production source is `7c2ceed495529d58c4038a3e153f6da8c2acdf28`, identical to the
isolated implementation `aeff26f7f25cf447353de6d16059ec199f183551`. The protected
store acquires and validates its exclusive synchronization handle before reading
candidate or registration entries, retains it for the store lifetime, and
releases it after attempting child and ancestor cleanup. A surviving matching
lock file grants no adoption, cleanup, recovery or execution authority.

The first Windows/386 executable, SHA-256
`080a61e8073ada4826ed11b4bd12dde1d3e1c7243f574508efeb990b56ee742e`, passed:

* `TestNativeStoreLifetimeLockExcludesOtherOwnerAndReacquires`
* `TestNativeStoreLifetimeLockExcludesAnotherProcess`
* `TestNativeStoreLifetimeLockRefusesUnsafeExistingObjects`, including directory,
  nonempty file, hardlink and wrong-DACL subcases.

Its fourth group failed because the fixture injected `windows.InvalidHandle`
(`-1`), which is also the Windows current-process pseudo-handle. Closing it did
not produce the required cleanup error. The failure assertion was preserved.

Fixture-only commit `36e0e84b3a0d18cf5b786e596eb4c23b92598992`, integrated as
`276c123672b993d76ec1e4e8164326f6efe48a0e`, replaced the injected handle with
null. Its failed-only native retry passed the production assertions but the
fixture's own final cleanup attempted to close null and failed. That result is
also retained as a failure, not a pass.

Fixture-only commit `aa4b7d5467e4cd035a515d30d46e1d68e0bacf94` separates the
injected pin from the fixture's ownership bookkeeping. Its Windows/386 executable,
SHA-256 `064e7268a1fd45c8f248dbe8cc29d5972d23571a77bf8853199a8849d63c35bf`,
passed only `TestNativeStoreLifetimeLockRechecksHeldMetadataAndReleasesAfterCleanupError`.
All assertions remain: mutated lock metadata refuses validation, the original
cleanup error remains available on repeated close, and the lock is released
despite the ancestor-close error. Production bytes did not change.

These are **four distinct top-level groups plus four nested subcases**, accepted
across the original three passing groups and the final failed-only retry. They
are not represented as a single passing full-suite invocation. Each retained
executable is 5,314,048 bytes, cross-compiled once with Go 1.27.1 for Windows/386,
CGO disabled, then executed natively on 64-bit Windows with PowerShell 5.1.
The test binaries do not contain an embedded VCS revision; file/source hashes
and the compilation/integration records bind their scope, not a production
binary attestation.

## Runner and evidence

The initial runner failed before testing because Windows PowerShell split bare
dotted Go arguments. Helper-only `3f978780` passes a string array. Helper-only
`272c1602` adds the failed-only selector; it does not rebuild the application.
Both fixture corrections reused that selector and left the three accepted
groups untouched.

`source-receipt.json` records actual raw-log and raw-receipt hashes. Raw logs
remain private; the tracked `.txt` files are UTF-8/LF conversions of the complete
test-only logs, inspected to contain no credentials or boat data. Original
PowerShell logs were UTF-16 with a BOM. The first flag-parsing failure was an
observed tool result without a separately retained raw-log hash.

Authenticated service provisioning, cross-account rights, installation/loader
custody, interrupted acquisition/orphan reconciliation and exact-package update
acceptance remain open. The requested expanded Claude source review has not run:
automatic approval refused the external transfer and exact-scope consent remains
pending. No application build, CI dispatch, public push, service activation,
visual/endurance sweep or physical command occurred. SCRUM-311 remains In Progress.
