# SCRUM-311 protected installed baseline — 2026-10-12

Production source is `67665e18779d5d76e3fa101c742872219df14cad`.
The Windows-only test mask comparison was corrected in
`f2a9e0515fc8c10f6c3115122c13eecad1579267`; no production input changed.
This records an implemented source increment and passing inert component checks.
It does **not** accept privileged enrollment, a real updater transaction, launch,
commissioning, a healthy-start handshake or a release. SCRUM-311 remains open.

## Actual source boundary

The optional independently approved client request now requires the fixed
`approved-installed-baseline.zip` in the protected operator artifact directory.
`brokerenroll.Run` validates the request, identical bootstrap trust/channel and
all authenticated baseline members before network or machine mutation. Only its
retained fresh enrollment may publish the paired fixed machine files:
`ProgramData/SKAGER/ClientRegistrationRequests/<SID>.json` and
`<SID>.baseline.zip`. Creation uses final Administrator/SYSTEM-write,
broker-read protection, `CREATE_NEW`, bounded copying, sync and exact readback.
Read-only deny-write/delete custody remains held through the stopped SCM handoff.
Partial publication is retained and refuses reuse; no existing enrollment is
adopted or repaired. The approval hash must be independently authenticated; an
adjacent self-hash, today's ACL or mutable LocalAppData state is not approval.

The stopped `register-client`, SID, approved-request-hash dispatch takes no path
and returns before `Running` or a pipe announcement. The actual broker principal
reads the fixed protected source with physical ancestry/identity checks, then
`ImportApprovedBaseline` reparses the original independently approved request and
validates the original recovery inventory, Lifecycle package manifest and product
bytes. It imports exactly the manifest's app/docs payload set, excluding the
portable marker. Recovery launcher/profile/root extras are not installed payload.
Original `FILE_SHA256.json` and `installer/package.json` are retained unchanged.
The shared `baselineevidence` validator is reused by import and retained reads;
no alternate generic parser or mutable installed-state record supplies authority.

`brokerstore.BeginInstalledBaselineFresh` reserves a permanent started fence
before creating the fixed `UpdateBroker/users/<SID>/installed/<generation>` tree.
At creation, payload members grant the broker/SYSTEM full access and the approved
client read/execute only. All directories and members remain retained, physically
identified and bounded; reparse points, hardlinks, replacement, additions and
namespace aliases refuse. Completion independently revalidates retained original
evidence and exact payload hashes, records creation identities and binds the
approved request plus completion record in the permanent marker. The baseline
completes before registration completion; owners remain held through that boundary.
Late cancellation cannot reverse successful durable publication. Actual attempted
publication/custody failure can be uncertain and may coexist with durable state;
partial namespaces and close errors stay retained, with no automatic retry/reset.
`OpenInstalledBaseline` requires that completion record, exact namespace and
independently approved pins. The host retains this owner through candidate staging.
`Directory()` conveys no launch authority.

`Installed.PackageSHA256` means the exact original `installer/package.json` bytes,
not the outer ZIP, inventory, a reserialization or a guessed payload hash.
`RecoveryIdentity.DeclaredPayloadSHA256` is the manifest declaration; it is not an
observation of a separate payload ZIP. Archive copy hashes corroborate transport
and readback; they do not replace independently approved original evidence pins.

Source entry points are under `tools/update-verifier`: `approved_baseline.go`,
`baselineevidence/evidence.go`, `brokerstore/installed_baseline*.go`,
`brokerenroll/client_baseline*.go`, `brokerenroll/enroll.go`,
`brokerhost/client_baseline_windows.go`, `brokerhost/client_registration*.go` and
`brokerhost/host_windows.go`. Individual source hashes are retained below.

## Recorded checks

| Check | Result | Provenance |
| --- | --- | --- |
| Five affected Linux packages, race | 111 top-level + 364 nested pass; zero fail/skip | [raw JSONL](portable-race.jsonl) |
| Root importer/signed-recovery subset | 20 top-level + 113 nested pass; zero fail/skip | [raw JSONL](approved-baseline-integrated-67665e18.jsonl) |
| Combined Linux component total | **131 top-level + 477 nested pass** | Counts of existing final logs; no rerun |
| Four Python packaging suites | **68 tests pass**, exit 0, 12.510 s | Coordinator-reported session 46334; no raw logfile retained |
| Windows/x86 broker + provisioner build | Pass at production source `67665e18` | [compile receipt](windows-compile-receipt.json) |
| Windows/x86 brokerenroll inert test compile | Pass | Same receipt |
| Windows/x86 brokerstore inert test compile | Initial failure, then pass after test-only `f2a9e051` | Original and repaired logs retained |
| Native InstalledBaseline allowlist | **5 top-level + 6 nested pass**, zero fail/skip | [receipt](native-windows/InstalledBaseline/receipt.json), [raw output](native-windows/InstalledBaseline/tests-stdout.log) |
| Native ClientBaseline allowlist | **5 top-level pass**, zero fail/skip | [receipt](native-windows/ClientBaseline/receipt.json), [raw output](native-windows/ClientBaseline/tests-stdout.log) |

The existing strict runner listed the exact anchored allowlist first, required
that exact set, then ran it once with `-test.count=1 -test.v -test.timeout=90s`.
It required every named top-level PASS and no SKIP/FAIL. Its source SHA256 is in
[native transfer manifest](native-transfer-manifest.json). OS was Windows
10.0.22631; these were x86 test executables under 64-bit Windows. Receipt
`process64` describes the PowerShell runner, not the test executable's ABI.
Per-case logs/receipts and both list outputs are retained. The lower-level native
file fixtures use a fresh temporary namespace under the current test account;
public native APIs explicitly refuse operator-console fallback. They do not
exercise actual broker service enrollment or authenticate historical first use.

Original OpenCPN PID 1852 was responding before and after, and no owned test
process remained ([before](native-stage.json), [after](native-after.json)). The
bounded inert checks did not activate a service, change the application/profile,
launch a candidate, send physical output or qualify application/MSVC behavior.
Compile receipt `nativeExecution: false` describes only the earlier compile phase;
subsequent native execution is evidenced separately above.

## Preserved failures and artifact identity

The initial [brokerstore compile failure](brokerstore-compile.log) was an
`ACCESS_MASK`/`uint32` test comparison. Root corrected the mask cast in the test
only. The [repaired log](brokerstore-compile-repaired-f2a9e051.log) is separate;
empty successful compiler logs are intentional and backed by exit-code receipts.
Production binaries were not rebuilt after that test fix because all production
inputs were unchanged. Exact commands/environment and four artifact SHA256s are
in the compile receipt. No executables are committed here.

The remote operations and both native groups succeeded. The first local
collector nevertheless exited 1 while extracting a zero-byte directory entry
encoded by `Compress-Archive` with a trailing backslash. `ZipInfo.is_dir()` did
not recognize it, so the suffix assertion refused. [Failure transcription and
provenance](collector-failure.txt) are retained separately from test outcomes.
Local extraction recognized these directory entries without rerunning tests.
The archive SHA256 matched [native packing output](native-results-pack.json):
`570210a86769e09cff04ab276b4902e5e4767117793760e35a8ca845a77b0415`.
Extracted log/receipt bytes are unchanged; duplicate ZIPs are omitted.

[Original input manifest](windows-source-inputs.json) and
[repaired test input manifest](windows-source-inputs-f2a9e051.json) record all
tracked module Go files, go.mod/go.sum and the inert runner. This is a conservative
input superset, not a minimal dependency closure. Their recorded canonical-map
aggregates are respectively `618ae5a2651ebd4fb33379aaf089cbea34e79da122d0a08b71820f580bf2dc8c`
and `c25d38a8ccd5ae3edb07583551626f45a7b7de72b9a0ff9d6779525dce249458`.
The unchanged production aggregate is
`107d7eb2063d8fb035232f66cf91e11e8477db5b9e906d01ad1549cee84be40a`.
These aggregates differ from the receipt files' own byte hashes.
[SHA256SUMS](SHA256SUMS) covers every committed evidence member except itself.
[Machine-readable summary](summary.json) distinguishes raw logs from session reports.

## Remaining acceptance boundary

No selected-generation/pending transaction owner, successor promotion or
interactive user-side authenticated baseline/launcher handoff is provided by this
increment. The fixed initial baseline is not proof of the old running LocalAppData
image or full loader custody. Actual privileged first execution and broker account
ACL/SCM provisioning, real update/fallback transaction, interactive launch,
commissioning and healthy-start handshake remain required. The historical root,
current metadata/expiry and rollback-floor checks retain their distinct roles.
No trust-floor reset, arbitrary path, pipe enrollment, candidate execution,
service activation, full app build, CI dispatch or release acceptance occurred in
this documentation/evidence increment. Earlier registration evidence remains in
[its own record](../scrum311-client-registration-20261012/README.md).
