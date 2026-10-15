# SCRUM-283 focused exposure parity, 2026-10-15

The initial input-binding proof below is superseded by the reviewed
[provenance repair](provenance-repair/README.md). Its historical receipts remain
unchanged; two fixture-provenance gaps were found and corrected afterwards.

Helper/fixture revision: `43fd4eb219438b1b49ee15fd495a3ef7cf62e730`.
Runtime/application source base: `1ffd09ddf2fa6588def2907485f6e14e89bc3a7d`.
This change compiles four isolated fixture executables. It does not build,
relabel or qualify an application, private DLL, installed product or boat.

The actual extracted `_UDWHAZ03`, `OBSTRN04` and `WRECKS02` bodies now run a
288-case matrix: EXPSOU absent/1/2/3, depth unknown/3.2/5/6 m, WATLEV 2/3/4,
and deterministic associated DEPARE DRVAL1 2/10 m, with safety contour 5 m.
The original 26 quality, sounding and isolated-danger cases and bridge/lifetime
assertions remain. The direct `_UDWHAZ03` promotion flag is recorded; the two
callers' internal flag is explicitly marked `not-exposed`.

## Recorded results

Linux, GCC 16.2.1, exact committed helper:

| Fixture | Result |
| --- | --- |
| Pinned OpenCPN core | 398 checks passed |
| Locked original private | 398 checks passed for its original behavior |
| Corrected private | 416 checks passed, including bridge/lifetime cases |
| Corrected source with adapter disabled | 398 checks passed; original output/layout retained |
| Original private required-repair negative | Exit 1, `association check 18`, retained |
| Core/original/corrected source mutations | Rejected before fixture compilation |

All 26 baseline and 288 exposure output/category records agree between core and
corrected private after the existing, explicit private Wk placement exception.
Original and corrected private differ in 93 exposure records, so the matrix
still distinguishes the original missing-query defect. The original required
repair failure remains in `original-private/required-repair-negative.log`.

`source-inputs.json` records seven core inputs checked against Git objects at
OpenCPN `37fd0cddb7334fe489e9f18aa163977a9c5c84f7`, 217 retained private
original files checked against the existing source lock, and the 217 derived
files reproduced with the existing production `apply_patches` function. The
three current patch hashes are retained. This is a local focused subset;
the complete lock has 219 entries and distributable-source acceptance is separate.

The initial resumed attempt refused the earlier derived subset because it
omitted three retained API files. A fresh copy of the 217 retained originals
through the production patch function supplied the complete matching subset.
No runtime repair was needed. Earlier evidence remains under
`/home/standard/Work/skager-scrum283-exposure-20261015/`.

`harness.json` binds the immutable fixture revision and file hashes separately
from the application source base. `receipt.json` retains compile commands,
actual source/body/header/extracted-input hashes, executable hashes and exits.
The output directory must be new, and assertions cannot be disabled with `-O`.
The local syntax and whitespace checks passed.

The existing change selector conservatively returns product/helpers/dependencies
true: both tests are compiled inputs and the helper path is currently unreviewed.
The coordinator owns any integration build decision. No application build or CI
dispatch was performed for this extension.

## Minimal native invocation (not yet run)

Use the existing authorized disposable Windows workflow with its successful
SDK authentication and source preparation steps. Preserve that workflow's
producer/run/attempt/artifact identities and receipt; this helper does not
replace SDK authentication. Supply its authenticated wx 3.2.8 vc14x x86 DLL
prefix, initialized MSVC x86 toolchain and available x86 VC runtime. Use the
exact fixture commit above and a pinned core checkout containing its Git objects.
The original/prepared private trees must come from the normal locked preparation.

From an MSVC x86 developer PowerShell session, after those prerequisites pass:

```powershell
python opennav-x/tools/verify-private-hazard-association.py `
  --core $PinnedCoreCheckout `
  --private-original $PreparedPrivateOriginal `
  --private-patched $PreparedPrivateSource `
  --wx-prefix $AuthenticatedWxPrefix `
  --native-sdk-receipt $SuccessfulSdkVerificationReceipt `
  --output $NewHazardEvidenceDirectory
if ($LASTEXITCODE -ne 0) { throw 'Private hazard fixture failed' }
```

The native branch calls `cl` with `/MACHINE:X86`, verifies each fixture PE is
I386, and records the supplied receipt identity and consumed wx import/runtime
files. It refuses a missing SDK receipt, a non-x86 developer environment, and
inherited compiler overrides. Those mechanisms have not been executed on native
Windows in this task. A future native fixture pass is component evidence only.

## Remaining SCRUM-283 acceptance

The list fixture does not execute the actual geographic associated-area query.
Native Win32/private-DLL runtime, actual area-query behavior, real-chart
selection and portrayal in software and OpenGL modes, and boat evidence remain
unpassed. The inherited reference-point limitation for line/area features also
remains. Missing or unavailable query results are not a safe-portrayal pass.
No physical command, website change, release, source publication or Jira closure
is authorized by this evidence. SCRUM-283 must not be closed from this matrix.
