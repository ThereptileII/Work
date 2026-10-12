# SCRUM-311 — protected application view

This is component evidence, not an installed updater, a healthy application
receipt, release qualification or permission to run the provisioner on the boat.

## Fresh enrollment

Integrated source `601dc155` (isolated `2e8e79b88a6df852f3a7ac320c8fed7cf280b26b`)
creates `ProgramFilesX86/SKAGER/Applications` as a fresh, protected sibling of
`UpdateBroker` before completing the enrollment file fence. The broker can write
the application subtree, but retains only read/execute permissions to its own
code and the parent. Ordinary authenticated users have read/execute, no mutation.
Existing namespaces are refused, never adopted or repaired.

Focused Linux enrollment race tests passed **19 top-level + 67 nested cases**;
raw JSONL is retained. Windows/386 test compilation passed. The fixture executable
SHA-256 is `bc21eee3a2935e18f8da34cb5dee3fc46c8d709b9c83f51f206a707b673efcb5`.

The exact allowlisted native `LaunchRoot` group passed **2 top-level + 4 nested
cases**, zero failures/skips, on Windows `10.0.22631`. It inspected actual native
descriptors and created a disposable temporary directory with the final
descriptor, retained/checked its pin, refused a second creation and checked handle
drainage. It did not create the real ProgramFiles application root or execute
privileged product enrollment. The source hash and collector receipt are in
`native-transfer-manifest.json`; complete listing, execution and case evidence are
under `native-windows/LaunchRoot/`.

The original OpenCPN PID **1852** stayed responsive, with no owned fixture process
left afterward. No application restart, service creation/activation, product
installation, profile change, network fault or physical output occurred.

## Materialization and host join

Product source `ac96ef54` joins store `c148e4d2` (isolated `256da46c`) and host
materialization (isolated `fe0f781`). The live, frozen signed candidate is copied
into a fresh protected public application view while its approved baseline and
source custody remain held. Original inventory/manifest/product/helper bytes are
retained. Private reservations, original parent-birth records and full physical
identities prevent adoption, partial reuse, aliasing and invented lineage. A
subsequent fresh generation can reuse only authenticated original parents.
Completion distinguishes signed candidate policy/Recovery/Setup hashes from the
original baseline approval; it grants no selected, launched or healthy status.

Independent source review found two defects before integration: missing
cancellation boundaries between fresh creations, and missing uncertainty
classification for completed-owner cleanup failures. Both were corrected.
Retained private records are also hash-checked at inventory/Validate boundaries.
The delegated store race check passed 5 top-level + 26 nested cases. The final
joined host/store race check passed **14 top-level + 64 nested cases**, zero
failures/skips, against the hashes in `joined-source-receipt.json`. The earlier
host overlay result is not relabelled as this final join. Windows/x86 store and
host component test compilation passed; those compilations are not MSVC or
native execution acceptance.

The first native Generation run passed 3/4 top-level and 10/11 nested cases. The
actual directory-descriptor mutation returned Access denied after opening its
write-security handle. Failure and receipts are retained under
`generation-native-initial/`. Test-only repair `f8fce3e5` (isolated `64462502`)
adds read-security access, real descriptor readback and separate mutation/readback/
cleanup errors; it preserves the retained validation rejection assertion. Missing
read access is a candidate explanation, not a conclusion established by the
original log. No production source was changed for that repair. Its native rerun
confirmed the mutation itself now succeeds, but the newly added readback incorrectly expected
unchanged descriptor inheritance flags. The actual inheritance-control flags
differ from the fresh descriptor. That failed readback is retained separately under
`generation-native-acl-attempt/`; it is not a passing native gate. Test-only
correction `d2a45ee7` (isolated `dca7394d`) now verifies exact requested
ACE count, order and bytes, and requires present/protected flags while allowing
OS-normalized `AUTO_INHERITED`. Production's exact retained-descriptor rejection
remains unchanged.

The final allowlisted Generation group passed **4 top-level + 11 nested cases**,
zero failures/skips, on native Windows `10.0.22631`. Original copies/helpers,
authenticated parent reuse across successive generations, unknown/partial
namespace refusal, byte/descriptor/identity/hardlink mutations and forged parent
records were exercised in disposable temporary fixtures. The exact final
Windows/x86 binary SHA-256 is
`4e79f5ad864e83571051f6845843fa1bf962fd62340ee81d04e47d6caab13777`;
runner SHA-256 is
`cc309da8acf4cf231c0578bc7b152baddd824d49297dde10009b0c199012fae6`.
All transfer, native parse, listing, execution, retrieval and process checks
passed; receipts are retained under `generation-native-final/`. These fixtures
bypass the actual broker account/native acquisition/fixed ProgramFiles discovery.
OpenCPN PID **1852** remained responsive, with no owned fixture processes left.
No product profile, service, application installation or physical output changed.

## Corresponding source

The fixed shared broker/provisioner source closure includes the concrete
application-view production members. Four affected inert producer/verifier suites
passed **68 tests** (32 + 12 + 10 + 14); their actual raw logs and result receipt
are retained. These fixture checks do not produce or qualify a product package.

## Acceptance boundary

SCRUM-311 remains In Progress. Durable pending/selection/spawn, interactive-user
launch, read-only loader custody, real profile restoration/commissioning, authenticated readiness and installed
update/fallback qualification must be evidenced separately. No full application
build, CI dispatch, release or boat deployment was performed for these checks.
The user approved Claude's frozen nine-file review; the attempt failed with the
provider's weekly limit and returned no findings. The independent Sol fallback
and exact-source comparison are recorded in
[the separate review comparison](../scrum311-client-registration-20261012/review-comparison.md).
Earlier evidence keeps its original revision and scope.

The materialized owner still holds writer handles; it does not provide the future
read-only loader transition. Previous lineage identifies the original approved
baseline, not an immediately selected/healthy predecessor. Historical sibling
records authorize parent membership only, not execution or readoption of their
old payloads. No failed/partial materialization is automatically deleted or reset.
