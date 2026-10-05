# Delivery efficiency — 2026-10-04

Scope: SCRUM-225, SCRUM-292, SCRUM-293 under SCRUM-290. These are delivery
infrastructure checks, not application, boat, visual or Production acceptance.
No application build, boat action or Production promotion was requested here.

## Focused qualification

Code: `0e6ee68aa70f82a5d29561cf8de346e0b28d7422`.
[Run 37235438463](https://github.com/ThereptileII/Work/actions/runs/37235438463)
passed **190 Linux checks and 190 native Windows checks**. Windows also parsed
`build-pristine-windows.ps1`, `package-preview-windows.ps1`,
`qualify-staging-windows.ps1`, the installer self-test script and
`verify-curl-bundle.ps1`. Job wall time was 19 seconds on Linux and 48 seconds
on Windows, including setup/upload.

| Suite | Checks per platform |
| --- | ---: |
| Release inventory | 16 |
| Authenticated release delivery / publish retry | 20 |
| Workflow and qualification policies | 24 |
| Installer retained-source and functional policies | 20 |
| Installer process completion | 12 |
| Changed-input selection using actual Git histories | 17 |
| Immutable SDK bundle / native metadata selection / exact consumer compatibility | 23 |
| Authenticated Actions downloads and ZIP boundaries | 16 |
| Retained compiled inputs | 13 |
| Exact-attempt retest selection | 2 |
| AIS dependency authority and native-runtime guard | 9 |
| OpenSSL immutable identity / live CPU separation | 5 |
| Existing same-job receipt and shared native setup | 13 |

The retained-input fixtures exercise exact-byte restoration on both platforms,
original product/source identity, independent helper identity, failed evidence,
missing/tampered/extra files, Windows aliases, linked archives and wrong producer
attempts. They do not claim an actual application build or desktop qualification.
The next real product candidate still runs the separated native application and
installer jobs; neither is waived by these results.

## Failed evidence preserved

- [37227864837](https://github.com/ThereptileII/Work/actions/runs/37227864837):
  Linux passed; Windows found two fixtures creating newline filenames and an
  embedded-provenance check comparing a resolved path to a short Windows path.
  Fixtures now construct real Git object histories without filesystem-invalid
  names; the verifier canonicalizes both paths. No failure assertion was removed.
- [37228280158](https://github.com/ThereptileII/Work/actions/runs/37228280158):
  an AIS test fixture lacked the newly required unchanged verifier record.
  The complete fixture now exercises the same tampered-prefix rejection.

- [37235243723](https://github.com/ThereptileII/Work/actions/runs/37235243723):
  190 Linux checks passed; Windows passed 189 and exposed a nondeterministic
  retained-archive manifest timestamp. Original payload entries had a fixed ZIP
  timestamp but the manifest used wall time. Both now use the same fixed metadata.
  The existing test was strengthened to force different clocks/source mtimes and
  compare full bytes, hashes and all member metadata; it fails before the repair
  and passes afterwards. No assertion was removed.

These corrections reran the small helper workflow, not an application build.

## Dependency-only native proof

The first cold native producer
[37227094736](https://github.com/ThereptileII/Work/actions/runs/37227094736),
source `966e7832ac326aaaf397046ea1dc9c2cff19569d`, failed during sealing after
48 minutes 39 seconds. Library builds/tests and native tool checks passed,
including curl 1,569/1,569. The sealing helper searched the entire CMake build
for compiler metadata, unlike the authoritative native check which searches
`CMakeFiles`; nested test builds made that selection ambiguous. No SDK was
uploaded and this producer is **not eligible for reuse**.

Failed evidence artifact `11312569303` has SHA-256
`504d998527dde244c6db5b8e85362f72ee1164c19b1525e2b10bad4186645f19`.
The selector correction must preserve missing/ambiguous authoritative-metadata
refusal. Future failures separately retain unqualified original native outputs
for investigation; these cannot be selected as a successful SDK.

Three obsolete exception-specific tests were replaced by three tests for nested
compiler metadata, ambiguous authoritative metadata and missing authoritative
metadata; the suite remains 19 checks.

The unused verifier-compatibility exception for the failed producer is removed:
there is no eligible bundle requiring an exception. Exact current verifier bytes
remain required; the corrected Windows short-path boundary stays in place.
Replacement producer [37230581131](https://github.com/ThereptileII/Work/actions/runs/37230581131)
passed at `1b25542aea3f9ab62c83c5d7a3ecdac9652f7d3e`:
OpenSSL 4,283, zlib 13 and curl 1,569 tests. The producer step took **50m30s**;
whole job 51m28s. SDK artifact `11314817073` contains 117,979,890 bytes and has
SHA-256 `27f689d55a6827e52826fb64f7f1c5081e17ee19ab35beaa219d406b43bfaf2e`.
The original bundle manifest SHA-256 is
`ec7cf8fae8f179bb8695f0352516db7dbe2ca97dd7cf8ad48644165822884d32`.

First cross-run attempt [37234040385](https://github.com/ThereptileII/Work/actions/runs/37234040385)
authenticated/restored the original bundle and passed OpenSSL/zlib native
reprobes, then failed curl's full version-output comparison. Diagnostic
[37234440416](https://github.com/ThereptileII/Work/actions/runs/37234440416)
executed the same verified OpenSSL binary and proved that **only CPUINFO differed**
on the second processor. OpenSSL 3.5.9
[version.c](https://raw.githubusercontent.com/openssl/openssl/openssl-3.5.9/apps/version.c)
and [info.c](https://raw.githubusercontent.com/openssl/openssl/openssl-3.5.9/crypto/info.c)
identify this as a live CPU observation. It is not an immutable build property.

An independent consumer probe now preserves original producer scripts and
receipts, compares all non-CPU version fields exactly, and records CPU state
separately with overrides refused. A narrowly pinned two-file consumer-only
compatibility record applies to this successful producer; the obsolete failed-
producer exception remains removed. No library is rebuilt for this correction.
Corrected warm proof [37235349391](https://github.com/ThereptileII/Work/actions/runs/37235349391)
passed with consumer `a2f8815ed2cb7d43f12184dd597c339c5a1ba71f` against the
unchanged original SDK. Authentication/download took 9 seconds; fresh native
verification/restaging took 65 seconds: **1m14s**, compared with the cold
producer's **50m30s**. Whole warm job was **2m11s** including setup and evidence
upload. This saves **49m16s in the dependency stage** in this measured pair;
it is not a measurement of the full application build.

The producer build step was skipped. Full inventory/hash/provenance checks,
OpenSSL parent/child, zlib parent/child and curl native tool reprobes passed.
Original build/test receipts remain unchanged; current CPU observations are
retained separately. Evidence artifact `11314943893` SHA-256:
`d86b5ee093ef3f13714bcc9ccb6028a542eaa2e91c77228291f50bb82a998721`.
The later helper commit changes only deterministic retained-archive metadata;
its dependency-consumer inputs are identical to the successful warm revision.

SCRUM-225 remains **Testing** until the next genuine application candidate consumes
and qualifies these inputs. No extra application build is scheduled solely to
close that broader gate. SCRUM-292 and SCRUM-293 helper acceptance is complete;
SCRUM-290's first complete Staging candidate remains separate verification.

## Workflow registration and preserved boundaries

Default-branch registration `771d0419aaa08c4de6757fa0e6699b382cc8660f`
changes only four `.github/workflows/` files. All existing firmware/project
files remain untouched. Follow-up registration
`2677b0661a92dd3435df1472fb77c9db1f0adc38` changes only the dependency
workflow to retain failed producer outputs. Registration
`a8525595f11273ebb942461c2487569514164f3b` changes only the baseline/helper
workflows to remove duplicate automatic builds from mirrored branches.
Registration `bd5ff5702f70ebf3249ff3eab86199f3ab96021d` changes only the
focused helper workflow to include current consumer and same-job boundary checks.
Software work uses the `staging` branch.

Staging remains the default delivery channel, with draft versioned Releases.
Production requires the explicit selected-candidate instruction. Design review
and endurance remain opt-in according to the existing user decisions. The boat
installation and physical hardware permissions are unchanged.
