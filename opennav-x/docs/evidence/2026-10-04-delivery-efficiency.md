# Delivery efficiency — 2026-10-04

Scope: SCRUM-225, SCRUM-292, SCRUM-293 under SCRUM-290. These are delivery
infrastructure checks, not application, boat, visual or Production acceptance.
No application build, boat action or Production promotion was requested here.

## Focused qualification

Code: `a7718df86b35a541919c5ccf7a53bc0c3f0c3583`.
[Run 37228443410](https://github.com/ThereptileII/Work/actions/runs/37228443410)
passed **167 Linux checks and 167 native Windows checks**. Windows also parsed
`build-pristine-windows.ps1`, `package-preview-windows.ps1`,
`qualify-staging-windows.ps1` and the installer self-test script. Job wall time
was 17 seconds on Linux and 43 seconds on Windows, including setup/upload.

| Suite | Checks per platform |
| --- | ---: |
| Release inventory | 16 |
| Authenticated release delivery / publish retry | 20 |
| Workflow and qualification policies | 23 |
| Installer retained-source and functional policies | 20 |
| Installer process completion | 12 |
| Changed-input selection using actual Git histories | 17 |
| Immutable SDK bundle / approved verifier correction | 19 |
| Authenticated Actions downloads and ZIP boundaries | 16 |
| Retained compiled inputs | 13 |
| Exact-attempt retest selection | 2 |
| AIS dependency authority and native-runtime guard | 9 |

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

These corrections reran the small helper workflow, not an application build.

## Dependency-only native proof

The cold native producer is
[37227094736](https://github.com/ThereptileII/Work/actions/runs/37227094736),
source `966e7832ac326aaaf397046ea1dc9c2cff19569d`. Native producer completion,
authenticated cross-run reuse and measured time savings are still pending.
The selected SDK lock will only reference a successful complete producer.
The exact consumer-only verifier correction is explained in
[build efficiency](../build-efficiency.md#reviewed-verifier-correction-for-the-first-producer).
No compiler recipe, library source, DLL, upstream test log or original manifest
is changed by that correction.

## Workflow registration and preserved boundaries

Default-branch registration `771d0419aaa08c4de6757fa0e6699b382cc8660f`
changes only four `.github/workflows/` files. All existing firmware/project
files remain untouched. Software work uses the `staging` branch.

Staging remains the default delivery channel, with draft versioned Releases.
Production requires the explicit selected-candidate instruction. Design review
and endurance remain opt-in according to the existing user decisions. The boat
installation and physical hardware permissions are unchanged.
