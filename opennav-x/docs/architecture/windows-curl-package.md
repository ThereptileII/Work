# Maintained curl and zlib package boundary — SCRUM-209

The disposable native build must publish `curl-build.json` and `zlib-build.json`
beside its installed DLLs. `tools/curl_package.py` verifies their exact reviewed
source identities, Win32/x86 shared runtime, build-step completion, full output
inventories, and curl's hashes of the OpenSSL and zlib producer manifests.
OpenSSL's existing independent validator must pass first.

Recovery packaging requires the original verified curl 8.22.0 and zlib 1.3.2
archives from `build/dependency-downloads`. It does not download dependencies.
Missing, modified or mismatched source archives, notices, manifests or runtime
DLLs stop packaging. The corresponding-source archive includes the exact
original archives, project build recipes, and integrated OpenCPN patches.
Unchanged upstream license texts and their provenance are included with the
recovery package and inherited by installer assembly.

The package is checked again after app-local runtime copying. Legacy
`libeay32.dll` and `ssleay32.dll` are refused anywhere inside the package. The
native PE closure gate still inspects all normal and delay imports, including
plugins. Omitting a legacy DLL while retaining a dependent binary must fail;
this change does not permit deleting stock or user-installed plugin files.

Manifest checks are build provenance consistency checks, not release signatures.
The controlled CI build and later signed release/update policy remain separate
trust boundaries. Runtime TLS caller behavior remains SCRUM-211; a maintained
library alone does not fix disabled peer verification.

Focused inert-fixture tests cover tampering and missing inputs. Actual
PowerShell producer/consumer interoperability is checked separately. Native
MSVC compilation, upstream dependency tests, real TLS trust, plugin behavior,
installer update/repair/rollback and exact-candidate Windows gates remain
required before acceptance. No boat deployment or public distribution is
qualified by these package checks alone.
