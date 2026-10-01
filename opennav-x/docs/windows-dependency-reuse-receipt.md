# Windows dependency receipt boundary (SCRUM-217)

`tools/windows_dependency_receipt.py` is a strict file receipt primitive for a
future same-job optimization. `capture` and `verify` take the same absolute
workspace root, a caller-derived context JSON file, and a sorted set of
relative installed prefixes. The context binds the GitHub run ID, attempt,
job, exact source commit, Win32 architecture, and SHA-256 maps for producer
inputs and toolchain components. The receipt binds that context, the absolute
workspace path, and every regular file under each prefix by size and SHA-256.
It rejects missing, added, changed, linked, malformed, and unbounded input.

The receipt is **not yet called by the native workflow and does not enable
reuse**. The next integration must derive the context from the actual GitHub
job, source tree and native toolchain; verify all three producer manifests,
successful upstream tests and their retained logs, source archive hashes,
transitive script and lock identities, complete installed inventories, cache
mapping, and Win32 import closure. Only after the first integrated application
checks succeed may it publish a receipt. A later production invocation must
verify that evidence before staging cached dependency files from the verified
prefixes. Any failed check must take the existing full producer path or refuse
the build. Product, package, installer, and security gates remain mandatory.

Native same-job execution and before/after timing are a later candidate gate;
this receipt alone provides no Windows or release qualification.

`tools/windows_dependency_evidence.py --root <workspace>` is a separate,
read-only producer check. It reuses the packaging source/notice/Win32 checks,
requires the flat integrated install to match each OpenSSL, zlib and curl
producer manifest byte-for-byte, hashes every declared producer output, and
checks curl's OpenSSL/zlib manifest links and retained import text. It also
requires nonzero successful test summaries in the retained OpenSSL, zlib and
curl logs; curl's log must match the SHA-256 in its manifest. The OpenSSL and
zlib producer manifests currently record `test=passed` but do not bind their
native log hashes or test counts. The verifier returns those log hashes for a
future receipt, but this increment cannot independently prove that their
logs were produced in the same job as their manifests. The native producer
and same-job orchestration work must close that gap before reuse is enabled.
The second `build-pristine-windows.ps1` invocation currently runs zlib's
`-VerifySourceOnly` preflight before dependency reuse could be checked. That
preflight overwrites `source-verification.json` with `mode=source-only`; the
producer verifier deliberately requires `mode=build`. It accepts one explicit
alternate through `--zlib-source-verification`:
`evidence/local/windows-zlib-1.3.2/first-success-source-verification.json`.
The future first-success receipt step must preserve that exact build record,
bind its bytes in the receipt, and select it after the later preflight. The
alternate receives the same bounded parsing, plain-path and source checks;
arbitrary paths and `mode=source-only` remain rejected. Producer-time native
tool facts must be retained from each producer's actual environment rather
than recaptured from a later GitHub step's different PATH. No orchestration
currently writes or selects the alternate, and reuse remains disabled.

`tools/windows-native-tool-facts.ps1` is a separate capture/reprobe primitive,
now called by the three dependency producers. OpenSSL records parent tar and
the NASM/Perl PATH choice before extraction, plus cl/link/nmake/Perl/NASM from
its `vcvarsall x86` child. zlib records the CMake/compiler environment from
its `vcvarsall x86` child and the parent's actually selected dumpbin. curl
records its direct CMake/build environment and its selected dumpbin. A small
observe-only `CMAKE_PROJECT_INCLUDE` writes the generator-selected instance,
Win32 SDK, platform toolset, compiler, linker and MSBuild identities after
top-level `project()`; the helper cross-checks those against the generated
cache, compiler metadata and project. Each fact file binds the producer and
helper bytes, selected executable paths/hashes/version probes, and a fixed
allowlist of nonsecret environment values or hashes. Its `Verify` mode
reprobes the live tools and compares the complete BOM-free record. These
facts are not yet consumed by the receipt or reused by the workflow. Native
Windows execution of the new hooks remains unqualified; no dependency build
can be skipped on this evidence alone.
