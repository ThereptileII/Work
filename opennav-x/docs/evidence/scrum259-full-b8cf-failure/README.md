# SCRUM-259: full b8cf native failure and independent package audit

The [native integration job](https://github.com/ThereptileII/Work/actions/runs/37114216075/job/111178794028)
failed during host OpenCPN configuration, **after the actual private adapter DLL
compiled, linked and packaged**. This is not an accepted application candidate.

Exact source: local `d5d71356d806ea8c3518644d10728a24f1334d1d`, published
`b8cfbf809450208f723095ffb4e00d7800b619a5`, tree
`111fac745157fd70cc83be81d09b0f3a6da7ecd5`. The audit used that frozen source's
four-export verifier, not later source with a fifth diagnostic export.

## Retained evidence

- Artifact `11271783971`, `windows-integration-b8cfbf809450208f723095ffb4e00d7800b619a5`:
  22,411,239 bytes, SHA256
  `ece7c5c9736e5126ca20635245ec512de3b81c4354c83d7f8c93dd0fc7459990`;
  all 5,812 ZIP entries passed CRC verification. The archive remains local;
  its binaries and source ZIP are not duplicated in Git.
- `windows-xnav-Win32.log.gz` is the exact downloaded native transcript,
  losslessly compressed. `salient-log.txt` preserves numbered excerpts.
- `package-manifest.json` and `windows-ocharts-first-build.json` are unchanged
  downloaded receipts. `audit.json` records independently measured hashes,
  source inputs, missing payloads and verification results.

## First causal failure

Completed job log: private DLL link succeeded at 10:56:37 UTC on 2026-10-03.
Host resource generation then completed at 10:58:04, followed by:

```text
tools/verify-ocharts-adapter-package.py:121
ValueError: Adapter chart resource manifest differs
src/integration/OpenCPN.cmake:420
Private chart adapter source/package verification failed
```

The transcript locates the actual DLL link at line 17970 and the host failure
at lines 18255–18263. OpenSSL's tests, all 13 zlib tests, and all 1,569 reported
curl tests had already passed. The earlier `cl` missing-source message is a
tool probe; it is not the causal failure of this build.

The exact frozen package verifier passes against the canonical completed
Linux-generated resources, without executing native code. It independently
checks the I386 PE import/export contract, DLL bytes, corresponding-source
inventory/CRCs/upstream blobs, exact current product inputs, resource files,
and dependency receipt schema/locked identities. The packaged DLL and the
actual native build output have identical SHA256
`6ae8618513007efa10318d41235bfe887ff07103c9147a4bc1ef2d5364068e60`
(1,325,568 bytes). This does not verify that DLL loading or rendering succeeds.

The package's resource manifest is
`12cfbce4686fce9a89e15f45105195c9f09a51ae4e3e9932aa7ba803ae205a52`
(85,084 bytes), and its header is
`fef00119d6b820abe8725be6b8243490da4c056dcaff25c89575f4e1b6c94266`
(934 bytes). Both match the independently audited native preflight and
completed Linux generation. The host's second resource set was **not uploaded**,
so this archive cannot reveal the exact differing bytes or prove pixel equality.

## Generation boundary under investigation

`build-pristine-windows.ps1:73` generates the first resources with PATH-resolved
`python`; transcript lines 426/433 include Python 3.12.10 on PATH. Host CMake
independently selects `C:/hostedtoolcache/windows/Python/3.14.7/x64/python3.exe`
(line 18248) and regenerates resources at `OpenCPN.cmake:259`. The private CMake
configuration also discovers that 3.14 executable (line 17760), but verifies
already prepared bytes instead of regenerating them.

The resource encoder uses runtime `zlib.compress(..., 9)`. The interpreter split
is a concrete reproducibility risk; the first interpreter's actual executable
and the second generated PNG bytes were not recorded. A resource-only native
proof must establish the compression/pixel difference before calling that exact
mechanism confirmed. No byte guard was relaxed and no product source changed
in this evidence commit.

## Existing dependency reuse does not apply

The artifact contains producer manifests and test logs, but zero entries beneath
`build/windows-openssl-3.5.9/install`, `build/windows-zlib-1.3.2/install`,
`build/windows-curl-8.22.0/install`, `build/dependency-downloads`, or
`build/xnav-install`. The private package records dependency hashes, not the
dependency DLL/import-library/header payloads. No
`windows-dependency-first-success-receipt.json` exists.

The existing `windows_dependency_reuse.py:100–165` requires the same GitHub
run, attempt, job, checkout SHA, input/toolchain hashes and complete prefix
inventory. The workflow captures this receipt only after the integrated fixture
and following fixture gates pass (`opennav-baseline.yml:540–545`), then uses
`-ReuseVerifiedDependencies` on the same job's production pass. This failed run
did not reach capture. `windows_dependency_stage.py` revalidates that receipt;
it is not an independent cross-run cache importer. Therefore no existing
qualified recipe permits reuse from this failed artifact for the next candidate.

Application compilation, fixture runtime, fixture-free production installation,
actual host module load, installer, Windows visual acceptance and boat validation
were not reached. No workflow was retried, ref changed, DLL executed or boat
operation performed during this audit.
