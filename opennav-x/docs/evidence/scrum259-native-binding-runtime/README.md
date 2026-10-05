# Native BindingState runtime comparison

Tool-only [run 37198821735](https://github.com/ThereptileII/Work/actions/runs/37198821735)
and job `111426110388` pass for remote
`a167e3049c68bc9443fcbd1b3f46efc2475988c0`, mapped local
`6e7407dbbbdaa478d42e4a653077fe3d0b28092f`. The complete 6,837-entry mapped tree
`17ef3d64d97f2f6156c8f586c4e315651fdf15cc` is independently reconstructed.
Only four proof files were added to the failed application candidate; this
workflow does not compile the application or access the boat.

The original artifact `11302176888` is retained unchanged as `original.zip`:
6,209 bytes, SHA-256
`32bb19524247e0819e30e3a27c646443e58abc34d243f06622ab6b755322bd2f`.
All 13 members have safe, unique regular paths and valid CRCs. The seven
recorded source identities independently match the frozen Git blobs with
Windows CRLF checkout normalization.

Both Win32 `/MD` programs use the unchanged actual `BindingState.h` and copied
ABI header. Their only build-definition difference is
`_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR`, already required by pinned OpenCPN's
`CMakeLists.txt:431–440`. Both load the identical app-local runtime files
recorded in the failed application's installed trust prerequisite:

| File | Version | SHA-256 |
|---|---|---|
| msvcp140.dll | 14.12.25810.0 | fa21058e50d0d6860da87d784f573670bf5d3efd65158145954ef96d0cd403cf |
| vcruntime140.dll | 14.12.25810.0 | 1b372f064eacb455a0351863706e6326ca31b08e779a70de5de986b5be8069a1 |

These bytes independently match the official pinned OpenCPN dependency
repository `e90cc5842b02d5502a549f0f90424e3e1614bf67`, `buildwin/vc/`.
Loaded paths, versions, executable hashes, compiler and commands are retained.
Microsoft documents this compatibility boundary in the
[VS 2022 17.10 STL release notes](https://github.com/microsoft/STL/releases/tag/vs-2022-17.10).

The unguarded child reaches `before-bind`, then exits `0xC0000005`; the
non-suppressing exception observer locates this proof's fault in MSVCP140.dll
at RVA 117802. The guarded child exits zero after all 13 ordered stages:
binding, pending status, initialization, selected status and repeated-bind
refusal. Neither child times out. This reproduces and corrects the actual
binding mutex/runtime incompatibility without inventing host API stubs.

This is not a stack trace for the earlier full application's crash and does
not prove that it had no additional defect. The original full-host fault
instruction remains unavailable. The replacement must still pass the actual
host import/bind/status/unload gate, all required native package checks and
the boat's private-chart/GPU/font/mode review. No stock/boat DLL was replaced.
Endurance remains explicitly skipped by user direction.
