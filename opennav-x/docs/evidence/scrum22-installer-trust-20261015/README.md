# Native installer trust handoff — SCRUM-22

[Run 38764328627 / attempt 1](https://github.com/ThereptileII/Work/actions/runs/38764328627/attempts/1),
job `116523477327`, passed on exact source
`b375e6bd3d743993a1415729becdc73ee3e47575`, Windows image `20261012.333.1`.
The focused `installer_trust_only` lane built the actual x86 GUI launcher and
exercised the original installer handoff functions on disposable owned fixtures.
This evidence commit is separate from that tested source.

The original artifact `11987078203` contains 9,138,815 bytes, SHA-256
`654c57d3233e007583837e2959123160d0bd949740b852a06af2acfe3f5e3b3b`.
API run/source/attempt/size/digest and all nine member CRCs were authenticated.
The original log SHA-256 is
`674ffff927ad6ff872ec15b4784406487730b28af05a2100d594215038633548`.
[receipt.json](receipt.json) records all member hashes, source identities and
native outcomes. Originals were downloaded once and remain private.

Both Windows PowerShell **5.1.20348.5622 Desktop** hosts passed **13/13** cases:

| Case | x86 host | x64 host |
| --- | --- | --- |
| Native owner wait before child / after disposal | Passed | Passed |
| Actual first initialization and unchanged repeat | 854 / 272 ms | 889 / 225 ms |
| Actual missing configuration remains inert | Passed | Passed |
| Actual held-lock failure exits without modal UI | Exit 1 | Exit 1 |
| Actual malformed trust refuses without modal UI | 422 ms | 374 ms |
| Inert child acquires lock while image/ancestors stay pinned | Passed | Passed |
| State changed during handoff refuses | Passed | Passed |
| Owner changed during handoff refuses | Passed | Passed |
| Ownership changed during handoff refuses | Passed | Passed |
| Stock changed during handoff refuses | Passed | Passed |
| Transaction journal appears during handoff: refuses | Passed | Passed |
| Armed update refuses before lock recreation | Passed | Passed |
| Owned child timeout terminates and drains | 15,112 ms | 15,105 ms |

All **six top-level Go tests and 28 subtests** passed, with zero skips or failures.
The launcher is PE32/I386, Windows GUI, Go 1.27.1, `GOARCH=386`, `CGO_ENABLED=0`,
with exact clean `vcs.revision=b375e6bd…`. Its SHA-256
`3d4a0ef108897f1d0c0cebd366563c1b9b37b16cbecaba40e7b582d9172e635b`
remained unchanged across both hosts. Both native receipts match the exact
Windows checkout hashes of `Lifecycle.ps1` and `UpdateTransaction.ps1`.

The positive and nonmodal cases used the real launcher. Separate inert children
exercised parent custody/refusal/timeout behavior. Fixture trust material contains
only a freshly signed public root and inert stock bytes; no private signing key
is included. Initialization performs no network or application launch.

This is a **component pass**, not final installer/package, release or boat
acceptance. It does not overwrite the
[original e80 installer failure](../scrum22-retained-e80-20261015/README.md).
The next candidate still requires its fresh same-run launcher, packaged installer
and complete native qualification. Read-only source comparison found all thirty
actual SDK inputs unchanged against both authenticated producers; the normal
native SDK image/toolchain/artifact checks remain required.
