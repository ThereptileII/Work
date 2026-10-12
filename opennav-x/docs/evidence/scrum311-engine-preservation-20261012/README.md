# SCRUM-311 — original execution-helper preservation

Source integration: `aa3e6bfeb88dba981fa88465b5b28816428ca609`.
Native allowlisted runner: `700e5dfd38c744c68ce5343265c6eac594e22a2d`.
This is component/source evidence, **not acceptance of a working installed updater**.

## Actual implementation

The NSIS producer snapshots `Lifecycle.ps1`, `UpdateSupervisor.ps1` and
`UpdateTransaction.ps1` before compilation. NSIS receives those exact paths;
post-compilation validation refuses changed snapshots. Recovery preserves the
original bytes under the three fixed `installer/engine/` paths and inventories
and checksums them. No reconstructed script or existing app/docs payload is
substituted. This binds producer inputs; it is not independent inspection of
Setup embedded resources or proof of publisher/first-execution authenticity.

Authenticated `StageRecovery` and independently approved baseline import copy
these bounded original ZIP members through the retained source after hash and
CRC checks. `baselineevidence` requires zero historical helpers or all three
exact helpers; partial sets, case aliases and unknown reserved paths refuse.
Inventoried helper digests participate in the measured retained namespace.
Historical zero-helper evidence confers no helper execution authority.

Candidate completion/sealing and installed-baseline completion/reopening verify
the original sidecars, product, full retained membership and helper bytes.
Native ancestry, file identity, links, descriptors and deny-write/delete handles
remain enforced. Recovery reconciliation recognizes only exact inert residue,
including partial fixed helpers; it does not adopt, delete or authorize it.
The source-package fixed closure now covers all 74 production members of the
actual imported local packages, including both new candidate-evidence files.

## Focused results

| Check | Result | Raw evidence |
| --- | --- | --- |
| Signed Recovery / approved import race | 23 top-level + 117 nested pass | `verifier.jsonl` |
| Shared original evidence race | 8 top-level + 31 nested pass | `engine-evidence-leaf-tests.jsonl` |
| Store / candidate / baseline race | 17 top-level + 81 nested pass | `store-engine-helpers-focused-20261012.jsonl` |
| Real packaging recipe with inert mocked NSIS | 25 pass | `recovery-helper-receipt-focused-20261012.log` |
| Four source-package producer/consumer suites | 68 pass | Delegated tool-session output only; no raw logfile retained |
| Windows/x86 inert fixture compilation | Exit 0 | `windows386-compile.log` |
| Windows/x86 broker + provisioner compilation | Exit 0 | `production-components-compile.log` |
| Actual native Windows custody/reconciliation | 5 top-level + 22 nested pass | `native-windows/EngineEvidence/` |

All recorded Go/native selections have zero failures/skips. Shared/store tests
ran in isolated worktrees; their integrated tested source bytes are unchanged.
`source-inputs.json` records the final integration inputs. The cross-compiled
Win32 fixture actually executed on native Windows 10.0.22631. Its SHA-256 is
`a63dcc4c81e3ea9072996a677e93437ddce0f94d86ef6fafa6f3ba0a3296577a`.
The PowerShell receipt's `process64` describes its runner, not the x86 test EXE.

The bounded collector checked exact transferred hashes, native PowerShell parse,
exact listed/executed test names and exit status. Transfer/operation/retrieval
records are retained. Existing OpenCPN PID 1852 remained responsive before and
after; no owned test process remained. No service, application, profile, installer,
update, network fault, screenshot or physical command was exercised.

`prior-source/recovery-evidence-store-attempt2.jsonl` preserves an earlier failed
stock-mutation fixture at the preceding source increment. Its corrected result
was supplied by that increment; it is not counted as a new passing test here.
No failed native attempt occurred for this exact selection.

## Remaining boundaries

Actual privileged first provisioning/source authentication, a real protected
install/update/fallback transaction, interactive user-side launch handoff,
commissioning, durable one-use selection/spawn and authenticated healthy startup
remain unfinished. Signing/private-channel custody and exact-package qualification
remain required. No full application/MSVC build, CI run, release or boat product
change occurred. SCRUM-311 stays In Progress. Claude's frozen nine-file comparison
still awaits exact transfer consent; no private source was sent to Anthropic.
