# Claude review workflow validation

Validated on 2026-10-10 in the Linux development environment. This qualifies
the developer review workflow, not SKAGER binaries, releases or boat hardware.

## Exact reviewed inputs

Base Git commit: `743275900b6849c834958a89ec440f20ca5217f4`.
These new working-tree files were reviewed as copied bytes before commit:

| File | SHA-256 |
| --- | --- |
| `tools/agents/claude-review` | `eac5d3560449d21edcb5e883619c17f04b7edb47e223d6d894374e0053f2d798` |
| `tools/agents/test_claude_review.py` | `8aade6ec5fcd1e9310c455253044ccb66684a080ab42eca5ce8fa714e6746781` |

Runtime: Claude Code 2.1.292, `sonnet` alias, high effort. The actual CLI init
reported `claude-sonnet-5-5`. Existing OAuth authentication was reused without
printing credentials or changing global Claude configuration.

## Checks and observed results

- `python3 tools/agents/test_claude_review.py`: **12/12 passed**.
- Real bubblewrap probe: snapshot writes fail; host workspace/home unavailable;
  inherited SSH agent and API endpoint overrides absent.
- Scope tests: traversal, control characters, symlinks, link swaps, hardlinks,
  binary/oversized inputs refused. Directory selection excludes untracked files
  and uses literal Git paths; inherited Git overrides cannot redirect selection.
- Snapshot remains unchanged after source edits. Timeout terminates the process
  group and fails with partial evidence retained.
- Real noninteractive security review completed successfully. Event records show
  Read/Glob/Grep plus Agent delegation to the configured security specialist;
  no shell/write/web/MCP tools were available. The child returned its review.
- A real Read of harmless `/etc/hosts` was denied by the file-tool policy.
- Separate live probes: Glob of `/home/reviewer`, Grep of disposable
  `/home/reviewer/.claude.json`, and Read through `/proc/self/root/...` were
  all denied. Tool-result events confirm all three failures. No credential
  was probed and no file contents were returned.
- `git diff --check` passed. No product build or Windows/boat test was needed.

The restricted outer command environment produced an `EAI_AGAIN` API failure.
The wrapper correctly rejected it despite the CLI result subtype saying success,
because `is_error` was true. A subsequent call through the authorized network
execution path passed with bubblewrap and Claude tool restrictions unchanged.
Do not disable containment to address an outer network restriction.

Private local evidence (not uploaded automatically):

- `/tmp/skager-claude-network-review/`: completed source review, receipt and
  event stream; receipt says `sourceChangedAfterCapture: false`.
- `/tmp/skager-claude-tool-boundaries/`: completed benign tool-boundary probes.
- `/tmp/skager-claude-final-review/`: failed network attempt, retained for diagnosis.

These local temporary directories may be cleaned later. This document preserves
the non-secret result; raw transcripts are intentionally not committed.

## Codex disposition of the independent review

The initial review prompted the link-race/hardlink protection, literal Git
pathspecs and environment isolation, certificate-only mounts, explicit outside
Read denies, atomic receipt writing and terminal control-character filtering.
The final reviewed hashes above include those changes.
After the live checks, one trailing space was removed from the wrapper; this was
the only subsequent wrapper change and did not change its behavior.

Claude's concern that Grep/Glob might bypass Read-deny rules was a hypothesis;
the installed CLI's live probes rejected the selected direct and alias paths.
This does not claim exhaustive verification of every possible CLI vulnerability.
Do not add a blanket Read deny over `/review`: denies take precedence over
allows. Likewise, missing unmounted host directories are not exposed files.

The installed Claude runtime must access the read-only OAuth mount and the
network to authenticate and call the model. The runtime, OS and service remain
trusted; this is not containment of a compromised Claude binary or a network
firewall. Credential refresh may require ordinary host-side authentication.
Only an isolated temporary home is writable, not the real home or working tree.

Scope/version preflight failures occur before a report directory is created.
If a selected source becomes unreadable during the final freshness comparison,
the workflow conservatively fails rather than accepting uncertain provenance.
Interrupts return failure, never a completed review. These are deliberate
fail-closed behaviors; Claude's low-severity suggestions do not change the
read-only boundary or require a product rebuild.

Codex retains final judgment. This review is advice, not automatic approval.

## Opus routing update — 2026-10-10

The user subsequently requested routine Claude delegation for complicated work:
Opus 5 for most assignments and Opus 5.5 for the hardest. This supersedes the
original Sonnet default above without changing the read-only containment.

- Default is pinned `claude-opus-5`; escalation is `claude-opus-5-5`.
- Child specialists explicitly inherit the parent's model.
- **14/14 focused tests passed**, including rejection of missing model evidence,
  unexpected fallback and a child using a different model.
- Two independent authenticated calls ran in parallel. Both receipts completed:
  `/tmp/skager-opus5-routing/receipt.json` records `claude-opus-5`, and
  `/tmp/skager-opus55-routing/receipt.json` records `claude-opus-5-5`.
- Opus 5's event stream also records its architecture child as `claude-opus-5`.
  Opus 5.5 reviewed the model-verification function and its focused tests.
- Opus 5.5 identified that accepting moving aliases left an exception to strict
  verification. The aliases were removed, missing assistant model metadata was
  made fail-closed, and the focused tests passed again. Both saved live event
  streams were then validated successfully by the final stricter function,
  avoiding unnecessary repeat model calls.
- AGENTS.md now calls for regular bounded Claude assignments alongside Codex
  subagents, with Codex owning prioritization, integration and final decisions.
  Implementation delegation within authorized tasks requires a separate
  worktree/branch and restricted implementation session; this review wrapper
  still never edits source.

No product, Windows, boat or release qualification was required for this routing
change. Raw local transcripts remain private; temporary paths may later expire.

## Source-batch reuse — 2026-10-11

The active source branch retains this wrapper unchanged from tooling commit
`847e78b4`. All **14 local isolation/model-routing tests** passed with private
project temporary storage. This did not contact Claude or exercise credentials.
The user's current Codex delegation preference is GPT-6.1 Sol; the Claude
wrapper retains its pinned Opus 5/5.5 routing and read-only boundaries.

Automatic egress review rejected a proposed five-file private plugin-loader
security review pending consent for its exact payload and Anthropic destination.
No source was transferred by that rejected call. It is not a successful review
and does not supersede the historical successful review evidence above.
