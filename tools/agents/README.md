# Independent Claude reviews

**Inactive by user decision, 2026-10-15.** Use Codex subagents only. Do not run
this wrapper, Claude Code or Claude child agents unless the user explicitly
re-enables Claude. The usage and isolation details below are retained for
reference and do not authorize invocation.

Codex leads implementation and makes final decisions. `claude-review` asks an
independent Claude Code specialist for evidence-based advice; it is never an
implementation, deployment, release-approval or physical-control entry point.

## Use

From the repository/worktree being reviewed:

```sh
tools/agents/claude-review --kind security \
  --path tools/agents/claude-review \
  --question 'Find concrete isolation or correctness defects; cite file and line.'
```

Kinds: `architecture`, `debugging`, `security`, `regression`, `code`.
Repeat `--path` for context, interfaces and relevant tests. Directory scopes include
**tracked files only**; explicitly name new files. Files are current working-tree
bytes, not necessarily HEAD. No Git history, automatic diff, ignored build logs,
private boat evidence, whole home directory or full repository is sent by default.
For a comparison, supply deliberately sanitized before/after text files explicitly.
The question and selected source are sent to Anthropic through the user's existing
Claude login. Check the scope for confidential content/secrets; filename exclusions
are a guardrail, not a general-purpose secret scanner.

Defaults: **Opus 5** (`claude-opus-5`), high effort, 10-minute timeout and $3 CLI
budget ceiling. Most Claude assignments use this default. For the hardest work,
use **Opus 5.5** with `--model claude-opus-5-5`. Examples include difficult
architecture tradeoffs, persistent concurrency/crash bugs and sensitive security
or safety boundaries. Child specialists inherit the parent's selected model.
Exact model IDs are checked against returned assistant metadata (including
children); unexpected fallback or missing model evidence fails the review.
Only these two exact IDs are accepted; moving aliases such as `opus` and
`sonnet` are deliberately refused so routing cannot silently change.
`--timeout 900` and `--max-budget-usd 5` allow bounded resource escalation.
The CLI's dollar budget is a runtime guard, not a billing guarantee for subscription
accounts. The wrapper never retries a failed review automatically; Claude's own
transport may retry within the outer timeout. There are no shell argument
passthrough options.
`--repo /absolute/worktree` selects another checkout; all scope paths are relative
to its Git root, even when invoked from a subdirectory. `--output-dir /new/private/dir`
retains results at a new path; otherwise the wrapper prints its private `/tmp` path.
Results: `review.md`, `receipt.json`, `events.jsonl`, `stderr.txt`. Do not upload raw
transcripts automatically. The receipt records exact selected-file SHA-256 values,
base commit, CLI version, requested/observed models and whether source changed after capture.
If it did, Codex must revalidate findings against the current source before applying
anything. Partial/failed reviews exit nonzero and are never reported as accepted.
If the surrounding execution environment blocks DNS/network access, use its
authorized network-access path for this command, retaining the wrapper's
bubblewrap and tool restrictions. An API error or CLI exit code alone is not a
successful review; require the completed receipt and nonempty review result.

## Read-only boundary

Requires Linux, Python 3, Git, bubblewrap and Claude Code **2.1.292 or newer**.
Existing `claude auth login` OAuth authentication is used without printing or
copying credentials into evidence. No installation or global Claude configuration
is changed. The credential file is mounted read-only for the Claude runtime;
file tools are additionally restricted to `/review`. An expired login requiring
credential refresh may need an ordinary host-side Claude login/refresh.

The wrapper captures bounded UTF-8 files (2 MiB/file, 16 MiB total, 1000 files),
rejects symlinks, hardlinks and private paths and runs against a read-only snapshot.
Directory Git pathspecs are literal; inherited Git overrides cannot redirect scope.
The real
working tree, Git directory, SSH agent, boat access and unrelated home files are
not mounted. An isolated temporary home holds Claude runtime state and is removed
afterward. Only system runtime/certificates/DNS, the Claude executable, the snapshot
and its OAuth credential are mounted. Network is available to the trusted Claude
runtime for the model API; this is not a network-domain firewall. The model gets
no shell, code-execution, edit, web-fetch, plugin or MCP tools. Host environment,
user/project settings and hooks are not inherited. No permission bypass is used;
no unsandboxed fallback is allowed.

Claude may use the five explicitly configured child specialists. They inherit
read-only instructions and have only Read/Glob/Grep. Children cannot run commands
or implement changes. The trusted boundary includes the installed Claude binary,
system runtime, kernel/bubblewrap and Anthropic service. This does not claim to
contain a compromised runtime or kernel.

## Ownership and conditional use (currently disabled)

Include the applicable user authorization and allowed source/destination scope
in each assignment, following the project AGENTS permission-handoff rule. Do not
ask again for an already approved transfer with the same scope and destination.
Codex sandbox approvals do not grant Claude new tools, network destinations or
write access; the wrapper's restrictions and current execution approvals remain
binding. A materially different transfer needs its own applicable authorization.

If the user explicitly re-enables Claude, it can assist with complicated
design/planning, architecture, debugging, security and regression work.
Give each assignment a distinct bounded question and acceptance criteria; Codex
owns prioritization and integration. Run independent work in parallel, avoiding
duplicate investigation and unnecessary review chains. Supply precise evidence;
request independent hypotheses
instead of steering the answer toward Codex's preferred conclusion. Allow bounded
parallel child reviews when they cover distinct questions. Skip trivial edits,
duplicate opinions and full-repository review by default. Claude findings do not
replace focused tests or mandatory Windows/boat/release gates.

Codex can continue independent edits after snapshot capture because Claude never
sees the mutable source. Never point an unrestricted Claude session at a worktree
Codex is modifying. Any future Claude implementation requires explicit
re-enablement and an authorized project task, using a dedicated Git worktree/branch, exclusive file ownership and
a separately restricted implementation session. Establish its file/tool boundaries
before allowing writes; Codex reviews and integrates the changes. This wrapper
intentionally has no implementation mode and its restrictions must not be relaxed.

Run focused safety tests with `python3 tools/agents/test_claude_review.py`.
The OS-boundary test runs real bubblewrap with an inert Python probe, no network
request, real credential or hardware. A successful real review is separately
required to verify authentication, CLI flags and subagent operation.

References: [CLI](https://code.claude.com/docs/en/cli-reference),
[subagents](https://code.claude.com/docs/en/sub-agents),
[model configuration](https://code.claude.com/docs/en/model-config),
[file permissions](https://code.claude.com/docs/en/permissions).
