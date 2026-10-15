# OpenNav X Agent Instructions

The authoritative product specification is:

`OpenNavX_Codex_Project_Specification.md`

The later user-approved [delivery policy](docs/delivery-workflow.md) takes
precedence over older delivery/channel and design-validation scheduling rules in
that original specification and the public-beta contract. Preserve both supplied
source documents and their historical evidence; do not rewrite them as if they
originally contained the later policy.

The authoritative visual reference is the unchanged supplied HTML prototype:

`docs/design/prototype/index.html`

Its original-file manifest is `docs/design/prototype-original.json`. Never edit
those original files. Put instrumentation in tools or a separate derived copy.

The current project goal is:

`PROJECT_GOAL.md`

## Jira-driven public-beta work

[Navigare / SCRUM board](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the sole product-development backlog. The public-beta requirements are captured in
`docs/public-beta-contract.md`; that contract and technical evidence are not a
second roadmap. Do not maintain a competing private TODO list.

### Product work versus internal administration

User decision: 2026-10-15. Routine internal administration does not need a Jira
ticket, board scan, transition or progress comment. Examples include creating or
maintaining Codex skills, agent instructions, local agent-tool setup and internal
workflow documentation. Make these changes directly, use ordinary Git history
where relevant, and report the outcome; a Jira key is not required in their
branch or commit names. Do not create retrospective admin tickets by default.

Track product features, customer bugs, product security/data-preservation work,
installer/updater changes, release/CI gates and acceptance evidence in Jira.
Classify by effect rather than filename: documenting a release gate or changing
a qualification script is product work; editing an agent guide is normally admin.
For mixed work, track the product part and handle incidental admin without a
separate ticket. Existing historical admin tickets need not be deleted or
rewritten. Authorization, secrets and safety rules still apply to both kinds.

At the beginning of each substantial product-development cycle:

1. Read the board, its actual column/status mapping, In Progress and Testing
   issues, and all unresolved `scope-public-beta` issues. Complete pagination.
2. Read the selected issues' acceptance criteria, dependencies and material
   comments. Never infer a decision from a label or an older comment alone.
3. Prioritize safety/security/data-loss blockers, then public-beta launch
   blockers at Highest, other public-beta Highest, High, Medium and Low.
   Dependencies may change the eligible order. Post-beta/Future work waits.
4. Record the selection in Jira before unrelated implementation. Create or
   update an issue for additional required work; split distinct deliverables
   into appropriate tasks/subtasks. Do not silently expand scope.
5. Move work to In Progress when implementing; Testing when implementation is
   ready but required verification remains; Done only after its acceptance
   criteria and evidence pass. Preserve failed evidence and open blockers.
6. Record exact commits, CI links, validation and remaining gates in Jira or
   linked technical evidence. A passing build is not feature completion.

The board's English columns currently map to Swedish Jira statuses:
Idea → Idea, To Do → Att göra, In Progress → Pågående,
Testing → Testning, Done → Klart. Read the current board mapping and available
transitions rather than assuming status-category queries identify each column.

## Development strategy and task sizing

Use many small, bounded parallel tasks when the work can be split cleanly.
The user's latest model instruction is **GPT-6.1 Sol** for Codex subagents, with
**Claude** as an independent specialist through the existing scoped read-only
review wrapper. This supersedes the earlier Astra High and Luna/Sol allocation.
Use isolated worktrees for delegated implementation; retain the wrapper's
snapshot, secret-exclusion and read-only boundaries for Claude reviews. Codex
owns final decisions and integration. The root agent is the single coordinator for
integration, full application builds and releases. Tie each delegated product task to its Jira
issue; internal admin tasks need no issue. Assign non-overlapping file ownership
and acceptance criteria, and return
focused test results to the coordinator. Subagents must not independently
start duplicate full builds, deliveries or boat operations. Integrate a coherent
batch and inspect the combined change; request one build/delivery when its
product inputs require it. Keep context limited and handoffs concise.

### Permission handoff to subagents

User decision: 2026-10-15. Existing user authorization follows delegated work
within its original scope. Include a short permission handoff in every initial
assignment, context-limited handoff and material scope update:

- State the applicable user authorization and the assigned task, worktree/files,
  permitted reads/writes, external destinations and focused checks.
- Identify coordinator-only actions and applicable exclusions, including boat
  operations, physical output, release publication and secret handling.
- Do not ask the user again for task scope already authorized. Forward the same
  authorization and limits when delegating further; a child cannot expand them.

Codex subagents inherit the parent session's current sandbox/approval policy;
this text carries intent, not an execution-permission override. Tool-specific
approvals still apply, and a one-command approval is not blanket authority for
other commands or agents. Use the normal scoped escalation mechanism when
required. If it is unavailable or denied, report the exact action and reason to
the coordinator; do not bypass it or claim a permission change occurred.
If the parent permission profile changes mid-task, verify the worker's actual
access before claiming propagation. An already-running worker may retain its
older environment. Preserve its handoff, stop only that worker and resume through
a fresh agent under the current policy when needed; this is not a way to evade
an approval denial. Verify access with a minimal task-relevant check rather than
assuming that a message or configuration edit changed runtime permissions.
Keep existing external job
handles; never restart CI or a boat operation merely to repair observation.

Claude is a separate runtime: use its existing scoped wrapper and approved
transfer scope, never copy Codex credentials or assume Codex execution approvals
apply there. Review assignments remain read-only. Implementation assignments
require the isolated worktree and exclusive ownership rules above. Do not put
secrets or approval tokens in prompts, repository files or evidence.

Use [the scoped Claude workflow](../tools/agents/README.md) for bounded
architecture, debugging, security, regression and code reviews. Its immutable
snapshot permits independent Codex work without concurrent edits by Claude.
The wrapper's pinned Opus model routing and read-only child specialists remain
in force; never bypass a denied external transfer or silently change models.

Choose checks from actual build and package inputs, following
[build efficiency](docs/build-efficiency.md). Ordinary documentation and test
helper changes need focused checks appropriate to their effect; they do not
request a new application package or qualify a new product/source revision.
Packaged release notes, licenses, runtime resources, installer code and build
recipes are product inputs even when a filename looks like documentation.
Unknown inputs or unavailable comparison history take the conservative build
path. For reversible, noncritical copy, layout or function changes, run focused
functional checks and an applicable smoke check. Run
design/visual-conformance review only when the user explicitly requests it. Avoid
redundant broad reruns and tests that only mirror the implementation. Preserve
all navigation, safety, data-loss, security, authentication, payment,
installer, updater, recovery, exact-revision, native Windows, boat, release,
and source-compliance gates. Do not remove or weaken existing failure
assertions. Batch full suites at integration and release milestones instead of
running them for every minor edit. Keep Jira as the sole backlog; do not create
a TODO file.

Update documentation when a change materially affects a contract, architecture,
delivery procedure, safety boundary or recorded evidence. Do not create a new
status report or rewrite historical specifications for every small edit. Preserve
failed and incomplete evidence, and record the package revision separately from
a later helper revision. Retain application outputs before downstream tests so
a helper repair can retest exact bytes without rebuilding or relabeling them.
Long-running suites require an explicit request and run as separate background
work; pending required results remain pending. The existing endurance-test skip
continues until the user changes it.

`SCRUM-97` governs this workflow. The user's 2026-10-04 decision in
[docs/delivery-workflow.md](docs/delivery-workflow.md) separates frequent Staging
delivery from Production readiness; `SCRUM-290` tracks its CI implementation.
Development/default delivery is **STAGING**. Standard delivery in both channels
uses versioned GitHub Releases. Start Production readiness/promotion only on an
explicit user instruction for a selected candidate. Promote its exact qualified
package bytes, with channel identity separate from the embedded package version.
Keep both Staging and Production releases **draft** while public access is closed;
Production promotion does not authorize public publication. Use
`opennav-baseline.yml` for Staging, manual `skager-production.yml` for promotion,
and `opennav-prototype.yml` only for explicitly requested design review.
Production promotion must not initiate design validation, prototype comparisons,
aesthetic refinement, screenshot review sets or design-only DPI sweeps unless
the user explicitly requests that work. This supersedes older blanket visual
promotion requirements. Functional chart rendering, usable controls, navigation
correctness, data preservation and security remain required. Do not reclassify
design review as a functional gate to bypass this instruction.
`SCRUM-14`, `SCRUM-15` and `SCRUM-16` retain their documented work and evidence;
unperformed design acceptance is not a pass and is not an automatic promotion
blocker under this decision. Functional/safety blockers remain blockers.
The approved public identity in `SCRUM-89` is **SKAGER / SKAGER App** with
`skager.app` selected; domain control and legal clearance remain separate gates.
Preserve internal XNav/OpenNav namespaces where renaming adds technical risk,
and never modify the immutable original HTML to apply branding.

Before public launch, qualify one exact release revision across product, native
Windows, boat PC, installer/updater, source/licenses, website, commerce, portal,
support, privacy/security and operations. OpenCPN remains a separately installed
prerequisite. Unqualified XNav hardware output must be unavailable/default-off;
SmartNav never steers. Keep public payment/download access closed until the
full readiness report is reviewed and the user explicitly gives GO.

## Core rules

- Preserve standard OpenCPN as **Legacy Mode**.
- Implement **XNav**, **Legacy**, and **Safe Mode** against the same underlying OpenCPN navigation/user data.
- The initial mode switch may use a controlled restart. Reliability is more important than live re-parenting.
- Do not rewrite OpenCPN functionality which can be reused safely.
- Keep direct upstream modifications narrow, reviewable, and documented.
- UI, Vessel Data, SmartNav, OpenCPN integration, and hardware adapters must remain separate.
- Never invent unavailable sensor, chart, AIS, device, or navigation data.
- Missing, stale, invalid, estimated, and uncertain data must be represented explicitly.
- No autonomous steering in the initial product.
- Safety-relevant commands require appropriate confirmation/acknowledgement handling.
- Develop primarily on Linux.
- Native Windows MSVC build is mandatory for Windows-facing milestones.
- Windows rendering is authoritative for release UI acceptance.
- Wine or cross-compilation may assist development but does not replace native Windows validation.
- A feature is not complete merely because it compiles.
- Keep the project runnable after every meaningful milestone.
- Work in small vertical slices.
- Inspect actual OpenCPN source before assuming APIs, symbols, class structure, or plugin capabilities.
- Do not silently alter OpenCPN navigation semantics while redesigning UI.
- Do not modify an unsupported or unknown OpenCPN build.
- Preserve charts, routes, tracks, waypoints, plugins, connections, and configuration across XNav/Legacy switching and product updates.

## Before a major change

1. Read the relevant specification section.
2. Inspect the corresponding pinned OpenCPN source.
3. Identify existing functionality that can be reused.
4. Identify whether the work belongs in UI, Vessel Data, SmartNav, an adapter, or the OpenCPN integration bridge.
5. Define how the change will be tested on Linux.
6. Define whether it requires a Windows gate.
7. Implement the smallest maintainable change.
8. Build and test it.
9. Capture evidence where required.
10. Update architecture/upstream-patch documentation if boundaries changed.

## Visual development

This section defines how to perform an explicitly requested design review.
It does not authorize automatic visual validation during Staging delivery or
Production promotion. The 2026-10-04 delivery policy above controls scheduling.

Reference resolution: **1280×800**.

Also qualify **1920×1080**, as requested by the user on 2026-10-01. Preserve
the primary 1280×800 composition and existing DPI gates; the larger workspace
must keep the chart, primary values, alerts and panels usable. Record native
Windows and boat-display evidence before claiming the added resolution passes.

Compare implemented XNav screens against canonical renders of the HTML at
1280×800, device scale factor 1. The user superseded the older image-based
design policy on 2026-09-28: safety/navigation correctness, then HTML prototype,
then extracted prototype specification, then written brief, older image, and
current implementation. Match exact computed values and interactions; do not
redesign the prototype. Its illustrative values/geography and simulated
equipment behavior are not production data or navigation semantics.

When design validation is requested, native Windows and the boat display remain
the visual acceptance authorities. Linux font fallback screenshots do not
qualify Windows typography. Keep the UI native.

Do not approximate XNav using visibly native desktop controls where the design specifies an XNav component.

## Linux / Windows rule

Use Linux for:
- normal Codex work
- source editing
- OpenCPN Linux build
- SmartNav and Vessel Data development
- simulator work
- unit tests
- static analysis

Use native Windows for:
- authoritative MSVC build
- wxWidgets/platform integration
- DLL/plugin loading
- installer/update/repair/rollback/uninstall
- registry/UAC/shortcuts
- fonts and DPI scaling
- authoritative 1280×800 screenshots
- final UI acceptance
- target-hardware testing

## Upstream OpenCPN

Keep the selected upstream revision pinned.

Recommended repository location:

`upstream/OpenCPN/`

Document every direct upstream modification in:

`docs/upstream-patches.md`

Do not add a new OpenCPN version to the supported compatibility manifest until it has passed the required Windows regression gates.
