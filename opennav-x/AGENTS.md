# OpenNav X Agent Instructions

The authoritative product specification is:

`OpenNavX_Codex_Project_Specification.md`

The authoritative visual reference is the unchanged supplied HTML prototype:

`docs/design/prototype/index.html`

Its original-file manifest is `docs/design/prototype-original.json`. Never edit
those original files. Put instrumentation in tools or a separate derived copy.

The current project goal is:

`PROJECT_GOAL.md`

## Jira-driven public-beta work

[Navigare / SCRUM board](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the sole development backlog. The public-beta requirements are captured in
`docs/public-beta-contract.md`; that contract and technical evidence are not a
second roadmap. Do not maintain a competing private TODO list.

At the beginning of each substantial cycle:

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

Use many small, bounded tasks when the work can be split cleanly: GPT-5.6-Luna
handles straightforward tasks, GPT-5.6-Sol handles heavier tasks, and the root
agent handles exceptional complexity and integration decisions. Tie every task
to its Jira issue and record explicit ownership and acceptance criteria. Keep
context limited and handoffs concise.

For reversible, noncritical documentation, copy, layout, or function changes,
run focused checks relevant to the change and a smoke or visual review. Avoid
redundant broad reruns and tests that only mirror the implementation. Preserve
all navigation, safety, data-loss, security, authentication, payment,
installer, updater, recovery, exact-revision, native Windows, boat, release,
and source-compliance gates. Do not remove or weaken existing failure
assertions. Batch full suites at integration and release milestones instead of
running them for every minor edit. Keep Jira as the sole backlog; do not create
a TODO file.

`SCRUM-97` governs this workflow. `SCRUM-14`, `SCRUM-15` and `SCRUM-16` retain
the existing prototype, chart-presentation and Online AIS acceptance gates.
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

Native Windows and the boat display remain visual acceptance gates. Linux font
fallback screenshots do not qualify Windows typography. Keep the UI native.

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
