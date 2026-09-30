# SKAGER / OpenNav X Agent Instructions

The current product goal is:

`PROJECT_GOAL.md`

The authoritative architecture/product specification is:

`OpenNavX_Codex_Project_Specification.md`

The approved visual references are under:

`docs/design/`

Jira project **Navigare / SCRUM** is the authoritative backlog, priority and release-status source.

## Operating model

At the beginning of a meaningful work cycle:

1. Read the relevant Jira sprint/backlog state and the selected issue.
2. Read the issue's latest comments/evidence and dependencies.
3. Confirm that public-beta blockers have priority over post-beta/future scope.
4. Select the smallest bounded task that can make real progress.
5. Choose the appropriate model/subagent tier using the routing policy below.
6. Work in an isolated branch/worktree when parallel work could mutate a candidate under qualification.
7. Run proportionate checks.
8. Record commit/test/evidence status in Jira.
9. Move an issue to Testing only when implementation is complete and acceptance remains.
10. Move to Done only when its acceptance criteria have actually passed.

Do not create a private roadmap outside Jira.

## Model and subagent routing

Use multiple small subagents aggressively where work can be decomposed cleanly.

### GPT-6 Luna — small, concrete tasks

Prefer **GPT-6 Luna** for narrowly scoped work with clear inputs/outputs and low architectural ambiguity.

Examples:
- inspect one file or one API;
- update a focused test;
- implement a small UI component adjustment;
- fix one deterministic bug;
- add a migration constraint;
- write/refresh bounded documentation;
- gather exact source/dependency provenance;
- prepare a small fixture or script;
- make a mechanical refactor with clear acceptance criteria;
- investigate one failing assertion with preserved evidence.

Luna tasks should normally:
- own one Jira subtask or one clearly bounded increment;
- avoid broad architecture decisions;
- return a concise result, changed files, verification and any blocker;
- escalate rather than guessing when scope expands materially.

### GPT-6 Sol — normal and substantial engineering

Use **GPT-6 Sol** for most feature implementation and technical work requiring non-trivial reasoning across several files/components.

Examples:
- implement a complete Jira Feature slice;
- refactor a subsystem while preserving contracts;
- integrate UI + presentation model + tests;
- implement installer/updater logic;
- authentication/backend service work;
- chart/OpenCPN integration;
- multi-platform debugging;
- dependency replacement once the boundary is understood;
- combine several Luna outputs into a coherent implementation.

Sol is the default for work that is not obviously tiny and mechanical.

### Astra — exceptional complexity and cross-cutting decisions

Use **Astra** only for genuinely difficult, high-consequence or deeply cross-cutting work.

Examples:
- architecture with multiple viable approaches and significant long-term consequences;
- complex safety/security threat analysis;
- difficult native Windows/OpenCPN/wxWidgets failures spanning several subsystems;
- release-critical integration with ambiguous root cause;
- major migrations or compatibility strategy;
- review/decision work where several Sol/Luna results conflict;
- final synthesis of large multi-agent work before a high-risk merge.

Do not spend Astra on routine implementation or ordinary debugging.

### Routing rules

- Start with the **smallest capable model**, not the largest available model.
- Split broad work into independent Luna/Sol subtasks where doing so reduces risk and improves throughput.
- Do not split work merely to maximize agent count; each subagent must have a crisp ownership boundary.
- One agent must own final integration for any change spanning multiple agents.
- A subagent must not silently broaden its Jira scope.
- Escalate Luna → Sol → Astra when complexity genuinely increases.
- If the named model/tier is unavailable in the current Codex runtime, use the closest available capability tier and record the substitution in the Jira work note rather than blocking progress.
- Model choice never relaxes test, security, safety or evidence requirements.

## Core product rules

- Preserve standard OpenCPN as **Legacy Mode**.
- Implement **XNav**, **Legacy** and **Safe Mode** against the same underlying OpenCPN navigation/user data.
- Do not rewrite OpenCPN functionality that can be reused safely.
- Keep direct upstream modifications narrow, reviewable and documented.
- Keep UI, Vessel Data, SmartNav, OpenCPN integration, hardware adapters and hosted services separated.
- Never invent unavailable sensor, chart, AIS, device, route, account or navigation data.
- Missing, stale, invalid, estimated and uncertain states must be explicit.
- No autonomous steering in the public-beta product.
- Unqualified safety-critical output must remain unavailable/status-only.
- Do not silently change OpenCPN navigation semantics while redesigning UI.
- Do not modify an unsupported or unknown OpenCPN build.
- Preserve charts, routes, tracks, waypoints, plugins, connections and configuration across XNav/Legacy switching and product updates.
- Core onboard navigation must continue when hosted services are offline.

## Before a major change

1. Read the Jira issue and acceptance criteria.
2. Read the relevant product/specification section.
3. Inspect the corresponding pinned OpenCPN/project source.
4. Identify functionality that can be reused.
5. Identify the owning architectural layer.
6. Identify safety, security, data-loss, licensing and compatibility implications.
7. Decide the smallest implementation boundary.
8. Decide the model/subagent decomposition.
9. Define focused development checks.
10. Define any mandatory Windows/boat/release gate.
11. Implement.
12. Review integration.
13. Record evidence and Jira status.

## Visual development

Reference resolution: **1280×800**.

The approved HTML prototype/design artifacts are the visual/interaction contract for primary XNav screens.

Do not approximate XNav using visibly native desktop controls where the design specifies an XNav component.

Prototype values are illustrative only. Never fake live product data to match a screenshot.

Validate major visual states in:
- Day;
- Dusk;
- Night;
- supported DPI/scaling;
- native Windows;
- physical boat-PC 1280×800 where required.

## Linux / Windows rule

Use Linux for:
- normal Codex work;
- source editing;
- OpenCPN Linux build;
- SmartNav/Vessel Data work;
- backend/web development;
- simulator work;
- unit/contract tests;
- static analysis;
- fast local UI/component iteration.

Use native Windows for:
- authoritative MSVC build;
- wxWidgets/platform behavior;
- DLL/plugin loading;
- dependency/import closure;
- installer/update/repair/rollback/uninstall;
- registry/UAC/shortcuts;
- fonts and DPI scaling;
- authoritative XNav screenshots;
- XNav/Legacy/Safe transitions;
- target-hardware and boat-PC qualification.

Wine/cross-compilation may assist development but never substitutes for required native Windows acceptance.

## Test strategy

Do not make every small change wait for the complete release matrix.

For reversible non-critical work:
- focused relevant tests;
- targeted smoke/visual review;
- diff/static checks where appropriate.

Run broader suites at meaningful integration boundaries.

Never reduce required gates for:
- navigation correctness;
- safety-critical behavior;
- security/authentication/payment;
- installer/updater/recovery;
- data-loss/migration risk;
- cryptographic/signing/update trust;
- dependency security;
- GPL/source/license publication;
- exact release acceptance;
- final Windows/boat qualification.

Do not weaken thresholds, add arbitrary retries or mask failures simply to make CI green.

Retain negative evidence until the replacement is independently accepted.

## Candidate isolation

When a candidate is undergoing CI/native/boat qualification:
- treat its source as immutable;
- perform unrelated/new fixes in an isolated worktree/branch;
- do not cancel valid evidence merely by publishing another candidate unless intentionally required;
- never transfer acceptance from one commit to another.

## Hardware safety

Remote/read-only inspection is allowed where already authorized.

Do not send physical autopilot, propulsion, radar-transmit or other safety-critical commands without explicit user authorization for that test.

Preserve Tailscale, SSH and RustDesk access on the boat PC. Do not make remote-access/network changes that risk losing recovery access.

## Upstream OpenCPN

Keep the selected upstream revision pinned.

Recommended repository location:

`upstream/OpenCPN/`

Document every direct upstream modification in:

`docs/upstream-patches.md`

Do not add a new OpenCPN version to the supported compatibility manifest until it has passed the required native Windows regression gates.

## Release discipline

A passing unit test is not a release.

Public Beta requires:
- exact source/candidate identity;
- required Linux/native Windows gates;
- UI/boat acceptance;
- security/dependency closure;
- installer/updater/recovery qualification;
- source/license artifacts;
- staging service acceptance;
- legal/operational readiness;
- explicit human GO/NO-GO.

Codex may prepare release evidence, but must not autonomously declare or publish a production/public release unless explicitly authorized.
