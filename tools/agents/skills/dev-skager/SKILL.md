---
name: dev-skager
description: Plan, implement and diagnose SKAGER marine-navigation changes using its Jira backlog, OpenCPN integration boundaries and lean Staging workflow. Use for SKAGER development and process decisions; route building, qualification, Jira operations and website deployment to their dedicated skills.
---

# Develop SKAGER

Deliver a useful, runnable increment with evidence proportional to the change.
This skill captures working methods, not a second backlog or release approval.

## Establish current authority

Find the intended Git checkout from the active workspace/task attachments. Confirm
it contains `opennav-x/AGENTS.md`; do not assume the shell starts inside the repo
or that a historical worktree is still current. Read root/project `AGENTS.md`,
`opennav-x/docs/delivery-workflow.md` and the selected Jira issue. Use
`opennav-x/docs/status.md` as a handoff, then verify drift-prone claims against
current Git, CI and Jira. Skills supplement these authorities; later explicit
user instructions win.

For a substantial cycle use [skager-jira](../skager-jira/SKILL.md) to select and
record work. Respect an explicit user reprioritization rather than silently
adding unrelated launch work. Inspect relevant pinned OpenCPN source before
changing its integration; preserve navigation/storage semantics and small
integration hooks. Do not build a parallel route, AIS or connection database.

## Make one coherent increment

- Inspect source and failed evidence before choosing a fix. Separate a product
  defect, test-fixture defect, build-input defect and transport failure;
  compilation or a helper exception alone does not identify an application crash.
- Split independent work into small bounded assignments when delegation is
  authorized. Give each an issue, isolated worktree, exclusive files and concrete
  acceptance. Root alone integrates, requests full builds and operates the boat.
  Check current model availability and AGENTS rather than pinning old model names
  here. No agent independently starts duplicate builds or deliveries.
- Use the existing scoped `tools/agents/claude-review` for a useful independent
  question. Read `tools/agents/README.md`; retain snapshot, transfer permissions,
  read-only tools and budget limits. Do not bypass a rejected transfer or share
  credentials/boat data. Claude advice is not acceptance or authority to deploy.
- Review combined changes before committing. Preserve user changes and keep
  runtime source separate from diagnostic artifacts and secret storage.

## Choose the next check, not every check

Use [skager-test](../skager-test/SKILL.md) for targeted verification and
[skager-build](../skager-build/SKILL.md) when actual build/package inputs changed.
Batch full suites at integration/delivery. Ordinary docs or helper corrections
do not justify a fresh application build. Corrected helpers can retest retained
exact bytes through the supported authenticated workflow; never relabel them.

Staging is the default. Production readiness/promotion requires explicit user
instruction for a selected candidate, reusing qualified package bytes. Do not
start prototype comparison, aesthetic refinement or design-only DPI sweeps
without an explicit request. Functional chart content, readable alerts and usable
controls remain required. Do not invent design acceptance for unperformed work.

Long-running qualification requires explicit authorization. When requested, run
it separately in the background and continue independent development; freeze its
inputs and retain logs. Pending is not passed. Honor the documented endurance
skip until the user changes it. A notification request needs a real app heartbeat
monitor, not a promise after the agent stops; keep it quiet until actionable or
complete and end the one-off monitor after reporting.

## Preserve the vessel and release boundary

Never convert missing/stale/invalid data into zero or live information. Owned
snapshots must not retain OpenCPN pointers. SmartNav remains advisory. Read-only
boat observation does not authorize actuator commands; use the current specific
user authorization and commissioning contract for any physical output. Preserve
SSH/Tailscale/RustDesk and a working Legacy/Safe escape path.

The approved Windows application/plugin ABI is Win32 on Windows x64 unless the
user explicitly changes it. Validate exact OpenCPN bytes against the current
manifest before installation, with verified recovery and user-data preservation.
Public downloads/payments stay closed until separate readiness and human GO;
Staging, promotion, or website deployment does not imply public-launch approval.

Report changes, focused results, product/helper revision distinction and remaining
gates. Keep Jira status consistent with actual acceptance. For recurring failures,
consult [source-backed lessons](references/development-lessons.md) instead of
repeating a full build to obtain the same diagnosis.
