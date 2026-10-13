---
name: skager-jira
description: Read, select, triage and update SKAGER work in Navigare/SCRUM using the actual Jira board, issue metadata and evidence. Use for backlog coordination and customer-feedback intake design, not as authorization to implement, deploy or publish.
---

# SKAGER Jira

Jira [Navigare / SCRUM](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the sole development backlog. Keep technical evidence in the repository or
linked artifacts; do not create a competing roadmap or TODO system.

Locate the SKAGER checkout and read its `AGENTS.md`, `opennav-x/AGENTS.md`,
`opennav-x/docs/delivery-workflow.md` and the relevant sections of
`opennav-x/docs/public-beta-contract.md`. Paths are relative to that checkout,
not this installed skill. The later delivery policy takes precedence over older
channel/design scheduling rules. This skill does not authorize external writes
outside the user's task, builds, tests, automations, delivery or public access.

## Establish the live state

- Use the connected Atlassian tools. Inspect their current schemas; discover an
  operation before using a generic runner. Pass the site URL above as `cloudId`,
  or resolve accessible resources once and cache the selected site for the session.
- Read board configuration with `getJiraBoardConfig` (board 1) and its issue
  dataset with `getJiraBoardIssueData`; include backlog when selecting work.
  Currently: Idea → Idea, To Do → Att göra, In Progress → Pågående,
  Testing → Testning, Done → Klart. Refresh this mapping; status categories do
  not distinguish these columns (To Do also has an indeterminate category).
- Read In Progress, Testing and all unresolved `scope-public-beta` issues.
  Use mapped status IDs/names in JQL; do not substitute category queries for
  columns or assume that the resolution field reflects the workflow correctly.
  Complete JQL/board pagination with `nextPageToken` until `isLast` is true.
  If any page fails, report the listing as incomplete; do not report a complete
  count or make exhaustive backlog claims from the partial result.
- For selected issues, fetch acceptance criteria, linked dependencies and
  material comments. `getJiraIssue` does not provide complete comments; discover
  `listJiraIssueComments` and page with `startAt` until complete. Resolve current
  decisions from the latest relevant evidence, not one label or an old comment.

## Select and coordinate work

Default order: safety/security/data-loss blockers; public-beta launch blockers
at Highest; remaining public-beta Highest, High, Medium, then Low. Dependencies
can change eligibility. Leave post-beta/Future work for its intended scope.
Honor explicit user reprioritization within the project's safety boundaries.

Record the selected scope in Jira before unrelated implementation. Search for
existing work before creating anything; update or link the existing issue for
the same deliverable. Split distinct deliverables into bounded tasks/subtasks
without silently expanding scope. Retain separate release-scope and domain/type
labels. When delegating, use one coordinator for Jira writes and integration;
workers return their issue, owned files, results and remaining gates.

## Write without losing state

- Discover `listJiraProjectIssueTypesMetadata`, complete its pagination, and
  inspect `getJiraIssueTypeMetaWithFields` for the selected type and required
  fields before creation; reuse current session metadata until a schema change or
  rejection warrants refreshing it. Current types are Epic, Subtask, Feature and
  Uppgift; do not assume Bug or English Task exists. Respect connector defaults;
  resolve account IDs only when a requested/missing reporter or assignee requires
  it, and inspect parent requirements rather than inventing values.
- Before editing, read current fields and editable metadata. Multi-value edits
  replace the whole value: preserve unrelated labels, components and links.
  Preserve rich bodies/media using the format actually returned; read HTML
  before a lossless body edit and follow the tool's content-format guide.
- Fetch `listJiraIssueTransitions` for the issue immediately before moving it.
  Match the destination status, then use the available transition's ID. A status
  ID, transition ID and transition name are different things. Move to In Progress
  when implementing, Testing when required verification remains, Done only
  after the issue's acceptance criteria and required evidence pass.
- Use the comment tool for evidence notes; a comment supplied in transition
  `update` can be silently dropped. Retain failed evidence and open blockers.
  Record product commit, package/hash identity, CI links, performed validation
  and remaining gates; distinguish a later helper revision from the product
  revision whose exact bytes were tested. Compilation alone never proves Done.
- On errors, inspect repair hints and actual returned state. A create may succeed
  while sprint assignment fails; an edit may succeed while ranking fails; a
  transition may succeed while a sprint move fails. Retry only the failed part.
  After an unknown write outcome, reconcile by reading/searching before retrying
  to avoid duplicate issues/comments. Stop dependent writes if state is uncertain.

Ordinary delivery remains Staging. Keep both channel releases draft while public
access is closed. A Jira status, successful build or Production promotion does
not authorize publishing, payments/download access or claim release acceptance.

## Customer feedback and the missing Bug type

For intake design, follow the contract's website/Jira and bug/feature sections.
Customers use an authenticated portal backend, never Jira credentials in browser
code. Expose only the customer's authorized reports and sanitized status;
exclude private/internal comments, unrelated issues, secrets and private evidence.
JSM role/group restrictions alone do not make a comment internal; when using JSM,
select `internalNote` explicitly for an internal note and verify its visibility.

The bug contract requires a Jira **Bug**; current create metadata lacks that type.
Record/update the existing platform gap with metadata evidence and required schema
work. Do not silently relabel an Uppgift as satisfying the Bug requirement or
mark the intake/rehearsal acceptance complete. Future implementation should
discover the deployed schema and create Bug when supported. If the user explicitly
accepts a temporary intake fallback, use an available type with `source-web` and
`customer-bug`, preserve the report and link the open migration/schema issue;
describe it as temporary and retain the unmet Bug gate. Do not change the project
schema or substitute a type merely because this skill was invoked.

Capture product/OpenCPN versions, OS, area, reproduction steps, expected/actual
behavior and optional sanitized diagnostics/screenshots. Feature requests use
the discovered Feature type, `source-web` and `customer-feature-request` in
backlog. Triage can recommend priority/scope, link duplicates and add technical
notes within authorization; it does not automatically close customer reports or
deploy fixes.
