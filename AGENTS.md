# SKAGER workspace instructions

Read and follow [the project agent instructions](opennav-x/AGENTS.md) before
working anywhere in this repository. They define the Jira backlog, Staging
delivery policy, safety boundaries, model delegation and required evidence.
Internal administration, including creating skills and maintaining agent
instructions, needs no Jira ticket or board scan. Product work remains tracked
under the scope rule in the project instructions.

The scoped read-only Claude specialist is [tools/agents/claude-review](tools/agents/claude-review).
Follow [its workflow and isolation rules](tools/agents/README.md). Codex owns
final decisions and integration. Every delegated task must carry the existing
user authorization and its limits using the permission-handoff rule in the
project instructions; do not ask again for already authorized task scope.
External review does not replace the required
functional, native Windows, boat, security or release gates.

Reusable Codex skill sources are versioned in [tools/agents/skills](tools/agents/skills/README.md):
`dev-skager`, `skager-build`, `skager-test`, `skager-jira` and `skager-web-publish`.
Provider operations use `skager-resend`, `skager-cloudflare`, `skager-supabase`
or `skager-stripe` as relevant; `skager-web-publish` coordinates publication.
Use the relevant installed skill when available; otherwise read its source
entrypoint. Load only the needed specialty and conditional references. These
guides supplement project authority and never grant new deployment, physical
control or public-launch permission.
