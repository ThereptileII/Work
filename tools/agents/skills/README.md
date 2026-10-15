# SKAGER Codex skills

Versioned source for the user-requested reusable development guides (SCRUM-371)
and provider workflows (SCRUM-372).
These are agent tooling, not application resources, test acceptance or a backlog.

| Skill | Use |
| --- | --- |
| [dev-skager](dev-skager/SKILL.md) | Plan/implement/debug SKAGER with source-backed development lessons |
| [skager-build](skager-build/SKILL.md) | Choose build inputs, verified SDK reuse and retained-output recovery |
| [skager-test](skager-test/SKILL.md) | Focused regressions, native fixture diagnosis and exact-byte retesting |
| [skager-jira](skager-jira/SKILL.md) | Select/update Jira work using live metadata and truthful evidence |
| [skager-web-publish](skager-web-publish/SKILL.md) | Cloudflare Free + Supabase invited-beta publication and TEST commerce |
| [skager-resend](skager-resend/SKILL.md) | Sending-domain DNS, Supabase SMTP and login-email delivery diagnosis |
| [skager-cloudflare](skager-cloudflare/SKILL.md) | Existing Worker/DNS publication and private R2 release access |
| [skager-supabase](skager-supabase/SKILL.md) | Auth, Edge gateway, private SQL/RLS and server configuration |
| [skager-stripe](skager-stripe/SKILL.md) | TEST Checkout, discount codes, signed settlement and entitlements |

Load only the relevant provider guide; `skager-web-publish` coordinates a full
publication. These project-specific guides supplement, not replace, the general
Cloudflare skills. Verify current source/provider state rather than exporting
SKAGER's configuration as a universal rule for other projects.

Install each named folder under `$CODEX_HOME/skills`, or `~/.codex/skills` when
unset, preserving its `SKILL.md`, `agents/` and conditional `references/` files.
Use a reviewed exact source revision. Refuse an existing destination until local
differences are inspected; do not overwrite personal changes or replace unrelated
skills. A plain copy is sufficient; no generated executable or runtime dependency
is installed. Restart a task/session if its available-skill catalog has not
refreshed. Automatic discovery remains enabled.

Validate each folder with the bundled skill-creator `scripts/quick_validate.py`;
also review links/metadata and realistic decisions. Validation does not invoke
SKAGER product tests. No credentials, private logs, boat records or keys belong in
these sources. Repository paths in instructions are resolved against the selected
checkout, while sibling skill/reference links remain valid after installation.

Skills are maintained instructions, not frozen live status: verify Git/CI/Jira
before claiming a candidate or provider deployment is accepted. Follow the current
project AGENTS and user delivery policy if these guides ever lag them. Desktop and
web candidate branches may differ; never combine their release identities.
