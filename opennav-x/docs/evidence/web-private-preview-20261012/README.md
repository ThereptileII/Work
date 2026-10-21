# Closed website increment

Date: 2026-10-12. Jira: SCRUM-33/35/38/55/60/64.

Website implementation: `c447e80ea60e3f83423703e8b480d540403c1da2`.
Web-only CI workflow revision: `16934d4e42244aec8d6333b82309a065c1d98035`.
These commits are on isolated `codex/web-beta-launch-20261012`; they do not
qualify a new native application candidate.

[CI run 38306803251](https://github.com/ThereptileII/Work/actions/runs/38306803251)
passes: **38 tests**, including PostgreSQL integrity/concurrency, authentication
and the seven marketing-page cases; no skips reported. Django checks, migration
drift, fresh migrations and reverse/reapply all pass. The existing web workflow
now includes marketing tests and isolated `codex/web-*` branches. It requests
no native application build. Eleven focused page/configuration tests also passed
locally; these overlap CI and are not added to its count.

Owner-private static design preview:
https://skager-beta-launch-preview.swedishcountrysideli.chatgpt.site

Sites project: `appgprj_6acd44a03b60819192aa8cde1e7b719b`.
Generated preview source: `811365ac36a9874d73f996ef8ce54c64921ccd48`.
Deployment: `appgdep_6acd482c0f548191a33f65b4a6d87b24`, succeeded.
Archive SHA-256:
`2b3c2f68959fc159a20f023fa4d2c54d64a869bc7d47caa0fc01f848610dac9d`.
Only 20 allowlisted rendered informational pages, their owned assets and exact
source/asset provenance are exported. No backend/private native source,
credentials, boat location or licensed chart files are included.

Browser smoke: local desktop homepage/pricing/help rendering; help search for
“transducer” leaves exactly one matching guide; phone viewport 390×844 displays
the responsive header/menu; menu opens and Escape closes it. The temporary
viewport was reset. [Desktop capture](home-desktop.jpg) is the local Django page,
not a screenshot taken after owner authentication at the deployed URL. The
deployed URL correctly requires ChatGPT sign-in; this browser did not have an
authenticated owner session. Owner-side post-login review remains pending.

The approved wordmark is unchanged. Chart artwork is an explicitly labelled
illustration, not an actual product screenshot. Real release screenshots remain
part of release content qualification. No fake vessel measurements are used.

Public checkout/downloads/account collection, ticket intake, analytics and vendor
services are closed/absent. Auth0 remains disabled by default. Legal/privacy pages
are review drafts. This establishes neither commerce readiness nor public-beta
GO. Selected production Django/PostgreSQL/Render architecture is unchanged.

Separate concurrent software helper: `d01a2618`, eight focused inert tests pass;
initial protected B0 native admission and B1 lifecycle still require their
independent prerequisites. Component product remains `4966e8ad` and passing
run38302791125, not the later helper revision. No boat state was changed.
