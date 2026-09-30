# Web database foundation

Status: first implementation slice of SCRUM-94. This is a schema and migration
foundation, not completion of the database, identity, payment, entitlement,
download, support, backup, deployment, or staging issues.

## Implemented records

- `Account` identifies an Auth0 account only by unique immutable `(issuer,
  subject)`. Contact email is optional display/contact data and is not unique.
  There is no local user or password table.
- `Order` stores the server-selected owner, environment, product, currency and
  amount in minor units. Those terms are immutable. Zero is reserved for an
  explicitly authorized 100% provider discount; a future payment service must
  verify that total. `ProviderTransactionBinding` is a one-to-one, immutable
  binding whose environment must match its order.
- `VerifiedEventInbox` deduplicates `(environment, provider, provider event
  ID)`. Its digest, minimal normalized payload, verification time and receipt
  identity are immutable while processing state may advance. Inbox rows cannot
  be deleted, preserving their provider idempotency keys. It stores no raw
  webhook body.
- Each `Entitlement` is a grant from one order. This permits a later paid order
  to grant access after an earlier order is refunded. `EntitlementEffect`
  requires its verified event to share the entitlement environment, deduplicates
  each event/effect, and is append-only. Selection of the effective
  hosted entitlement, monotonic transitions, and delayed-event handling belong
  to the transactional entitlement service and are not implemented here.
- `ReleaseArtifact` is an immutable exact commit/binary/source record. `Release`
  publishes that artifact at a channel/version with that publication's signed
  manifest and signature object, allowing the same reviewed binary to move from
  beta to stable without reusing channel-specific signed metadata. Manifest and
  signature object keys are nonempty and globally unique across publications.
  `DownloadAudit` records allowed and
  denied checks; an allowed row must name an entitlement and that entitlement
  must belong to the account. Source and license locations are part of the
  artifact and remain conceptually public; URL-serving policy is not present.
- `SupportSubmission`, `JiraMapping`, and `JiraOutbox` provide the customer
  ownership and sanitized-command mapping needed by a later server-only Jira
  adapter. No Jira credentials or remote behavior exist.
- `AuditEvent`, entitlement effects, release records and download audits are
  append-only in PostgreSQL. Foreign keys use `PROTECT` where deletion could
  erase financial, ownership, release, or audit context.

The schema contains no card data, passwords, vendor secrets, raw bus data,
navigation data, telemetry, or email-based identity/purchase join. Application
services must allow-list every JSON payload field; database JSON columns are
not permission to persist raw vendor, Jira, or security-sensitive payloads.

## Versioned migrations and environments

`0001_initial_schema` creates the portable model structure and constraints.
`0002_postgresql_integrity` adds PostgreSQL ownership and immutability triggers.
`0003_close_reviewed_integrity_gaps` enforces effect/event environment matching,
retains webhook receipts against deletion, and constrains signed release references.
The production settings require explicit hosts, a strong secret, PostgreSQL
credentials, `sslmode=verify-full`, and a trusted CA path. They reject wildcard
hosts and expose no URL routes.

Development, staging and production must use separate databases and vendor
environments. A migration moves toward production only through this process:

1. CI checks migration drift and runs migrations plus integrity tests against
   a fresh PostgreSQL database.
2. Apply the reviewed migration to staging, then run schema checks and the
   complete synthetic checkout/refund/download/support journey when those
   services exist. Record failures instead of promoting them.
3. Confirm a current automated backup and complete a timed restore rehearsal
   into an isolated database. Verify order, entitlement, release and audit
   integrity there.
4. Apply backward-compatible migrations to production before the application
   version that needs them, observe migration and health checks, and retain the
   preceding application image for rollback. Destructive migrations require a
   separately reviewed, tested data and recovery plan.

Automated backups, retention, point-in-time recovery, restore tooling, staging,
deployment, health checks, reconciliation, privacy export/deletion and incident
procedures are not implemented in this slice. They remain SCRUM-94 and linked
launch gates; this document does not claim a restore or staging rehearsal.

## Validation baseline

The initial migrations and eight integrity cases passed on a disposable,
Unix-socket-only PostgreSQL 18.6 cluster on 2026-09-30. The tested cases cover
identity and webhook idempotency, money and immutable order terms, provider
binding, ownership, denied and allowed download audit structure, append-only
audit data, repurchase after refund, and artifact promotion. They test database
invariants only; no paid/refund/out-of-order service workflow exists yet.

After independent review, exact merged revision `d0105a1` passes all eleven
integrity cases through loopback TCP with SCRAM authentication, plus fresh
migration, reverse to 0001 and reapply through 0003. See
[retained local evidence](../evidence/scrum-94-schema-local.json). Actual GitHub
CI and the open operational gates above remain required.

Primary references used to select the pinned stack:

- <https://docs.djangoproject.com/en/5.2/releases/5.2/>
- <https://docs.djangoproject.com/en/5.2/ref/databases/#postgresql-notes>
- <https://pypi.org/project/Django/5.2.17/>
- <https://pypi.org/project/psycopg/3.3.6/>
