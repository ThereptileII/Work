# Goal: Take the project from development prototype to a production-ready paid public beta

The current internal project name **XNav / OpenNav X is temporary**.

The overall goal of this development phase is to turn the existing software into a complete, credible, secure and maintainable **public beta product**, including the Windows application, installer/update infrastructure, commercial website, customer portal, support system, legal/open-source compliance, release infrastructure and launch material.

The work is complete when a member of the public can discover the product, understand what it is, pay for beta access, install it on a supported Windows/OpenCPN system, use it successfully, receive updates, obtain support, report bugs/request features, and access all required legal and open-source information without manual intervention from the developer.

---

# 1. Jira is the source of truth

The Jira project:

```text
Navigare
Project key: SCRUM
Board: SCRUM board
```

is the authoritative development backlog.

Do not maintain a competing private roadmap or TODO system outside Jira.

At the beginning of every substantial work cycle:

1. Read the Jira board.
2. Inspect current `In Progress` and `Testing` work.
3. Inspect unresolved `scope-public-beta` work.
4. Check blockers and dependencies.
5. Select the highest-value eligible issue according to the rules below.
6. Update Jira before beginning unrelated work.

If implementation discovers additional required work:

> create or update the appropriate Jira issue.

Do not silently expand scope without representing the work in Jira.

---

# 2. Jira workflow

Use the board states as follows:

```text
Idea
Captured requirement or future idea.

To Do
Approved and ready to implement.

In Progress
Codex is actively implementing it.

Testing
Implementation is complete but verification or acceptance remains.

Done
Acceptance criteria and required evidence have passed.
```

Do not move an issue to Done merely because the code compiles.

---

# 3. Work prioritization

Default priority order:

```text
1. safety/security/data-loss blockers
2. scope-public-beta + launch-blocker + Highest
3. other scope-public-beta Highest
4. scope-public-beta High
5. Medium
6. Low
7. scope-post-beta
8. scope-future
```

Dependencies may modify this order.

Do not begin Future features while unresolved Public Beta launch blockers remain unless the user explicitly reprioritizes them.

Examples of Future work include Cloud Sync, cloud backup, remote diagnostics and mobile companion functionality.

Preserve them in Jira, but do not let them derail launch.

---

# 4. Work already in progress

The current prototype/UI work must be finished rather than abandoned.

Current major workstreams include:

```text
SCRUM-14
HTML prototype conformance

SCRUM-15
XNav chart presentation

SCRUM-16
Online AIS / AISStream

SCRUM-97
Jira-driven Codex development workflow
```

Complete these according to their Jira acceptance criteria before treating the current UI stage as finished.

The supplied HTML prototype remains the authoritative visual reference.

---

# 5. Choose the final product identity

Before public launch, replace the development name XNav/OpenNav X with a real product name.

Treat this as a product/brand decision, not a casual code rename.

Required work includes:

* shortlist strong product names;
* search for obvious marine/software naming conflicts;
* check practical domain availability;
* assess trademark risk;
* select a final product name;
* secure the primary domain;
* define wordmark/logo direction;
* define how the product credits OpenCPN;
* create a migration plan from internal XNav terminology.

Do not unnecessarily rename low-level internal namespaces if doing so creates technical risk.

The customer-facing product, installer, website, marketing and documentation must use the final product name consistently before launch.

---

# 6. Finish the Windows product

The Windows application must reach a production-quality public-beta state.

This includes:

* prototype-conformant native UI;
* XNav chart presentation;
* Standard S-52 fallback;
* real charts;
* routes and waypoints;
* AIS;
* AISStream online AIS;
* instruments;
* energy;
* alerts;
* anchor mode;
* settings;
* system health;
* diagnostics;
* Legacy Mode;
* Safe Mode;
* physical boat-PC validation;
* real data-source validation;
* robust stale/unavailable semantics.

Do not add major new navigation features unless Jira identifies them as required launch blockers.

Prefer finishing, testing and polishing existing functionality.

---

# 7. Safety-critical capabilities

No unqualified safety-critical control path may ship accidentally.

This includes:

* autopilot commands;
* propulsion commands;
* radar transmitter control;
* switching outputs;
* other physical actuators.

A capability may be included in the public beta only when its specific safety boundary and commissioning requirements have been satisfied.

Otherwise it must be:

```text
disabled
unavailable
or clearly status-only
```

by default.

Never interpret public-beta completion as permission to add autonomous steering or autonomous collision avoidance.

---

# 8. Physical boat-PC qualification

The actual Windows navigation PC is a required acceptance environment.

Before release, validate:

* real Windows install;
* supported OpenCPN installation;
* real chart database;
* real ENC rendering;
* 1280×800 target display;
* physical touchscreen;
* actual GPU/OpenGL path;
* navigation profile preservation;
* real NMEA/N2K inputs;
* AIS where available;
* Online AIS;
* mode transitions;
* repair/update/rollback;
* recovery.

CI does not substitute for boat-PC acceptance.

Do not modify Tailscale, SSH or RustDesk in ways which risk remote access.

---

# 9. OpenCPN remains a prerequisite

Do not bundle OpenCPN into the public XNav installer.

The customer flow is:

```text
Install supported OpenCPN
        ↓
Install the product
        ↓
Installer verifies compatible OpenCPN
        ↓
Product integrates with the existing installation
```

The installer must detect and validate the supported OpenCPN version/configuration.

If the installed version is unsupported:

do not modify it.

Provide clear instructions instead.

---

# 10. OpenCPN attribution

OpenCPN must be credited respectfully and transparently.

The website should explain that the product builds upon / integrates with OpenCPN and requires OpenCPN to be installed separately.

Provide links to the official OpenCPN website/source.

Do not imply that OpenCPN endorses the product unless explicit permission exists.

Attribution should also appear where appropriate in:

* About;
* installer;
* Legal/Open Source;
* documentation;
* website.

---

# 11. Production installer

The public-beta installer must feel like a real commercial Windows product.

It must support:

* supported OpenCPN detection;
* installation;
* version validation;
* update;
* repair;
* rollback;
* uninstall;
* recovery.

The customer should not need developer knowledge to install the product.

All destructive operations must be transactional and recoverable.

---

# 12. Automatic updater

Implement the startup updater specified in Jira.

Expected user flow:

```text
Launch product
      ↓
check for update
      ↓
new version available?
      ↓
startup update popup
      ↓
Update Now / Later
```

The decision happens before XNav enables its own control-capable hardware adapters.

If Later is selected:

continue startup and do not interrupt that session again.

Support:

* Beta/Stable channels;
* signed/verified manifests;
* SHA-256;
* package verification;
* transactional update;
* startup-success handshake;
* automatic rollback;
* offline operation;
* manual rollback.

Never execute an unverified update.

---

# 13. Release infrastructure

One exact release commit must produce the complete release set.

At minimum:

```text
Setup.exe
Portable-Recovery.zip
corresponding-source.zip
SHA256SUMS.txt
update manifest
manifest signature
release notes
license/notice package
```

Every binary must be traceable to its exact source revision.

Do not publish artifacts from mixed commits.

---

# 14. Paid public beta

Initial commercial offer:

```text
Public Beta Access
USD $20
```

Support discount/promo codes.

The purchase grants access to the official product ecosystem, including as appropriate:

* official beta downloads;
* official update channel;
* customer portal;
* knowledge base;
* support;
* bug reporting;
* feature requests;
* roadmap/release information.

Do not frame payment as removing GPL rights from software the customer has received.

---

# 15. Website

Build a production website for the final product.

It must visually belong to the same family as the application.

Required areas include:

```text
Home
Features
How it works
Screenshots
Pricing
OpenCPN prerequisite
FAQ / Support
Login
Customer portal
Open Source
Legal
```

The site must be:

* responsive;
* fast;
* accessible;
* secure;
* production deployable;
* maintainable by a small team.

---

# 16. Website visual quality

The website should feel like the web version of the software.

Reuse the visual principles from the XNav prototype:

* dark premium marine aesthetic;
* clean typography;
* restrained cyan/navigation accent;
* excellent information hierarchy;
* high-quality motion;
* sophisticated but not gimmicky presentation.

Do not use a generic SaaS template aesthetic if it conflicts with the product identity.

---

# 17. Promotional assets

Create professional launch assets.

Required examples:

* real product screenshots;
* photorealistic product-on-boat images;
* chartplotter/helm installation scenes;
* hero imagery;
* social/launch imagery;
* promotional video.

The promotional video should move between realistic sailing scenes and product functionality.

Example narrative:

```text
boat underway
      ↓
route/navigation
      ↓
changing conditions
      ↓
instruments
      ↓
nearby traffic
      ↓
AIS
      ↓
energy/range awareness
      ↓
XNav helm display
```

Do not depict functionality which does not actually exist.

Do not imply autonomous navigation where none exists.

---

# 18. Customer account system

Implement secure account/authentication infrastructure.

Required capabilities include:

* account creation;
* login;
* account recovery;
* secure sessions;
* entitlement lookup;
* purchase association;
* account management.

Prefer proven authentication infrastructure over custom password security.

---

# 19. Customer entitlement

Payment must create a server-side entitlement.

The entitlement controls access to hosted commercial services such as:

* official download portal;
* official updater channel;
* paid support resources;
* customer feedback features.

The entitlement system must be:

* auditable;
* idempotent;
* resilient to webhook retries;
* compatible with refunds;
* secure.

Do not use entitlement to falsely revoke GPL rights to already distributed GPL-covered code.

---

# 20. Store and payments

Implement production checkout through an appropriate provider.

Prefer a Merchant of Record if this materially reduces VAT/tax/compliance overhead.

Required support includes:

* $20 purchase;
* discount codes;
* successful payment;
* failed/cancelled payment;
* refund;
* webhook verification;
* idempotency;
* receipt/invoice path;
* entitlement provisioning.

Never process or store raw card details directly.

---

# 21. Customer portal

Provide an authenticated customer area.

It should contain at minimum:

```text
Account
Beta access status
Downloads
Latest release
Release notes
Support
FAQ
My bug reports
My feature requests
Billing/order information
```

Keep the interface simple and visually consistent with the marketing site.

---

# 22. Website → Jira integration

Jira is the internal development system.

Customers should not need Jira accounts.

The customer portal backend should mediate all communication with Jira.

Never expose Jira credentials to browser clients.

---

# 23. Bug reporting

A paid beta user should be able to select:

```text
Report a bug
```

and submit structured information.

The backend should create a Jira **Bug**.

Include appropriate metadata such as:

* source-web;
* customer-bug;
* product version;
* OpenCPN version;
* operating system;
* area;
* description;
* reproduction steps;
* expected result;
* actual result;
* diagnostic bundle where supplied;
* screenshots where supplied.

Do not expose private Jira comments or unrelated issues to the customer.

---

# 24. Feature requests

A paid beta user should be able to select:

```text
Request a feature
```

The backend creates a Jira **Feature** in backlog with:

```text
customer-feature-request
source-web
```

and suitable product-area labels.

Provide a customer-facing sanitized status in the portal.

---

# 25. Release planning taxonomy

Use release-scope labels consistently:

```text
scope-public-beta
scope-beta-patch
scope-post-beta
scope-future
```

Use domain/type labels separately.

Examples:

```text
customer-bug
customer-feature-request
security
crash
data-loss
ui-ux
installer
updater
charts
ais
nmea
autopilot
web
portal
billing
```

Do not overload one label with multiple meanings.

---

# 26. Daily Codex feedback triage

Build the Jira-driven feedback process specified in the backlog.

A daily Codex workflow should review newly created or materially updated:

```text
customer-bug
customer-feature-request
```

issues.

The workflow may:

* summarize;
* categorize;
* identify likely duplicates;
* identify likely regressions;
* recommend priority;
* recommend release scope;
* add technical investigation notes;
* link relevant known issues.

It must not automatically:

* close customer issues;
* deploy fixes;
* dismiss safety issues;
* downgrade security/data-loss issues;
* fabricate reproduction evidence.

---

# 27. Knowledge base and support

Create a searchable support library.

Cover at least:

* OpenCPN prerequisite;
* installation;
* update;
* rollback;
* uninstall;
* charts;
* AIS;
* Online AIS;
* NMEA/N2K;
* system health;
* recovery;
* diagnostics;
* known limitations;
* licensing/open source.

Support content should be version-aware.

---

# 28. Open-source compliance

Treat licensing compliance as a release gate.

OpenCPN contains GPL and other licensed components.

Maintain:

* applicable license texts;
* copyright notices;
* third-party notices;
* dependency/license inventory;
* corresponding source;
* exact release/source mapping.

Do not publish a binary if its corresponding source cannot be identified and supplied as required.

---

# 29. Source-code publication

Public release pages should provide a clear path to the corresponding source.

It does not need to dominate marketing.

A suitable flow is:

```text
Footer
→ Open Source / Legal
→ Source code
→ version-specific source
```

Also expose the source/legal location from the application About/Legal screen.

Keep source artifacts tied to exact release versions.

---

# 30. Legal documents

Prepare production-ready drafts for legal review.

Required documents include:

* Terms of Service;
* Privacy Policy;
* Refund Policy;
* Public Beta terms/disclaimer;
* Open Source / Third-Party Notices;
* source-code information;
* cookie/analytics information where applicable.

These documents must not impose restrictions which conflict with the applicable open-source licenses.

Before accepting real public payments, flag these documents for qualified legal review.

---

# 31. Privacy / GDPR

Design the system according to data minimization.

Document and implement:

* what account data is collected;
* payment-provider references;
* support data;
* diagnostic uploads;
* logs;
* analytics;
* retention periods;
* deletion;
* export;
* processors.

Navigation position or vessel telemetry must not become marketing analytics.

---

# 32. Production backend

Implement the minimum backend required for:

```text
authentication
entitlements
orders/payment webhooks
downloads
update authorization where applicable
Jira feedback bridge
support metadata
audit state
```

Use versioned database migrations.

Maintain clear trust boundaries.

Do not make the browser a trusted backend.

---

# 33. Secrets

No production secrets may live in Git.

Protect at minimum:

* payment secrets;
* Jira credentials;
* authentication secrets;
* database credentials;
* signing keys;
* deployment credentials.

Use a proper secrets manager / protected CI environment.

Enable secret scanning where practical.

---

# 34. Security

Before public launch perform a focused security review covering:

* authentication;
* session handling;
* authorization;
* payment webhooks;
* entitlement manipulation;
* download URLs;
* updater;
* upload handling;
* Jira integration;
* admin functions;
* API rate limiting;
* XSS;
* CSRF;
* SSRF;
* secrets leakage;
* replay/idempotency.

Block launch on unresolved critical security findings.

---

# 35. Operations

Create production operations infrastructure for:

* hosting;
* DNS;
* TLS;
* deployments;
* monitoring;
* logs;
* alerts;
* backups;
* restore;
* incident handling;
* rollback.

Provide separate:

```text
development
staging
production
```

environments where appropriate.

---

# 36. Staging launch rehearsal

Before production launch, run the entire customer journey in staging.

Required end-to-end rehearsal:

```text
visit homepage
→ pricing
→ checkout
→ payment
→ account
→ entitlement
→ download
→ OpenCPN prerequisite
→ XNav install
→ startup
→ updater
→ submit bug
→ Jira receives Bug
→ submit feature request
→ Jira receives Feature
→ portal shows status
→ refund
→ entitlement updates
```

Do not open the public beta until this flow succeeds.

---

# 37. Public beta release candidate

Once all implementation work is present:

enter feature freeze.

Do not add interesting new product features at this stage.

Focus on:

* defects;
* UX inconsistencies;
* installation;
* reliability;
* security;
* performance;
* recovery;
* documentation.

Create one explicit Public Beta Release Candidate.

---

# 38. Release candidate qualification

The release candidate must pass relevant:

* Linux CI;
* native Windows CI;
* fixture-free builds;
* installer lifecycle;
* upgrade/rollback;
* chart tests;
* DPI tests;
* touch tests;
* endurance tests;
* boat-PC tests;
* real data tests;
* source/package validation;
* security/compliance checks.

Failures must be investigated rather than hidden by weakening tests.

---

# 39. Launch-blocker policy

An unresolved issue labelled:

```text
launch-blocker
```

prevents public beta release unless the user explicitly accepts the risk.

Automatically defer nothing which involves:

```text
security
safety
data-loss
installer corruption
update corruption
source/license compliance
payment integrity
authentication bypass
```

without explicit human approval.

---

# 40. Future backlog

Retain, but do not implement merely because they are attractive:

* Cloud Sync;
* Cloud Backup;
* multi-device management;
* remote vessel diagnostics;
* hosted crash analysis;
* weather routing;
* mobile companion;
* fleet management.

These belong under `scope-future` / `scope-post-beta` until the public beta is healthy.

---

# 41. Definition of Public Beta Ready

The project is **Public Beta Ready** only when all of the following are true:

## Product

* final product name selected;
* customer-facing XNav development naming replaced;
* HTML prototype-conformant UI accepted;
* XNav chart presentation accepted;
* real charts work;
* real boat-PC acceptance passes;
* no known critical navigation/data regressions;
* unqualified hardware control is unavailable/default-off.

## Installation

* OpenCPN is installed separately;
* installer correctly detects the supported OpenCPN version;
* install works;
* update works;
* repair works;
* rollback works;
* uninstall works;
* stock OpenCPN remains recoverable.

## Updater

* startup update check works;
* Later works;
* offline works;
* manifests/packages are verified;
* broken updates rollback;
* Beta channel works.

## Distribution

* reproducible release artifacts exist;
* SHA-256 exists;
* source artifact exists;
* release notes exist;
* license package exists.

## Website

* final domain works;
* HTTPS works;
* homepage works;
* pricing works;
* product information works;
* OpenCPN prerequisite is clear;
* support/legal/source links work;
* responsive/mobile QA passes.

## Commerce

* $20 checkout works;
* discount codes work;
* payment webhooks are verified;
* entitlement is created;
* refund flow works;
* billing information is available.

## Account/Portal

* login/account recovery works;
* downloads are gated correctly;
* customer dashboard works;
* release/download experience works.

## Support

* knowledge base exists;
* bug form works;
* feature request form works;
* Bug creates Jira Bug;
* Feature request creates Jira Feature;
* portal status works;
* daily triage process exists.

## Open source

* OpenCPN attribution is complete;
* corresponding source is published;
* third-party notices are complete;
* public source link works;
* source maps exactly to distributed binary.

## Legal/privacy

* required legal drafts are present;
* legal review blockers are closed;
* Privacy/GDPR flows exist;
* user export/delete path exists;
* beta/safety disclaimers are present.

## Security/Operations

* production secrets are protected;
* production/staging environments work;
* backups/restores have been tested;
* monitoring exists;
* critical security findings are closed;
* incident/release runbooks exist.

## Marketing

* real release screenshots exist;
* hero imagery exists;
* product visuals match actual software;
* promotional video/assets are ready;
* launch copy is ready.

## Jira

* no unresolved `scope-public-beta + launch-blocker` remains unless explicitly accepted by the user;
* completed work is correctly represented as Done;
* remaining non-blocking work is correctly placed in Beta Patch/Post-Beta/Future.

---

# 42. Final go/no-go review

When every required Public Beta criterion appears complete:

STOP implementation.

Do not automatically publish the product.

Produce a final **Public Beta Readiness Report** containing:

* final product name;
* release version;
* commit SHA;
* qualified Windows installer;
* checksums;
* source artifact;
* CI links/results;
* boat-PC qualification;
* updater validation;
* website production URL;
* checkout result;
* portal result;
* web→Jira result;
* open-source compliance status;
* legal-review status;
* security-review status;
* known issues;
* remaining Jira issues grouped by release scope;
* every accepted risk;
* explicit recommendation for a human **GO / NO-GO decision**.

Wait for the user's explicit approval before opening payment/download access to the public.

---

# Core principle

The target is not:

> “all planned features are finished.”

The target is:

> **A trustworthy, polished, supportable and legally/commercially complete public beta product that real customers can buy, install, update, use, recover, get help with and provide feedback on.**

Use Jira to drive the path from the current state to that outcome.
