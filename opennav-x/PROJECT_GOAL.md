# SKAGER / OpenNav X — Codex Goal

Build **SKAGER** (the customer-facing product currently implemented in the OpenNav X / XNav codebase) into a polished, trustworthy, commercially distributable marine navigation product on top of OpenCPN.

This project is no longer in initial-prototype mode. The current milestone is **Public Beta readiness**.

The primary job now is to finish, qualify, package, secure and launch the complete public-beta product without losing the strong OpenCPN foundation or weakening navigation safety.

## Source of truth and current milestone

Jira project **Navigare / SCRUM** is the authoritative backlog and release-planning source.

Use Jira to decide:
- what is in the current Public Beta scope;
- priority and dependency order;
- what is actively being implemented;
- what remains blocked;
- what has reached Testing;
- what is genuinely Done.

The active sprint is the current public-beta release effort. Post-beta/future epics and release sprints are planned, but must not displace unresolved public-beta launch blockers unless the user explicitly reprioritizes them.

Repository documentation remains authoritative for architecture, safety contracts, implementation detail and evidence, but it must not silently override Jira scope/status.

## Product objective

The finished Public Beta must feel like a cohesive commercial marine navigation product built on OpenCPN — not a skin, plugin bundle or developer preview.

It must:

- Preserve OpenCPN's proven chart, route, waypoint, track, AIS, connection, navigation-data and plugin capabilities.
- Provide the modern, touch-first XNav/SKAGER interface as the primary experience.
- Treat the approved HTML prototype and design references as the visual and interaction contract for the primary XNav views.
- Preserve normal OpenCPN as **Legacy Mode**.
- Provide **XNav**, **Legacy** and **Safe Mode** against the same underlying OpenCPN user/navigation data.
- Keep UI, Vessel Data, SmartNav, hardware adapters, web/backend services and OpenCPN integration cleanly separated.
- Use truthful vessel/navigation data. Never invent sensor, chart, AIS, route, propulsion, device or service state.
- Explicitly represent stale, unavailable, invalid, estimated and uncertain information.
- Support electric propulsion and battery data with tested range, reserve and arrival-SOC calculations.
- Keep SmartNav advisory-only for the initial product. No autonomous steering or autonomous collision-avoidance commands.
- Integrate autopilot/radar only through isolated, reviewable adapter boundaries.
- Keep unqualified safety-critical output disabled or status-only in public-beta builds until separately commissioned and accepted.
- Remain useful offline for core onboard navigation even if cloud, account, store or update services are unavailable.

## Public Beta priorities

Work on the current public-beta backlog in this order unless Jira dependencies require another order:

1. **Safety, security and data-loss blockers**
   - unqualified physical output;
   - insecure or obsolete runtime dependencies;
   - authentication/payment/security boundaries;
   - installer/update/recovery correctness;
   - source/license compliance.

2. **Native product qualification**
   - HTML-prototype UI conformance;
   - chart presentation;
   - DPI/touch behavior;
   - XNav/Legacy/Safe switching;
   - native Windows build/runtime behavior;
   - real boat-PC 1280×800 acceptance.

3. **Real marine operation**
   - real charts;
   - real marine data;
   - AIS;
   - propulsion/battery;
   - data-source health;
   - physical boat-PC performance and stability.

4. **Installer, updater and recovery**
   - supported-OpenCPN detection;
   - real Windows installation;
   - startup update popup;
   - signed/verified update metadata and packages;
   - transactional update;
   - rollback, repair and clean uninstall;
   - preserved charts/routes/settings/plugins;
   - portable recovery package.

5. **Public-beta service layer**
   - web prerequisite/install journey;
   - account/auth;
   - entitlement and checkout;
   - customer portal/downloads;
   - support and Jira intake;
   - operational monitoring/backups.

6. **Legal, licensing and launch readiness**
   - GPL/corresponding source;
   - third-party notices;
   - OpenCPN attribution;
   - privacy/terms/refund/safety material;
   - final staging rehearsal;
   - human GO/NO-GO.

Do not let marketing, cloud convenience or future features become the dominant workstream while navigation, security or native release blockers remain open.

## UI and interaction contract

Primary XNav screens must be implemented against the approved HTML prototype/design reference, including:

- Navigation
- Route / Passage
- AIS
- Instruments
- Energy / Propulsion
- Radar
- Anchor
- Autopilot/status-only states
- Alerts
- Settings / System
- commissioning, update and recovery surfaces where applicable

The prototype is a design/interaction contract, but real application data must remain truthful. Illustrative prototype values must never be copied into the product as fake live state.

Windows behavior is authoritative for final:
- fonts;
- DPI;
- wxWidgets behavior;
- layout;
- touch/pointer interaction;
- plugin/DLL loading;
- installer behavior;
- screenshots;
- XNav/Legacy/Safe transitions.

The physical boat-PC display is the final authority for 1280×800 usability.

## OpenCPN integration

Keep the supported OpenCPN baseline pinned and pristine.

Prefer small, documented integration hooks over invasive upstream rewrites.

Before changing an OpenCPN-facing subsystem:
1. inspect the pinned upstream source;
2. verify the actual API/behavior;
3. reuse existing OpenCPN functionality where practical;
4. document direct upstream changes;
5. preserve Legacy Mode behavior.

Never patch or update an unknown/unsupported OpenCPN installation.

Preserve charts, routes, tracks, waypoints, plugins, connections and configuration through install, update, rollback and mode switching.

## Distribution and commercial readiness

The user must be able to install and maintain SKAGER without developer tooling.

Public Beta distribution must provide a coherent exact-revision release set such as:

- Windows installer;
- portable recovery package;
- checksums;
- signed update metadata/package verification;
- release notes and known issues;
- corresponding source;
- required license/notices;
- exact XNav/OpenCPN revision identity.

The software updater should check at startup. When an update is available, show the update decision before normal XNav operation/control-capable adapters are enabled. Choosing **Later** must continue startup without interrupting that session again.

Core GPL-covered software rights and corresponding source must not depend on a paid hosted entitlement.

## Web/backend boundary

The selected public-beta service architecture is documented in:

`docs/architecture/public-beta-web-commerce.md`

Follow that decision unless Jira explicitly reopens it.

Current direction:
- Django 5.2 LTS modular monolith;
- PostgreSQL;
- managed Auth0 OIDC;
- Paddle Merchant of Record;
- hosted web/worker infrastructure;
- private object storage;
- server-side Jira integration.

Treat authentication, entitlement, payment webhooks, signed releases, customer uploads and privacy boundaries as security-sensitive work.

Core onboard navigation must not depend on these services being reachable.

## Testing and evidence

Do not declare a feature complete because it compiles or because one platform passes.

Use the smallest appropriate verification during implementation, then run broader qualification at meaningful integration/release boundaries.

For ordinary reversible changes:
- focused unit/contract checks;
- targeted smoke or visual review;
- no unnecessary full-suite reruns.

For safety/navigation correctness, security/auth/payment, installer/updater/recovery, data-loss risk, dependency replacement, licensing/source publication and release acceptance:
- preserve the full relevant gates;
- do not weaken assertions to obtain green CI;
- retain failed evidence;
- qualify the exact candidate revision.

Native Windows remains mandatory for Windows-facing release claims.

Boat-PC acceptance is separate from CI acceptance.

## Definition of Done

A Jira issue is Done only when its own acceptance criteria and applicable evidence have passed.

A Public Beta release is not ready until:
- primary XNav UI has accepted native Windows + boat-PC conformance;
- real chart/navigation/data behavior is qualified;
- safety-critical output policy is accepted;
- security launch blockers are resolved;
- installer/update/repair/rollback/uninstall pass;
- corresponding source and license artifacts are tied to the exact release;
- web/account/entitlement/support flows required for launch pass staging;
- legal/privacy/safety launch materials are ready;
- release/incident/backup procedures are exercised;
- a human performs the final GO/NO-GO decision.

Codex must never infer release approval from passing tests alone.

## Future roadmap

Post-beta release sprints already exist for Connected/Backup, Companion/Remote, Energy Planner, Weather/Tides, Sailing Performance, Voyage Intelligence, Radar/Awareness, Anchor/MOB/Docking, Vessel Analytics and Platform/Fleet.

Keep these designs compatible with current architecture, but do not implement them ahead of unresolved public-beta launch blockers unless the user explicitly changes priority.

## Working principle

Prefer many small, bounded, reviewable changes over large speculative rewrites.

Continuously:
- read Jira;
- select the smallest valuable unblocked task;
- implement it in the correct architectural layer;
- verify proportionately;
- record evidence/status;
- move on.

When a new defect or prerequisite is discovered, create or update the corresponding Jira work rather than hiding it inside an unrelated change.

The goal is not merely to make SKAGER feature-rich. The goal is to make it **beautiful, trustworthy, maintainable, recoverable and safe enough for real onboard use and a paid public beta**.
