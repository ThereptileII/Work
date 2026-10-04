# SKAGER App — paid public beta objective

Deliver a trustworthy, polished and supportable public beta that a customer can
discover, purchase for USD 20, install over a separately installed supported
OpenCPN, use, update, recover, obtain support for, and give feedback on. Preserve
all applicable open-source rights and supply exact corresponding source.

The full user-provided objective is [the public-beta contract](docs/public-beta-contract.md).
It includes the Windows application, installer/updater, release infrastructure,
website, accounts, commerce, customer portal, support, legal/open-source
compliance, privacy/security, operations and launch material. Completion means
the entire customer journey is qualified, not merely that existing navigation
features compile.

The user's later 2026-10-04 [delivery policy](docs/delivery-workflow.md) governs
Staging/Production delivery: development defaults to **STAGING**, and both
channels use versioned GitHub Releases. Production readiness/promotion requires
an explicit user instruction and reuses the exact qualified Staging package.
Keep channel identity separate from the immutable package version. Releases in
both channels stay draft until the separate public-access GO is given.
Promotion performs release-readiness work only; design validation requires its
own explicit request. This later policy takes precedence over older delivery and
blanket visual-promotion requirements in the preserved source documents without
changing functional, security or data-preservation gates.

[Navigare / SCRUM](https://swedishcountrysideliving.atlassian.net/jira/software/projects/SCRUM/boards/1)
is the authoritative and sole backlog. Select work, record dependencies and
priorities, and track acceptance there according to [AGENTS.md](AGENTS.md) and
[SCRUM-97](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-97).
Repository contracts and evidence explain implementation and validation; they
do not replace Jira planning or maintain a competing TODO system.

The product-owner decision in [SCRUM-89](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-89)
selects **SKAGER** as the brand, **SKAGER App** as the product and **skager.app**
as the primary domain. Its comments contain the approved wordmark direction
and remaining domain/trademark/legal gates. Do not restart naming research or
claim domain control or legal clearance from the naming decision alone.
XNav/OpenNav X remains an internal/historical identifier where appropriate.

Continue from the accepted OpenCPN integration and engineering contracts. Keep
the immutable HTML prototype as the visual authority, preserve Legacy/Safe,
navigation data/configuration and supported Win32 plugin ABI on Windows x64,
and verify Windows and actual boat-PC behavior. Never fabricate missing data,
expose an unqualified physical-control path, or implement autonomous steering.

When every public-beta acceptance criterion has evidence, stop implementation
and present the full **Public Beta Readiness Report** for a human GO/NO-GO.
Do not open public payments or downloads without explicit user approval.
