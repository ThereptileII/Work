# Boat boundary for signed updates

SCRUM-311 separates an authenticated update package from permission to start it
on the boat. The existing bootstrap qualification does not qualify live signed
offers, Update Now, or automatic rollback under boat commissioning.

`tools/boat/StartupLauncher.ps1` intentionally rejects configured update trust
and pending updates. It retains the selected `state.json` against replacement
and verifies one exact commissioned generation. Those guards must remain.

The installed Go launcher runs the authenticated installer after Update Now,
then immediately invokes `UpdateSupervisor.ps1 -Action LaunchPending`. The
supervisor starts the successor with `--xnav` before collecting its health
receipt. It has no boat commissioning check or pause before process creation.
After rollback, the launcher can also start the previous generation. A health
receipt establishes startup identity and readiness, not output safety.

The existing commissioning proof pins the installed generation, current profile,
plugin roots, complete reviewed trees and child launch environment. Changing the
generation invalidates that proof. Reviewing only the signed payload is also
insufficient: `Lifecycle.ps1` copies unbundled stock plugins and preserves
previous-generation additions. Every resulting DLL, helper and dependency in the
actual loader closure needs review, including disabled plugin candidates.

Therefore, removing the bootstrap trust check or releasing its state handle
does not make the live Update Now path safe. Observing the successor and auditing
it after startup is too late. No stage-only product mode or relaxed boat guard
is introduced by this finding.

Before live signed-flow boat qualification, an independently reviewed pre-launch
boundary must bind the exact authenticated release and installed successor,
preserve input-only profile state, and verify a fresh target-generation source,
plugin and loader-environment audit before application process creation. The
same protection must cover fallback startup; a previously healthy generation
does not waive current profile and audit checks. Existing commissioning cannot
silently transfer its active transaction to the new generation.

Until that boundary is implemented and qualified, use only the separately
authorized installation and fresh commissioning sequence that already has
evidence. Completed settings preservation restores configuration; it grants no
launch permission. Bootstrap/package evidence must not be reported as live
signed-offer acceptance on the boat.
