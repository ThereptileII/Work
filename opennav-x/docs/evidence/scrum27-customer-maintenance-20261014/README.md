# SCRUM-27 fixed customer maintenance component — 2026-10-14

Exact source `91d647c383ce662815b0e4e9fab79c5ad0f32ad6` passes
[focused run 38521029142](https://github.com/ThereptileII/Work/actions/runs/38521029142),
attempt 1. Linux passes **140 top-level / 422 nested** cases. Native Windows
Server 2022, Go Win32/386, passes **206 top-level / 565 nested** cases. No failures.
Three existing non-Windows-boundary tests skip on Windows; all 65 explicitly
checked current/prior controls pass, including every selected new native test.
All 34 checked portable controls pass. `results.json` records exact names.

Both original provider artifacts were downloaded. Their source/run/attempt,
length, SHA-256, complete JSON events and ZIP CRCs were verified. The workflow's
broker source-closure check also passes. Local combined race checks pass for
the enrollment and CLI changes; the separately executed provisioner source
closure passes 14 tests. Read-only retained-baseline inspection has local race
coverage; the focused workflow does not select the root Go package and does not
establish native execution of those root-package tests.

## Change and original failure

Original enrollment retains its signed-policy helper in the protected code
directory and commits an original per-install maintenance completion after the
running-service checks. Customer Uninstall uses a fixed native entry with no
caller-supplied SID, approval hashes or cleanup paths. It retains original
completion/baseline custody, joins the live broker terminal conversation and
uses the existing installed cleanup owner. Same-account elevation binds the
original live process, token, session, creation and code; parent loss, timeout
and close uncertainty cannot report completion or trigger a fallback.

[Original run 38520213641](https://github.com/ThereptileII/Work/actions/runs/38520213641)
at `c6bb2e21c1534cab7f79084e44bda9017719b372` remains **failed**. Linux passed
140/422; Windows passed 203/555 with two top-level failures in running-image
source/publication acquisition (`held enrollment native identity changed`).
`failed-results.json` preserves the authenticated original artifacts and failures.
The old source constructor used process DOS spelling to construct exact physical
paths, which cannot represent all Windows short/case aliases. The correction
derives canonical DOS and NT names from the first retained original image handle,
then retains its ancestry. Full file identity, creation, size, permissions,
sharing exclusions, hash and actual-process corroboration remain enforced.
The replacement native run passes the original cases and an explicit alias /
identity-substitution regression. No assertion was weakened.

Independent read-only reviews covered the original authority handoff, completion
producer/consumer, elevation and final source correction. A cancellation watcher
race found during review was corrected before the first native run and has a
portable regression which refuses even if a later parent check recovers.

## Acceptance limits and next installed gate

This is a **component qualification**, not full application, actual UAC prompt,
Maintain wizard, installed lifecycle or boat acceptance. No SCM installation,
physical output or boat operation was performed by these inert native fixtures.
SCRUM-27 remains open.

Source inspection of `test-installed-protected-b0-startup-windows.py` establishes
the next minimum installed fixture: install genuine matching B0 Setup first,
perform original startup enrollment, qualify/close B0, drain retained launch
handles, then invoke installed Maintain from the same unelevated account and
observe actual UAC. The existing driver requires an elevated token and does not
create the LocalAppData integration, so it does not yet exercise that journey.
Assertions must cover stock/profile/recovery and modified/unknown preservation,
retired lineage, uninstall registration and final service state. An updated /
rolled-back lineage needs its separate installed scenario.

Old retained packages lack this helper/completion/Maintain join. The operator kit
and compiled application commit enforce matching product identity; repackaging
or relabelling those old bytes cannot qualify this source. A coherent new
candidate is required, with eligible dependency reuse. No full application
rebuild, delivery, acceptance transition or public release was requested for
this focused run.
