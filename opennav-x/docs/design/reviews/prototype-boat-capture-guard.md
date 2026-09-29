# Prototype boat capture guard — qualification pending

The installed display helper identified the older Menu/Navigation shell and
rejected every overlapping top-level window. The prototype has an eight-item
left rail and native owned chart tools/sheets. Weakening the overlap check to
allow any window from the process would also allow unknown plugin/dialog UI.

The revised helper recognizes the exact eight sibling rail captions under a
direct frame child. The older shell path remains. Mixed, missing or duplicate
identities are refused. Capture admits only five source-reviewed surface types:
chart tools, chart orientation, follow boat, Passage, and vessel traffic. Each
requires the exact process, immediate native owner, visible/enabled state,
matching DPI, full frame-client containment and fixed control signature. At
most one Passage/traffic sheet may be visible. Unknown overlapping windows still
refuse capture; a title alone is insufficient. Native handles, captions,
signatures, geometry and DPI are checked before and after copying pixels.

This is capture authorization, not authorization to press sheet controls.
Existing fixed navigation actions map to the prototype rail, and palette
selection is scoped to the status bar. No hardware, route mutation, credential
or arbitrary-click action is added. Owned chart-control input and new sheet
selection remain unqualified and are not automatically enabled here.

The outer PowerShell guard still requires the installed generation, build and
executable hashes, an audited recent launch, exact PID/creation time/path and
interactive session, private evidence paths and reviewed helper hashes. No
boat access, profile mutation or remote-service change occurs in these tests.

Linux: the review policy/interop compilation suite passes 212 checks. Twelve
pure suites and eight explicitly selected portable suites pass. The first
unqualified invocation of Windows-only suites was correctly refused; native
review-staging remains a Windows gate. Records are under
`evidence/local/prototype/review-guard-policy/`.

Native qualification adds 17 prototype cases to all 31 existing disposable
display-window cases. Unknown/incorrectly owned, duplicated, clipped, changed,
modal and ambiguous surfaces must fail without callbacks. Separate actual
application screenshots run the same capture guard in disposable CI, retaining
its exact HWND/geometry proof alongside the screenshot. The CI-only wrapper
cannot authorize a boat launch or installed-profile action.

Native run36496486152 passes all48 marker cases and212 pure checks. Actual
product navigation Day/Dusk/Night passes; Passage refuses capture. The first
launcher did not retain the child error stream, so its refusal cannot yet be
attributed to a specific guard. The corrected CI wrapper explicitly saves the
stream and the isolated application's state on failure. Bounded guard reasons
include only allowlisted surface titles and boolean results; they never print
arbitrary child text or typed credentials. No rejection predicate was relaxed.

Actual-product qualification remains pending. This record does not authorize deployment or
claim physical display acceptance. No upstream hook changes are involved.

Run `36498559587` at `42255ef` retains the actual refusal: the first navigation
capture cannot identify a unique shell. Its newly added decorative rail panel
adds the wxMSW default native caption. Source inspection of
[wxWidgets 3.2.8 MSWCreate](https://github.com/wxWidgets/wxWidgets/blob/v3.2.8/src/msw/window.cpp#L3745)
confirms that a null title becomes the window name. This also explains the
earlier drawer's structural heading/body captions. The production correction
explicitly clears only those three structural labels. Action labels, hierarchy,
ownership and all guard predicates remain unchanged. Marker forms already use
empty structural captions; actual native replacement captures must qualify the
correction. All eight failed-run artifacts and27 PNG hashes verify.

Run `36503170626` passes the actual native product guard for all twelve primary
and four additional states. Its overall run fails an independent route-paint
test, so this is only capture qualification. The next Settings increment adds
one exact owned Preferences signature with a single Close header. The same
PID, immediate owner, DPI, containment, caption and before/after HWND checks
apply. At most one of Preferences/Passage/Traffic may be visible. Pure checks
increase to 217, including fixture-catalog reachability; native marker cases increase to 50 (replacement Windows
execution pending). No new boat input or arbitrary-control action is authorized.
