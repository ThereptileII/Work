# Prototype interaction contract

The original [inventory](prototype/UI-INVENTORY.md) and
[navigation audit](prototype/NAVIGATION-AUDIT.md) enumerate the supplied suite.
They are preserved unchanged. This mapping connects their visible operations
to native behavior; it does not authorize mock hardware or installer actions.

| Prototype entry | Actual native boundary | Return / error |
|---|---|---|
| Chart / brand | Existing ChartCanvas; show navigation | Close the transient context, keep chart position |
| Passage / next-turn / horizon | OpenCPN route snapshots; existing route model operations | Root Close; selected leg Back; invalid progress unavailable |
| Chart point / Go To | Geographic position copied on GUI thread; upstream temporary route | Focused confirmation; cancel leaves existing route untouched |
| Waypoint / edit / remove | Existing identity-based waypoint commands | Back to selected point; removal confirmation, shared persistence |
| Create passage / undo / finish | Existing OpenCPN route-creation state | Undo one point; confirm discard; name before save |
| Traffic / list / target | Owned aggregated AIS state; onboard CPA/TCPA retained | Target Back to list; source/age visible; lost target cannot appear current |
| Chart target selection | Current target identity; separate online overlay | Target drawer directly; Back to list; Show on chart closes drawer after a validated jump and retains highlight |
| Energy | Existing advisory model and valid Vessel Data/route inputs | Root Close; stale required input suppresses dependent predictions |
| Instruments / metric | Vessel Data assessment, configurable four rail slots | Close; editing a slot returns to rail configuration |
| Anchor | Existing OpenCPN anchor-watch boundary | Explicit arm/disarm; no claim of advanced drag detection |
| Autopilot summary | Existing adapter state and manual command gate | Default output disabled; actual acknowledgement only; no remote physical commands |
| Radar | Adapter availability/capabilities | Unavailable when absent; no synthetic sweep or transmitter command |
| Health / Sensors | Existing source registry and diagnostics | Nested source Back, root Close; online AIS separate from onboard |
| Alerts / banner | Existing episode/acknowledgement model | Acknowledgement never hides an unresolved critical cause |
| Chart layers / orientation / follow | Existing OpenCPN presentation and chart actions | Retain shared chart model; XNav/Standard is a presentation preference |
| Theme button / Display | Day → Dusk → Night → Day | Applies UI and proper chart presentation; no bright primary sheets |
| Settings sections | Existing validated settings store | Focused editing, validation inline, cancel preserves previous value |
| System / Diagnostics | Real build, sources, age, logs and recovery | One contextual return; technical metadata stays here |
| Legacy / Safe | Existing controlled restart and crash recovery | Explain restart; preserve shared configuration and navigation objects |
| Repair / rollback / uninstall | Existing hash-gated transactional Windows maintenance | Real progress/verification only; retain recovery backup |

The stage does not implement the prototype's future update service, fictional
chart packages, symbol-placement toys or mock restore/sample-data buttons.
Unavailable capabilities have disabled controls with an explanation or are
absent; they must never pretend to execute. The original remains unchanged.

Shared input rules: one Close at root, one Back for nested content; wizard
Cancel/Back/Done stays in its footer. Escape and Alt+Left invoke the same
contextual return. Unsaved edits require confirmation; critical causes remain
visible. Blank-chart selection closes transient selection when safe. Active
chart tools replace context navigation with Cancel/Done and restore the opener.

`tools/prototype/render.py` drives actual DOM controls into deterministic
reference states, recording visible labels/actions and computed dimensions in
`capture.json`. Native equivalents must retain keyboard/accessibility names
even where the visual control is an icon. The prototype's mock action code is
not shipped or invoked by the native product.

The Preferences migration preserves the real chart while switching among its
eight sections. Sensors delegates to the existing source list, upstream
connection editor and source-health diagnostics; it never manufactures a
configured-sensor count. System retains diagnostics/export/plugins and existing
controlled Legacy/Safe restart entry points. Display uses actual theme,
fullscreen and rail preferences. Back from a remaining advanced product page
returns to the last Preferences section. Vessel/Navigation inline forms are
still pending, and existing validated editors stay reachable until migrated.
