# AIS drawer — in progress, not accepted

Reference: immutable Traffic and AIS Target states, 1280×800 DPR1. The drawer
occupies x682/y80, 398×674, with a 352px body, 48px segmented sort control,
89px root header or 139px nested header. It overlays the existing chart and
horizon without moving the chart, rail or alert controls.

The first Linux capture replaced the old full-page AIS screen with an owned
native drawer. Review found an automatic focus outline, square segment
background and missing key/settings interaction coverage. Corrections preserve
keyboard focus indication while removing automatic/mouse focus decoration,
use the segment's 10px type and 6px button radius, and retain a single root
Close or nested Back. Escape/Alt+Left use that contextual return. Theme changes
preserve the drawer; primary navigation and chart selection explicitly replace
it, following the supplied interaction model.

The second capture failed despite a visible geometry record: the sheet was
temporarily behind the chart. Retained failed images prevent treating native
visibility flags as evidence. Added exact drawer bounds and painted-background
probes, without weakening the full-image comparison. A diagnostic pass recorded
a 0.29s additional first-paint wait; the other settings captures needed none.
The host now uses the same owned-frame arrangement as the chart controls, with
no global topmost or repeated focus activation. The fifth capture pass retained
the 0.29s first-show delay on bare Xvfb; it remains a performance finding, not a
visual acceptance waiver. All later settings states painted immediately.

The live settings path exercised default OFF, Enabled with no credential
(explicit key-needed state, no socket connection), OFF, Day/Dusk/Night, Back and
Close. The product contains no synthetic AIS source. A missing source leaves
the list empty and risk/range sorting unavailable. Online-only CPA/TCPA remain
unavailable; onboard values remain upstream copies. The title is “Vessel
traffic” because chart-area internet targets need not be near ownship; the
prototype's fictional risk advisory is withheld without onboard evidence.

Second visual review covers the fifth-pass Traffic Day and Online AIS Night
images. Exact drawer bounds, painted probes and settings interactions pass;
Night contains no bright native primary surface. Eight prototype comparison
sets retain reference/current/diff without masking missing real-world data.

Native Windows replacement `3db9704` passes 112 integrated tests and the twelve
product captures, including settings interaction. Downloaded Traffic Day and
Online AIS Night images were reviewed. The drawer paints immediately on Windows,
but its heading exposed UTF-8 mojibake. The next correction explicitly decodes
the heading/separator and adds service/receipt-age labels, callsign/destination,
aging/lost labels and identity-based selection when the target sheet opens.
Linux replacement builds, 120 integrated and 76 portable cases, repeated empty
source captures and retained full-image comparisons pass; populated-target and
replacement Windows evidence are still required.

Outstanding: populated target/list interaction, online chart symbols and
selection, exact font/spacing/shadow/motion review, and physical boat captures.
Service settings are additional functional captures,
not invented HTML reference states. No conformance PASS is claimed.
