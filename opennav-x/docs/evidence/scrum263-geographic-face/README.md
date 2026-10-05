# SCRUM-263 explicit land-label face

Source commit `e61c29e19a862bb21d46ed1dce098b2a170cc13c` changes exactly two
expressions, based on `9dee9b1`. Jira selection: 10741.

Immutable `docs/design/prototype/index.html:12` explicitly sets `.chart-label`
to Segoe UI/sans-serif. `.chart-water-label` has no family override; its later
line-20 rule changes only size, retaining the root Segoe UI Variable Display,
Segoe UI, Arial stack. The prior geographic resolver used that root stack for
both roles, which would select Variable Display for land on a machine having it.

Land now uses the existing explicit `light_face` selection: installed Segoe UI,
otherwise Arial. Water retains its exact previous root-stack selection. LIGHTS,
all sizes, weights, styles, tracking, opacity, font ownership, handlers and
fallbacks are unchanged. No resource, preference, chart data or UI change.

Both actual production objects compile: core ChartPresentation.cpp and private
ChartPresentationAdapter.cpp. `objects.json` retains commands and object hashes.
The focused collector executes each verbatim resolver in eight fresh processes
covering every combination of the three relevant installed-family flags.
All **336 assertions** pass: all four geographic classes, unchanged water,
factory/custom LIGHTS, ordinary labels and TE exclusion. Restoring only the
original Land selection fails both source-derived negative controls when
Variable Display and Segoe UI are both available.

The fixture substitutes font availability and records FontMgr factory arguments;
it does not claim actual Windows font substitution. The first fixture compile
incorrectly returned wx's const stock-font pointer through a mutable mock API;
copying it into fixture-owned storage corrected the harness. The initial log is
retained privately. Production objects were unaffected and compiled successfully.

`check.py` is the retained local collector, run from repository root under the
existing isolated Xvfb/sysroot environment. No new workflow or broad test group
was added. Native full-candidate and boat font acceptance remain separate.
