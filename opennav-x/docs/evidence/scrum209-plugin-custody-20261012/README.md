# SCRUM-209 original plugin inspection custody — 2026-10-12

Product source: `4f98ce2838cbe04274ada36e2876b906b783441d`.

The actual OpenCPN container now owns original-plugin/import inspection handles
through factory export resolution, subsequent declared delay loads and checked
module unloading, including detach callbacks. A failed `FreeLibrary` retains the
module and its inspection handles. If a container is destroyed in that uncertain
state, it deliberately retains both until process exit instead of retrying or
releasing protection. This is an exceptional retained owner, not a claim that
unload succeeded or that arbitrary Windows loader behavior is constrained.

Independent Codex review identified the original helper-stack lifetime gap. Root
integration additionally found that the source-local `OPENNAV_X` definition would
make the changed container layout disagree with other model readers. The narrow
`SKAGER_PLUGIN_CUSTODY` definition/include now propagates through `_OCPN_MODEL`;
the header rejects an integrated unit missing that shared layout definition.
A final bounded independent review found no further concrete blocker in these
paths. Claude has not reviewed this increment.

## Focused evidence

`results.json` binds all six changed source inputs to the product revision.
The entire twelve-patch chain applies to the pinned OpenCPN Git index without
mutating any checkout. The reconstructed actual `plugin_loader.cpp` and header
pass Linux syntax compilation with current integration headers. A non-integrated
configuration also passes; deliberately omitting the shared custody definition
while retaining `OPENNAV_X` fails with the intended explicit header error.

These syntax probes use cached generated Linux configuration/dependency flags
from an earlier production build. They are not a clean integrated build, native
Windows build or loader execution. Commands and compiler output are retained.

A small Linux smoke probe compiles and executes the actual production fallback
helper and callback registry against real wxWidgets and the local system math
library. Nine checks cover ordinary load/export lookup, clean failed replacement,
callback residue refusal and checked cleanup. The initial probe compilation
omitted wxWidgets' target macro and failed before execution; its original output
is retained. The corrected flags pass. This probe does not simulate or qualify
Windows file sharing, forwarded exports or `FreeLibrary` behavior.

The existing native MSVC harmless-forwarder fixture now uses the actual fallback
rather than a manually retained outer guard. It checks denied write/delete access
before and after native export resolution and released custody after checked
unload. The injected invalid-module-handle case checks retained custody on unload
failure. These updated native cases have **not yet executed**.

No full application build, CI dispatch, public push, installed package change,
boat application/profile change, visual/endurance suite or physical command
occurred. Exact-revision native fixture execution, full integration, installed
closure, supported plugin compatibility and installer/recovery gates remain open.
SCRUM-209 stays In Progress.
