# Preserve the installed stock resource default after commissioning

The first installed development session closed normally with a retained native
handle and measured exit 0. No OpenCPN or chart-helper process remained. The
closed navigation database retains SHA-256
`8eada8eb11d17fc91e687307f713a7bdb90a38ade31e7d6f272198fff360ebbc`.
The input-only INI changed in four keys: AUI perspective, chart-canvas height,
pinned build marker and `Directories/BaseShapefileDir`.

Source review: `OpenCPNIntegration::SelectSharedProfile` and
`InstalledResources::ApplyResourceDefaults` fill only empty resource choices
from the owned `OPENNAV_INSTALLED_STOCK` locator. Pinned `navutil.cpp::UpdateSettings`
persists that path using the normal wxFileConfig escaping. The boat's initially
empty basemap preference became the stock `basemap_shp` directory. Other chart
paths and connection fields remain identical. Transient GPU/toolbar startup
changes returned to their original values at close; no new permission for those
keys is added.

Cold inspection now verifies the current generation, ownership/state hashes,
exact supported stock executable, owned locator bytes and required resource files.
It freezes this proof in the hash-bound inspection. Adoption separately requires
an exact before/after per-key source review and rechecks the live resource proof.
Historical lineage validates the recorded proof against the immutable prepared
context, so a later update/uninstall can retire that generation without destroying
the lineage. It does not reauthorize launch, hardware output or arbitrary directories.

Ordinary restoration still refuses a changed protected path. An inspection with
this resource proof requires explicit adoption, preserving every migrated byte
while reversing only the temporary COM8 direction byte. Custom/nonempty paths,
missing keys, another stock path, changed locator/ownership/generation, chart paths
and connection changes remain refused.

Portable parser/identity/migration tests and actual disposable native transaction
coverage are extended. The native fixture includes proof capture, refusal of
unreviewed erasure, interrupted adoption after atomic publication, recovery, and
new-baseline reuse. No test accesses the boat or sends marine commands.
Native qualification and actual boat restoration are pending; a local policy
pass is not permission to claim those gates have passed.
