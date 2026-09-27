# Preserve the installed stock resource default after commissioning

The first installed development session closed normally with a retained native
handle and measured exit 0. No OpenCPN or chart-helper process remained. The
closed navigation database retains SHA-256
`8eada8eb11d17fc91e687307f713a7bdb90a38ade31e7d6f272198fff360ebbc`.
The input-only INI changed in four keys: AUI perspective, chart-canvas height,
pinned build marker and `Directories/BaseShapefileDir`.

Source review: `OpenCPNIntegration::InitializeResourceDefaults` and
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
The initial local policy pass was followed by the native qualification and
actual boat restoration recorded below; those remain distinct evidence gates.

The first tooling candidate `ef30481262c3def373b90501acdd39d426dd7d31`
passed all 19 native maintenance suites, including 163 baseline-adoption checks
(112 policy, 25 existing stock transactions, 26 installed-resource transactions),
but its broker job failed: the copied dependency closure omitted the newly
shared `InstalledResourceReview.ps1`. It is not fully qualified or deployed.
The dependency is now explicitly hashed/copied, and a portable test imports the
actual copied audit, including refusal when that transitive module is missing.
That failure required replacement native broker/Prepare/Arm qualification,
subsequently completed below.

Replacement tooling `1439b5b2dac669090b5de4ef956eaffa0f42c542` passes all nine
native jobs in run 36299584290. All nine artifacts match API identity, upload
digest, downloaded digest/size and ZIP integrity. The 163 adoption cases, 114
installed-runtime checks, 17 copied-closure checks, 51 actual broker checks and
47 actual Prepare/Arm checks pass. [Exact scoped evidence](../evidence/beta2-resource-adoption-1439b5.json).

Actual boat restoration subsequently completed using that exact tooling. An
independent byte comparison confirms only the temporary COM8 direction byte was
reversed; the four reviewed migrations remain. All five plugin DLLs returned to
their original locations with exact hashes. A separate postcheck confirms the
adopted INI, unchanged navigation/chart databases, no active transaction marker
and no OpenCPN/chart-helper process. SSH, Tailscale and RustDesk remain running
with automatic startup. A short network timeout recovered without infrastructure
changes. Another application launch requires a fresh read-only source/profile audit.
