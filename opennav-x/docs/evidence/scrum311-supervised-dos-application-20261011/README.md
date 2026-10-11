# SCRUM-311 supervised application spelling correction

Base: `3d13f1e93f40099df20cc75edb8d77755b9a270a`.

The coordinator's native diagnostic [38166517938](https://github.com/ThereptileII/Work/actions/runs/38166517938)
reported Win32 87 for GLOBALROOT application spellings and all four
held-file-derived volume-GUID application spellings. A conventional DOS
application with GLOBALROOT argv[0] and CWD succeeded while suspended; no child
was resumed. This correction adopts that one compatible application spelling.
The diagnostic is an input to this change, not qualification of the changed owner.

`UpdateImageOwner.Acquire` derives `ApplicationImage` exactly once from the
already held physical image with `GetFinalPathNameByHandle` flags 0
(`FILE_NAME_NORMALIZED | VOLUME_NAME_DOS`). It strips exactly `\\?\`, requires a
local drive path with safe components and the exact held native volume suffix,
and refuses unavailable or unsupported results. It never derives the application
from `OriginalImage` or retries with another spelling. Existing `NativeImage`
flags 2 and physical image/ancestor handles remain unchanged.

Every `Validate` reopens the selected DOS root and image and checks their native
names, volume/file identities, type and reparse status against the existing
physical pins; the image additionally retains its single-link and size checks.
The DOS root must equal the held physical NTFS volume root. The only changed
`CreateProcessW` parameter is `lpApplicationName`; argv[0] and CWD keep their
GLOBALROOT spellings. Suspended creation, raw kernel-handle/process identity
checks before the one resume, health custody, one-use state and cleanup ordering
remain unchanged. The DOS namespace is not pinned; wrong-image resolution is
still refused before resume through the existing raw native child identity checks.
This is a compatibility contract adjustment, not immutable application-name,
general DLL/resource/environment or mapped-section custody.

The [portable receipt](results.json) and [log](policy-only.log) retain **37 passing
checks** on Linux: exact embedded production C# compilation, direct pure parser
acceptance/refusal cases and pending-record refusal checks. No native API was
invoked. The existing native fixtures are retained. New native cases require the
SUBST-acquired application's selected spelling to match the actual physical
image, then simulate selected-name redirection/unavailability against real held
pins without remapping a host volume drive. They require refusal before creation
and no name substitution. These native cases have not been run locally.

Actual Windows acquisition, creation/resume, original-input SUBST remap, health,
selected-name refusal, uncertain-resume and normal/interrupted cleanup remain
pending the coordinator's combined native workflow. No full application build,
release, boat operation, network access or independent review was performed for
this focused source increment.
