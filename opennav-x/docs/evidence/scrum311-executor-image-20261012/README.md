# SCRUM-311 fixed interactive executor image binding

Source `b1bed3d4`, native runner `605b0bea` (the exact native source receipt is
retained). The sealed `GenerationExecutorImage` originates only from a live
read-only generation's fixed `app/skager-start.exe` original. It has no path,
hash, file handle or receipt constructor. Native comparison checks the live
process and original creation FILETIME, actual process image's physical path,
single-link/plain file and full file ID against retained original custody.
Generation contents are checked before and after comparison; original peer
mismatch does not poison a sound generation. Owner/Store closure revokes it.

Focused portable race **3 top-level + 5 nested passed**, no failure/skip.
Windows/x86 component compiled. Exact native Windows fixture **1 passed** with
no failure/skip: a protected copy of the test PE is created suspended and never
resumed. Its original creator handles terminate and drain only that inert fixture
process. Different process/creation refuses; the actual original matches;
closing generation revokes the image capability. No product/application
entrypoint, service, profile or physical output was invoked. OpenCPN PID1852
remained responsive and no fixture process remained.

Native binary SHA-256:
`523ad80f99e0a18b58ce02d169ca880131f6b1f41871cfbb140076e096f03829`.
Exact binary/archive/runner hash, discovery, timeout, transfer and retrieval
records accompany original raw logs. Test binaries/ZIPs are not committed.

The approved SCM registration path is separately joined to initial read-only
public baseline materialization after private registration writers close.
Its native host component compiled; actual SCM enrollment remains pending.
A post-registration view failure is explicitly uncertain and never authorizes
registration retry or removal of its permanent fence.

## Limits

Image comparison is only one authentication boundary. It is not a selected
version, commissioned profile, spawn permission, healthy commit or restore.
The pipe layer must independently retain and validate its actual process,
pipe, token and logon session. Full authenticated execution integration,
MSVC/application/installer/signed-update/boat acceptance remain pending.
No full application build, design review or endurance run occurred.
