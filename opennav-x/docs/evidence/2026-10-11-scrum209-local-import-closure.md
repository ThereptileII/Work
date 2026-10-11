# SCRUM-209 same-directory import closure — source evidence

Base: source-batch `8280f3dd7edbbeac89e18be1ed785f240417c1ec`.
This focused slice extends the already integrated original-plugin pre-load
refusal. It does not qualify a release or the currently running Staging build.

The guard now reads recursively named sibling DLLs when a plugin's normal or
delay import names a simple basename. It holds their read handles until the
original load returns. Direct or indirect legacy `libeay32`/`ssleay32` imports,
malformed local PE, redirected/unreadable siblings, invalid import names and
budget exhaustion refuse the original load. It does not rewrite any plugin.

Focused portable checks: `python3 tools/test-win32-pe-imports.py` passed
**9,366** deterministic checks. `LSAN_OPTIONS=detect_leaks=0 python3
tools/test-win32-pe-imports.py --sanitize --cxx clang++` passed the same 9,366
checks with ASan/UBSan; LeakSanitizer itself cannot run under this host's
ptrace environment. Python syntax and `git diff --check` passed. The normal
and indirect native DLL fixtures passed **42 groups** under native MSVC Win32
in [focused run 38146240771](https://github.com/ThereptileII/Work/actions/runs/38146240771)
for exact source commit `840e1932fd2531f508d43459c89d39bce98fb496`.
The retained `summary.json` SHA-256 is
`f4595833dff7417459908f2623c30b81ef12701160d5f7c288111f4bbde8034e`.
All 20 source receipts match that source checkout after accounting for Windows
CRLF checkout conversion. The receipt explicitly sets product acceptance false.

Independent source review found an extension-less import alias that the first
walk would miss: `helper` may resolve as `helper.dll`. The guard now normalizes
that name before sibling lookup and case-insensitive deduplication. Normal and
delay import negatives plus an alias deduplication check cover this correction.

The test-only `libeay32.dll` is an inert locally compiled symbol stub, not
OpenSSL. The new indirect fixture chain is original → sibling helper → that
stub, with a benign sibling-control variant. No target boat, real vendor DLL,
network or production application was executed for this slice. The benign
native helper in the 42-group predecessor is preloaded to control DLL search
resolution; those checks do not prove a first dependency mapping while its
inspection handle is held.

The separate first-mapping control initially stopped before the new load in
[run 38158753749](https://github.com/ThereptileII/Work/actions/runs/38158753749)
at `1f756424`: a zero-capacity `GetDllDirectoryW` query was incorrectly treated
as an actual nonempty path. That failed evidence is preserved. The bounded
string-query correction passed **43 native MSVC Win32 groups** in
[run 38159170047](https://github.com/ThereptileII/Work/actions/runs/38159170047)
at exact source `e615446e2c1f547fd2e01d87bcf0621da0c8500a`.
The new helper was not mapped and had no application-directory duplicate.
While inspection handles remained live, the actual original load mapped it
from the inspected sibling path with matching volume/file index and SHA-256;
write/delete attempts were refused. Unload, released inspection handles,
unchanged input bytes and restored process search state passed.
The receipt explicitly keeps product acceptance false. Raw downloaded
[summary](scrum209-first-map-native-20261011/summary.json),
[native receipt](scrum209-first-map-native-20261011/native-tests.json) and
[log](scrum209-first-map-native-20261011/native-tests.log) are retained.
Summary SHA-256:
`96995dbb0e9957d6001820973fca71caa386ef200b3c2e5f080836bc39720fc3`.
All 21 source receipts match after Windows CRLF checkout conversion; this is
not a claim of identical raw Linux checkout hashes. No application rebuild,
vendor execution or boat action occurred.

Limits: the observed sibling graph is not a complete Windows loader graph.
Application, PATH, already mapped, SxS/manifest, KnownDLL and API-set choices;
dynamic `LoadLibrary`, export forwarders and child processes remain outside
this check. Native loader behavior and retained-plugin lifecycle must be
qualified before the guard is accepted. SCRUM-209 stays In Progress.
