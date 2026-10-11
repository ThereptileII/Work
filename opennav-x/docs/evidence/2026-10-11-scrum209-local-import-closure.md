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
and indirect native DLL fixtures are prepared but have **not** yet run under
MSVC; the native receipt requires 42 groups.

Independent source review found an extension-less import alias that the first
walk would miss: `helper` may resolve as `helper.dll`. The guard now normalizes
that name before sibling lookup and case-insensitive deduplication. Normal and
delay import negatives plus an alias deduplication check cover this correction.

The test-only `libeay32.dll` is an inert locally compiled symbol stub, not
OpenSSL. The new indirect fixture chain is original → sibling helper → that
stub, with a benign sibling-control variant. No target boat, real vendor DLL,
network or production application was executed for this slice.

Limits: the observed sibling graph is not a complete Windows loader graph.
Application, PATH, already mapped, SxS/manifest, KnownDLL and API-set choices;
dynamic `LoadLibrary`, export forwarders and child processes remain outside
this check. Native loader behavior and retained-plugin lifecycle must be
qualified before the guard is accepted. SCRUM-209 stays In Progress.
