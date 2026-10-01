# SCRUM-209 source-build boundaries

This record binds the reviewed zlib producer and curl consumer revisions to
their source provenance and bounded checks. The zlib worktree is at commit
`a1121903a7886e6e4f721e1412875942f691481e`; the read-only curl worktree was at
`cd07b8d0eaf4d76ad2924845a39e9c88811547a8`.

The verified sources are zlib 1.3.2 (`bb329a0a...f0119d16`, 1,502,830 bytes)
and curl 8.22.0 (`f7ef3ae8...a2a5f4f7`, 2,953,092 bytes). Both detached
signatures were locally `GOOD/VALIDSIG` with the official maintainer
fingerprints recorded in the JSON evidence.

The generated zlib CMake wrapper configured and built under Linux with the
repository tool environment, and upstream CTest passed all 14 discovered tests.
A second configure/build passed when the wrapper path contained spaces and `&`.
The zlib PowerShell builder passed the repository PowerShell AST parser.

The actual zlib manifest construction was fed into the actual curl consumer
validation statements. A UTF-8 BOM manifest was accepted. The consumer rejected
a damaged DLL, damaged public header, wrong source SHA-256, and wrong version.
The existing verified OpenSSL interop fixture was accepted as the dependency
fixture. The harness stopped before `vswhere`, native compiler actions, DLL
execution, or curl build commands.

Evidence logs are retained under
`/home/standard/Projects/evidence/local/scrum209-boundaries`; their hashes and
the exact transient harness hash are in the JSON record. This evidence covers
source identity, producer-consumer schema/bytes checks, and Linux wrapper
behavior. It does not qualify native Windows MSVC output or the full curl
dependency closure.
