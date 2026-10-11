# SCRUM-311 native installed/private store check — 2026-10-11

Exact source from `d55641cf` was copied through the authorized `ssh boat` alias
into a fresh isolated source-test directory. All five transferred SHA-256 values
were read back and matched the local committed files; see [results.json](results.json).
The actual default host reports Windows 11 build 22631, PowerShell 5.1.22621.5624
and an eight-byte pointer. The second architecture used the explicit SysWOW64
Windows PowerShell executable.

Both architectures passed 42 installed-store and 245 private-store assertions:
574 native assertions in total. The installed cases exercise real private ACLs,
file sharing, physical marker/record/lock/journal access, and disposable SUBST
remaps after capture. A nested alias target makes the rename test exercise an
ancestor above that target. Private-store cases exercise real native ownership,
ACL/share/link and persistence behavior. No broad application suite was run.

The tests create disposable files and test-owned aliases only. They do not start
SKAGER/OpenCPN, read or change the normal navigation profile, inspect chart data,
send hardware commands, modify installed binaries or change remote access.
The five source files remain in the isolated checks directory for reproduction;
the suites remove their own temporary fixtures and drive mappings.

This evidence qualifies these exact store source bytes on the observed machine,
not an MSVC/application/package build, CI acceptance, the new previous-restoration
operation, complete loader namespace custody or the full signed-update broker.
Native qualification of those separate operations remains pending. Public source
push, release publication and Production promotion are not implied.
