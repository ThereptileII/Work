# Bounded serial writer regression

This offline native target executes `SendMessage`, `PayloadToName` and the
serial Actisense encoder extracted verbatim from the patched pinned OpenCPN
source. It links the real pinned `N2kMsg.cpp`. Only external message/framework,
queue and listener collaborators are minimal stubs. No serial port,
network, wx application, hardware profile or installed application is opened.

From the repository root:

```sh
python tests/pilot_serial_bounds/run.py \
  --upstream /path/to/pinned/OpenCPN \
  --build-dir .local/pilot-serial-bounds --sanitize
```

The runner reads the six required files from exact commit
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`, copies them into the specified build
directory, checks/applies the patch there, and builds/runs the target. It leaves
the shared upstream checkout untouched. Omit `--sanitize` for ordinary native
MSVC. `evidence.json` records pin, source/patch hashes and completed checks.

The first target covers actual writer/serializer, the new serial pilot API,
receive handler, physical-write wrapper, production connection/queue state and
framer. Tests include queue capacity, default/session OFF, pending limit, disable
cancellation, reconnect purge/epoch, expiry, partial/throwing writes, no output
purge after write, bounded DLE framing and original worker timestamp preservation.
The second target extracts the actual `OpenCPNPilot` final sink/session methods
and exercises the real ST4000 adapter with a fake registry/serial endpoint. It
checks observed exact identity, session enablement, six commands, unsupported
modes, heading requirements, endpoint isolation and disable cancellation.

The helper copies the pinned driver and message headers as well as N2kMsg.
CMake builds two offline tests on Linux or native MSVC. It does not qualify a
real port, actual gateway acceptance, physical pilot response or the full
application event loop. Full translation-unit compilation and physical tests
are separate evidence. The original short-payload ASan regression is retained
in the earlier evidence and commit; this expanded harness requires the patched
serial API and is not an unpatched-source build target.
