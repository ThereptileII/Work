# Bounded serial writer regression

This offline native target executes `SendMessage`, `PayloadToName` and the
serial Actisense encoder extracted verbatim from the patched pinned OpenCPN
source. It links the real pinned `N2kMsg.cpp`. Only external message/framework,
queue, listener and clock collaborators are minimal stubs. No serial port,
network, wx application, hardware profile or installed application is opened.

From the repository root:

```sh
python tests/pilot_serial_bounds/run.py \
  --upstream /path/to/pinned/OpenCPN \
  --build-dir .local/pilot-serial-bounds --sanitize
```

The runner reads the four required files from exact commit
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`, copies them into the specified build
directory, checks/applies the patch there, and builds/runs the target. It leaves
the shared upstream checkout untouched. Omit `--sanitize` for ordinary native
MSVC. `evidence.json` records pin, source/patch hashes and completed checks.

The harness covers a three-byte ISO request, a thirteen-byte addressed command,
the legal maximum payload including heavy DLE escaping, malformed input, unknown
source identity on transmit notifications, and queue-return semantics. It does
not qualify the real worker, physical serial I/O, connection epochs, reconnect
queue invalidation, session permission barriers or boat operation. The retained
`0x94` notifications are transmit attempts, including on queue rejection; they
are never `0x93` received feedback. A true return proves queue acceptance only.

For the original regression, configure this target against an isolated
unpatched copy of those same pinned files with sanitizers enabled. The first
three-byte request must fail in the original `PayloadToName` eight-byte read.
Do not use that diagnostic build for application qualification.
