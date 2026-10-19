# Observable synchronization regression — SCRUM-376

This isolated fixture compiles the actual pinned `observable.cpp` and headers,
with only the Observable patch applied, against wxWidgets 3.2. It uses the real
wx console event queue; a receiver pauses its virtual `QueueEvent` dispatch to
make registration/removal overlap with an in-flight notification.

The four bounded scenarios check:

- unregistering cannot return until the current enqueue returns; the owned
  pending event remains deliverable, and later notification after handler
  destruction cannot reach it;
- registering through another Observable wrapper cannot invalidate an active
  traversal and begins with the next notification;
- move assignment removes its previous registration, transfers one registration,
  preserves self move, and leaves no registration behind at destruction;
- four producer threads and 500 registration/destruction cycles retain exact
  delivery to a persistent receiver without queuing to retired receivers.

Run from a checkout with native wxWidgets 3.2, CMake and a C++17 compiler:

```sh
python opennav-x/tests/observable_concurrency/run.py \
  --upstream /path/to/pinned/OpenCPN \
  --build-dir /path/to/disposable/observable-fixture
```

`--sanitize` enables AddressSanitizer and UBSan on GCC/Clang. For a separate
negative-control directory, `--negative-control` requires the original pinned
source to fail the unregister and replacement-move assertions. The runner
records source/patch/fixture hashes and its bounded result in `evidence.json`.
Native Windows may use `--cmake-arg=-A --cmake-arg=Win32` and the normal wxWidgets
root/configuration arguments. `--cmake` and `--ctest` select retained tools.

This proves the shared-list race and replacement-registration defect are
corrected at this boundary. It does not establish the unique cause of the
captured startup crash, qualify Windows/boat startup, or make arbitrary handler
lifetimes safe: owners must unregister before destroying their wxEvtHandler.
No physical input/output or plugin binary is involved.

Local evidence on 2026-10-19: GCC 16.2.1 / Linux with the retained wxWidgets
3.2.11 toolchain passed all four scenarios, including the AddressSanitizer +
UBSan variant with recovery disabled. The original pinned source failed both
negative controls at their expected assertions. Patch SHA-256:
`29b6ff13dc9791774b7ed40c9c29aa17cedfda82e56321c08b4c83346923cd04`.
The isolated source/fixture hashes and CTest logs are retained under
`.local/observable-concurrency`, `.local/observable-sanitized` and
`.local/observable-negative` in the implementation worktree.

Initial fixture failures were corrected without changing the product patch or
weakening assertions: `ObservedEvt` needs wx core as well as base, pending events
must be drained by the app, and a console fixture must use
`wxAppConsole::GetInstance()` instead of the GUI `wxTheApp` downcast. The initial
sanitizer log remains retained separately. Native Windows and application/boat
startup are still unperformed here.
