# Native navigation-store failure fixture

Run against **prepared OpenCPN source containing the current patches**:

```sh
python3 tests/navobj_persistence/run.py --source build/integration-source --output build/navobj-persistence
```

The wrapper configures, builds and runs CTest for this fixture only, using
Release by default. CMake, Python 3 and a C/C++ compiler are required. SQLite is
compiled directly from the prepared source's `libs/sqlite/src/sqlite3.c`, using
its matching `libs/sqlite/include/sqlite3.h`; no system SQLite package is used.
`inputs.json` in the output directory records the native and SQLite input hashes.

Native Win32/MSVC example (from the `opennav-x` directory):

```powershell
python tests/navobj_persistence/run.py --source build/integration-source --output build/navobj-persistence-win32 --generator "Visual Studio 17 2022" --architecture Win32
```

Use separate output directories for different generators/architectures. The
wrapper accepts `--config`, `--parallel`, `--cmake` and `--ctest` overrides.
Direct CMake use is also supported with `-DOPENCPN_SOURCE_DIR=<prepared-source>`.

This isolated C++/SQLite fixture extracts the actual native update methods and
helpers, schema, savepoint guard and `ChartCanvas::FinishRoute`. Value objects
and notifications/UI use inert stubs; SQLite statement execution, triggers,
transactions, constraints, authorization failures and commit refusal are real.
The database is in memory. No application, profile, serial driver or hardware
connection opens, and no test target is installed or linked into the product.

Assertions cover atomic route names/point attributes/point order, waypoint name
edits, track rename/visibility/hyperlinks, track point replacement, failed
statement preparation, failed commits, colliding hyperlink identities, nested
transactions, recovery after each injected fault, and native route Finish
behavior across new routes, append, Legacy, rejected saves and short drafts.
The native Finish fixture checks that XNav's successfully prepared new route is
not saved again without checking the result.

This is a focused persistence and native caller check. It does not qualify the
full wx/OpenCPN application, filesystem failure recovery, Windows compiler/ABI,
installed profile migration, GUI name editing or boat behavior. Root coordinates
those candidate gates and the complete upstream patch-stack preparation.
