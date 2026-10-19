# Native navigation-store failure fixture

Run against **prepared OpenCPN source containing the current patches**:

```sh
python3 tests/navobj_persistence/run.py --source build/integration-source --output build/navobj-persistence
```

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
