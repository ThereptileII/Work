# Navigation database preservation audit

`tools/boat/audit-navigation-copy.py` audits an **offline, closed-session copy**
of the real `navobj.db`. It never opens the live boat database, repairs text,
changes schema or writes database data. Copy the closed database through the
existing private evidence path and independently verify its SHA-256 against the
boat-side file before running the tool. Keep the database and reports private.

The pinned OpenCPN `model/src/navobj_db.cpp::CreateTables` defines eight route,
waypoint, track and hyperlink tables. The audit requires those tables, checks
SQLite integrity, then hashes schema and all ordinary table contents. Stored
text is compared as bytes with its SQLite storage type. Some existing Windows
names are not UTF-8; a decode error is not permission to rewrite those names.
Row order is ignored, duplicate rows are retained, and a changed value with an
unchanged row count is detected. Reports contain counts and digests, without
names, positions or machine paths.

Example using independently supplied private-copy identity:

```bash
python tools/boat/audit-navigation-copy.py /private/before.db \
  --expected-sha256 "$BEFORE_SHA256" > /private/before-audit.json
python tools/boat/audit-navigation-copy.py /private/after.db \
  --expected-sha256 "$AFTER_SHA256" \
  --baseline-report /private/before-audit.json > /private/after-audit.json
```

The comparison exits 2 if stored navigation content or schema changed. This
requires investigation against intended operations; it does not automatically
mean corruption. A chart-view change should not alter stored navigation data.
An intentional route/waypoint edit requires separate evidence for the expected
change and is never silently adopted as an unchanged baseline.

The audit refuses mismatched copy hashes, symbolic links, journal/WAL siblings,
missing required tables, virtual tables, excessive size/table/row bounds and
integrity failures. It uses a read-only immutable SQLite connection with trusted
schema and extension loading disabled, and verifies the file hash again after
reading. It cannot prove the source was closed: process-exit and source-copy
evidence remain required. This audit supplements the original file backups,
profile/chart/plugin inventories and installer lifecycle tests.

`python tools/boat/test-navigation-copy.py -v` passes 11 isolated Linux cases,
including non-UTF-8 text, storage-type changes, same-count changes, duplicates,
row-order independence, schema changes and refusal paths. These checks access
no boat and send no device commands. The initial private real-profile copy also
passes integrity and preserves its source hash. This establishes a baseline;
post-launch and installer preservation comparisons remain pending.
