# Pilot integration follow-up

Source `ac11f5ea3a18af9031cccc9030a7271259bd4b8c` corresponds to local
`44261a7`. [Focused CI](https://github.com/ThereptileII/Work/actions/runs/37374840798)
qualifies only the declared contracts/components. It produces no installer.

[Evidence](evidence.json) separates Linux, native and actual boat observations.
The installed boat candidate remains `c0d8d85`; it has not acquired these changes.
No physical command, profile change, plugin restoration or restart was performed.

The first local CTest selection included a settings executable which had not yet
been built. That case was NOT RUN; after building its explicit target it passed.
Final focused selection passes all14 tests. An initial source-package check
invocation omitted its required archive argument; with the existing hash-locked
archive, its actual test passes. Those invocations do not indicate app failures.

The exact original serial writer fails ASan on a three-byte request. The corrected
writer/serializer passes ASan/UBSan with the real pinned N2kMsg implementation.
Its worker/listener collaborators are fake, so this result establishes neither
serial delivery nor reconnect safety. Three complete changed translation units
also compile using retained fixture-enabled Linux flags and generated headers;
this is not a linked application or normal-product qualification.

[Integration contract](../../pilot-boat-integration.md) explains the AutoTrack
comparison, event-driven identity claims and remaining physical boundary.

Native contracts (14), presentation (6), and button checks (161) passed in the
first run. Its final serial harness failed at CMake parsing of a Windows path,
before compilation. The helper-only forward-slash/typed PATH correction passed
[serial-only native CI](https://github.com/ThereptileII/Work/actions/runs/37375801162)
on `0a6bd2b30ea20aef91e010741c0270872d6f86a5`. The production serial patch is
unchanged; passing earlier suites were not rebuilt or rerun.
