# Real-profile read-only commissioning transaction

This maintenance transaction temporarily prepares the actual recovered OpenCPN
profile for separately reviewed, read-only commissioning. It never launches an
application, opens a connection, invokes a plugin/helper, changes remote access,
or sends a vessel command. It is not an alternative to the existing limited-user
launch audit, and it does not claim physical bus silence.

The exact stock 5.12.4 executable, recovered working INI, interactive account and
current installed generation must match. The fixed INI baseline is
`a2e416b7d35d6d2f82dcf6b3a97136a8fba5a133435c2047e07e09022426aef9` (21,380 bytes).
The earlier zero-filled corrupt file is never an undo baseline.

## Source boundary

The pinned OpenCPN `model/src/conn_params.cpp` stores serial port at field 5,
protocol at field 4, direction at field 8 and enabled state at field 17. Only the
known enabled serial NMEA2000 COM8 connection can change direction `1` to `0`.
Its Garmin driver-mode flag must be off. The separate serialized GarminUpload
preference is preserved exactly, including the inspected baseline's value `1`;
the pinned core only deserializes/serializes that per-connection field. Explicit
Send-to-GPS paths also have a separate global upload preference. They are outside
this commissioning session and must never be invoked. A byte-offset edit changes one ASCII byte without
normalizing UTF-8, BOM, line endings, filters, other connection fields or GPS.
The strict profile parser and existing input-only audit validate the result.

`model/src/plugin_loader.cpp` loads DLLs and calls `create_pi` before reading the
enabled flag. Loader directories are recursive; helper/dependency directories
also enter PATH. Disabling a plugin checkbox or renaming a subdirectory inside
the plugin root is therefore insufficient. Inventory includes all files below:

* The interactive account's managed plugin directory.
* The exact supported stock application's plugin directory.
* The current installed XNav generation's plugin directory, when installed.

An install/update/generation change invalidates preparation. XNav, Legacy and
Safe use the same integrated executable and need the same live profile audit.
Starting the separate stock executable requires its own launch audit.

Only explicitly listed, hash-pinned `*_pi.dll` files move. Their original bytes
are first copied to a private recovery directory; same-volume rename then moves
each selected DLL outside all loader roots and PATH. Shared dependencies and
helpers remain in place, with every byte pinned by the complete inventory.
No helper may be executed merely because its associated plugin was isolated.
All application and known/path-owned plugin helper processes must be closed.

The recorded application environment has an explicit working directory (the
actual executable directory) and child-only PATH containing that directory,
the operating system's System32 directory and Windows directory. Known-folder
and WINDIR identity must agree. The interactive launcher must use those exact
verified fields; it must not inherit an ambiguous SSH/service PATH. OpenCPN adds
its reviewed plugin roots as usual. This does not edit the machine/user PATH or
the SSH process environment. Quarantine is checked against the actual recorded
child search directories and all recursive loader roots. Empty/relative PATH
entries remain refused rather than silently ignored.

Input-only connection configuration alone does not block a plugin calling
`WriteCommDriverN2K`. The remaining DLLs therefore need individual source and
binary review. OpenCPN's serial N2K driver still sends gateway initialization
and manufacturer-query traffic; the reviewed gateway/protocol boundary must be
recorded separately. No actuator command, Send-to-GPS operation, route activation,
pilot identity request, radar transmission or plugin installation is authorized.

## Review plan

`tools/boat/commission-read-only.ps1 -Action Inventory -Workspace C:\XNav`
creates a private complete inventory without altering profile/plugins. The
operator constructs a private JSON plan; no source review is invented by the tool:

```json
{
  "schema": 1,
  "owner": "OpenNavX.ReadOnlyCommissioning.Plan.1",
  "inventoryPath": "<returned private inventory path>",
  "inventorySha256": "<returned SHA-256>",
  "reviewedUtc": "<actual review UTC>",
  "plugins": [
    {
      "path": "<exact DLL path from inventory>",
      "sha256": "<exact DLL SHA-256>",
      "decision": "retain",
      "reason": "<specific reviewed session boundary>",
      "sourceBoundary": "<constructor, Init, callbacks and helper review>",
      "sourceRevision": "<40-character reviewed source revision>",
      "startupAndIdleReadOnly": true,
      "evidencePath": "<private source/binary review document>",
      "evidenceSha256": "<document SHA-256>"
    }
  ]
}
```

Every candidate DLL, including disabled candidates, must appear exactly once.
`quarantine` entries must set `startupAndIdleReadOnly` to false; unknown source
revision may be null only for quarantine. A vendor helper trust boundary must
remain explicit in evidence, rather than being called an audited implementation.
An incomplete review, changed dependency, unknown retained revision, truthy
string instead of a boolean, or changed source document fails closed.

Do not commit these plans, inventories, INI files or review diffs. They contain
private paths, configuration and possibly chart/license information. Recovery
directories grant access only to the current account, administrators and SYSTEM.

## Transaction

1. Complete stock upgrade, plugin cold backup and the separate exact INI repair.
   Close applications/helpers normally. Inspect/remove any pending launch task;
   no application may restart while controls are being restored.
2. Inventory the final installation and complete the source/binary review plan.
3. Run `-Action Prepare -Plan <plan> -ExpectedPlanSha256 <hash>`. This copies the
   immutable plan/evidence, original DLLs, baseline INI and one-byte edited INI.
   No original file moves or changes. The result gives a prepared record/hash.
4. Run `-Action Apply -Record <prepared> -ExpectedRecordSha256 <hash>`. All
   identities, hashes, whole trees, current ACL and closed processes are checked
   again. An exclusive active-transaction marker and flushed journals precede
   each mutation. Only reviewed DLLs move; only the COM8 byte changes. The tool
   verifies remaining DLL completeness and returns `launchAuditStillRequired`.
5. Independently create the existing short-lived read-only launch attestation
   from the real modified INI, exact build and copied review evidence. These
   tools never fabricate that attestation or start XNav. Review each mode's
   legitimate profile writes before issuing another attestation.
6. Close all applications/helpers. Run `-Action InspectRestore` with the same
   prepared record/hash. It preserves the actual post-session INI, full current
   profile inventory and a private before/after key diff. Changes to connection
   configuration or chart directories are refused, not silently undone.
7. Inspect that diff. Run `-Action Restore` with the prepared record/hash plus
   `-Inspection <inspection> -ExpectedInspectionSha256 <hash>
   -ReviewedCurrentIniSha256 <explicitly reviewed current hash>`. An intervening
   edit fails without overwriting it. The exact working baseline INI and selected
   DLLs return; unrelated current profile files remain unchanged. Post-session
   configuration remains in private evidence. Final verification precedes removal
   of the active-transaction marker.

Restoration returns the original output/control configuration. **Do not launch
automatically afterward.** Leaving a persistent read-only configuration would be
a separate deliberately documented decision.

## Interrupted operations and refusal

No failed Apply is replayed. The active marker, per-file intents, exact DLL
presence/hashes and original backups remain available. InspectRestore can inspect
a partial Apply when the INI is either the untouched baseline or valid input-only
configuration. Restore only moves files whose original/quarantine presence is
unambiguous and whose bytes/permissions remain exact. If both paths exist,
neither exists, a hash changes, the installation changes, or navigation/source
configuration changes, stop and inspect the private journal. The tool never
deletes an unexpected file or restores the whole profile from an old backup.
Completion records use unique names. If interruption occurs after durable
completion but before active-marker removal, a fresh InspectRestore/Restore
rechecks all state and can finalize without overwriting the earlier evidence.

## Tests

`test-commissioning.ps1 -PortableContracts` passes twelve Linux contract groups for
byte preservation, malformed and ambiguous input, source review completeness,
dependency/evidence changes, safe restore constraints and quarantine boundaries.
The native suite additionally runs the actual entrypoint against a unique
temporary tree with fixture context only: public baseline refusal, prepare,
changed-DLL refusal, apply, replay refusal, reintroduced-DLL collision refusal,
post-inspection user-edit refusal and exact restore. Injected interruptions cover
the first Apply DLL move, edited-INI publication, restored-INI publication, first
DLL return, and completion journaling before marker removal. Each interruption
must recover through a fresh InspectRestore/Restore. The native test substitutes
no transaction logic and never queries real profile, registry, processes, serial
ports, tasks or hardware. `-IsolatedLocal` explicitly enables this disposable test
outside CI; native results and actual commissioning remain separate evidence.

The current implementation passed **12/12 Linux portable groups** and **21/21
native Windows disposable filesystem groups** on the boat PC. The latter ran
only temporary fixtures with explicit fixture PATH restored afterward; it did
not access the actual profile, execute plugins or perform commissioning. Private
local evidence is retained as `commissioning-portable-final.json` and
`native-commissioning-final.json` under `evidence/local/boat-beta2/`. Corresponding
CI qualification and actual installed launch/restore evidence are separate gates.

The exact private recovered baseline was also transformed in memory, without a
file write: the result remained 21,380 bytes and changed exactly one byte to
SHA-256 `8fa87a4550a645521f0833155371f497c4631577d354466d41808f7fea7fe0fe`.
The serialized Garmin upload preference remained unchanged. This verifies the
specific configuration mapping; it does not claim hardware or launch acceptance.
