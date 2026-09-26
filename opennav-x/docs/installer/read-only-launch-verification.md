# Launch verification after read-only preparation

Normal remote XNav, Legacy and Safe launches require both the existing independent
read-only audit and the completed commissioning transaction. Preparation alone
does not authorize a launch. Portable display reviews retain their separate,
connection-free profile verification.

The private `boat-target.json` read-only audit includes an explicit binding:

```json
{
  "readOnlyAudit": {
    "commissioning": {
      "record": "<private prepared.json returned by Prepare>",
      "recordSha256": "<exact prepared record SHA-256>",
      "appliedSha256": "<exact applied.json SHA-256 returned by Apply>"
    }
  }
}
```

This supplements the existing reviewed profile hash, build commit, review time,
connection/route assertions and complete plugin list. It never replaces those
fields or invents an operator approval. Keep the record private.

`verify-commissioning-launch.ps1` verifies the active owned transaction, exact
installed generation/context, complete applied record, copied source-review plan,
cold inventory, every saved review document, the recovered baseline and exact
one-byte input-only profile proof. All quarantined DLLs must be absent from their
original locations and retain their exact quarantine and backup hashes.

It then compares the complete managed, stock and installed-generation plugin
trees against the reviewed inventory, subtracting only verified quarantine moves.
This includes helper executables, runtime DLLs, resources and directories. A
changed chart helper therefore blocks launch even if its plugin DLL is unchanged.
Partial Apply, missing ownership, a started Restore, and completed Restore all
block launch. The existing per-plugin audit is also repeated.

The current INI must match the independently renewed launch audit. Normal UI
persistence may be reviewed for another mode launch; changing the connection or
chart-path configuration requires separate preparation. Restoring an output
connection cannot be authorized merely by updating the audit hash. Both source
review and launch review must be current and must not be dated in the future.

The checks run before scheduling and again inside the interactive task immediately
before `Process.Start`. The verifier returns only the exact recorded child
environment. `ProcessStartInfo` sets its working directory to the verified
application directory and PATH to that directory plus the known Windows system
directories. OpenCPN may prepend its reviewed plugin roots during initialization.
The inherited SSH PATH, including empty entries, is not used. No global user or
system environment is changed.

This remains a scoped read-only commissioning boundary. The pinned OpenCPN serial
gateway initialization and explicitly documented vendor chart helper behavior
remain as described in the source review. It is not electrical bus silence,
physical equipment acceptance, or permission for actuator commands.

`test-commissioning-launch.ps1` executes the real verifier against isolated
filesystem fixtures, with only native identity/environment discovery substituted.
Its eleven groups exercise changed helpers in all roots, returned control DLLs,
changed evidence/records/recovery files, incomplete Apply, restoration boundaries,
reviewed view persistence, prohibited profile changes, expired reviews and changed
installation context. `-PortableContracts` runs on Linux; native Windows uses
disposable CI or explicit `-IsolatedLocal`. Neither mode opens an application,
serial port, marine transport, or actual boat profile.

Local validation on 2026-09-26 passed all eleven groups on Linux PowerShell and
all eleven on native Windows PowerShell 5, using isolated temporary files. The
existing twenty-one native boat-tool guard groups also passed with the updated
shared launch code. Windows exposed an array-pipeline difference which is now
avoided by reading the parsed multi-plugin plan directly. These are local
filesystem/contract results; qualification of the published commit in CI and
actual read-only application commissioning remain separate gates.
