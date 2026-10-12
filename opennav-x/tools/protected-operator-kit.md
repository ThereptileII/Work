# Protected operator kit

On native Windows, use an original authenticated launcher-only install from the
same clean commit as this checkout:

```powershell
python tools/package-protected-operator-kit.py --source-install C:\producer\launcher-install --output C:\delivery\protected-operator-kit.zip --commit <exact-40-lowercase-hex-commit>
```

When the same native run already produced the complete launcher → broker →
provisioner → baseline-handoff install, add
`--components-install C:\producer\broker-install`. This read-only reuse mode
invokes **no producer or compiler**. It verifies the existing three components
with the existing package verifiers, requires their launcher/source closure to
match the original launcher install exactly, copies only fixed component
binaries/records into the private stage, and retains the already-copied shared
source archive with the same verified hash. Both original installs' file
identities, sizes, hashes and exact inventories are checked before and after.
Unknown entries, links, mixed commits or changed original objects refuse. The
option never discovers an install or silently falls back to rebuilding.

The output must be new, outside either source install, with an existing plain parent
directory. The producer verifies and copies the original launcher closure into a
private install, builds the broker, provisioner and baseline handoff with the
existing fixed producers unless explicit verified components are supplied, and
verifies their exact dependency/source chain. It
rechecks the unchanged original launcher files before and after production.
Neither the original launcher nor the application install is modified.

The deterministic ZIP contains only `skager-update-broker.exe`,
`skager-update-provisioner.exe`, `skager-baseline-package.exe`, their original
build records, the shared exact corresponding-source archive, its exact license
and source reference, and `SHA256SUMS.json` (schema, product commit and each
payload's byte count/SHA-256). The outer ZIP byte count/hash is printed as data.
There are no launcher binaries, update trust/configuration, credentials, profiles
or sensor data copied into the kit. Unexpected source-install entries refuse.
Source archives retain their original corresponding-source contents.

This kit **must be paired with the independently authenticated same-commit
recovery/launcher prerequisite for verification and use**. It is not a standalone
approved installer or evidence that initial enrollment is safe. Existing package
verifiers require that launcher prerequisite alongside the extracted tools in a
new separate verification copy; preserve the original producer install.
Authenticate the original native producer/transfer separately before any first
execution. The checksum manifest detects byte changes; it does not authenticate
the publisher, approve source/request/broker pins, or grant mutation authority.

Packaging never executes or elevates any output, enrolls/starts/stops a service,
creates trust, supplies operator approvals, or touches application/profile data.
Enrollment, commissioning and explicit authenticated maintenance keep their
existing independent approvals and native ownership checks. Service quiescence
alone does not authorize repair/uninstall or establish a cold profile.
