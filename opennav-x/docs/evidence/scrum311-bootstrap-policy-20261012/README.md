# SCRUM-311 purpose-bound broker publication and verification

Integrated source: `75a770b1d16bb0d23768005d75a25cd3f98f1952`.

* `766d72f8`: strict bootstrap policy/identity parsers and target-path helper.
* `609db5ed`: actual go-tuf reader with independent caller pins and no redirects.
* `a4ea380d`: explicit operator publisher/CLI and retained declaration/floor.
* `75a770b1`: actual publisher-to-reader TLS integration regression.

Three isolated GPT-6.1 Sol tasks supplied schema, reader/integration test and
independent read-only source review; Codex coordinated publisher implementation,
integration and decisions. No Claude review of these new files occurred.

## Observed results

The retained focused Linux run uses pinned Go 1.27.1 and passes **37 top-level
groups, 438 test pass events including nested cases**, with zero failed or skipped
tests across four packages. This is not a full-application regression count.

| Package | Top-level groups | Pass events including nested cases |
| --- | ---: | ---: |
| Actual verifier | 22 | 136 |
| Release/installation policy | 4 | 264 |
| Operator repository | 10 | 37 |
| Operator CLI | 1 | 1 |

[Exact command and source](linux-result.json), [raw test events](linux-tests.jsonl)
and stderr are retained. The selection includes the new tests and existing
affected ordinary release, transport, identity, tamper, rollback, publication,
symlink and concurrent-writer regressions. Earlier affected repository/CLI checks
also passed during implementation; they are not added to this final count.

The actual producer-to-reader test initializes a temporary repository with four
ephemeral signing keys, publishes inert app/bootstrap artifacts and valid notice
ZIP structure, and reads the declaration through the actual bounded go-tuf
bootstrap API over local TLS. It verifies exact whitespace-preserved owned bytes,
typed/signed identity and unchanged ordinary release verification in the same
repository/cache. Fixture `passed` strings satisfy format only and do not qualify
any product artifact. Nothing is executed or installed.

Linux-hosted Windows/386 reader test compilation, operator-command compilation
and affected-package vet pass. [Commands and scope](windows-cross-result.json)
explicitly record `nativeExecution: false`. The operator command's mutations
remain Linux-only. These results do not establish native Windows execution,
service principal behavior or installed update acceptance.

## Independent review and negative control

The independent Codex reviewer found two concrete publisher defects before final
integration: existing immutable destination bytes could be rejected only after a
new floor was reserved, and historical differently cased filenames could alias
on Windows. Both now fail before reservation. New regressions require the exact
prior state to remain unchanged and a subsequent ordinary app publication to
retain the original bootstrap policy. The reviewer rechecked both corrections
and the shared verifier refactor without further findings; it ran no tests.

A private tracked-source copy of the exact integrated revision removed **only**
the destination-preflight invocation. Both new historical-collision subcases
then failed as expected. [Negative-control result](negative-control-result.json)
and [raw failure events](negative-control.jsonl) are retained. Root source remained
unchanged. This is a deliberate fault-injection result, not a failed candidate
hidden by weakening an assertion.

## Authority and remaining gates

The [contract](../../installer/broker-bootstrap-policy.md) separates the app target
from the broker-enrollment declaration. Operator-approved raw policy bytes,
version floor and public-config binding survive ordinary publication and failed
attempts. Declared artifacts have no execution authority. Existing role keys
remain the TUF signing authority; purpose custom metadata is a type discriminator.

Independently authenticating the initial privileged verifier/provisioner, actual
native protected service enrollment, installation/loader custody, launcher
handoff, authenticated healthy-start receipt, full recovery and installed
Windows/boat Now/Later/no-update/offline acceptance remain unfinished. The new
declaration does not make the installed updater ready. SCRUM-311 remains In Progress.

No real operator repository/signing material, full application build, CI dispatch,
push, release publication, boat application/profile change, service activation or
hardware command occurred. The earlier scoped Claude comparison is separate
historical evidence. Expanded broker-source transfer remains pending exact-scope
consent after automatic approval rejection and has not run.
