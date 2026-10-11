# SCRUM-311 inactive broker package prerequisite — 2026-10-12

This is a source increment and component check, **not** a qualified Windows
package, provisioned service or working installed updater. Exact source,
checker and retained component identities are in `source-receipt.json`.

## Package boundary

The native probe retains a separate `updater-native/broker-install` directory.
It first uses the existing verified launcher/source transfer, then adds exactly
`skager-update-broker.exe` and
`opennav/third-party/update-broker/build.json`. The ordinary `install` output
remains launcher-only; the application build's retained-package transfer and
its refusal of unexpected files are unchanged. No normal product installer
installs or enables this service.

The producer requires native Windows, the exact clean commit, canonical pinned
Go 1.27.1, Windows/386/SSE2, CGO disabled and the fixed GUI build recipe. It
reuses the original corresponding-source archive instead of regenerating or
downloading it. That archive now also includes the broker producer, verifier
and their required `updater_package.py` helper. Current source, module-cache
and Go source inputs are compared with the retained inventory before and after
compilation. Publication refuses existing outputs and preserves partial failures.

The consumer checks fixed paths, duplicate-free bounded records, actual binary
hash, Win32 PE32 GUI format, actual embedded inline Go identity, same commit,
locked dependencies and exact source descriptor/inventory. It returns the
unchanged existing source descriptor. These are consistency checks under the
caller's authenticated successful same-revision producer prerequisite. They
are not independent publisher authenticity or reproducibility evidence.

## Real emitted-byte checks and corrections

A single Linux cross-compilation produced the Windows/386 broker at
`5e260b4a75a2bf3e96e5d6c7baad49aea00703f7`. This did not execute on Windows.
The real embedded metadata exposed a fixture error: Go deliberately omits
`-ldflags` when `-trimpath` is enabled, as implemented by pinned
`cmd/go/internal/load/pkg.go:2469–2480`. The original checks incorrectly
required that field. Producer and consumer now follow the pinned omission,
reject contradictory metadata, and independently check actual GUI PE format.
The prescribed build command still sets `-H=windowsgui`; omitted metadata is
not claimed to attest linker flags.

The actual read-only module-graph inspection returned 82 dependencies, with
63 lacking optional checksum metadata in the committed module context. The
launcher intentionally retains supplementary checksums in its private
`provisioning/go.sum` instead of rewriting the product checkout. Requiring
those optional fields from the product-context graph was incorrect. The broker
now binds and reuses the already-retained provisioning checksums for its offline
cache audit, while keeping compilation in the original clean source tree.

The initial evidence helper also assumed every graph record contained `Sum`;
its failure is retained separately. The repaired component check compares the
12 actually linked dependencies with committed `go.sum`, compares the parser
with actual `go version -m` output and keeps the binary's original source
identity. The same compiled bytes were reused after failures; there was no
application rebuild or relabeling as a newer binary.

An independent bounded Codex review inspected the disjoint source changes.
Its initial metadata-format inspection missed the conditional omission above;
the real emitted executable supplied the counterexample. The separate earlier
[Claude review](../scrum311-acquisition-claude-comparison-20261012/README.md)
does not cover these later files.

## Focused results

| Check | Actual result |
| --- | --- |
| Broker producer policy and offline graph/input/cache failures | 30 tests PASS at `17435429`. Fake runner; not native packaging. |
| Broker consumer, actual-metadata consistency and unsafe source/package inputs | 11 tests PASS at `bd15f02e`. |
| Affected existing launcher source producer | 14 tests PASS at `75c6859d`; its source is unchanged by the later broker-only graph repair. |
| Existing updater copied-package boundary | 16 tests PASS at `5e260b4a`; that consumer source remains unchanged. |
| Workflow strict YAML/security keys and same-run launcher package reuse | 2 checks PASS against the final workflow bytes. |
| Actual retained Windows/386 GUI PE, metadata and 12 linked dependency checksums | PASS. Linux cross-compilation and parser/producer checks only; no Windows execution. |

There are **73 distinct focused Python tests**, not a sum of repeated runs.
Only changed or failed checks were rerun. The real 82-module retained-archive
audit and complete native producer have not executed; portable fake-runner
coverage does not qualify them. The failed real metadata check and evidence
helper are retained, with no recompilation after either failure.

## Remaining qualification

Native producer execution, authenticated artifact transfer and native service
identity/ACL/IPC remain unqualified. No approved committed public bootstrap
root/origins or authenticated privileged installer-to-user/install handoff was
found; fixture roots and documented host examples cannot supply that authority.
Fresh provisioning must initialize trust under the actual broker account, not
an administrator/SYSTEM substitute or an adopted precreated channel.

Activation, full loader/installation custody, crash/orphan reconciliation and
exact-package Windows/boat update acceptance remain open under SCRUM-311.
No full application build, CI dispatch, boat deployment, service activation,
physical command, publication or Production promotion occurred.
