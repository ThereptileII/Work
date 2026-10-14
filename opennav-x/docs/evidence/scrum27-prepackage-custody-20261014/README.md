# SCRUM-27 original prepackage custody, 2026-10-14

The original compiled applications remain available, but the original Setup and
completed package inputs are absent. After focused native qualification of the
receipt-helper repair and its fixtures, the coordinator selected **one new
candidate through the existing Staging workflow**. No new build was dispatched
by this investigation. The original failed candidate remains unqualified.

[custody.json](custody.json) records the authenticated identities, source
inventories, hashes, native compiled-test results and external source locks.

## What was verified

GitHub API metadata bound run `38529654044`, attempt `1`, Windows job
`115743280774`, and recovery artifact `11880282958` to exact source
`faa211b568f0e79e1d1065ba3fb76c3b1e801fd6`. The run and job remain failed. Both
application compilation steps and prepackage recovery retention/upload succeeded;
the sole failed job step was installer generation/receipt retention. Normal
compiled-input sealing and its artifact upload were skipped.

The downloaded outer artifact matched its authenticated size **427,230,985
bytes** and SHA-256
`edd1c3398eb91aa766afefefacafa642d3e09bfb07040463f495d38f839d0333`.
Outer and inner ZIP CRC checks passed. The retained receipt's inner archive and
manifest hashes matched, and every one of **16,790** retained files matched its
recorded size and SHA-256, totaling **892,845,008** uncompressed bytes. Inspection
executed no original binary and extracted no application or source tree.

Both native compiled XML suites contain **146 cases, zero failures/errors and
zero skips**. These compilation/component results do not establish installer,
desktop, Staging, public-release or boat acceptance.

The product snapshot contains **8,938** actual tracked files with clean-source
provenance; **48** root workflow files also have clean provenance. The pinned
OpenCPN integration snapshot retains **4,392** actual tracked files, with **86**
tracked paths recorded as changed by integration. Its remaining three references
are one Flatpak gitlink and two link-text entries. This expected integration dirt
does not change the recorded product commit. Arbitrary untracked source was not
captured.

## Why original compilation reuse is not a simple retest

`COMPILED_RECOVERY.zip` was retained before updater, portable and Setup packaging.
It is deliberately distinct from `STAGING_BUILD_INPUTS.zip` and has no accepted
restore/qualification path. It excludes the original Setup, payload/manifest,
portable ZIP and corresponding-source ZIP. It also excludes the OpenSSL, curl,
zlib and wx source/download caches needed by the existing packaging recipe.
Private-chart installed source/manifest, generated headers and chart resources
are retained. The same-run qualified updater remains a separate artifact.

Packaging succeeded later, and the installer step subsequently failed in the
receipt helper after NSIS generation. That does not preserve the generated Setup
bytes. Recreating packages would produce **new package bytes around the original
compiled applications**, requiring separate packaging/run/helper custody and all
never-run deferred native Windows qualification. The final app-local CRT package
selection would also need fresh explicit provenance. The operator kit is a
separate approval input and cannot replace the missing installer closure.

Prepackage recovery contains the original peer-CLI and AIS receipts and the
private-chart trust evidence. Later peer-buffer and public-downloader evidence
must be authenticated from the separate original build-evidence artifact; their
successful job steps do not manufacture missing package inputs.

The existing source packager requires the product/recipe checkout to remain
clean at the original commit and the integration tree to retain its pinned Git
baseline. Copying a later repaired helper into that checkout would violate those
invariants. A resumed package would need original product source and separately
retained packaging-harness source, without relabeling the original application.

The ordinary sealed-input CLI also requires the actual producing run/job, and
the staging fetcher requires its successful producer job. Impersonating original
Actions metadata or admitting this failed job through that path is unacceptable.
An accepted resume would therefore need coordinated new provenance/seal, fetch,
receipt-consumer and composition contracts plus native checks. That work is larger
than a receipt-helper repair. The observed existing Windows producer took about
**54 minutes**; a preflight-qualified new candidate through its established gates
is the lower-risk and likely faster delivery path. This is a scheduling decision,
not permission to skip any existing gate or a claim that recovery bytes are lost.

The audit retained its original artifact and detailed local inspection under
`/tmp/skager-candidate-38529654044/`. Those local paths are operational evidence,
not durable package authority; the immutable GitHub identities and hashes above
remain the reproducible selection.
