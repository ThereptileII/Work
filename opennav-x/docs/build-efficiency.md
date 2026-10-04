# Build selection, dependency reuse and retained application checks

This is an operating contract under [the delivery policy](delivery-workflow.md),
not a backlog or a claim of completed native verification. Jira remains the
backlog. Actual run identities, outcomes and measured timings belong in linked
evidence; implementation or an offline contract test does not prove a Windows
build, an accepted package or a performance improvement.

## One coordinator per coherent delivery batch

The coordinator assigns bounded parallel tasks with explicit file ownership,
integrates their changes and focused checks, and owns the full application build
and release. Check the combined changes before requesting that build. A subtask
completion does not independently request another build, release or boat action.
Update documentation only when contracts, architecture, delivery, safety or
material evidence change. Preserve historical specifications and failed evidence.

Development defaults to Staging. Production readiness or promotion requires the
user's explicit instruction for an exact candidate. Both release channels remain
draft while public access is closed. Design review, prototype comparisons and
design-only DPI sweeps remain explicit opt-in. A functional defect still requires
its relevant functional check; it does not authorize an appearance review.

## Select work from actual inputs

`tools/ci_changes.py` compares resolved Git commits and emits product, helper,
dependency and documentation decisions with changed paths and reasons. Local
project paths map to `opennav-x/` in the published monorepo; workflows are at the
repository root. Both sides of a rename and deleted inputs participate. Missing
or ambiguous history and unreviewed paths select the conservative build path.

| Change | Required scheduling boundary |
| --- | --- |
| Ordinary docs or recorded evidence | Relevant content checks; no new application package or release. |
| Workflow or test/retest helper | Focused contract/helper checks; reuse exact retained candidate bytes when needed. |
| Application, installer, runtime resources or packaging recipe | Build a new candidate and run relevant functional/package gates. |
| `docs/beta2/`, third-party notices or `LICENSE` | Treat as packaged product inputs, even though they are documentation. |
| Dependency producer, source lock or ABI/configuration input | Invalidate affected SDK reuse and require a verified matching producer before consumption. |
| Producer workflow alone | Check the workflow and invalidate its reuse fingerprint; do not build the application just to verify scheduling. |
| Independent web project | Run that project's checks; no application acceptance follows. |

The corresponding-source archive includes more files than the runtime package.
Docs/helper-only edits may therefore change the next source archive without
requesting a new package now. They do not qualify a new source revision, relabel
an older candidate, or transfer that candidate's source/license acceptance.
When a new package is produced, all exact-source and license gates still apply.
Manual product selection is a Staging build request and does not request design
validation or Production promotion.

## Verify dependency SDKs before consumption

`skager-windows-dependencies.yml` produces the maintained native Win32
OpenSSL/zlib/curl SDK separately from the application. Reuse requires a successful
authenticated producer run, attempt, job and artifact; sealed hashes and complete
inventories; unchanged source, recipe, options and ABI inputs; and successful
current native tool/environment reprobes. Verify the restored producer prefixes,
headers, import libraries, runtime DLLs, source archives, notices and test evidence.
A matching name or cache key cannot replace those checks.

Reject mismatched, missing, expired, tampered or relocated inputs. Select or build
a valid SDK explicitly rather than silently accepting partial restoration.
Keep same-job fixture-success receipts separate from cross-run SDK provenance.
The AIS runtime gate accepts either authority explicitly, reprobes a reused SDK,
and preserves the existing compiled TLS, transport and session checks before the
GUI build. SDK verification alone grants no application or boat acceptance.

## Retain once, test the same bytes

Retain immutable application outputs and their build/input identity before
running downstream functional and package qualification. Download and verify
those exact outputs in downstream jobs. A failed installer or helper check must
not require recompiling an unchanged application just to repair the check.
Record original build revision, run/attempt and hashes alongside the separate
helper revision and its result. Never rewrite embedded version or source identity
as if a helper rerun produced a new executable. Promotion reuses the exact
qualified package bytes and corresponding source.

Run focused checks for small changes and batch full suites at integration and
release milestones. Navigation, safety, security, recovery, data preservation,
installer/updater, supported ABI and exact-source requirements retain their
failure assertions and native Windows/boat evidence boundaries. Requested
long-running suites run as separate background work with retained logs; report
pending, failed and skipped gates truthfully. The user-directed endurance skip
continues until explicitly changed.
