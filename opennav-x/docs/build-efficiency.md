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
Automatic application delivery runs only on `staging`; mirroring a commit to a
historical branch must not create a second build or a duplicate draft Release.
Manual dispatch remains available for an explicitly selected historical branch.
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
| `tests/**` | Conservatively select product/build checks: fixture support is linked into executables and compiled tests need build targets; dedicated narrower targets are not yet selected. |
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

For an explicitly requested new Staging candidate after helper-only corrections,
put the exact trailer `Skager-Staging-Build: true` in the final trailer block of
the pushed commit. The selector accepts it only once, on a `push` to
`refs/heads/staging`, from the exact checked-out HEAD. Prose mentions, duplicate
or conflicting trailers, pull requests, other branches and tags do not force a
build. This selects the same product/dependency gates as `--force-product`;
design and extended scenarios stay off unless explicitly requested separately.
Ordinary helper edits continue to skip product builds.

The native AIS reprobe stages maintained curl before the application driver runs.
Upstream `win_deps.bat` uses `cache/buildwin/libcurl.dll` as the marker for its
entire stock support bundle, including LibArchive. Before the fixture build,
`prepare-windows-stock-deps.py` authenticates the restored SDK and pinned batch,
checks that this disposable marker matches the maintained Win32 curl prefix,
and removes only that marker when stock support is incomplete. Unknown bytes
or redirected paths fail closed. The unchanged driver then provisions stock
support and restages authenticated TLS before compilation; producer prefixes
and their fingerprints are unchanged. The focused AIS workflow checks this cold
cache sequence and full native CMake configuration without compiling/installing
the application; private chart adapter compilation and release gates remain separate.

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
and preserves the existing compiled TLS, transport and session checks before desktop
qualification. SDK verification alone grants no application or boat acceptance.

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

Before Windows updater/ZIP/Setup packaging, retain and upload a distinct
`compiled-recovery-<commit>-run<run>-attempt<attempt>` artifact. Its
`COMPILED_RECOVERY.zip` contains both compiled install trees, generated identity
headers/test binaries, and actual tracked product/integration/recipe bytes with
HEAD references and dirty-source provenance. It excludes arbitrary untracked
source and is explicitly unqualified; it cannot substitute for the normal sealed
Staging inputs. Investigate a failure against these bytes without claiming a
successful package or relabeling their producer.

The native updater is compiled once in the clean, same-run updater-contracts
job. The integrated application job requires that job's success, downloads its
exact commit-named artifact, and verifies/copies only the launcher, build record
and corresponding-source ZIP. Do not rebuild it in the integrated checkout:
that tree contains fetched upstream/cache inputs and is intentionally unsuitable
for the updater's strict clean-source build check. Keep that check unchanged and
keep Go setup out of the immutable dependency reprobe environment.

## Preserve failed producer outputs

A failed producer never becomes an eligible SDK. If compilation/tests complete
but final verification or sealing fails, retain a separately named
`unqualified-dependency-recovery-*` artifact containing original recipes,
source archives, installed dependency outputs, native metadata and evidence.
It is recovery material only: the normal downloader rejects its name and failed
producer status. Recovery requires independent investigation and qualification;
there is no automatic fallback to those files. This keeps a sealing-helper
failure from discarding the completed native outputs.

## Live processor observations during reuse

OpenSSL 3.5.9 `version -a` includes live `CPUINFO`, so identical verified binaries
can report different CPU capabilities on different hosted Windows machines.
The read-only consumer probe compares every other version/build/configuration
field exactly and retains both original and current CPU observations separately.
It rejects malformed, missing, duplicate or overridden CPU fields; it never sets
`OPENSSL_ia32cap` to imitate another machine. The original producer manifest and
test evidence stay unchanged. Original curl producer/native-tool-fact scripts
remain byte-identical and their native environment reprobe still runs.

The first successful SDK uses an explicit two-file compatibility record for the
reviewed consumer-only orchestration/verifier change. It is bound to exact
producer commit and old/current file hashes, including separate LF/CRLF checkout
bytes; any unreviewed change fails. Compiler recipes, native receipts, library
source, options, ABI and binary hashes are not exempted. This enables a corrected
consumer check without rebuilding the already verified libraries.

## Actions operation

Use the **staging** source branch in GitHub's “Use workflow from” selector; the
repository's default branch also contains a separate firmware project.

- **SKAGER Staging**: ordinarily automatic for product inputs; manual dispatch
  forces a Staging build. The change selector prints why each check was selected.
- **SKAGER verified Windows dependencies**: `produce` builds/tests and uploads
  a new SDK only. Select its exact successful run/attempt/artifact digest in
  `tools/windows-dependency-bundle.lock.json`, then run `verify` from the consumer
  revision before relying on it. Never select “latest” or a partial failed SDK.
- **Re-run failed jobs**: a desktop-qualification failure inherits the completed
  build's original artifact and attempt. The successful compiler job stays done.
- **SKAGER retained Staging checks**: for a corrected test helper, choose its
  source branch, original build run and attempt, and `installer`,
  `installer-charts` or `all` scope.
  This authenticates/downloads the original compiled inputs, uses the candidate's
  installer engine and records the new helper revision. It creates diagnostic
  evidence, never a new executable or an automatically accepted release.
  The narrowly named `skager-staging-retest` push branch uses a committed exact
  artifact selection and runs only installer plus charts. It is useful when
  those were the failed and subsequently skipped checks; it does not repeat
  completed application compilation or earlier functional checks.
- **SKAGER composed Staging delivery**: after those replacement checks pass,
  an explicit committed request authenticates original and replacement evidence
  and assembles a draft Release from frozen inputs. Product and helper identities
  remain separate, and the original failed run is preserved. This is not a way
  to waive a failed gate or claim a new binary was tested.
- **SKAGER Production**: remains the separate explicit named-candidate promotion
  flow. No design review, rebranding or application compilation is added to it.

Expired SDKs, a changed native image/toolchain, a changed producer recipe, or
missing retained inputs stop reuse with a clear failure. They require a new
verified producer; they do not authorize a weaker fallback. Future Staging runs
still have to pass their application, native desktop and installer gates before
creating a candidate Release.

## Measured workflow verification

The [2026-10-04 evidence](evidence/2026-10-04-delivery-efficiency.md) records
190 focused checks per platform and the first authenticated cross-run SDK proof.
In that measured pair, dependency production took 50m30s and verified reuse took
1m14s; the entire warm job took 2m11s. This does not qualify a new application or
promise the same time for every runner. The next real Staging candidate retains
its application, native functional, installer and exact-source gates.
