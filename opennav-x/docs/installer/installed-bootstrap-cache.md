# Original installed bootstrap cache binding

New native enrollment with startup completion emits original-maintenance schema 2.
It retains the cache actually supplied to the original enrollment verifier: its
canonical local path and full NTFS directory identity (volume, file ID, creation
time and descriptor digest). The original input owner retains protected ancestry
through completion. No directory is created, reset, copied or adopted by this
binding. The existing immutable completion bytes and once marker authenticate it;
there is no second customer approval record.

The cache is bound before native service initialization and included when the
original completion is prepared. Missing retained input custody refuses new
completion rather than silently producing a schema-2 record without a binding.
The native reopening seam requires a genuine original completion owner, reopens
that exact path with existing ancestry checks and compares the complete original
directory identity. A different protected directory is not a replacement cache.
Existing TUF metadata and rollback floors remain in the original `metadata` and
`targets` locations. The later admission and command integration below connects the dedicated
Setup/Maintain component action; actual installed customer acceptance remains pending.

Schema-1 completion bytes remain unchanged and accepted for existing maintenance
and explicit operator operations. Existing installations, including any boat
installation still on schema 1, have no authenticated cache mapping in their
completion. They must refuse automatic customer-cache discovery. A new empty
cache cannot preserve their earlier rollback floors. A future compatible
migration must independently prove the actual old cache and preserve its native
identity, existing contents and floors, with a separately reviewed native
binding procedure. Current ACLs, a locally declared path or neighboring hashes
cannot supply that proof. No automatic migration or workaround is implemented.

Portable tests cover schema-2 binding, missing/tampered identity/path refusal and
unchanged schema-1 serialization. Inert Windows component tests cover original
versus replacement directory identities and missing producer custody; these do
not qualify installed cache migration, SCM, UAC or the customer journey.

## Explicit fresh preparation

Fresh enrollment requires an independently authenticated original provisioner and
an absent cache under a protected parent. Its explicit operator preparation step
creates and verifies the original root before enrollment; it is distinct from
this completion binding. See [bootstrap preparation](broker-bootstrap-policy.md).
No existing-cache verification, installed customer action or schema-1 reader can
invoke preparation as a recovery shortcut. Native first-enrollment and installed
customer acceptance remain separate from the new focused helper checks.

## Authenticated successor admission

`AcquireInstalledCodeAdmission` is a fixed elevated installed-helper entry with
no caller path, SID or approval pin. It opens genuine original completion and
its bound cache, discovers the distinct authenticated upgrade target and binds
its predecessors to original code. Only the exact signed provisioner is acquired
into that same protected cache root; its retained read-only download/native pins
and existing PE loader checks precede execution. Owned policy/trust bytes and
comparison digests are returned by a checked snapshot, which grants no execution
authority by itself.

`ConsumeForLaunch` explicitly drains ordinary original fixed-image read pins and
transfers cache/acquired-image custody into a separate launch owner. The initial
owner becomes permanently unusable. The command-side native launcher must retain
that owner through the actual same-account child exit, authenticate its original
process/token/session and retain the original helper's rename-compatible observer.
An acquisition is not removed while the admitted successor is running. The fixed
successor entry independently reopens original completion/cache/trust, verifies
the exact signed policy and actual running new provisioner, then drains preparatory
fixed-image pins before calling the existing `UpgradeEnrolledCode` engine. It
accepts comparisons only and currently supports fresh transitions; it does not
automatically resume uncertain operations.

A verified absent optional signed target yields `ErrInstalledCodeUpgradeUnavailable`
only after successful authenticated metadata refresh and a fresh local authority
check. Network, expiry, root/floor, signature and partial-state failures remain
errors. `ErrInstalledCodeUpgradeCurrent` requires the genuine installed reader's
complete successor-ready closure and the actual code pair matching the signed
successor. Both results close all owners without downloading/executing code;
cleanup failures suppress the no-change classification.

A reusable zero-byte `.installed-code-refresh.lock` under the original bound
cache serializes native customer and operator refreshes. Its share-zero native
handle spans root selection/New/Refresh, refuses concurrent or malformed lock
objects, and is never deleted/reset/retried. The admission parent releases its
refresh owner before spawning the child. The child releases its preliminary
verification lock before the existing upgrade engine acquires the same common
operator lock and freshly verifies again; cache and running-image custody remain.
Portable tests cover admission/rejection, exact tuple ownership, cancellation,
no-change classification and release ordering. Native lock fixtures cover
exclusive/reusable custody and malformed objects. The combined component/command
native gate passes at `7dde3b15` in run `38628998074`; this does not replace
actual installed Windows/customer-journey acceptance.
