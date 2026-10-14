# Customer rollback of a protected installation

SCRUM-27 adds the fixed customer `rollback-current` action to the installed
Maintain wizard. This is application selection only. Repair remains separate;
legacy Lifecycle never mutates protected enrollment.

Maintain's Rollback action invokes only the fixed protected
`skager-update-provisioner.exe --maintenance=rollback-current`. An incomplete
protected helper namespace refuses. There are no customer paths, SIDs, target
hashes, approval inputs or elevated private commands to supply. The original
unelevated helper retains its original same-account UAC child and waits for that
child; the elevated child retains and checks that same original parent. Declined
UAC, identity drift, cancellation and uncertain child completion never authorize
another execution.

The elevated child retains the original completed installation/service owner,
then makes one read-only `previous-qualified` broker request. The broker holds
its existing operation lock and original registered operator, store and selected
journal owners. It walks actual selected-head linkage to the most recent earlier
different selected release with original successful qualification and closure.
It never selects by lexical generation ID, directory order, version sorting or
caller data. Failed-start recovery advances history without qualifying the failed
candidate. After a manual selection, the prior actually selected release is the
previous target; the action can therefore return to that release after another
explicit confirmation. A baseline without a prior qualified release refuses.

The Windows dialog shows the current version, target version and exact target
commit. No is the default. It explains preservation of CURRENT navigation,
plugins, connections and profile, and that the application stays closed. This
application dialog does not approve or replace Windows UAC. Its late answer
cannot resume an expired/cancelled transaction.

After Yes, the child sends the existing `manual-rollback` request exactly once,
using a fresh ID and the original observed current-head/target-completion pins.
The existing native transaction independently reauthenticates the registered
operator, stock compatibility, original ordinary application exit, absence of
other profile writers, legacy armed-marker denial and frozen CURRENT profile.
It preserves broker/registration ownership, old generations and metadata floors;
it never restores historical profile data, starts the target, repairs files or
reinstalls anything. Selection changes during confirmation fail the expected-head
check. A lost mutation reply is uncertain and is never retried.

A separate fresh `selected-history` request must corroborate the committed head,
selected generation/completion, version and commit. Original client owners must
close successfully before the one success result is emitted. Failure retains
partial state for diagnostics; it does not repeat rollback or fall back to legacy
maintenance. Selecting a historically qualified release is not proof of its
compatibility or successful startup with today's profile. Those remain actual
installed journey checks.

## Fixed bootstrap compatibility

`previous-qualified` is an additive optional maintenance operation. Existing
selected-history, manual-rollback and uninstall request/response schemas are
unchanged. An older fixed broker rejects this unknown operation before mutation;
a mismatched or incomplete response refuses before confirmation. An older fixed
provisioner rejects the unknown `rollback-current` action. Neither case falls
through to legacy Lifecycle.

This feature is executable only after a **fresh matching broker/provisioner
operator kit has been independently authenticated and enrolled**. Updating an
application B1 generation does not update the fixed B0 broker or provisioner.
Their original installed hashes remain bound to the enrollment completion.
There is no broker replacement, re-enrollment, hash rewriting or migration path
in this change. Existing older bootstrap installations therefore retain an
explicit customer-rollback upgrade blocker, even if their application payload
is updated. Their existing independently authorized operator manual-rollback
operation is a separate route, not evidence that the customer wizard works.

Portable tests cover lineage order, failed/manual history, wire isolation,
unsupported bootstrap, target confirmation, original ownership, one mutation,
readback mismatch and cancellation/refusal. NSIS source checks and Windows x86
test compilation are helper checks. Native same-account UAC, actual dialog,
protected B1-to-prior selection/start/close and preservation against genuine
previous packages remain pending; no product or boat acceptance is inferred.
