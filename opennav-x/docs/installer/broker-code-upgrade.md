# Existing-enrollment broker/helper code transition

This is an **unqualified explicit operator foundation for SCRUM-311**. It is not
wired to ordinary Setup Update, Maintain, application startup or public downloads.
Native installed-service, installer, profile-preservation and boat qualification
remain required. The inert NTFS tests exercise disposable files and reader
selection; they do not install or qualify a service or execute the installed
customer journey. Cross-compilation does not establish Windows behavior.

## Independent authority

Authenticate the new provisioner's final bytes, publisher and loader chain before
its first elevated instruction. Separately approve its SHA-256, the exact signed
upgrade-target SHA-256, unchanged original trust bytes/SHA-256 and exact original
installed-completion SHA-256. UAC, a self-hash, adjacent package JSON and a CLI pin
manufactured from an untrusted local file do not establish initial authority.

The reusable signed target is `bootstrap/broker-code-upgrade-beta.json` (or the
corresponding stable target). Its exact schema is:

```
{schema: 1, purpose: "machine-update-broker-code-upgrade",
 previousBrokerSha256: <exact allowed predecessor broker hash>,
 previousHelperSha256: <exact allowed predecessor provisioner/helper hash>,
 bootstrap: <full existing BrokerBootstrapPolicy>}
```

The TUF target's custom identity has exactly `purpose`, `channel`, `version`,
`commit`, and `trustConfigSha256`; the purpose is the distinct code-upgrade
purpose and the other fields equal the embedded bootstrap. The nested enrollment
policy supplies the fixed service/account/KnownFolder IDs, new broker/provisioner,
source and notice tuple. A signed enrollment target alone cannot replace existing
code. Both executable hashes must differ from their predecessor hashes. There
are no wildcard predecessors or per-customer identifiers in the reusable target.
Publisher monotonicity and current TUF metadata/root expiry and durable rollback
floors remain mandatory. The consumer does not infer a broker version from the
original application's version.

The local original-completion pin and genuine protected native completion bind
this transition to the actual user, original request/baseline/publication and
full old code identities. The exact original trust bytes must match. An expired
*original* bootstrap anchor may be read only after this native installed-completion
check; current TUF root/metadata validation remains live and unchanged. No fresh
enrollment parser is relaxed and no floor or cache is reset.

## Operator entry and bounded sequence

The independently authenticated new provisioner accepts one explicit operation:
`--code-upgrade=true`, or `--code-upgrade-resume=true`, plus all of:

- `--trust-config`: exact unchanged public trust input.
- `--bootstrap-cache`: already protected durable operator TUF cache.
- `--artifact-directory`: already protected acquisition directory.
- `--upgrade-policy-sha256`: independent exact signed upgrade-target approval.
- `--provisioner-sha256`: independent new running-provisioner approval.
- `--trust-config-sha256`: exact unchanged original trust approval.
- `--original-completion-sha256`: independent immutable original-completion approval.

No replacement path, caller SID, baseline substitution, service name, store reset,
registration or initialize-store input exists. Duplicate, mixed and unknown
options refuse before opening inputs. Ordinary maintenance retains STOP-only SCM
rights; this separate native entry obtains only the existing service's required
query/config/start/stop rights, with no service creation/deletion authority.

1. Verify the distinct purpose-bound TUF target, exact approved bytes and actual
   running new-provisioner source/PE loader. Verify the genuine original installed
   completion, unchanged trust and exact allowed predecessor pair.
2. Flush immutable protected intent before staging. Copy/hash/flush both new files
   under the retained protected code parent, with exclusive names and native
   identity checks. After each complete copy, flush a distinct intent-bound staged
   receipt containing its full native identity before staging the next file.
   Flush a reservation binding original completion/request,
   policy bytes/hash, original full code IDs and staged full new IDs.
3. Disable the exact original service. Its original retained process/token/image
   must drain and exit with code zero. Flush/read back a protected clean-exit
   record bound to the reservation and actual original PID/creation time before
   releasing any old image owner or renaming any code.
4. Hand off read custody to exact-ID DELETE-capable owners, retaining protected
   ancestry and descriptors. Nonreplacing native relative renames retain old
   broker/helper as `broker.previous.exe` / `helper.previous.exe` and publish the
   two staged successors only at the fixed broker/helper filenames. No foreign
   destination is overwritten. NTFS creation-time tunneling is handled only for
   a known original handle move or exact reserved identity at a fixed name.
5. Flush a separate successor receipt. Validate the already initialized store
   through the original virtual-account service's stopped `verify-store` mode;
   never initialize or register again. Flush durable complete-pair selection
   before restoring AUTO_START. Start normally once, retain/recheck the actual
   new process/token/image and require an authenticated registered-client history
   response. Only then flush the final successor-ready marker.
6. The fixed installed helper reader requires original immutable completion plus
   exact intent/reservation/drain/successor/pair/ready closure and both actual
   fixed full code identities. Missing, malformed or partial successor state
   never falls back to original code.

The original completion/once marker, B0 launcher/startup publication and links,
original request/baseline, trust roots and floors, registrations, generation
histories/journals/source closure and application/profile bytes are preserved.
Service account/image arguments/recovery configuration stay exact. The transition
permits one immutable successor; a second *new* code upgrade refuses. Ordinary
application version updates are not restricted to one version by this contract.
A later broker-code generation needs an extended authenticated chain.

## Recovery and current limits

Explicit resume may instead terminate an **early** transaction while the exact
original fixed broker/helper IDs remain and the original service is AUTO_START
and normally RUNNING. The same independently approved target, running new
provisioner, unchanged trust and original completion remain mandatory. A
query-only service owner retains the actual original broker process/token/image
through authenticated registered-client history readback and exclusive durable
`.code-upgrade.aborted.json` publication. It never stops, configures or starts a
service, replaces/deletes a file, restages code or removes transaction evidence.

Early abort accepts canonical intent alone with both stages/receipts absent,
one complete broker stage with its exact receipt, or both complete stages with
their exact receipts. A complete reservation is permitted only when its bytes
equal that same intent and both receipt-bound full native IDs. Any old backup,
drain, successor, pair, ready or normal-start-attempt namespace forbids early
abort. Staged bytes without their original completion receipt, including legacy
stages and genuinely partial copies, remain unprovable and refuse. A receipt
is written only after complete copy/hash/flush; interruption between file
creation and that receipt is deliberately not adopted. A torn intent, receipt,
reservation or abort write also refuses.

The installed reader admits original code only after rechecking the exact
immutable abort closure, unchanged actual full original code IDs and every
retained completed staged-file identity. Unknown/malformed state does not gain
fallback authority. Abort restores original reader eligibility; it does not
complete the upgrade. `ErrCodeUpgradeAborted` reaches the caller as a distinct
non-success outcome. The retained intent/abort permanently consumes this one
transition: neither a fresh invocation nor explicit resume can silently reset
it or retry staging. A future authenticated transition protocol is separate.
Actual installed SCM/IPC/customer acceptance remains unqualified by the inert
file-closure and portable owner fixtures.

Within the same live transaction, a rename failure can restore only its retained
old/new handles to original/staged positions while the original service remains
disabled and stopped. It does not restart old code automatically or remove any
intent/reservation/staged evidence. A native rename that takes effect before
readback/rebinding fails can leave an uncertain path; the explicit resume entry
re-observes actual known positions instead of blindly retrying that mutation.

Explicit resume reauthenticates the **same** target, new provisioner, unchanged
trust and original completion. Native observations select its bounded
phases. Disabled rename recovery requires the full reservation and protected
clean-old-exit record, checks that the recorded original PID/creation identity
has not unexpectedly survived, and retains the original service/registry/config
with exact DISABLED and clean STOPPED state. Only the original/new native IDs,
hashes, lengths and descriptors may occupy the fixed/staged/previous names.
Unknown/missing/duplicate/mixed foreign files refuse; no path is adopted from a
current ACL or hash alone. Resume finishes that exact reserved pair and repeats
read-only initialized-store/readiness qualification before terminal completion.
Existing successor/pair publications may be reused only if their exact bytes
match this same transaction. Already completed invocation is not a second upgrade.

A separate terminal finalizer handles AUTO_START with the exact successor already
normally RUNNING, the complete pair/receipt present and ready marker genuinely
absent. It independently checks the original completion, exact approved policy,
intent/reservation/clean-exit/pair closure, retained old backup identities and
both actual fixed successor identities. It binds the actual running successor
process, creation time, primary token and image, performs an authenticated
registered-client selected-history query, then rechecks the same process and
native custody immediately before exclusive ready-marker creation and after
flush/readback. Any existing ready namespace (including empty, partial, foreign
or already-completed state) refuses; it is never adopted or overwritten.

Finalization has a separate capability and a service handle limited to
QUERY_CONFIG, QUERY_STATUS and READ_CONTROL. It cannot configure, start or stop
the service, and failure/cleanup releases owners without changing its running
state. It neither downloads replacement broker bytes, renames code nor reruns
verify-store. The complete pair marker and actual running broker's authenticated
history response remain required; SCM RUNNING alone is insufficient. Public
installed readers continue to require the final ready marker.

These query-only **service rights** do not mean zero machine effects. Completion
publishes the immutable ready marker and uses the established cooperative
executor-query access for authenticated readiness. Its process/TOKEN_QUERY ACEs
remain for those objects' lifetime; normal folder-query cleanup restores the
temporary TOKEN_IMPERSONATE addition to the exact query-only descriptor. Abrupt
process death can leave that shared-token addition, an inherited limitation of
this readiness mechanism. Finalization does not add a new grant or cleanup path.

A separate stopped-start owner accepts exact AUTO_START plus clean STOPPED
with the same complete authenticated pair closure and absent ready marker. Its
service rights are START, QUERY_CONFIG, QUERY_STATUS and READ_CONTROL only. Before
the sole normal StartService call it exclusively creates, flushes and reads back
canonical protected `.code-successor.start-attempt.json`, binding original
completion, approved policy, successor receipt and complete-pair digests. Original
normal activation writes this same fence before its StartService call. Any existing
namespace, including an empty or partial write, forbids another START across
invocations. DISABLED recovery rejects any attempt namespace; RUNNING finalization
and public successor readers permit absence for legacy transactions or an exact
matching attempt. A start error, including already-running or timeout, consumes
the attempt and refuses adoption. Stopped-start failure never disables, stops,
replaces code or runs verify-store. Clean SCM STOPPED is not process-exit proof.
After successful start, the first actual RUNNING process is bound once and retained
through query-only readiness/publication; process loss never rebinds a later instance.

This implementation supports the early terminal abort boundaries above, the
four disabled rename boundaries, one fenced
normal start from an exact automatic clean-stopped successor, and terminal
finalization of an already-running automatic successor with an absent ready
marker. It deliberately **does not claim recovery of every phase**:

- Interrupted staging lacking exact complete-file receipts, including partial
  copies and unprovable legacy stages, refuses; early abort covers only the
  authenticated complete boundaries above while original code is still running.
- A partial immutable successor/marker write whose bytes do not exactly match
  the same transaction refuses adoption; it is not silently repaired.
- Process loss after DEMAND_START is selected for `verify-store` refuses the
  DISABLED-only recovery entry.
- AUTO_START with pending/unclean STOPPED or any previous start-attempt namespace
  refuses another start; a failed or uncertain StartService is never retried.

Those phases require additional authenticated abort/service-phase recovery before
normal customer Setup activation. Actual installed SCM/UAC/IPC and customer
journey qualification for terminal finalization remains separate from inert
NTFS/process-observation fixtures. An uncertain SCM configuration effect is never
blindly retried by cleanup. A failed live operation attempts containment only
after rechecking the same authenticated service/config custody; otherwise it
reports uncertainty and preserves all evidence. Do not delete namespaces,
re-enroll, repair ACLs, rewrite original completion, reset trust/floors, kill a
process or restart a service to manufacture missing evidence.
