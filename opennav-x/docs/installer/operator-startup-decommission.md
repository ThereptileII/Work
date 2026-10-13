# Explicit original-startup restoration during service decommission

SCRUM-27 adds an optional, independently approved restoration input to the
existing elevated `skager-update-provisioner --maintenance=decommission-service`
operator action. It joins the existing native original-shortcut restoration to
the **same retained, already-fenced service/process owner** used for terminal
decommission. It is an operator capability, not public Setup/uninstall UI,
automatic application cleanup, repair or a completed uninstall qualification.

Authenticate the final provisioner, its publisher/loader and original transfer
before elevation. The existing registered client, original request, actual
principal/token, protected registration, selected head, live broker witness and
native owner checks are unchanged. An administrator's path, request self-hash,
adjacent checksum or serialized witness supplies no authority.

## Separate approval of the original completed publication

The new optional pair is:

```text
--original-startup-publication <original independently reviewed publication file>
--original-startup-publication-sha256 <separately approved original SHA-256>
```

Supply the original canonical `shortcut-publication-complete.json` bytes retained
by successful startup publication. This is distinct from its original pre-move
`publication.json` manifest. The completed publication binds the original client
request/SID, baseline generation/commit/package, prepublication manifest, native
menu/recovery/launcher identities and all three published shortcut identities,
including the original normal shortcut when one existed. Do not reconstruct the
record from a current directory scan, create a replacement record or calculate
a fresh local hash and call it independent approval.

Both options are required together for explicit `decommission-service` or
`uninstall-protected`. The service-only action keeps them optional; protected
retirement requires the pair because omission cannot prove original absence. Missing/empty options, malformed/uppercase approval
pins, duplicate/unknown options and conflicting arguments refuse before reading
records or obtaining SCM ownership. The original request is read through the existing bounded reader,
owned and checked against its separately approved original pin. The publication
is also owned, limited to 32 KiB and passed with **both independent pins and the
original request bytes** to `ParseApprovedStartupPublication` before opening any
SCM owner or requesting the terminal broker fence. The existing canonical parser
rejects changed bytes, unknown/duplicate fields, incomplete native identities,
wrong original-client bindings and alternate encodings. The shared file reader's
existing maximum remains the bounded client-request limit; the narrower
publication limit is enforced before parsing or native ownership.

This invocation illustrates the explicit operator input set; placeholders are
not approvals or an execution instruction for an unqualified machine:

```powershell
skager-update-provisioner.exe `
  --maintenance=decommission-service `
  --broker-sha256 <independently-approved-installed-broker-sha256> `
  --client-registration-request C:\approved\original-client-request.json `
  --client-registration-request-sha256 <separately-approved-original-request-sha256> `
  --request-id <fresh-32-lowercase-hex-request-id> `
  --expected-selected-head-sha256 <observed-original-selected-head-sha256> `
  --original-startup-publication C:\approved\shortcut-publication-complete.json `
  --original-startup-publication-sha256 <separately-approved-original-publication-sha256>
```

The source CLI uses the actual native implementation and checks the original
publication against current native objects. The supplied file path only selects
historical comparison bytes; it cannot select alternate menu/recovery roots,
replacement shortcuts, another service, an application generation or a profile.

## Ownership and mutation order

After input validation the action:

1. Opens and checks the original decommission service/process owner.
2. Requests the terminal fence from that same authenticated original broker and
   binds its sealed live witness through `BindDecommission`.
3. When explicitly requested, calls `BindStartupRestoration` on that **same**
   already-fenced owner with the parsed original publication. Binding checks and
   retains native shortcut owners; it does not perform shortcut or SCM mutation.
4. Calls the existing `Decommission` once. A bound restoration must complete the
   owned shortcut retirement and original normal-shortcut restoration before
   service disable/STOP. If no original normal shortcut existed, its original
   absence is restored. The live witness and retained identities are checked
   around each non-replacing move. Only afterward does the same owner disable
   the service, observe its clean original process exit and mark it for removal.
5. Requires all retained owners to close successfully before emitting one
   structured result. No trailing generic completion message is added.

A failed or cancelled publication parse/binding stops this action. Once
restoration was requested, it cannot be omitted to fall back to service-only
decommission. No second enrollment, alternate owner, retry, cleanup or adoption
is attempted. A partial move, disable, shutdown, deletion mark or close failure
may already have durable effects; retain every original menu/recovery/service
object and evidence for operator review. Do not delete recovery fences or reset
trust to make another attempt succeed.

With the pair omitted, the existing service-only action, lack of publication
reads/binding and exact four-field JSON result remain unchanged. With the pair
explicitly supplied and completed, the result adds only:

```json
"startup": "original-shortcuts-restored"
```

The common service result is still `marked-for-removal`: Windows `DeleteService`
does not prove immediate disappearance of the SCM entry. The additional startup
observation reports only the bound owned-shortcut transaction; it does not claim
continuous application/profile custody, deletion of application files or a
working repair/uninstall path. The public installer UI, complete application and
registration cleanup, data-preserving repair, native end-to-end decommission and
full uninstall acceptance retain their separate required gates. Never route this
qualification to the boat or physical equipment.

## Focused verification

Portable checks:

```sh
go test ./cmd/skager-update-provisioner -count=1
go test ./brokerenroll -run 'Test.*(Startup|Binding|Decommission|Restoration)' -count=1
```

CLI tests reject incomplete/ambiguous pairs and every other action before reads,
exercise real parser refusal before SCM ownership, and use private inert
parser/owner/witness-close fakes to check exact ownership/mutation order,
cancellation, refusal, no fallback/retry and unchanged service-only output.
Production always uses the existing canonical parser and sealed native witness
close; zero fake objects in ordering tests grant no native authority.

The brokerenroll controls separately exercise the existing actual restoration
reducer and owned-move fault boundaries. Windows cross-compilation proves only
compilability. A dedicated exact-source native Windows run and actual retained
SCM/shortcut integration remain required; these portable checks do not perform
installation, service mutation, shortcut moves, full application builds or boat
operations.
