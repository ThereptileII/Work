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
`targets` locations. This component does not yet wire customer Setup/Maintain or
admit/execute a downloaded successor.

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
