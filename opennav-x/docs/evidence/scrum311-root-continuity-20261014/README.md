# Retained TUF root continuity

SCRUM-311. Independent signed-fixture reproduction accepted root v2 revoking
v1's root key, removed the remote rotation link, and then observed direct
bootstrap discovery restoring v1. The installed generation-state control
preserved v2. The original failing reproduction is retained privately.

Fix `cfb9fc64` (reviewed original `805bb834`) reads the bounded retained root
before go-tuf construction. Invalid, missing, oversized or regressed roots in an
existing cache refuse without resetting it. Signed rotation observed before a
later refresh failure remains retained. An expired original anchor can advance
to a valid retained root; expired current authority still refuses normally.

Focused Linux evidence passes **20 top-level and 96 subcases**, no failures or
skips. Four new groups cover repeated purpose-specific checks, failed refresh,
expiry and ten damaged-cache cases. Windows x86 cross-compilation passes; native
execution and customer integration remain pending. [Receipt](receipt.json).

Rotated-root provenance comes from the caller's original authenticated protected
cache/history, not a self-signature or current ACL alone. Caller serialization
remains required. The ordinary installed state's locking and generation hashes
are unchanged. Native operator inputs already require an existing protected
cache: seed exact approved root bytes only at independently approved original
cache creation. Existing customer upgrades cannot initialize or reset it.
