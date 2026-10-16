# SCRUM-212: exact 854 unavailable-peer containment

**Recommendation:** the retained evidence supports the explicitly unavailable
software alternative for product
`854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`. This is containment acceptance,
not authenticated sharing or general security/release approval. The coordinator
owns the final Jira decision. No additional implementation defect or new broad
software test requirement was identified by this bounded review.

The [live acceptance criteria](https://swedishcountrysideliving.atlassian.net/browse/SCRUM-212)
permit an explicit safe disabled/unavailable boundary. Comment 10157 selects it;
comment 11414 records independent source review. The current evidence task is
comment 11857. The [source contract](../../architecture/peer-unavailable-boundary.md)
and [original lifecycle inspection](../../architecture/local-peer-trust-inspection.md)
document the choice and preserve the insecure upstream behavior as historical
findings, not an accepted protocol.

The constant-false policy applies to integrated XNav, Legacy and Safe modes.
The production startup caller gates certificate generation, REST startup and
peer mDNS advertisement. UI/forced handlers, all four model entrypoints and CLI
key commands refuse before credentials, serialization or peer transport. Existing
navigation data and saved peer identity are preserved; pristine OpenCPN remains
unchanged. There is no runtime override, automatic trust or insecure fallback.

[receipt.json](receipt.json) binds the following existing results without reruns:

| Scope | Exact retained result |
| --- | --- |
| Native model | Four real peer-entrypoint cases pass in both fixture and fixture-free CTest builds: empty/nonempty transfer refusal, no object serialization, retained-version clearing and direct-key refusal. |
| Linux model | The same four cases pass in each corresponding integrated test build. |
| Native CLI | Generate/store-key operations refuse with the unavailable reason, without key stdout or disposable-config mutation. Receipt SHA-256 `3582737fb18a396f2f171b96e31b4730a1c0126e329cb8b49394a4a3b18f05f8`. |
| Native concurrency | One ordinary and one portable instance, distinct profiles/live PIDs, preserved navigation/peer state and original certificate/key at clean exit; no owned TCP listeners observed. Report SHA-256 `74bd9723a898f4e8a34ca3f58c29c7b86b5cd717961764c41d5e2c1f946a7310`. |
| Linux concurrency | Two ordinary instances with the same containment/preservation assertions. Report SHA-256 `7fade64535edb6d530e2661b69191ab4895bf455fddbc40b38d32a9d15b3b922`. |
| Mode transitions | Five peer-preservation observations on each platform; no owned TCP listeners observed. The concurrent harness also checks application-log/stdout/stderr for fixture credential sentinels. |

The Windows concurrent executable is
`714cd164d50cecd29f3b07f017c26e96a4842168a5befe9030bbb9e40be5812e`.
The final fixture-free package executable is separately
`8ad205a452683316330549bb93a5d0c039b2a2acb4d59eaa6aa4c5d3b85bc511`.
Its [package/recovery qualification](../scrum290-composed-854-20261016/README.md)
and [actual Update/stock restoration](../scrum17-854-install-20261016/README.md)
are separate evidence. Neither changes which executable ran the concurrent test.
The original whole run remains failed; its passing peer prefix is retained as
such, with later exact-package composition separately recorded.

Pairing cancellation, malformed pairing-response JSON, changed-certificate
confirmation, replay/relay protection and credential migration were **not run or
passed as a protocol**. They are unreachable through the selected production peer
entrypoints. The native response-buffer pass covers allocation/bounds only. A
future sharing implementation must review and test those authentication scenarios
before enabling the feature.

The listener contract forbids peer TCP ports 8443/8444. Observed empty TCP lists
do not establish absence of all UDP/mDNS/outbound traffic or arbitrary plugin
networking. Sentinel checks cover the supported retained streams, not every
possible log sink. Windows ordinary-plus-portable evidence is not proof that two
ordinary installed instances can run.

Full installer lifecycle, fresh installed ordinary-profile/boat observation and
public-release qualification remain separate. This receipt performs or approves
none of them. No tests, builds, CI, boat actions, new protocol or product changes
were made for this documentation. Private paths, profile contents, credentials
and raw streams are omitted.
