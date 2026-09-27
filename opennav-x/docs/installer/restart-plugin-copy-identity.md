# Restart review identifies each installed plugin copy

The first real installed inventory contains four stock bundled DLLs and four
separately built XNav copies, as well as the managed chart plugin. All nine
retained files have their own startup/idle review. Restart preparation previously
keyed shutdown review by basename and would reject this legitimate installation
before launch. This was found during source/preparation inspection; no failed
transition or hardware command was attempted.

Shutdown review schema 2 binds each entry to the full Windows DLL path, binary
SHA-256, plugin name, reviewed source revision/hash and shutdown behavior. The
closed chart-helper trust boundary remains explicit. Same-name copies in different
directories require separate entries; missing entries, duplicate paths (including
case variants), borrowed hashes and mismatched source revisions are refused.
Canonical path syntax is checked without loading a DLL. The existing complete
commissioning inventory still verifies actual paths, file bytes and helper trees.

Historical basename-only records are not automatically upgraded or reused. A
new independently reviewed schema 2 record is required before preparing the cold
restart session. The broker still requires the exact current audit, normal parent
exit, unchanged marine-output configuration, one-use permit and actual child
identity. No product code or normal mode-switch behavior changes.

Portable policy tests cover distinct stock/bundled copies and substitutions. The
native marker fixture now contains three inert same-name DLLs with different
bytes in stock, managed and generation directories. Actual Prepare/Arm/Broker
checks must succeed for separate reviews and refuse a borrowed-copy hash without
launching a marker. Native qualification and real boat transitions remain pending.
