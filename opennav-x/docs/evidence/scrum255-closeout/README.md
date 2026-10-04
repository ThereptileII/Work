# SCRUM-255 bounded closeout audit

Audit base: `5d19314fe51d7192ea3cfc2fa55d1674c559f406`. Jira selection
`10916` and test-maintenance authorization `10918` were read with all six earlier
material comments. The frozen candidate remains local
`0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4` → remote `17ab044`, run
`37164360050`. This audit neither polls nor changes that candidate.

| Acceptance | Retained evidence and finding |
| --- | --- |
| Preserve actual failure | `../scrum255-gettext-failure/`: remote `9fcd3db54ee6913cc144ecfc09ec2152078a6eff`, run `37080314681`, job `111080247315`, artifact `11260087803`; HTTP504 at 00:12:49 followed by missing-tool refusal at 01:09:09. ZIP 20,009,975 bytes, SHA256 `e081a53c6638e20a5b852659d88fc5c8857ec9f564faaf30b507fb77e9fe6485`. |
| Early usable tools and bounded acquisition | Current `tools/build-pristine-windows.ps1` invokes `Initialize-WindowsGettext -Mode Ensure` before curl preflight, UI compilation, stock dependencies and maintained dependencies. `tools/windows-parent-environment.ps1` binds this to `windows_gettext.py ensure --allow-install`, fails on a nonzero exit, and selects the receipt directory. |
| Provenance and refusal guards | Unchanged `tools/windows_gettext.py` retains known ProgramFiles roots, paired msgfmt/msgmerge probes, regular-file/reparse refusal, exact identities, pinned existing Chocolatey provider/Poedit 3.9.1, three attempts, finite timeouts and no timeout retry. Provider/version pinning does not imply independently authenticated installer provenance. Direct verification before CMake and both explicit tool paths remain. |
| Focused native proof before full rerun | `../scrum255-gettext-native-pass/`: local `36ab8e7b2e59e94dd35fc0486c5bc400315779e9` → remote `4330cae78c53707310e94ded4d1b933d0fb519a8`; run `37087435049`, job `111100530739`, artifact `11261311795`; 10,885 bytes, SHA256 `850564d42deeeab8861f933be77444d234328e6585cc40c7ff12909f8cdec131`. All 25 native contracts passed, actual tools acquired and UTF-8 catalog merged/compiled. This precedes integrated run `37088759582`, as recorded in `docs/status.md`. |
| Evidence integrity | This audit rehashed all 30 retained native entries against `verification.json`, with zero mismatches, and independently read the native MO translation as `SKAGER översättning`. Helper, native harness and contract file match proven local `36ab8e7` and frozen `0a52a6c` byte-for-byte before maintenance. Native executable hashes remain recorded observations, not independent binary rehashes. |
| Later caller refactor | `970671d1f66a5cb2bca2b46c242f7d93fd8b9a35` extracts shared parent setup without moving the gate. `../scrum278-native-parent-context/` retains run `37155593528`, job `111298215075`, artifact `11285546611`: actual bounded acquisition and verify-only receipt reuse passed. |
| Documentation and scope | Windows boundary/dependency documentation and `docs/status.md` record the repair and retained failures. AIS, application, installer, native visual/DPI, boat and release acceptance remain independent mandatory gates. |

The initial closeout found one stale ordering contract. Its source search still
expected the inline ensure call removed by the shared-parent refactor. The single
authorized focused execution is retained in `original-failure.log` and
`original-failure.json`: one test, `ValueError`, exit 1. This is an observed local
test-maintenance failure, not a frozen-candidate product or CI failure.

At this audit commit, keep Testing until that existing assertion is adapted to
the shared helper and the focused positive/negative proof passes. No production
source, full suite, application/dependency build, workflow, or boat action was
performed. Root controls the final issue transition.

## Authorized test maintenance and final recommendation

The follow-up changes only the existing
`test_script_orders_gate_before_every_expensive_step`. It follows the actual
shared initialization call, requires Ensure mode and the same receipt, checks
the helper's Python/ensure/acquisition/failure binding, and retains all four
original expensive-step ordering targets and both explicit CMake tool arguments.
It also protects ordering before the actual `Build-PrivateOCharts` invocation.

The first focused correction exposed another obsolete lexical assumption in
that method; `inline-cmake-search-failure.*` retains its one-test assertion
failure. The first textual `Run cmake` is now inside the private adapter function
definition. The final assertion targets the actual application configure call
using `$Source`/`$Build`, which consumes the verified gettext paths. Direct
receipt verification must precede this consumer. No production order changed.

`corrected-pass.*` records the final source hash and the sole selected test
passing. `check-ordering-negatives.py` runs that same assertion against two
in-memory caller mutations, without changing or executing production scripts:
moving Ensure after curl preflight and moving Verify after application configure.
Both are rejected by assertion, with no test errors; exact output and mutation
hashes are retained in the two logs and `negative-results.json`.
The reviewed preflight mutation inserts Ensure after the complete two-line
preflight invocation, preserving the hypothetical statement structure. Only the
two negatives were repeated for this collector correction; the passing contract
source and its positive evidence are unchanged.

After this focused maintenance, **SCRUM-255's own acceptance is complete and
eligible for Done**, subject to root review. The native prerequisite result
belongs to proven local `36ab8e7` / remote `4330cae`; the subsequent passing
source-order maintenance is local evidence based on `5d19314`, not a new native
run. Frozen `0a52a6c` / remote `17ab044` is unchanged and remains subject to its
full qualification gates. No full-candidate restart is required solely for this
test maintenance. Earlier failures remain retained; no assertion or gate was
removed. No full suite, CI/build, production source, system or boat action was
performed. Jira transition remains root-owned.
