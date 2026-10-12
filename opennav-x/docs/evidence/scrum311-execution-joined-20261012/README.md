# Joined protected updater execution source — SCRUM-311

This is bounded source/component evidence, **not** an accepted application,
installer, update transaction, native service or boat release.

Integrated component commits: `8522a631` (linked pre-spawn recovery), `336c404c`
(original executable capability), `e4abbdaf` (fixed query access), `8d745fba`
(interactive executor), and `d2c0a287` (explicit native profile fixture harness).
The source manifest records the actual checked source bytes, including the
package/plugin qualification changes made after that base commit.

- Joined Linux race selector: `Test(ExecutionSelection|ExecutionCapability|GenerationApplicationQualification|BrokerExecution|BrokerQueryAccess)` in brokerhost, brokerstore and launcher.
- Result: **28 top-level + 126 nested passed**, zero failures/skips.
- Joined Windows x86 launcher test component compiled; binary SHA-256 is in the receipt. It was **not executed** by that check.
- Package provenance boundary: **7 tests passed**, including actual copied-image digest, missing/invalid/Boolean capability refusal and no sidecar publication on refusal.
- Fixed broker source-package boundary: **12 tests passed**.
- Focused startup harness: **6 tests + 9 subtests passed**. Native MSVC execution is a separate gate.
- All **13** upstream patches apply in order to the exact pinned revision in a temporary index. The earlier failed include-hunk check remains retained separately.

Native process/token grants, actual SCM execution, full-profile backup and
restoration, initial baseline qualification, current startup/shortcut integration,
healthy durable selection, fallback and the complete installed flow remain
unaccepted. Unsupported executor phases refuse; report labels and sidecars do
not provide those effects. Qualification defers plugins and suppresses outputs;
it must not replace ordinary Later/offline startup.

No full application build, release, installed product/profile/service change,
boat action, design/endurance run or physical command occurred in this increment.
