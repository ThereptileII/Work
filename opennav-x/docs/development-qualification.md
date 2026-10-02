# Background qualification and delivery

User direction, 2026-10-02. Jira remains the authoritative backlog.

Long-running builds and endurance tests run asynchronously. Continue independent
eligible implementation, review and evidence work while they run, using isolated
worktrees and bounded Astra High subagents during the Windows stabilization and
boat-test delivery stage. One agent owns integration.

Record the candidate commit, live CI/job handle, completed checks and remaining
gates. Freeze that candidate. Do not modify its build directories or test profile,
interfere with its resources, or restart a healthy job merely because an
observation timed out.

When a Windows prerequisite fails, reproduce the exact failure with a focused
native diagnostic and verify the repair before another full application build.
Qualify one integrated Windows path before starting additional expensive UI
builds. Retain failures and distinguish build/test failures from runtime crashes.

Use focused tests for small reversible edits and batch broad suites at meaningful
integration milestones. Dependency reuse must satisfy the existing exact-input
and provenance rules. Do not reduce safety, security, navigation, preservation,
installer/recovery, native Windows or source-compliance checks.

## Development boat review

An explicitly labelled pending-endurance package may be used once its mandatory
functional, security, installer/recovery, display, chart and packaging gates pass.
Verify the artifact's source identity and hashes, then complete the established
boat backup, input-only audit and chart/mode/recovery smoke checks before handoff.
Record remaining qualification. Do not send physical actuator commands.

Endurance can continue independently of this supervised development review. If
a remaining gate fails, assess the deployed candidate and suspend testing or use
the recovery procedure when warranted. Retire older installations only after the
replacement and its Legacy/Safe recovery paths are known-good.

## Public release

All required full-duration tests and actual boat acceptance still apply to the
exact release candidate. A short development smoke check does not replace them,
and a new commit cannot inherit the earlier commit's release acceptance.
Public access remains closed until the full readiness review and human GO.

Website implementation remains on hold until the user's software/boat milestone
is satisfied. Background testing does not change project priorities.
