# User-directed development boat-test candidate

On 2026-10-04 the user changed the goal to: “Deliver a stable SKAGER Windows
boat-test candidate as fast as possible. skip endurance testing.” This supersedes
the earlier endurance scheduling work in SCRUM-287 for the immediate SCRUM-217
candidate. The change starts from frozen source
`0a52a6cfe3bd3b4a1e6253d609bd9dc046016bd4`; it does not integrate that scheduling branch.

`release/qualification.json` explicitly sets `enduranceEnabled: false`. Both
Linux and native Windows workflow locations record `status: skipped` with the
user direction and exact CI revision/run/attempt. They do not invoke the soak
harness, shorten its duration or create a passing endurance result. The retained
10800-second duration applies only if endurance is explicitly enabled again.

Every other existing workflow job, dependency and functional/package assertion
remains in place, including native Windows production, fixture exclusion,
installer/update/recovery, DPI, restart, private-chart/source closure and
Linux checks. Candidate assembly still validates the native-tested installer
and recovery archive hashes. Its qualification note identifies a development
boat-test candidate with endurance skipped. All six payload hashes remain the
output of the existing artifact collector.

`publishNamedRelease` remains false. No public release, boat acceptance or
endurance qualification is claimed. Physical boat operations and the actual
boat-PC visual/read-only acceptance gates are unchanged. This policy change
does not diagnose or resolve the preceding native production build delay.

Validation: parse the workflow and policy, verify identical job dependencies and
all unrelated steps against the frozen parent, exercise both disabled-policy
blocks in temporary directories with subprocess invocation forbidden, and check
that the candidate note changes no product payload. No application build, CI
dispatch, public publication or boat operation is part of this change.
