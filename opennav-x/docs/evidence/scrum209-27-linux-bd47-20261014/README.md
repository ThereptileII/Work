# Exact bd47 Linux integration evidence

Candidate `bd47aa729aa728c6834072047eec7e00a65c6f13`,
[run 38563357629, attempt 1](https://github.com/ThereptileII/Work/actions/runs/38563357629),
has a successful original Linux integrated job `115849938492`.
Both original CTest reports pass **154 tests**, with zero failures/errors/skips.
Their test-name sets are identical: fixture-enabled and fixture-free executions
of the same suite, not 308 unique tests.

Original artifact `11895553164` is 14,621,662 bytes. Its SHA-256
`60582bf60db74054506bf2408640c356c8cd938fc4d8b8fc349bebf0ec53728a`
matches authenticated GitHub artifact metadata and the original producer upload
log. All 3,984 ZIP members pass CRC verification. The adjacent records retain
original run/job/artifact identity, individual CTest report hashes/counts, the
original fixture-free loader self-test, and sanitized original log excerpts.
That self-test reports this exact candidate commit and pinned OpenCPN `37fd0cdd`;
its reported executable hash is preserved without claiming a separate binary
download verification.

The overall run is **failed and unqualified**. Windows job `115857263896` passed
the early SDK check, then failed in prerequisite setup before application
compilation. No Windows application, installer or installed journey acceptance
follows from the Linux result. The original run is not rerun or relabelled, and
this evidence does not qualify later helper/product commits. Endurance remains
explicitly skipped; design review was not requested.
