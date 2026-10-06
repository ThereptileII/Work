# Exact SDK reuse correction

Product run [37418273537](https://github.com/ThereptileII/Work/actions/runs/37418273537)
at `e3a85a2bc6102bf0cae76fd28108ed61a6ac13ea` stopped before Windows application
compilation. The immutable SDK verifier rejected changed fingerprint inputs.
The existing native startup, serial, recovery and Windows contract jobs passed;
that does not qualify the integrated application.

Comparison with the exact producer tree
`1b25542aea3f9ab62c83c5d7a3ecdac9652f7d3e` found three changed input files:
the two previously reviewed consumer helpers and `windows_dependency_reuse.py`.
The latter differs only by adding the application's pilot serial patch to the
same-job receipt inventory. The dependency-only producer does not apply that
patch or compile OpenCPN. All other declared SDK inputs and the producer workflow
match the producer's Git blobs. No dependency source, recipe, ABI or options change
was identified.

The correction retains that complete same-job inventory and extends the explicit
consumer policy to the exact three original/current file hashes, separately for
LF and CRLF checkouts. It does not normalize input files or alter the immutable
producer artifact. Authentication, runner/toolchain identity, complete inventory,
source tests and fresh native consumer probes remain mandatory.

Focused checks: 24 immutable-bundle tests and 13 same-job receipt tests pass.
Negative cases reject arbitrary changes to every allowed helper, altered original
hashes, different producers and a changed dependency recipe. The reconstructed
Windows producer fingerprint exactly matches the selected artifact:
`cf9d9024a09681e06bc77a5e8a7b93ba4bd171191a68a5fceee3ff6f4bf15cf4`.
Native restore/reprobe passed at `38b199614735b2beec48a16d6223255f2e2264ee`
in [run 37419757974](https://github.com/ThereptileII/Work/actions/runs/37419757974).
The recorded restore and fresh native probes took 79.8818824 seconds without
compiling replacement dependency libraries. Downloaded artifact `11393170150`
passed ZIP integrity and the authenticated SHA-256:
`ab237203fd3ca139af77b3e19ee4fa87c295100141b2c5c25d56fd297a617ebb`.
This qualifies dependency reuse, not the application. Integrated qualification
remains pending after the separately discovered pilot timeout regression.
