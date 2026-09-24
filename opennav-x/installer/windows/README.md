# Alpha Windows installer qualification

The native NSIS setup and PowerShell 5.1 lifecycle engine install beside the
original supported OpenCPN. The public allowlist remains empty until disposable
Windows gates pass; the candidate fixture is not release support.

See [transaction contract](../../docs/installer-transaction-contract.md) and
[design decision](../../docs/installer-alpha-design.md). The maintenance wizard
supports offline repair from its retained package, rollback, diagnostics and
uninstall. Updates use the newer Setup. Modified/custom files and logs remain after
uninstall in this Alpha; original OpenCPN and navigation data are untouched.

Build with `tools/package-alpha-installer.py` on native Windows. Exact-hash
preflight, resource self-test and `tools/smoke-installer-windows.py` must pass
before release artifacts or compatibility entries are accepted.
