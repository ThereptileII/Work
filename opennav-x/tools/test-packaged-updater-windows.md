# Same-version package mechanics and selected public trust

`test-packaged-updater-windows.py` is called by the existing disposable native
installer smoke. It runs the original bootstrap health, exact Setup supervised
same-version update, corrupt-candidate refusal/rollback and restored launcher
checks on the exact package. Its scope remains local installation/recovery
mechanics; it does not qualify a protected broker installation, signed release
selection, real B0/B1 upgrade or the four-phase updater sequence.

The producer may explicitly package the committed private-Staging public trust.
Smoke passes the original PRODUCT_BUILD record and original recovery app directory
to the helper. If trust is declared, the original and every installed generation
must retain its exact configuration/source digest, product commit, channel and
fixed source path. Installed ownership must include that same trust digest. If
trust is undeclared, an unexpected file refuses. Missing, changed, linked or
oversized trust refuses. No file is stripped, rewritten or silently ignored.

All original mechanics still run for both package forms. The unchanged Setup
intentionally invokes `skager-start.exe --initialize-trust` after every committed
health-capable generation (`installer/windows/Lifecycle.ps1`, post-commit path).
With selected trust, `launcher/provision.go` creates the canonical once marker,
private state/artifact directories and the original bootstrap identity. Later
Setup runs validate that retained identity instead of reinitializing it. The
helper verifies these expected initialized objects; it never suppresses, removes
or resets them. Only unconfigured packages require those objects to be absent.

The marker's exact bytes and physical identity, and the original state/artifact
directory identities, must remain unchanged. State identity is bound to the
original configuration's exact raw `bootstrapRoot` span SHA-256, metadata URL and
channel. Before the first launcher invocation, the retained metadata must contain
only `root.json`, with those exact bootstrap bytes, as `InitializeState` produces.
Current metadata inventory/hashes must agree with state, and an
interrupted `refresh.pending` refuses. Cache generation and metadata may advance
normally; already observed signed metadata version numbers must not decrease or
disappear. These comparisons supplement the native verifier's checks and do not
replace signature, ACL, expiry or rollback-floor validation.

The original normal `--xnav` launcher remains enabled. After its healthy receipt
and configured trust, `launcher/launcher.go` may perform a bounded online check
and present an actual newer offer if one is available. This fixture neither
disables that product behavior nor infers zero network activity from runner
reachability. An unexpected offer that prevents the expected ordinary startup
refuses this mechanics test. Real signed-offer and protected B0/B1 acceptance
still use their separate controlled publication and protected-install drivers.

The result records `trustConfiguration` as `unconfigured` or `packaged-public-trust`
with the original configuration digest. Successful local mechanics records
`legacyTrustState: initialized-original-identity` for configured packages, or
`unconfigured` for packages without trust. It always retains
`nativeProtectedUpdaterAcceptance: false` and
`protectedUpdaterGate: pending-real-B0-B1`, even when its own scoped `status` is
`passed`. A green installer mechanics result does not close that required gate.

Portable inert checks:

```sh
python tools/test-packaged-updater-trust.py -v
python tools/test-installer-completion.py
```

Native same-run installer execution against these exact bytes remains required.
Portable tests neither execute Setup nor claim the mechanics passed on Windows.
# Retained startup failure diagnosis

The existing retained Staging workflow has an explicit
`installer-startup-diagnostic` scope for failed candidate
`854078a5de2ef8ed61a6529b7f2bb200ed6cf1bc`, producer run `38765418126`, attempt `1`.
It reuses authenticated retained inputs and the unchanged disposable fresh-install
fixture. At bootstrap it calls the original installed `UpdateSupervisor.ps1`
`QualifyCurrent` once, using the WOW64 I386 Windows PowerShell host selected for
the x86 launcher's contract, exact application working directory and unchanged
security checks. It does not repeat the original launcher or later lifecycle
phases. The original failed qualifier remains authoritative.

`packaged-updater.json` retains the original failure reference, installed input
hashes, host identity, exit/timeout observation and bounded, redacted stdout and
stderr. No startup challenge or environment dump is exported. A surviving app
can keep a pipe open; complete lines already received remain available and the
report marks incomplete output. The supervisor's own cleanup can mask an earlier
receiver failure; this observation does not reconstruct the masked cause.
Even zero exit or a newly created receipt remains diagnostic-only: this scope
intentionally fails qualification and uses the existing disposable cleanup.
