# Alpha installer design

Alpha installs an exact-version-gated integration **beside** the stock OpenCPN
application, under the current user's LocalAppData. It does not overwrite the
stock executable or its system DLLs. This is a deliberate refinement of the
specification's replace-and-back-up flow: it provides XNav/Legacy/Safe against
the same normal OpenCPN profile, while leaving the verified original executable
available for recovery. No elevation is needed for OpenNav-owned files.

The stock installation is still mandatory. Preflight discovers its location,
reads the PE architecture and version, and compares its executable SHA-256 to
the compatibility manifest. Version strings and registry entries are hints,
never authorization. Unsupported installations receive a clear refusal before
any OpenNav install location, shortcut or profile is modified. The 5.12.4 stock executable is now qualified by the complete disposable native
lifecycle at `7bc36e426a55926045ea1aece0ebe96ef9417863`. Its exact hash is the
only public allowlist entry; every release still runs the full lifecycle.

The package uses a conventional Windows setup wizard and an independently
exercisable transaction engine. Implemented invariants:

* Verify the integration payload and every owned file before publication.
* Reject path traversal, reparse points, unexpected ownership and running app
  processes; never follow a redirected installation/profile path during writes.
* Stage a new application tree, run its real executable loader/resource self-test,
  and publish an immutable generation through an atomic state record and durable
  journal.
* Preserve stock application files, normal navigation/profile data, user plugins
  and connections. Copy compatible system plugin additions into the integration
  tree with provenance; do not overwrite unknown additions on repair/update.
* Back up the prior OpenNav-owned generation before update/repair. Rollback
  restores that generation, or removes the integration for the first install,
  without restoring old navigation data over newer user changes.
* Uninstall removes only verified OpenNav-owned files and shortcuts, preserving
  unknown additions for inspection. Stock OpenCPN remains intact and launchable.
* All lifecycle actions log bounded paths, versions, file hashes and outcomes;
  no raw sensor stream or user navigation coordinates are collected by default.

The normal shared profile continues to be located by OpenCPN itself. The
portable package retains its separate profile marker and never enters this
installer path. An installed XNav Legacy shortcut uses the integrated executable
with `--legacy` so the return-to-XNav menu remains available. The original
OpenCPN shortcut still launches the untouched stock executable.

Native disposable tests must cover exact official stock hash, unknown/tampered
stock refusal, existing navigation/connection/plugin fixtures, all three modes,
repair, update, rollback, uninstall, interrupted publication/recovery and stock
hash equality after every action. The final manifest must identify only the
stock binary actually tested, its Win32 ABI, pinned upstream revision and the
matching integration package. Native Setup and prior-version fixtures compile;
PowerShell 5.1 filesystem contracts pass in 32/64-bit hosts. The exact official
prerequisite now passes [native installation checks](evidence/installer-stock-8ae303c-gate.json).
The full Alpha lifecycle has passed, including untouched stock launch after
uninstall, preserved fixtures and custom harmonic sources. See
[qualification evidence](evidence/alpha-installer-7bc36e4-qualification.json).
The manifest-bearing release must additionally pass its own same-commit gates
and download verification.

Beta retains these identities for upgrade continuity and expands the native
matrix with real locks, NTFS permission denial, corrupt payload, partial
extraction and a missing required DLL. Atomic state failures clean their own
temporary record while preserving the last durable state/journal. Unpublished
stages remain identifiable diagnostic residue, never an active installation.
[Failure matrix and endurance contract](beta-robustness.md).
