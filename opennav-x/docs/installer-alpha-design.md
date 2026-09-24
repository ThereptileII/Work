# Alpha installer design (implementation and native gates pending)

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
any OpenNav install location, shortcut or profile is modified. Inspection of the
5.12.4 stock release hash is not yet acceptance; the public allowlist stays empty
until disposable native lifecycle tests pass.

The package uses a conventional Windows setup wizard and an independently
exercisable transaction engine. Planned invariants:

* Verify the integration payload and every owned file before publication.
* Reject path traversal, reparse points, unexpected ownership and running app
  processes; never follow a redirected installation/profile path during writes.
* Stage a new application tree, run its real executable loader/resource self-test,
  and publish it with a durable journal and recoverable directory swaps.
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
matching integration package. This document records intended architecture, not
a claim that these gates or the installer currently exist.
