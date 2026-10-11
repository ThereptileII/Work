# SCRUM-211 native plugin orchestration: portable validation only

Product source base: `12acdd0f4a00de3ffe6c05686fd6fca50dd4fc70`.
Date: 2026-10-11. No CI workflow changes, product build, push, deployment or boat access.

- 19 portable orchestration contract tests passed (retained inventory, external pin,
  candidate binding, TLS producer match, paths/aliases/links, native build source,
  imports/header boundaries, host restrictions, OpenSSL config, case receipt names).
- PowerShell 7 parser accepted `plugin-guard-owned-trust.ps1`.
- Python byte compilation and `git diff --check` passed.
- Real Linux PluginHandler/Downloader fixture: 13/13 cases passed with libcurl
  8.22.0 and libarchive 3.8.9. Failed-download guard-bypass mutant rejected because
  it entered archive installation. Full case/hash receipts: `linux-results.json`.
- Initial restricted attempt could not open a loopback socket (EPERM), before
  any TLS case. The bounded loopback rerun passed after approved sandbox elevation.

The case-name receipt test catches the previous proxy-loop variable overwrite;
no transport/extraction failure assertions were weakened. Windows config paths
now use quoted forward slashes; owned certificate generation is time-bounded.

Native MSVC execution, system-store import/removal (including injected failure),
Windows DLL loading and exact candidate SDK provenance remain untested here.
This evidence does not establish native/product/release/boat acceptance or close
SCRUM-211. See the architecture document for prerequisites and invocation.
