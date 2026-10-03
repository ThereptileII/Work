# SCRUM-278: restore the producer parent environment before the AIS gate

Source base: `593464106bd110248813777a7be87d3727c3187e`. This tooling change
does not change the AIS provider, dependency producers or their trust checks.

The preserved run `37147671879`, frozen remote `63c1029584a325a961fa89794071cbf053c2e966`,
failed before the AIS wrapper configured or executed. The exact assertion was
`Native tool facts changed: openssl-parent`. The independently retained artifact
`11285152311` is 53,015,878 bytes, SHA-256
`a6031fb49dd1c83016431fc904264f5f49116b3d45de4e664033510f33091136`.
The only captured/observed fact difference was `environment.PATHSha256`; the
producer, helper, tool, PowerShell and Visual Studio identities matched.
Original evidence remains in the earlier failure record; nothing is recaptured
or relabelled here.

The original build prepended native Perl, then Poedit Gettext, before invoking
producers. The standalone AIS workflow omitted those steps. The two small
functions in `tools/windows-parent-environment.ps1` now supply the same boundary
to both callers, preserving the build's intervening upstream/architecture checks
and exact prefix order: `Gettext;NativePerl;inherited PATH`. There is no PATH
normalization, deduplication or equality relaxation. Native Perl still must be
the preselected application, and the selected curl-test Perl must exist.

The original build retains its explicit Gettext acquisition permission. The AIS
caller uses only `windows_gettext.py verify` against the existing
`windows-gettext-xnav.json`; it neither installs tools nor refreshes that receipt.
Verification writes its normal probe logs. The new helper is bound by the
same-job receipt's source inputs and the AIS wrapper's retained source inventory.
The existing receipt verification, producer parent/child reprobes and exact
environment comparisons remain active.

Focused local checks:

- Actual PowerShell helper execution: 13 checks, with only external Perl
  discovery and Gettext subprocess behavior replaced by fixture boundaries.
  This checks exact prefix bytes/order, verification versus acquisition arguments,
  unchanged receipt bytes, failed-probe handling and all Perl refusals. Both
  changed PowerShell production scripts also parse successfully.
- Existing same-job receipt suite, extended with helper tamper refusal and caller
  ordering/verify-only assertions: 12 cases passed.

Commands: `pwsh -NoProfile -File tests/windows-parent-environment-tests.ps1 -Python <absolute-python>`
and `python tools/test-windows-dependency-reuse.py`.
These are not native Windows, producer, TLS/lifecycle, app, boat or release
acceptance. The separately prepared short native gate must prove actual captured
Windows facts reject omitted prefixes and pass after this shared initialization.
