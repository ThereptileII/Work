# SCRUM-216 System landing flow

Selected by Jira comment 10485, based on `1579a382`. The active immutable
prototype routes Settings System through `index.html:465,469`, a navigation
list. Its earlier renderer at line 357 contains direct Legacy/Safe buttons but
is superseded. This increment removes those duplicate landing buttons; the
existing Interface & recovery destination retains Open Legacy OpenCPN, Restart
XNav and Safe Mode with their original callbacks and restart behavior.

Advanced / Legacy Settings now invokes the existing `actions_.advanced`
callback directly instead of opening the XNav NavigationSettings page. When
that callback is absent, the row is disabled and says “OpenCPN settings
unavailable.” No installer, update, backup or setup-wizard controls were added.

The directly affected Settings component scenario now checks the absent
callback and activates Advanced settings, verifying one upstream invocation
without an intervening XNav page request. Its 14 capture identities and
single-shot timer behavior remain unchanged. Validation comprises a focused
Linux compile/component run (181 checks), four passing capture-wrapper checks
and visual inspection of the System capture; retained local results are in
`.local/system-flow/capture/`. The initial wrapper invocation lacked Pillow;
the same checks passed with the bundled Python runtime without changing tests.
No full application build, native Windows run, publication or boat action was
performed. Remaining System composition work and Windows/boat acceptance are
still open.
