# Official stock first-start warning: native tooling test

`tools/test-stock-welcome-windows.ps1` is a disposable GitHub Actions test. It
does not run on the boat or qualify its real profile. The separate
`native-stock-welcome` tooling job executes the exact official OpenCPN 5.12.4
binary, not a recreation of its wxWidgets warning.

The test verifies the official setup SHA-256
`e949f55de57611afe2fc0dad5a8ac33795c46ba488cb40ca07b65f639a07b8aa`, then extracts
its payload into a new runner temporary directory without invoking an installer
or changing registry entries. The executable must match
`7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c`; its original
resources and DLLs accompany it. Only the four bundled plugins may be present,
and each remains disabled.

The child starts with the explicit `--portable --no_opengl` arguments, an empty
marine connection list, and private application-data directories. A generated
5.12.2 version setting deliberately causes the normal first-start warning. The
fixture refuses if the disposable runner already has a normal OpenCPN profile.
It never changes or substitutes the production stock launch/audit functions.

Before invoking the strict warning helper, the test saves actual native window
classes, control IDs, captions and ownership for the exact child. The unchanged
`Save-StockWelcomeCapture` and `Invoke-StockWelcomeAgreement` functions capture
the HTML warning, reject wrong PID/handle/geometry/pixel evidence, record one
durable intent, and send only the existing Agree action. The test then verifies
the warning was dismissed, waits for this launch's completed startup marker,
saves the enabled stock frame, and requests normal close. It never force-stops
an application. Failed cases retain metadata for the disposable runner teardown.

The artifact contains before/after screenshots, actual control metadata, the
generated portable INI before/after, exact helper hashes and individual results.
The HTML body must be reviewed from pixels; a native window caption is not proof
of its text. No native success is implied merely by adding this harness: the
exact subsequent commit, run and downloaded artifact must pass and be reviewed.

This qualifies only the fixed warning primitives against the official stock
application. The real shared profile, commissioning transaction, source-reviewed
plugins and physical display retain their independent boat validation gates.
