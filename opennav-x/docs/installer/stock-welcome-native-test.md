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
5.12.2 version setting with `NavMessageShown=1` deliberately causes the normal
version-upgrade warning in separate English (`en_US`) and Swedish (`sv`) jobs.
This matches the recovered boat profile. The version mismatch still requires
the warning; it does not skip or accept it. Pinned `ocpn_app.cpp:1715` adds a
portable chart directory only for first-install state, which the upgrade
fixture correctly avoids. The
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

The shared selector admits exactly two complete title/Agree/Cancel tuples:
`Welcome to OpenCPN` / `Agree` / `Cancel`, or `Välkommen till OpenCPN` /
`Acceptera` / `Avbryt`. Mixed tuples and all other languages refuse. The Swedish
fixture verifies the official `share/locale/sv/LC_MESSAGES/opencpn.mo` SHA-256
`c57052ffd88fde07d379f2f27256eb77d77511fe34ff4d519cea898bc639dec4` and `wxstd.mo`
`f3f490b0ac48373fb4ef3a54babd8957263024f9ddcd3df5756affa50851e09e` before launch.
Source translations agree with pinned `po/opencpn_sv_SE.po`. Each captured text,
class and control ID is rechecked along with handles, geometry and foreground.
A changed language requires a new inspection and pixel review. No boat locale
or configuration is translated by these tools.

Run `36285947258` (source `0dd5fe8716763f32d65faced0b4a0a3a3e337eb9`) proved the
actual English warning's source-derived selectors and one captured Agree action:
18 checks passed before an unrelated first-install chart database dialog blocked
startup. The full job failed and timed out; it is not a completed stock gate.
The downloaded artifact SHA-256 is
`3bbbb0a0a936b77f78dfe146d5f118fe5e91e2ac58f428dbf0ae4bc74d73256a` (51,598 bytes),
verified against the API and upload log. Both corrected upgrade cases must still
pass on their exact subsequent commit. Child output is explicitly redirected so
a failed fixture does not keep the CI console pipe open; normal close is attempted
only for its verified, enabled owned frame, never through an unknown modal.

Both corrected upgrade jobs subsequently passed on tooling source
`2b2c2c63a2e4ee1657161611355c8ac5815cb7a7`, run `36286949649`: English 38 checks,
Swedish 40 checks. The downloaded English artifact SHA-256 is
`4a00554e0ef7533eef0a4292200774d5e28273397e1500be0def14fc569777d5` and Swedish is
`11e17419e8c15e729bc63f5aeb0eeea12af0adcd329ec5e8980d6819573ca114`; both match
the API and upload logs, pass ZIP integrity, and their actual warning/after-frame
pixels were reviewed. Each app closed normally with exit zero. This is primitive
qualification, not evidence that a real boat warning has been accepted.

If capture refuses, diagnostics distinguish captured-field changes from a
foreground mismatch. They report changed field names or numeric HWND/PID values
only, never an unrelated foreground window's title. The identity predicate and
human inspection / one-use acknowledgement sequence remain unchanged.

Saved inspection acknowledgement regression: the boat's separate acknowledgement
process refused before capture, intent creation or input because PowerShell 5.1
tried to set the serialized native rectangle's read-only `Width` and `Height`.
The shared stock/installed acknowledgement primitive now explicitly reconstructs
only writable native fields and requires both serialized derived dimensions to
match the captured edges exactly. Unknown fields, malformed numeric types and
inconsistent geometry refuse before capture.

The actual English/Swedish fixture now writes the real C# observation to JSON and
acknowledges it from a fresh Windows PowerShell 5.1 process. That process verifies
the hash-bound packet, exact temporary portable executable/process/profile and
captured pixels, then calls the same production primitive. Earlier native results
qualified the live-object path; they did not qualify this process/JSON boundary.
The converter's 141 portable warning checks and 91 installed policy/runtime checks
pass locally; fresh native evidence is required before boat acknowledgement.
