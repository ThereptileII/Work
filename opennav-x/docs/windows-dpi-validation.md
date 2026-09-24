# Native scaling validation

The Alpha gate uses a disposable Windows Actions desktop, not a user's machine.
OpenCPN's pinned manifest already declares PerMonitorV2 awareness. The test-only
`opennav-test-dpi.exe` is never installed or packaged. It queries the monitor,
requests 100/125/150% scaling, and restores the original scale in `finally`.

The source-DPI packets are undocumented Windows interfaces, isolated in this
helper and based on the original investigation linked in its source. An
unsupported driver/session or failed request fails this gate explicitly. The
harness accepts a scale only if the real application `GetDpiForWindow` and wx
DPI both report 96/120/144. It does not substitute image resizing, font overrides
or environment-variable guesses for a native DPI test.

The application window remains 1280×800 physical pixels. Captures cover Day,
Night, route, energy, instruments, settings, the actual unresized modal sheet,
Legacy, returned XNav, Safe and Safe→XNav at each scale. Button bounds and touch
height, content-pane visibility, shared-profile persistence and deterministic
land/water rendering are asserted. SDK touch injection presses Menu and the
application's resulting page must change. Physical touchscreen operation and
boat-PC display/driver validation remain separate unaccepted gates.

Evidence is `dpi-results.json`, original native PNGs and their capture metadata,
plus the isolated profile/logs. Human visual review remains required even when
all automation passes.
