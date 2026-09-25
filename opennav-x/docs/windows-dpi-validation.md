# Native scaling validation

The Beta gate uses a disposable Windows Actions desktop, not a user's machine.
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

## Beta 1 qualified revision

`a3e6e0812e01d2aee8f0b83807527b9a0c0fc79a` passes native MSVC and every scale in
[run 36137990012](https://github.com/ThereptileII/Work/actions/runs/36137990012). The application reported 96/120/144 DPI, minimum
48-DIP navigation targets, successful SDK touch tap/pan, stable final menu
endpoint and mode/chart returns. The gate retained 78 DPI captures; 40 images
across the complete Windows suite were individually reviewed, including a DPI
subset. Their hashes and findings are in [Beta 1 acceptance](evidence/beta1-a3e6e08-accepted.json). Fullscreen used the
1920×1080 host desktop and returned to the 1280×800 window.

Long pages/data rails intentionally scroll; at 150% even speed can require
scrolling when the alert strip is present. At 150% the System popup partly
covers the Alerts button, while critical text remains visible. Dismiss it with
Escape/outside click before opening Alerts. This minor presentation issue and
the old Alpha warning caption in Diagnostics are deliberate documented deferrals.
No physical touchscreen, wet/gloved use or target GPU acceptance is claimed.
