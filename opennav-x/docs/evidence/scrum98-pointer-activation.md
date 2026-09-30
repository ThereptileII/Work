# SCRUM-98 pointer activation evidence

The retained native failure in `evidence/local/scrum98-19cf-ci-first/job-109979206742.log`
showed a valid Preferences viewport descendant, but the main frame was still the
foreground immediately after `SetForegroundWindow(surface)`. The scroll branch
had no foreground wait even though the normal candidate branch already waited.

`tools/windows-ui.py` now waits for the existing eight-second deadline, then
re-reads the foreground HWND, viewport and selected-target rectangles, and calls
`WindowFromPoint` again. It sends the wheel event only when the foreground is
exactly the Preferences surface, the hit is the viewport or its descendant, the
selected target remains its viewport descendant, and both rectangle identities
are unchanged. Existing NULL-safe evidence and refusal assertions remain.

The focused fake-native guard covers delayed activation, denied activation,
an overlay after activation, and a moved viewport. It does not establish that
the native Windows gate is fixed. It corrects the harness assumption that
`SetForegroundWindow` is synchronously effective across input queues; see the
[Microsoft explanation of asynchronous activation](https://devblogs.microsoft.com/oldnewthing/20161118-00/?p=94745).
