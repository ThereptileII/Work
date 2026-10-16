# SCRUM-17 — actual warning review awaits the desktop

The exact 164 native-qualified files from [38897699400](https://github.com/ThereptileII/Work/actions/runs/38897699400)
were staged separately, with completion hash
`cfd6f39e6309c223fa97a095c89d9cbfb25527825a1c5171193ce930e480de33`.
The active restart dependencies and installed 854 executable remain unchanged.

The same process started at 11:47:20 UTC remains the only launch. Its guarded
Focus action refused because the warning caption was covered by a visible
Windows `WebExperienceHostApp.exe` CoreWindow. Source inspection establishes this
refusal precedes `SendInput`; the intent record is not evidence of a click.
The separate existing Inspect action then requested ordinary modal activation,
but observed no foreground window and an inaccessible input desktop. It refused
capture and acknowledgement. Access denial alone does not establish that Windows
is locked. No warning was accepted and no fresh chart/plugin acceptance follows.
The ordinary desktop must be made accessible before another guarded inspection;
there is no generic keystroke, foreign-window dismissal or desktop-policy bypass.

A separate owned read-only diagnostic from 10:28:36 UTC remained alive. Its exact
decoded command matched the retained script which loads two complete lifecycle
logs and embeds them in JSON. At termination it held **26,107,596,800 private
bytes**. Root bound an open process handle, start ticks, executable path and
command hash, stopped only this diagnostic and observed its exit at 12:13:42 UTC.
The application, profile and services were untouched. A subsequent observation
records the same responding application and running SSH/Tailscale/RustDesk, with
5,737,988 KiB free physical memory. Private bytes are not resident RAM; no stack
or allocation trace proves a specific allocation function or application leak.

The [selected receipt](receipt.json) pins original private evidence. The raw logs,
profile and user content are not committed. Failed result timestamps are populated
before the operation; do not report them as failure-completion times. Pending
commissioning remains input-only, with its recovery records intact. Do not launch
a second application or automatically restore output settings and relaunch.
