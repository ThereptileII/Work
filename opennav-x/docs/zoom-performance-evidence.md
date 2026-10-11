# SCRUM-317: bounded Windows zoom evidence

This procedure observes an **already running disposable test process**. It does
not authorize a boat session, deployment, chart/profile edits, a build, or a fix.
Keep SCRUM-317 in Testing until the native/physical acceptance evidence exists.
The source at `bf68b355` already has the 1:150,000 and 1:600,000 XNav declutter
thresholds. The three Jira comments through 2026-10-11 do not establish a
remaining hang cause. `ZoomCanvas` / `DoZoomCanvas` / `Full_INIT` are investigation
locations, not a diagnosis.

## Short reproducible capture

1. Use an explicitly authorized desktop test session with the exact installed
   candidate and an already prepared disposable profile. Verify hardware output
   is unavailable and no operational navigator shares this process/profile.
   Identify the PID, full executable path and its existing diagnostic JSON.
   The helper cannot prove the PID/profile association: the operator must do so.
   Do not point it at the live boat profile or change the profile to enable data.
2. Record candidate package/installer SHA-256, embedded commit, Windows version,
   CPU/GPU/driver, display dimensions/DPI, theme, actual software/OpenGL renderer,
   enabled plugins and versions, chart product/type/edition and view position in
   a private run note. Record the displayed **1:N scale** at each step. The
   helper retains chart type IDs/native scales and viewport pixels/metre, but
   current diagnostics lack the viewport denominator. Native chart scale is not
   the current viewport scale. Do not calculate a denominator from PPM without
   the application's display calibration.
3. From a separate terminal start the passive collector, replacing the values
   below with the already verified test process. The output directory must be
   new and outside the disposable profile. The helper opens a query/read handle,
   reads existing diagnostics and writes only evidence. It does not launch,
   stop, suspend, debug, inject input, or send window messages to the process.

   ```powershell
   py tools/collect-zoom-evidence.py --pid 1234 `
     --exe C:\Test\SKAGER\opencpn.exe --package-commit EXACT_EMBEDDED_COMMIT `
     --disposable-profile C:\Test\ZoomProfile `
     --diagnostics C:\Test\ZoomProfile\opennav-diagnostics.json `
     --output C:\Test\Evidence\zoom-before-01 --seconds 60 --confirm-disposable
   ```

4. Use the same ordinary UI gesture for each run: 10 seconds idle; one zoom-out
   step every 5 seconds across 1:150k and 1:600k; one short pan; return to the
   starting view with corresponding zoom-in steps; then idle. Fit the sequence
   within 60 seconds; do not retry a freezing gesture repeatedly. Record actual
   UTC/time offsets, input type (wheel/button/touch), number of steps, scale,
   rendered chart type and any stall/recovery at each step. A test that cannot
   cross those thresholds should say so. Keep chart set, view, input data,
   overlay settings, plugins and renderer identical for before/after comparison.
5. If the application stalls, stop sending input. Ctrl+C ends only the
   collector; it never kills the application. Keep partial `samples.jsonl`.
   Preserve native stack/frame evidence described below. Do not declare a cause
   from high CPU or a stalled diagnostic file alone. Repeat the short comparison
   on a repaired exact candidate only after a source-grounded repair exists.

## What the collector proves and leaves unknown

`metadata.json` pins PID/process creation time, actual executable SHA-256,
package commit and collector SHA-256. The existing JSON must report the exact
requested commit. JSON has no PID, so matching commits alone do not establish
process ownership. Each `samples.jsonl` row records process CPU on a one-core
basis (100% is one fully used logical CPU; values may exceed 100%), resident
bytes, publication/file age, UI callback ticks and durations, and the available
chart/overlay inputs. It excludes positions, routes, target identities, chart
filenames, credentials and the rest of the diagnostic payload.

The first publication observation is a baseline, not proof of freshness.
Subsequent unchanged duration and file age expose stale evidence. A missing file
or an atomic-replacement access race produces an error row, never fabricated
zero counts. A changed embedded commit also produces an error row. A process
handle is retained for the whole capture, so PID reuse cannot silently retarget
CPU samples. `completion.json` records why observation ended; no result marks
performance acceptance passed.

Current GUI-thread diagnostics publish at most once per second. They can show
lack of progress, but cannot distinguish a UI stall from slow diagnostic I/O,
and cannot measure subsecond input latency. `ui_update.last_ms/mean_ms/maximum_ms`
measure the Shell callback, **not chart frame time**. Mean and maximum are
process-lifetime statistics, not statistics of the capture interval.

Online AIS `chart_marks` is the stored supplemental overlay population; cached
and in-radius positions are different populations. Active-route waypoint count
and weather wind-record count are model inputs. They are **not drawn-object or
label counts**. Local AIS, all route/waypoint overlays, drawn chart-object/label
counts and chart frame times are unavailable in existing diagnostics and remain
JSON `null`. Do not interpret absence as zero or add costly render instrumentation
without a separate reviewed change.

## Required native frame/stack evidence

Alongside the passive collector, a separately authorized Windows performance
capture must measure the same exact PID and time window. Use an already installed
and verified PresentMon/WPR setup; record its version, configuration and raw
trace/CSV SHA-256. Do not install a profiler or start a system-wide trace as an
implicit part of running this helper.

PresentMon's `--process_id` filters one process, and its `MsBetweenPresents`
column describes the interval between presents. Keep rows separated by swap
chain and retain the raw timing axis. Check that this renderer actually emits
capturable presents; missing events or an idle static chart are not 0 ms frame
time. Presentation intervals alone do not measure chart-paint execution or
input-to-visible response. See the [official PresentMon console documentation](https://github.com/GameTechDev/PresentMon/blob/main/README-ConsoleApplication.md).
Report sample count, median, p95 and maximum for the marked active zoom/pan
intervals, along with timing gaps and tool/renderer limitations. Do not substitute
DWM/compositor frame cadence, callback duration, or file polling interval.

For a stall, preserve sampled native UI-thread stacks over that same interval
using the established Windows performance profiler, with matching binary/PDB
identity and thread ID. An instrumented frame/paint trace or supported present
trace plus an input timeline is needed; missing frame telemetry leaves this
acceptance gate open. Any debugger/dump that suspends the process needs separate
handling and is not passive collector output.

The next boat evidence still required is: exact installed bytes and candidate,
real chart/type/scale and zoom sequence; actual overlay population and available
vs missing drawn counts; real renderer/plugins; native UI-thread stack samples
at a stall; synchronized frame/input timings plus CPU/resident samples; and
before/after recovery/detail-restoration results for Day/Dusk/Night on the target
Windows hardware at 1280×800. Keep private charts, positions and traces out of
public issue attachments. This helper alone does not close SCRUM-317.
