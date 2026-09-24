# First Task for Codex

Do not begin by redesigning many screens.

Complete this sequence first:

## Phase 0 — Baseline

1. Read:
   - `START_HERE.md`
   - `AGENTS.md`
   - `PROJECT_GOAL.md`
   - the full `OpenNavX_Codex_Project_Specification.md`
   - `docs/design/OpenNavX_Design_Reference.png`

2. Select and pin the exact upstream OpenCPN release/commit for the first supported version.

3. Build pristine, unmodified OpenCPN on Linux.

4. Create `docs/baseline.md` containing:
   - OpenCPN version/tag/commit
   - repository URL
   - build toolchain versions
   - dependency versions
   - exact configure/build/run commands
   - plugin API level detected from source
   - executable/config/plugin/data paths
   - any build warnings that may matter
   - baseline screenshots if available on Linux

5. Inventory the relevant OpenCPN source areas before making changes.

6. Create `docs/upstream-patches.md` before the first core modification.

## Phase 1 — First XNav vertical slice

Implement only:

- XNav/Legacy/Safe mode configuration foundation
- XNav shell/frame
- unchanged OpenCPN chart canvas hosted inside the XNav shell
- XNav top status bar
- right-side data rail populated from simulated Vessel Data
- day/night theme toggle
- a reliable route back to Legacy Mode
- developer diagnostics showing build/mode/data source information

The slice must be runnable.

## Phase 2 — Evidence

Before expanding the scope:

- clean Linux build passes
- unit/smoke tests pass
- XNav launches
- Legacy launches
- Safe Mode path is present or stubbed with explicit status
- simulated data shows correctly
- stale data behavior is tested
- screenshot evidence is saved
- the exact commit is sent to the Windows build lane
- native Windows build status is recorded

Do not begin SmartNav, advanced autopilot, radar fusion, or installer patching until this slice is stable.
