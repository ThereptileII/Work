# Milestone Test Checklist

Copy this checklist into milestone reports.

## Common
- [ ] Exact commit SHA recorded
- [ ] OpenCPN upstream tag/commit recorded
- [ ] Clean build completed
- [ ] Unit tests pass
- [ ] Relevant integration tests pass
- [ ] Application launches
- [ ] No new unexplained warnings/errors
- [ ] Documentation updated if architecture changed

## Data
- [ ] Simulated valid data renders correctly
- [ ] Missing value renders as unavailable, not zero
- [ ] Stale value transitions correctly
- [ ] Source and timestamp can be inspected
- [ ] Multiple-source precedence behaves as designed

## XNav UI
- [ ] 1280×800 layout is usable
- [ ] Touch targets meet specification
- [ ] Day mode checked
- [ ] Night mode checked
- [ ] Chart remains readable
- [ ] Critical alerts cannot be hidden by normal panels
- [ ] No accidental duplicate commands from repeated input

## Modes
- [ ] XNav works
- [ ] Legacy works
- [ ] Safe Mode works or milestone status explicitly documents incomplete path
- [ ] Mode switch preserves required state

## Windows gate
- [ ] Same commit builds under native Windows/MSVC
- [ ] Windows screenshot captured for UI changes
- [ ] DPI/font behavior checked when relevant
- [ ] DLL/plugin load checked when relevant
- [ ] Installer test checked when relevant

## Safety
- [ ] No fabricated navigation/sensor data
- [ ] Estimated values identified as estimated
- [ ] Failed hardware command does not display false success
- [ ] SmartNav remains advisory
