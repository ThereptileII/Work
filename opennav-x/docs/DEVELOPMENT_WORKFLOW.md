# Development Workflow

## Primary loop — Linux

1. Pull/update branch.
2. Build pinned upstream/OpenNav configuration.
3. Implement one small vertical change.
4. Run unit/integration tests.
5. Launch with simulator/test profile.
6. Capture evidence where relevant.
7. Commit focused changes.

## Windows gate

For any milestone touching:
- UI
- OpenCPN integration
- plugin ABI
- DLL loading
- platform code
- XNav/Legacy/Safe startup
- installer/updater
- release packaging

send the exact same commit to the native Windows build/test lane.

A Linux-passing commit is not considered release-ready until the applicable Windows gate passes.

## Upstream discipline

- Keep upstream OpenCPN revision pinned.
- Avoid mass formatting of upstream files.
- Prefer new OpenNav files plus small integration hooks.
- Record every direct upstream modification in `docs/upstream-patches.md`.
- Rebase/merge upstream only in deliberate upgrade work.
