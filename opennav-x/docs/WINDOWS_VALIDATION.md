# Windows Validation Requirements

Windows is the release authority for OpenNav X.

## Required native Windows checks

### Build
- MSVC build completes for the pinned OpenCPN/OpenNav commit.
- Correct build configuration is used for OpenCPN/plugin ABI expectations.
- No missing DLLs at launch.
- Companion plugins/modules load successfully.

### UI
Validate on native Windows:
- 1280×800 reference resolution
- 100%, 125%, 150% DPI scaling where practical
- fonts and text metrics
- touch/mouse hit areas
- window chrome/full-screen behavior
- chart canvas sizing
- XNav day/dusk/night
- context cards and panels
- no unexpected native bright dialogs in primary night-mode workflows

### Modes
Test:
- XNav start
- Legacy start
- Safe Mode start
- XNav → Legacy switch
- Legacy → XNav switch
- navigation/configuration state preserved as specified
- crash-loop/recovery path

### Packaging
Test on a clean/disposable Windows environment:
1. install supported OpenCPN
2. install OpenNav X
3. launch XNav
4. launch Legacy
5. launch Safe Mode
6. run diagnostics
7. run Repair
8. apply Update
9. run Rollback
10. uninstall OpenNav X
11. verify normal OpenCPN remains usable

### Release evidence
Retain:
- build log
- commit SHA
- OpenCPN version
- installer version
- test results
- 1280×800 screenshots
- install/repair/rollback/uninstall logs
- relevant hashes

Linux or Wine results may support debugging but do not replace these native Windows checks.
