# Read-only loaded module path observation — SCRUM-209

`tools/win32_module_observation.py` is a separate, opt-in native Windows helper
for an **already running, explicitly selected process**. It never launches the
application, loads a plugin, changes plugin state, edits PATH, or writes target
files. It is not wired into packaging, workflows or runtime enforcement.

The source boundary was inspected at pinned OpenCPN
`37fd0cddb7334fe489e9f18aa163977a9c5c84f7`:
[`model/src/plugin_loader.cpp`](https://github.com/OpenCPN/OpenCPN/blob/37fd0cddb7334fe489e9f18aa163977a9c5c84f7/model/src/plugin_loader.cpp)
`setLoadPath()` prepends plugin library directories to the inherited Windows
PATH; `LoadAllPlugIns()` visits `PluginPaths::Libdirs()`;
`LoadPlugInDirectory()` discovers `*_pi.dll`; and `LoadPlugIn()` calls
`m_library.Load(plugin_file)` before obtaining and calling `create_pi`.
Consequently, a declared-directory inventory or a log of candidate paths cannot
establish the dependencies actually selected by the Windows loader. A module
snapshot supplies an additional OS-reported path observation, without changing
that upstream loading sequence or establishing that a plugin initialized.

Example in a native Windows Python shell, with the selected process already
running (replace all illustrative values, including the full 64-digit hash):

```text
python tools/win32_module_observation.py --pid 1234 --expected-exe "C:/Review/SKAGER/opencpn.exe" --expected-sha256 <reviewed-executable-sha256> --require-module "C:/Review/SKAGER/plugins/example_pi.dll"
```

The executable expectation refers to its **disk file**, not its mapped image.
The helper opens the process with only `PROCESS_QUERY_LIMITED_INFORMATION` and
`SYNCHRONIZE`. It holds that handle throughout capture, checks liveness, and
compares PID, executable path and process creation FILETIME before and after
capture. PID reuse, exit or identity change makes the observation incomplete.
It takes two
[Toolhelp module snapshots](https://learn.microsoft.com/en-us/windows/win32/api/tlhelp32/nf-tlhelp32-createtoolhelp32snapshot)
with both native and 32-bit flags, retaining each reported path, base address
and mapped image size. This does not inspect process memory. A 64-bit observer
is recommended for the approved x86 application on x64 Windows; unsupported
cross-bitness/access failures remain errors. `ERROR_BAD_LENGTH` retries are
bounded to three attempts. Enumeration errors and possibly truncated module
paths are rejected rather than silently producing partial success.

Every observed module path is separately read through the inventory's bounded,
symlink/reparse-refusing disk reader. Its nested `disk` record contains the
path, byte count and SHA-256 of exactly those captured bytes. A missing or
unreadable file keeps the OS observation with a `diskError`, never an invented
hash. Process identity, the module set (including addresses/sizes), and observed
disk metadata must stay unchanged for the helper's observation status to pass.
A missing `--require-module` path fails the observation. It does not imply an
import closure even when all requested paths are present. The CLI records
Python/platform/pointer width and the helper source files' on-disk hashes;
source drift during capture fails the observation. These source hashes are
provenance of the captured files, not executable-code attestation.

The stdout JSON schema is `SKAGER.WindowsModuleObservation.1`.
`module-paths-and-disk-files-observed` / exit 0 means only the requested
observation completed; failures use `observation-incomplete` / exit 1.
`runtimeEnforcement`, `loaderClosureAccepted` and
`mappedImageIdentityEstablished` remain false in every report. Neither status
establishes originally loaded bytes, memory contents, plugin activation,
signatures, publisher/installer ownership, TLS policy, or trust. Two matching
snapshots cannot prove that no load/unload occurred between them. Toolhelp can
omit transient modules, datafile loads, manually mapped images and child
processes. Disk replacement checks do not create an atomic filesystem snapshot.
The inventory and this observation remain distinct evidence records: a module
path can be supplied as an inventory root, but the static graph cannot be
upgraded to loader closure by association.

Focused validation: `python tools/test-win32-module-observation.py -v` covers
separate path/disk identity, pinned executable mismatches, PID creation-time
change, exit, changed module path/address/size, required-module absence,
access failure, missing disk files, disk/source mutation, reparse refusal,
budgets, duplicate/empty results, and non-Windows CLI refusal. Fixtures never
execute their target bytes. The native self-process smoke test is explicitly
skipped on Linux. It only observes the already running Python interpreter and
does not qualify SKAGER/plugin behavior.

**Remaining native gate:** run the helper tests with native Windows Python,
then retain observations for the exact installed application and its expected
plugin paths, including ambiguous/retained dependency locations and lifecycle
changes. Review these against exact package/installer ownership and the static
inventory. Source loading, full dynamic/import closure, TLS/certificate checks
and clean/update/repair/rollback/uninstall acceptance still require their own
SCRUM-209 evidence. No installed-host result is claimed by the Linux fixtures.
