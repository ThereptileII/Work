# SCRUM-259: bind resource generation to the entry Python

Failed source d5d71356d806ea8c3518644d10728a24f1334d1d maps to remote
b8cfbf809450208f723095ffb4e00d7800b619a5, run 37114216075. The linked private
adapter passed its own package verification, but host configure refused it at
verify-ocharts-adapter-package.py with “Adapter chart resource manifest differs.”
The exact guard remains unchanged.

## Evidence and bounded diagnosis

The downloaded failed artifact 11271783971 contains the sealed private package.
Its resource manifest identity is 12cfbce4686fce9a89e15f45105195c9f09a51ae4e3e9932aa7ba803ae205a52
(85,084 bytes), matching independently retained canonical resources. The monitor
verified the entire source/PE package against those resources without loading it.
The failed artifact omits host-generated resources, so its exact differing
manifest fields and PNG bytes cannot be reconstructed from that artifact alone.

The actual windows-xnav-Win32.log PATH at lines 426/433 identifies Python 3.12.10;
CMake selects Python 3.14.7/python3.exe at lines 17760/18248. The first generator
uses PATH-resolved python; the host independently calls find_package(Python3).
The generator uses zlib.compress(...,9). The official [Python 3.14 release notes](https://docs.python.org/3.14/whatsnew/3.14.html#zlib)
confirm that default Windows binaries switched from zlib to zlib-ng. Different
compressed PNG bytes are therefore a concrete risk, but are not falsely claimed
as a byte-proven reproduction of the failed host outputs. The native focused
probe below must establish the observed difference before a full retry.

No resource copy/newline conversion explains this boundary: generator output is
written as bytes, prepared resources are copied with shutil.copyfile, and the
package records raw hashes. Host generation runs immediately before verification.

## Repair and proof

build-pristine-windows.ps1 captures sys.executable at entry, verifies an absolute
existing executable, and records its SHA256, version and compression-library
facts. All its Run python calls use that executable. Both private-adapter CMake
and host CMake receive an explicit Python3_EXECUTABLE FILEPATH; the same host
command applies to development and production. Private-package same-job reuse
also pins interpreter path/hash. No semantic-only resource comparison is added.

The existing build-wiring test passes with a new interpreter-drift refusal,
17 rejection cases total. The cheap real CMake contract passes locally: unpinned
discovery is recorded and two explicit configurations select the captured
executable. A same-sized different-byte inventory is rejected. The first local
attempt lacked CMake on PATH; its receipt is retained. Repeating with the existing
sysroot toolchain PATH passed. No application or dependency build occurred.

The existing skager-ocharts-loader workflow accepts the push marker
`[resource-python]` on its existing branch. This selects only its new resource job
and skips the native-DLL job; normal loader behavior is unchanged without the
marker. Workflow dispatch resource_only=true selects the same bounded job.

The exact native command is:

```powershell
& 'C:\hostedtoolcache\windows\Python\3.12.10\x64\python.exe' opennav-x/tools/test-chart-resource-python.py --entry-version 3.12.10 --evidence opennav-x/evidence/local/chart-resource-python
```

It fetches only five source-lock-qualified public resource inputs, generates the
entry set, then uses actual CMake discovery to generate an unpinned witness and
two pinned development/production sets. It requires an actual unpinned byte
difference and exact equality of all seven pinned generated files. Interpreter,
zlib, hash/size and decoded-pixel witnesses are retained; decoded equivalence is
diagnostic only and never substitutes for hash equality. Logs/resources upload
on failure. No SDK, compiler, producer, plugin/helper/app execution or full suite.

Future full-build failure artifacts now retain private input/prepared resources,
both host resource directories and host CMake caches, alongside the interpreter
receipts. Native reproduction and the next full build remain pending. No CI was
published/dispatched by this change and no boat state was touched.
