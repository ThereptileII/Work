# Native resource interpreter reproduction and repair

[Run 37119161157](https://github.com/ThereptileII/Work/actions/runs/37119161157),
attempt 1, completed successfully on Windows Server 2022. Exact local source
`eced8dae855d477a9ea356ef600a958322bbc1d2` maps to remote
`c7f3616c5b9d385d39edf25b51f89a3508906071`. Resource job 111191742361 passed;
the old native-DLL job was skipped. No producer, compiler, app, plugin or boat ran.

The downloaded artifact 11273051001 is 4,007,959 bytes; its independently verified
SHA256 is `7149133bedf1601364c4c9984a22535f9cf68b2ad1fcb3497af6422af97f87b6`.
All 102 ZIP CRCs passed. Original bytes remain in the private worktree at
`.local/resource-native/artifact.zip`, with all four complete resource sets
extracted under `.local/resource-native/artifact`.

## Actual reproduced difference

Entry Python 3.12.10 uses zlib 1.3.1. Unpinned CMake selected Python 3.14.7 using
zlib-ng 2.2.4 (compatibility version 1.3.1.zlib-ng). Their exact paths and CI-reported
binary hashes are in native-result.json. Executable bytes are not in this artifact;
those Python binary hashes are not claimed as independently rehashed here.

| File | Entry bytes | Unpinned bytes |
|---|---:|---:|
| Day PNG | 199,897 | 196,379 |
| Dusk PNG | 174,289 | 173,348 |
| Night PNG | 169,663 | 168,069 |

All three PNG byte hashes changed. Independent Pillow decoding found **all
21,600,000 RGBA bytes identical**, with dimensions 1500×1200, and PNG non-IDAT
chunks were also byte-identical. XML and RLE were byte-identical. Manifest metadata
was identical except the three PNG hash/size records; these changes also altered
the generated resource header. This demonstrates compressor encoding drift,
not changed chart pixels. Pixel equality remains diagnostic, not an acceptance
substitute for exact packaged byte identities.

The entry manifest was the original sealed package identity
`12cfbce4686fce9a89e15f45105195c9f09a51ae4e3e9932aa7ba803ae205a52`;
the unpinned manifest was
`34b1c89f95f856e8fa5ddf5465d27765955af1eb9972bf91b358e3b18abedf90`.
Original b8 failure evidence still lacks its host-generated files; this separate
source-qualified native reproduction proves the same identified boundary without
pretending to recover those missing historical bytes.

## Exact repair result

Explicitly pinned development and production configurations both selected the
entry Python executable and each reproduced all seven entry files byte-for-byte.
Every file in all four sets (28 files) and all five pinned source inputs were
independently rehashed after download, matching the native receipts and manifests.
The collector, generator and source-lock runtime hashes match the exact local
commit under Windows CRLF checkout conversion. Existing PowerShell build wiring
passed all 17 refusal cases, including development/production interpreter drift.

Full receipts, both differing manifests/headers and complete native logs are
retained here. No assertion was weakened, no rerun occurred, and no artifact
failure was discarded. The initial commit-runs convenience query returned an empty
list; direct Actions API lookup found the correct run. This did not change CI.

This closes the bounded resource identity reproduction and pinning proof. It does
not qualify a full application build, module lifecycle, physical GPU, encrypted
charts, or boat operation. The [official Python 3.14 release notes](https://docs.python.org/3.14/whatsnew/3.14.html#zlib)
record the Windows zlib-ng transition which this native receipt now corroborates.
