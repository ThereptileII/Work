# SCRUM-259: bounded adapter source/build preparation

This increment prepares an independently named **skager-ocharts-adapter.dll**.
It does not qualify native loading, rendering, licensing, shop operations or boat
use. No original plugin, helper, data, credentials or charts are replaced.
The parent application owns selection of the original exact plugin identity and
its original helper/data paths. Safe/Legacy/Standard policies remain host-owned.

## Exact source and wrapper delta

`tools/ocharts-adapter-source.lock.json` pins 216 open plugin files at
`bdbcat/o-charts_pi:c98bf5f0654a6aee52cd9a6b9fabc0d1cf8047e8` and three API17
files from its exact `opencpn-libs` gitlink
`leamas/opencpn-libs:38762f4e7b8faf39133cc40efc99b12b5200f04b`.
Every original file is checked against both Git blob SHA-1 and byte count.
The API17 `msvc-wx32/opencpn.lib` was inspected as COFF I386. It supplies the
existing host API; the adapter exports its own private bind/status functions.

The minimal CMake wrapper preserves the plugin's production source list,
private static S52 renderer, GL/GLEW definitions, wxCurl networking and plugin
version. It explicitly links **ocpn::api_wx32**, because the upstream wrapper
otherwise selects the old API import library unless its package name contains
`wx32`. It uses C++17 for the owned integration helpers, `/MD`, Release, Win32
and locked wxWidgets 3.2.8. A module-definition file specifies the four exact
C factory/binding/status names, all of which the actual PE validator requires.

The recipe defines `DECL_EXP` empty for every private compilation unit before
adding the static libraries. The pinned API17 header honors this override with
`#ifndef DECL_EXP`; otherwise its Windows `__declspec(dllexport)` annotates host
API classes and generates unwanted exported copy/inline methods. A read-only
parse of the accepted original DLL (`99edcfd4419d606ef5c3fd554759e853cfda1fe4f38c05e26266f335a5a5b875`)
found 116 named exports, including these methods. A `.def` alone does not hide
them, so the previous recipe could not satisfy the strict four-export gate.
The override leaves the `.def` factory entries and the adapter's explicit
`SKAGER_ADAPTER_EXPORT` bind/status declarations intact. It does not change
`DECL_IMP`, calling conventions or the API17 import library. The pinned import
library supplies ordinary function thunks as well as `__imp_` symbols (checked
for `GetGlobalColor`, `GetpSharedDataLocation` and `opencpn_plugin` constructors),
so plain external function declarations still resolve to the same host DLL.
The API's separate data-import declaration for `wxEVT_DOWNLOAD_EVENT` is
unchanged. Preprocessing the exact pinned macro block with and without the
override confirms the difference; this is a source/COFF review, not a native
link claim. The native package gate must still confirm exactly four exports.

The wrapper excludes upstream packaging/installation, `libs/oeserverd`, old
prebuilt curl/zlib libraries and all helper/licensing/chart payloads. It has no
install target. Only the upstream public API import library and pinned OpenCPN
GLEW SDK are fetched as binary build inputs; no proprietary binary is fetched.
There is no `*_pi.dll` output. The current qualified application's modern
networking runtime supplies the adapter's dependencies.

Preparation retains immutable original blobs separately from the build copy.
Only that derived text copy normalizes CRLF to LF before applying the exact
presentation and trust patches. An isolated private Git directory prevents
`git apply` from accidentally using/skipping paths in a parent checkout.
Verification re-derives patched source from original blobs and checks the
complete prepared inventory, rather than trusting its receipt alone.

## Existing dependency paths and commands

Run only in a disposable native MSVC Windows build environment, after the
existing maintained dependency producers have passed. Reuse their exact
prefixes; do not rebuild networking or use old plugin import libraries:

- `build/windows-curl-8.22.0/install`
- `build/windows-zlib-1.3.2/install`
- The OpenSSL prefix and its executable/manifest must remain at the location
  recorded by the qualified curl producer. The existing curl validator checks
  that relationship. Missing/restored-invalid prefixes fail preparation.

From the product checkout, with its already generated, verified chart resources:

```powershell
python tools/prepare-ocharts-adapter.py --output build/ocharts-prepared --cache build/ocharts-source-cache --curl-prefix build/windows-curl-8.22.0/install --zlib-prefix build/windows-zlib-1.3.2/install --resources build/opennav-chart-style/v1
cmake -S cmake/ocharts-adapter -B build/ocharts-native -G "Visual Studio 17 2022" -A Win32 -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DSKAGER_PREPARED="$PWD/build/ocharts-prepared"
cmake --build build/ocharts-native --config Release --target skager-ocharts-adapter --parallel 2
python tools/prepare-ocharts-adapter.py --prepared build/ocharts-prepared --package-dll build/ocharts-native/Release/skager-ocharts-adapter.dll --output build/ocharts-package
python tools/verify-ocharts-adapter-package.py --package build/ocharts-package --resources build/opennav-chart-style/v1 --header build/generated/SkagerOChartsPackage.h
```

Output directories must be new. Configure revalidates prepared inputs. The
package validator compares current canonical product/patch inputs, resources,
DLL bytes, actual PE32/I386 imports and exports, and corresponding source. It
rejects extra package payloads, delayed imports, forwarded exports, unknown
side-by-side DLLs and `libeay32`/`ssleay32`. `libcurl.dll` keeps its supported
name; its receipt must identify maintained 8.22.0 source. **Actual runtime DLL
hashes still must match the qualified application's dependency manifests at
native load/package gates**; an import filename alone cannot prove that.

The validator emits `skager_ocharts::{available,sha256,bytes}` only after byte
identity checks. This is a build-time trust header, not an acceptance receipt.
The build pipeline must retain failed configure/link/import/TLS evidence and
must not treat successful header generation as those gates passing.

## Mandatory private wxCurl trust correction (SCRUM-209/211)

A no-wxCurl build was considered and rejected. The plugin's existing fallback
passes a `file://` URI for thumbnails (`ochartShop.cpp:1403`) whereas Windows
host `OCPN_downloadFile` treats its output as a filesystem path. Host
`OCPN_postDataHttp` also converts parameters with `ToAscii`. Switching that
branch would introduce unrelated shop/authentication behavior changes.

The recipe retains `__OCPN_USE_CURL__` and builds the original open wxCurl sources
against the existing maintained curl headers/import library. Inspection found
its own trust defaults separate from the core's SCRUM-211 fix:
`libs/wxcurl/src/base.cpp:828` used a cwd-relative `curl-ca-bundle.crt`, with
`VERIFYPEER=true` and no explicit hostname policy. The mandatory
`patches/ocharts-wxcurl-trust.patch` ports the inspected core policy:

- Windows native CA store; explicit peer `1L` and hostname `2L` verification.
- No cwd CA path and no production test-CA override.
- Any failed option blocks `Perform`; reset/reinitialize establishes a fresh
  handle state. A missing handle also refuses transfer.
- At most five redirects, with initial HTTPS restricted to HTTPS redirects.
  Supported initial HTTP/FTP/Telnet behavior is retained.

The existing native gate now accepts a bounded adapter selection. After fresh
preparation and the maintained runtime dependency manifests are available:

```powershell
pwsh -NoLogo -NoProfile -File tools/test-downloader-trust-windows.ps1 -IntegrationSource build/integration-source -Install production-install -OChartsPrepared build/ocharts-prepared
```

`-OChartsPrepared` verifies the complete prepared source receipt and independently
re-derives the exact patched source before any owned CA is installed. The probe
compiles the actual private `libs/wxcurl/src/{base,http}.cpp` with its actual
`src/wx/curl` headers, `/MD` and the maintained curl import library. CMake repeats
preparation verification. It uses the prepared locked wxWidgets SDK, requires
the cache curl manifest to equal the preparation's curl manifest byte-for-byte,
and checks copied wx DLLs against the prepared release DLLs. No test-CA compile
definition or source substitution is introduced.

The existing core Downloader probe remains a control; its cases and every
wxCurl assertion remain unchanged. The native harness runs only on a disposable
GitHub Actions Windows runner and uses owned loopback TLS peers. It retains
trusted GET/HEAD, HTTPS redirect, wrong host, untrusted, expired, HTTPS downgrade,
local-file redirect, different cwd, failed-option refusal and trust-removal
checks; exact CurrentUser CA cleanup remains mandatory in finally. No public
shop, account, credential, chart or helper operation is performed.

Private results are isolated under
`evidence/local/ocharts-private-wxcurl-trust-windows/`, with actual private
base/http/header and probe hashes, the preparation receipt hash, the selected
source kind, curl manifest hash and staged runtime DLL hashes. Default invocation
without the argument still selects the core sources and its existing evidence
directory. This is a short probe build, not a new application/dependency build;
`production-install` must already contain the exact manifested maintained DLLs.
The existing strict dependency checks refuse an unrelated older runtime.

The adapter now compiles the exact shared `ChartNameAlphaWindows.cpp` helper
from alpha correction `8b9e0a45c569d75d0c4f4c7a71a263cf5a910f95`, with the matching
`ChartNameText.h`, and links `gdiplus`. Both source files participate in the
canonical input receipt. The helper's platform/renderer guards need no additional
compile definition; no shared S52 header gains Windows SDK macros.

Local follow-up checks pass: fourteen focused preparation/selection/policy tests,
PowerShell parser validation, unchanged core/TLS case assertions, and exact alpha
source comparison. These are wiring checks; **the private native TLS command has
not been run here and remains a required gate**. The previously passing standalone
Windows alpha run does not qualify this adapter DLL or private TLS copy.

## Source/compliance artifact

The three-file package contains DLL, `manifest.json`, and deterministic
`corresponding-source.zip`. The ZIP contains original pinned source/import
blobs, original COPYING/COPYING.gplv2/debian copyright notices, all owned source
and patches, and the exact build/validation recipe. It retains the upstream
per-file copyright/license headers. No closed helper or chart is included.
Current OpenCPN, wxWidgets, GLEW and maintained curl/OpenSSL/zlib source bundles,
licenses and notices remain mandatory in the application package; this adapter
receipt does not replace or remove them. Dependencies identify their upstream
source/archive/signature pins and consumed output hashes in the manifest.

Host integration accepts an explicit `SKAGER_OCHARTS_PACKAGE` only on native
MSVC Win32. Configuration emits the exact DLL trust header; build and install
revalidate the complete package against current source and resources. A missing
option compiles an unavailable adapter state. Recovery packaging independently
checks the installed DLL against that header, corresponding source, actual
curl/zlib runtime bytes and installed chart resources. Its source ZIP is also
included in the standalone corresponding-source artifact. Adapter-free builds
refuse stray adapter payloads. `gdiplus.dll` is an explicit Windows-system
import for the qualified translucent-name path; no broad import wildcard was
introduced.

Focused local checks: all 219 original blobs verified; both patches applied to
a fresh normalized source copy; actual patched private wxCurl base/http compiled
with maintained curl 8.22.0 headers on Linux. Fourteen focused test methods cover
PE architecture/import/export/delay/forwarder/truncation refusals, unsafe paths,
blob mismatch, extra payload, package roundtrip, DLL/source/resource/source-ZIP
mutation, legacy dependency identity, trust-patch policy, and isolated CRLF patch application below a parent Git checkout.
An actual-source negative control changed patched bytes and its receipt together;
independent re-derivation still rejected it. No native DLL,
shop/network credential operation, helper execution, application build, CI
publication or boat change is claimed by this preparation increment.
