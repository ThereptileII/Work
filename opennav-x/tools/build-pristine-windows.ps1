param([ValidateSet('Win32', 'x64')][string]$Architecture = 'Win32', [switch]$Integration, [switch]$Production,
      [switch]$PrototypeObjectFlow, [switch]$ReuseVerifiedDependencies, [switch]$VerifyPeerCli, [switch]$PrivateOCharts)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Root = Split-Path $PSScriptRoot -Parent
$Source = Join-Path $Root 'upstream/OpenCPN'
if ($Production -and -not $Integration) { throw 'Production requires Integration' }
if ($VerifyPeerCli -and (-not $Integration -or $Production)) {
    throw 'Peer CLI isolation verification requires the initial integrated fixture build'
}
if ($ReuseVerifiedDependencies -and (-not $Production -or -not $Integration -or
    $env:GITHUB_ACTIONS -cne 'true' -or $env:GITHUB_JOB -cne 'windows-integration')) {
    throw 'Dependency reuse is only available to the explicit same-job CI production invocation'
}
if ($PrototypeObjectFlow -and (-not $Integration -or $Production -or $env:GITHUB_ACTIONS -ne 'true')) {
    throw 'The prototype-only object flow is a disposable CI development gate, not a production/release gate'
}
if ($PrivateOCharts -and (-not $Integration -or $env:GITHUB_ACTIONS -cne 'true' -or
    $env:GITHUB_JOB -cne 'windows-integration')) {
    throw 'Private adapter build requires the explicit disposable Windows integration job'
}
$Variant = if ($Production) { 'production' } elseif ($Integration) { 'xnav' } else { 'pristine' }
$Evidence = Join-Path $Root 'evidence/local'
New-Item -ItemType Directory -Force $Evidence | Out-Null
Start-Transcript -Path (Join-Path $Evidence "windows-$Variant-$Architecture.log")
function Run([string]$Program, [string[]]$Arguments) {
    & $Program @Arguments 2>&1 | Tee-Object -FilePath (Join-Path $Evidence 'windows-native-output.log') -Append
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
function Digest([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "Required file missing: $Path" }
    (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}
function Assert-ManifestRecord([object]$Record, [string]$Path, [string]$Label) {
    $Properties = @($Record.PSObject.Properties.Name | Sort-Object)
    if (@(Compare-Object @('bytes','sha256') $Properties).Count -or
        $Record.sha256 -cne (Digest $Path) -or $Record.bytes -ne (Get-Item -LiteralPath $Path).Length) {
        throw "$Label differs from its producer manifest"
    }
}
function Build-PrivateOCharts([bool]$Reuse) {
    $Prepared = Join-Path $Root 'build/ocharts-prepared'
    $NativeBuild = Join-Path $Root 'build/ocharts-native'
    $Package = Join-Path $Root 'build/ocharts-package'
    $Resources = Join-Path $Root 'build/ocharts-chart-style/v1'
    $ReceiptPath = Join-Path $Evidence 'windows-ocharts-first-build.json'
    $Identity = [ordered]@{
        run = $env:GITHUB_RUN_ID; attempt = $env:GITHUB_RUN_ATTEMPT
        job = $env:GITHUB_JOB; commit = $env:GITHUB_SHA
        script = Digest (Join-Path $PSScriptRoot 'build-pristine-windows.ps1')
    }
    foreach ($Value in $Identity.Values) {
        if (-not $Value) { throw 'Private adapter requires complete same-job identity' }
    }
    if ($Reuse) {
        $Previous = Get-Content -LiteralPath $ReceiptPath -Raw | ConvertFrom-Json
        foreach ($Key in $Identity.Keys) {
            if ($Previous.identity.$Key -cne $Identity[$Key]) {
                throw 'Private adapter build belongs to different job/source inputs'
            }
        }
        Assert-ManifestRecord $Previous.preparation (Join-Path $Prepared 'preparation.json') 'private preparation receipt'
        foreach ($Name in @('manifest.json','skager-ocharts-adapter.dll','corresponding-source.zip')) {
            Assert-ManifestRecord $Previous.package.$Name (Join-Path $Package $Name) "private package $Name"
        }
    } else {
        foreach ($Path in @($Prepared,$NativeBuild,$Package,$ReceiptPath)) {
            if (Test-Path -LiteralPath $Path) { throw "Private adapter first build requires a fresh output: $Path" }
        }
    }
    # Generate from the same pinned integration input before the host configure.
    # The host repeats generation; its package verifier requires exact equality.
    Run python @((Join-Path $PSScriptRoot 'generate-xnav-chart-style.py'),
        '--source', (Join-Path $Source 'data/s57data'), '--output', $Resources)
    if (-not $Reuse) {
        Run python @((Join-Path $PSScriptRoot 'prepare-ocharts-adapter.py'),
            '--output', $Prepared, '--cache', (Join-Path $Root 'build/ocharts-source-cache'),
            '--curl-prefix', (Join-Path $Root 'build/windows-curl-8.22.0/install'),
            '--openssl-prefix', $OpenSslPrefix, '--zlib-prefix', $ZlibPrefix, '--resources', $Resources)
        Run cmake @('-S', (Join-Path $Root 'cmake/ocharts-adapter'), '-B', $NativeBuild,
            '-G', 'Visual Studio 17 2022', '-A', 'Win32', '-DCMAKE_POLICY_VERSION_MINIMUM=3.5',
            "-DSKAGER_PREPARED=$Prepared")
        Run cmake @('--build', $NativeBuild, '--config', 'Release', '--target',
            'skager-ocharts-adapter', '--parallel', '2')
        Run python @((Join-Path $PSScriptRoot 'prepare-ocharts-adapter.py'),
            '--prepared', $Prepared, '--package-dll', (Join-Path $NativeBuild 'Release/skager-ocharts-adapter.dll'),
            '--output', $Package)
    }
    # Re-derive original+patch source and inspect every prepared byte on BOTH
    # passes. A package receipt alone is never proof of available SDK/runtime.
    Run python @((Join-Path $PSScriptRoot 'prepare-ocharts-adapter.py'), '--verify-prepared', $Prepared)
    foreach ($Library in @('curl','zlib')) {
        $Prefix = if ($Library -eq 'curl') { Join-Path $Root 'build/windows-curl-8.22.0/install' } else { $ZlibPrefix }
        $Manifest = Join-Path $Prefix "$Library-build.json"
        if ((Digest $Manifest) -cne (Digest (Join-Path $Prepared "sdk/$Library-build.json"))) {
            throw "Private adapter $Library manifest differs from same-job producer"
        }
        $Facts = Get-Content -LiteralPath $Manifest -Raw | ConvertFrom-Json
        foreach ($Item in $Facts.outputs.PSObject.Properties) {
            Assert-ManifestRecord $Item.Value (Join-Path $Prefix $Item.Name) "$Library producer $($Item.Name)"
            Assert-ManifestRecord $Item.Value (Join-Path $Prepared "sdk/$($Item.Name)") "private $Library SDK $($Item.Name)"
        }
    }
    Run python @((Join-Path $PSScriptRoot 'verify-ocharts-adapter-package.py'),
        '--package', $Package, '--resources', $Resources,
        '--header', (Join-Path $Root 'build/ocharts-verification/SkagerOChartsPackage.h'))
    if (-not $Reuse) {
        $Files = [ordered]@{}
        foreach ($Name in @('manifest.json','skager-ocharts-adapter.dll','corresponding-source.zip')) {
            $Path = Join-Path $Package $Name
            $Files[$Name] = @{sha256=(Digest $Path);bytes=(Get-Item -LiteralPath $Path).Length}
        }
        $PrepPath = Join-Path $Prepared 'preparation.json'
        [ordered]@{identity=$Identity; package=$Files
            preparation=@{sha256=(Digest $PrepPath);bytes=(Get-Item -LiteralPath $PrepPath).Length}
        } | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ReceiptPath -Encoding utf8
    }
    $script:OChartsPackage = $Package
}
$OChartsPackage = ''
try {
    if ($ReuseVerifiedDependencies -and -not $PrivateOCharts -and
        (Test-Path -LiteralPath (Join-Path $Evidence 'windows-ocharts-first-build.json'))) {
        throw 'Production pass must preserve the first pass PrivateOCharts selection'
    }
    if ($Integration) {
        if (-not (Test-Path -LiteralPath $env:SKAGER_NATIVE_PERL -PathType Leaf)) {
            throw 'The native OpenSSL build Perl was not selected before MSYS2 setup'
        }
        $NativePerl = (Resolve-Path -LiteralPath $env:SKAGER_NATIVE_PERL).Path
        $env:PATH = "$(Split-Path $NativePerl -Parent);$env:PATH"
        if ((Get-Command perl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source -ine $NativePerl) {
            throw 'OpenSSL build Perl differs from the preselected native tool'
        }
        if (-not (Test-Path -LiteralPath $env:SKAGER_CURL_TEST_PERL -PathType Leaf)) {
            throw 'MSYS2 curl test Perl was not selected'
        }
    }
    Run python @((Join-Path $PSScriptRoot 'verify-upstream.py'))
    if (-not [Environment]::Is64BitOperatingSystem) { throw 'Windows x64 host required' }
    if ($Architecture -eq 'x64') {
        throw 'OpenCPN 5.12.4 ships Win32 dependencies. An x64 dependency and plugin ABI port is not validated; refusing to mislabel Win32 as x64.'
    }
    # Stop before curl preflight, stock win_deps or any expensive producer if
    # the existing Poedit provider cannot supply both usable gettext tools.
    $GettextReceipt = Join-Path $Evidence "windows-gettext-$Variant.json"
    Run python @((Join-Path $PSScriptRoot 'windows_gettext.py'), 'ensure',
        '--allow-install', '--receipt', $GettextReceipt)
    $GettextFacts = Get-Content -LiteralPath $GettextReceipt -Raw | ConvertFrom-Json
    $Gettext = $GettextFacts.directory
    $env:PATH = "$Gettext;$env:PATH"
    if ($Integration) {
        # Exercise the unchanged curl source tests with the reviewed native/MSYS
        # environment before any maintained dependency compilation. The real curl
        # producer repeats them against its own generated configurehelp.pm.
        $CurlPreflight = Join-Path $Evidence ("windows-curl-source-early-$Variant")
        Run pwsh @('-NoProfile', '-File', (Join-Path $PSScriptRoot 'test-curl-source-preflight.ps1'),
            '-ProductionOnly', '-TestPerl', $env:SKAGER_CURL_TEST_PERL, '-Evidence', $CurlPreflight)
        # Fail on a changed zlib source before the costly OpenSSL build. The
        # normal zlib build below repeats the same guard and upstream tests.
        & (Join-Path $PSScriptRoot 'build-zlib-windows.ps1') -VerifySourceOnly
        # Also verify the producer/consumer lock contract before building dependencies.
        & (Join-Path $PSScriptRoot 'test-zlib-source-verification.ps1')
    }
    if ($Integration) {
        Run python @((Join-Path $PSScriptRoot 'prepare-integration.py'))
        $Source = Join-Path $Root 'build/integration-source'
        if (-not $ReuseVerifiedDependencies) {
            # Compile complete changed units with native Windows/wx headers
            # before spending time on maintained dependency producer suites.
            Run python @((Join-Path $PSScriptRoot 'test-windows-changed-units.py'),
                '--ui', '--evidence', (Join-Path $Evidence "windows-changed-units-$Variant"))
        }
    }
    # Upstream's batch file can continue after a failed wget/7z operation.
    # Prepopulate the exact supported wx bundle with checked, retryable fetches.
    $Wx = Join-Path $Source 'cache/wxWidgets-3.2.8'
    $Downloads = Join-Path $Source 'cache/opennav-downloads'
    New-Item -ItemType Directory -Force $Downloads | Out-Null
    $WxLock = Get-Content (Join-Path $PSScriptRoot 'windows-wx.lock.json') -Raw | ConvertFrom-Json
    foreach ($Item in $WxLock.archives) {
        $Archive = Join-Path $Downloads $Item.file
        if (-not (Test-Path $Archive) -or ((Get-FileHash $Archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $Item.sha256)) {
            Run curl.exe @('--fail', '--location', '--silent', '--show-error', '--retry', '3',
                '--retry-all-errors', '--connect-timeout', '20', '--max-time', '180', '--output', $Archive, $Item.url)
        }
        if ((Get-FileHash $Archive -Algorithm SHA256).Hash.ToLowerInvariant() -ne $Item.sha256) {
            throw "Dependency checksum mismatch: $($Item.file)"
        }
        Run 7z @('x', '-y', "-o$Wx", $Archive)
    }
    if (-not (Test-Path (Join-Path $Wx 'include/wx/version.h'))) { throw 'wxWidgets headers missing after extraction' }
    Copy-Item (Join-Path $PSScriptRoot 'windows-wx.lock.json') (Join-Path $Evidence 'windows-wx-provenance.json')
    Push-Location $Source
    try {
        Run cmd @('/c', 'buildwin\win_deps.bat')
    } finally { Pop-Location }
    if ($Integration) {
        # Replace the stock dependency bundle only in the disposable integrated
        # tree. Each consumer runs only after its producer manifest and output
        # records have been verified.
        $ZlibPrefix = Join-Path $Root 'build/windows-zlib-1.3.2/install'
        $ZlibManifestPath = Join-Path $ZlibPrefix 'zlib-build.json'
        $OpenSslPrefix = Join-Path $Root 'build/windows-openssl-3.5.9/install'
        if ($ReuseVerifiedDependencies) {
            Write-Output "Same-job dependency reuse verification begin: $([DateTime]::UtcNow.ToString('o'))"
            # The normal source/consumer and stock dependency preflights have
            # already run. Verify immutable producer evidence before reading
            # generated CMake tool records, then reprobe live tools in the
            # same parent process and original producer order.
            Run python @((Join-Path $PSScriptRoot 'windows_dependency_reuse.py'), 'verify', '--root', $Root)
            & (Join-Path $PSScriptRoot 'build-openssl-windows.ps1') -IntegrationSource $Source -VerifyToolFactsOnly
            & (Join-Path $PSScriptRoot 'build-zlib-windows.ps1') -VerifyToolFactsOnly
            $BeforeCurlPath = $env:PATH
            try {
                $env:PATH = "$(Split-Path $env:SKAGER_CURL_TEST_PERL -Parent);$env:PATH"
                & (Join-Path $PSScriptRoot 'build-curl-windows.ps1') -IntegrationSource $Source `
                    -OpenSslPrefix $OpenSslPrefix -ZlibPrefix $ZlibPrefix -ZlibManifest $ZlibManifestPath `
                    -VerifyToolFactsOnly
            } finally { $env:PATH = $BeforeCurlPath }
            # This helper rechecks the receipt/evidence and inventories every
            # source prefix before replacing stock win_deps cache payloads.
            Run python @((Join-Path $PSScriptRoot 'windows_dependency_stage.py'), '--root', $Root)
            Write-Output "Same-job dependency cache restage passed: $([DateTime]::UtcNow.ToString('o'))"
        } else {
            & (Join-Path $PSScriptRoot 'build-openssl-windows.ps1') -IntegrationSource $Source 2>&1 |
                Tee-Object -FilePath (Join-Path $Evidence 'windows-openssl-native-output.log') -Append
            if ($LASTEXITCODE -ne 0) { throw 'Pinned OpenSSL source build failed' }
            & (Join-Path $PSScriptRoot 'build-zlib-windows.ps1') 2>&1 |
                Tee-Object -FilePath (Join-Path $Evidence 'windows-zlib-native-output.log') -Append
            if ($LASTEXITCODE -ne 0) { throw 'Pinned zlib source build failed' }

            $ZlibManifest = Get-Content -LiteralPath $ZlibManifestPath -Raw | ConvertFrom-Json
            $ZlibCache = Join-Path $Source 'cache/buildwin'
            $ZlibMappings = [ordered]@{
                'include/zlib.h' = 'include/zlib.h'
                'include/zconf.h' = 'include/zconf.h'
                'lib/zlib1.lib' = 'zlib1.lib'
                'bin/zlib1.dll' = 'zlib1.dll'
            }
            foreach ($Mapping in $ZlibMappings.GetEnumerator()) {
                $Produced = Join-Path $ZlibPrefix $Mapping.Key
                Assert-ManifestRecord $ZlibManifest.outputs.($Mapping.Key) $Produced "zlib $($Mapping.Key)"
                $Cached = Join-Path $ZlibCache $Mapping.Value
                $null = New-Item -ItemType Directory -Force -Path (Split-Path $Cached -Parent)
                Copy-Item -LiteralPath $Produced -Destination $Cached -Force
                Assert-ManifestRecord $ZlibManifest.outputs.($Mapping.Key) $Cached "cached zlib $($Mapping.Value)"
            }

            $BeforeCurlPath = $env:PATH
            try {
                $env:PATH = "$(Split-Path $env:SKAGER_CURL_TEST_PERL -Parent);$env:PATH"
                & (Join-Path $PSScriptRoot 'build-curl-windows.ps1') -IntegrationSource $Source `
                    -OpenSslPrefix $OpenSslPrefix -ZlibPrefix $ZlibPrefix -ZlibManifest $ZlibManifestPath 2>&1 |
                    Tee-Object -FilePath (Join-Path $Evidence 'windows-curl-orchestration.log') -Append
                if ($LASTEXITCODE -ne 0) { throw 'Pinned curl source build failed' }
            } finally { $env:PATH = $BeforeCurlPath }
        }
        $CurlManifestPath = Join-Path $Source 'cache/buildwin/curl-build.json'
        $CurlManifest = Get-Content -LiteralPath $CurlManifestPath -Raw | ConvertFrom-Json
        if ($CurlManifest.library -cne 'curl' -or $CurlManifest.version -cne '8.22.0' -or
            $CurlManifest.architecture -cne 'Win32' -or $CurlManifest.abi -cne 'x86' -or
            $CurlManifest.buildSteps.configure -cne 'passed' -or
            $CurlManifest.buildSteps.compile -cne 'passed' -or
            $CurlManifest.buildSteps.test -cne 'passed' -or
            $CurlManifest.buildSteps.install -cne 'passed' -or
            $CurlManifest.importOutput -notmatch '(?im)^\s*libssl-3\.dll\s*$' -or
            $CurlManifest.importOutput -notmatch '(?im)^\s*libcrypto-3\.dll\s*$' -or
            $CurlManifest.importOutput -notmatch '(?im)^\s*zlib1\.dll\s*$' -or
            $CurlManifest.importOutput -match '(?i)ssleay32\.dll|libeay32\.dll') {
            throw 'Maintained curl manifest does not prove the reviewed dependency closure'
        }
        foreach ($CacheOutput in @('libcurl.dll','libcurl.lib')) {
            Assert-ManifestRecord $CurlManifest.outputs.($(if ($CacheOutput -eq 'libcurl.dll') { 'bin/libcurl.dll' } else { 'lib/libcurl.lib' })) `
                (Join-Path $Source "cache/buildwin/$CacheOutput") "cached $CacheOutput"
        }
    }
    if ($PrivateOCharts) { Build-PrivateOCharts ([bool]$ReuseVerifiedDependencies) }
    $Wx = Join-Path $Source 'cache/wxWidgets-3.2.8'
    $Build = Join-Path $Root "build/$Variant-windows"
    $Install = Join-Path $Root "build/$Variant-install"
    # Reprobe the exact approved files after dependency setup; no late PATH
    # substitute or installer retry is allowed here.
    Run python @((Join-Path $PSScriptRoot 'windows_gettext.py'), 'verify', '--receipt', $GettextReceipt)
    $env:PATH = "$Gettext;$env:PATH;$Wx\lib\vc14x_dll;$Source\cache\buildwin"
    $OpenNavArgs = @()
    if ($Integration) {
        $Fixtures = if ($Production) { 'OFF' } else { 'ON' }
        $OpenNavArgs = @("-DOPENNAV_ROOT=$Root", "-DOPENNAV_ENABLE_ROUTE_SCENARIO=$Fixtures", "-DXNAV_ENABLE_TEST_FIXTURES=$Fixtures", "-DXNAV_ENABLE_PILOT_LOOPBACK_TESTS=$Fixtures", "-DSKAGER_OCHARTS_PACKAGE=$OChartsPackage")
    }
    Run cmake (@('-S', $Source, '-B', $Build, '-G', 'Visual Studio 17 2022',
        '-A', $Architecture, '-DCMAKE_POLICY_VERSION_MINIMUM=3.5', '-DCMAKE_BUILD_TYPE=Release',
        "-DwxWidgets_ROOT_DIR=$Wx", "-DwxWidgets_LIB_DIR=$Wx/lib/vc14x_dll",
        '-DwxWidgets_CONFIGURATION=mswu', '-DOCPN_CI_BUILD=ON',
        "-DGETTEXT_MSGFMT_EXECUTABLE=$Gettext/msgfmt.exe",
        "-DGETTEXT_MSGMERGE_EXECUTABLE=$Gettext/msgmerge.exe",
        '-DOCPN_BUILD_TEST=ON', '-DOCPN_BUNDLE_WXDLLS=ON',
        '-DOCPN_BUNDLE_DOCS=OFF', '-DOCPN_BUNDLE_GSHHS=ON',
        '-DOCPN_BUNDLE_TCDATA=ON', "-DCMAKE_INSTALL_PREFIX=$Install") + $OpenNavArgs)
    Run cmake @('--build', $Build, '--config', 'Release', '--parallel', '2')
    Run cmake @('--install', $Build, '--config', 'Release')
    if ($Integration) {
        $OpenSslManifest = Get-Content (Join-Path $Source 'cache/buildwin/openssl-build.json') -Raw | ConvertFrom-Json
        $ZlibManifest = Get-Content (Join-Path $Root 'build/windows-zlib-1.3.2/install/zlib-build.json') -Raw | ConvertFrom-Json
        $CurlManifest = Get-Content (Join-Path $Source 'cache/buildwin/curl-build.json') -Raw | ConvertFrom-Json
        Copy-Item (Join-Path $Source 'cache/buildwin/openssl-build.json') (Join-Path $Install 'openssl-build.json') -Force
        Copy-Item (Join-Path $Root 'build/windows-zlib-1.3.2/install/zlib-build.json') (Join-Path $Install 'zlib-build.json') -Force
        Copy-Item (Join-Path $Source 'cache/buildwin/curl-build.json') (Join-Path $Install 'curl-build.json') -Force
        $InstalledRecords = [ordered]@{
            'libssl-3.dll' = $OpenSslManifest.outputs.'bin/libssl-3.dll'
            'libcrypto-3.dll' = $OpenSslManifest.outputs.'bin/libcrypto-3.dll'
            'zlib1.dll' = $ZlibManifest.outputs.'bin/zlib1.dll'
            'libcurl.dll' = $CurlManifest.outputs.'bin/libcurl.dll'
        }
        foreach ($Item in $InstalledRecords.GetEnumerator()) {
            Assert-ManifestRecord $Item.Value (Join-Path $Install $Item.Key) "installed $($Item.Key)"
        }
        foreach ($Legacy in @('libeay32.dll','ssleay32.dll')) {
            if (Test-Path -LiteralPath (Join-Path $Install $Legacy)) {
                throw "Disposable integration install retained legacy TLS runtime: $Legacy"
            }
        }
    }
    if ($VerifyPeerCli) {
        # Run before any installed application/model test: even GUI launches
        # with --configdir create the normal home directory in InitializeLogFile.
        # The CLI test must still refuse every pre-existing common-data profile.
        Run python @((Join-Path $PSScriptRoot 'peer-cli-receipt.py'), 'capture',
            '--cli', (Join-Path $Install 'opencpn-cmd.exe'),
            '--receipt', (Join-Path $Evidence 'windows-peer-cli-receipt.json'))
    }
    if ($Integration) {
        # Offline painter processes; no chart/profile/input or hardware output.
        Run (Join-Path $Build 'Release/chart_name_text_test.exe') @((Join-Path $Evidence "chart-names-$Variant.png"))
        Run (Join-Path $Build 'Release/chart_light_label_test.exe') @((Join-Path $Evidence "chart-lights-$Variant.png"))
        Run (Join-Path $Build 'Release/skager_wordmark_test.exe') @((Join-Path $Evidence "skager-wordmark-$Variant.png"))
        Run (Join-Path $Build 'Release/ui_font_resolution_test.exe') @()
        Run (Join-Path $Build 'Release/chart_route_label_test.exe') @((Join-Path $Evidence "chart-route-labels-$Variant.png"))
        Run (Join-Path $Build 'Release/onboard_ais_body_test.exe') @((Join-Path $Evidence "onboard-ais-$Variant.png"))
    }
    Run ctest @('--test-dir', (Join-Path $Build 'test'), '-C', 'Release', '--output-on-failure', '--no-tests=error',
        '--timeout', '90', '--output-junit', (Join-Path $Evidence "windows-$Variant-tests.xml"))
    Get-FileHash (Join-Path $Build 'Release/opencpn.exe') -Algorithm SHA256 |
        Format-List | Out-File (Join-Path $Evidence "windows-$Variant-executable-sha256.txt")
    Run python @((Join-Path $PSScriptRoot 'verify-upstream.py'))
    if ($PrivateOCharts -and $Production) {
        # Installed maintained runtime is now available. Keep the private probe
        # separate from the unchanged core TLS gate and its evidence directory.
        Run pwsh @('-NoLogo', '-NoProfile', '-File', (Join-Path $PSScriptRoot 'test-downloader-trust-windows.ps1'),
            '-IntegrationSource', $Source, '-Install', 'production-install',
            '-OChartsPrepared', (Join-Path $Root 'build/ocharts-prepared'))
    }
    if ($Integration -and -not $Production) {
        if ($PrototypeObjectFlow) {
            # Additional targeted development job. The default full integrated
            # release workflow below remains mandatory and unchanged.
            Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--objects')
            # Actual upstream route projection/paint, not a separate geometry
            # model. Every variant also keeps the complete progress lifecycle.
            foreach ($theme in @('Day', 'Dusk', 'Night')) {
                Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--route-fixture', '--theme', $theme)
            }
            Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--route-fixture-standard')
        } else {
        Run python @((Join-Path $PSScriptRoot 'smoke-modes-windows.py'))
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'))
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--route-fixture')
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--route-fixture-standard')
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--instruments')
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--n2k')
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--boat')
        Run python @((Join-Path $PSScriptRoot 'smoke-signalk.py'))
        Run python @((Join-Path $PSScriptRoot 'smoke-recording.py'))
        Run python @((Join-Path $PSScriptRoot 'smoke-pilot.py'))
        Run python @((Join-Path $PSScriptRoot 'smoke-navigation.py'), '--objects')
        Run python @((Join-Path $PSScriptRoot 'smoke-recovery.py'))
        & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1') -Variant xnav -Mode legacy -Name '11-legacy-mode'
        & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1') -Variant xnav -Mode safe-mode -Name '12-safe-mode'
        }
    } elseif ($Production) {
        Run python @((Join-Path $PSScriptRoot 'smoke-pilot.py'), '--production')
    } else {
        & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1')
    }
} finally { Stop-Transcript }
