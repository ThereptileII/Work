param(
    [Parameter(Mandatory=$true)][string]$IntegrationSource,
    [Parameter(Mandatory=$true)][string]$OpenSslPrefix,
    [Parameter(Mandatory=$true)][string]$ZlibPrefix,
    [Parameter(Mandatory=$true)][string]$ZlibManifest,
    [switch]$VerifyToolFactsOnly
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$Root = Split-Path $PSScriptRoot -Parent
$ProducerScript = $PSCommandPath
. (Join-Path $PSScriptRoot 'windows-curl-environment.ps1')
$Evidence = Join-Path $Root 'evidence/local'
$NativeLog = Join-Path $Evidence 'windows-curl-native-output.log'
if (-not $VerifyToolFactsOnly) {
    $null = New-Item -ItemType Directory -Force -Path $Evidence
    Set-Content -LiteralPath $NativeLog -Value '' -Encoding UTF8
}
$LockPath = Join-Path $PSScriptRoot 'windows-curl.lock.json'
$Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
if ($Lock.version -cne '8.22.0' -or $Lock.configuration -cne 'Win32 shared OpenSSL' -or
    $Lock.opensslVersion -cne '3.5.9' -or $Lock.zlibManifestSchemaVersion -ne 1) {
    throw 'Unexpected curl lock identity; refusing an unreviewed build'
}

function Resolve-Directory([string]$Path,[string]$Label) {
    if (-not (Test-Path -LiteralPath $Path -PathType Container)) { throw "$Label missing: $Path" }
    (Resolve-Path -LiteralPath $Path).Path
}
function Resolve-File([string]$Path,[string]$Label) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "$Label missing: $Path" }
    $Item = Get-Item -LiteralPath $Path -Force
    if ($Item.LinkType) { throw "$Label must not be a link: $Path" }
    $Item.FullName
}
function Digest([string]$Path) {
    (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}
function FileRecord([string]$Path) {
    $Path = Resolve-File $Path 'Required curl output'
    [ordered]@{ sha256=Digest $Path; bytes=(Get-Item -LiteralPath $Path).Length }
}
function Assert-Record([object]$Record,[string]$Path,[string]$Label) {
    $Properties = @($Record.PSObject.Properties.Name | Sort-Object)
    if (@(Compare-Object @('bytes','sha256') $Properties).Count -or
        $Record.sha256 -cne (Digest $Path) -or $Record.bytes -ne (Get-Item -LiteralPath $Path).Length) {
        throw "$Label differs from its verified manifest"
    }
}
function Assert-Win32Image([string]$Path) {
    $Bytes = [IO.File]::ReadAllBytes($Path)
    if ($Bytes.Length -lt 64 -or $Bytes[0] -ne 0x4d -or $Bytes[1] -ne 0x5a) { throw "Not a PE DLL: $Path" }
    $Pe = [BitConverter]::ToInt32($Bytes,0x3c)
    if ($Pe -lt 0 -or $Pe + 6 -gt $Bytes.Length -or [BitConverter]::ToUInt32($Bytes,$Pe) -ne 0x00004550 -or
        [BitConverter]::ToUInt16($Bytes,$Pe + 4) -ne 0x014c) { throw "curl dependency image is not Win32/x86: $Path" }
}
function Invoke-Checked([string]$Program,[string[]]$Arguments) {
    & $Program @Arguments 2>&1 | Tee-Object -FilePath $NativeLog -Append
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}

$IntegrationSource = Resolve-Directory $IntegrationSource 'Integration source'
$OpenSslPrefix = Resolve-Directory $OpenSslPrefix 'Verified OpenSSL prefix'
$ZlibPrefix = Resolve-Directory $ZlibPrefix 'Verified zlib prefix'
$ZlibManifest = Resolve-File $ZlibManifest 'Verified zlib manifest'
$OpenSslManifestPath = Resolve-File (Join-Path $OpenSslPrefix 'openssl-build.json') 'Verified OpenSSL manifest'
$OpenSslManifest = Get-Content -LiteralPath $OpenSslManifestPath -Raw | ConvertFrom-Json
$OpenSslLock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'windows-openssl.lock.json') -Raw | ConvertFrom-Json
$OpenSslSource = $OpenSslManifest.source
if ($OpenSslManifest.library -cne 'OpenSSL' -or $OpenSslManifest.version -cne $Lock.opensslVersion -or
    $OpenSslManifest.architecture -cne 'Win32' -or $OpenSslManifest.abi -cne 'x86' -or
    $OpenSslManifest.configuration -cne 'VC-WIN32 shared' -or
    $OpenSslManifest.buildSteps.configure -cne 'passed' -or $OpenSslManifest.buildSteps.compile -cne 'passed' -or
    $OpenSslManifest.buildSteps.test -cne 'passed' -or $OpenSslManifest.buildSteps.install -cne 'passed' -or
    $OpenSslSource.url -cne $OpenSslLock.url -or $OpenSslSource.archive -cne $OpenSslLock.archive -or
    $OpenSslSource.sha256 -cne $OpenSslLock.sha256 -or $OpenSslSource.bytes -ne $OpenSslLock.bytes -or
    $OpenSslSource.signingPrimaryFingerprint -cne $OpenSslLock.signingPrimaryFingerprint) {
    throw 'OpenSSL prefix is not a verified OpenSSL 3.5.9 Win32 shared build'
}
$OpenSslHeader = Resolve-File (Join-Path $OpenSslPrefix 'include/openssl/opensslv.h') 'OpenSSL version header'
$OpenSslSslLib = Resolve-File (Join-Path $OpenSslPrefix 'lib/libssl.lib') 'OpenSSL SSL import library'
$OpenSslCryptoLib = Resolve-File (Join-Path $OpenSslPrefix 'lib/libcrypto.lib') 'OpenSSL crypto import library'
$OpenSslSslDll = Resolve-File (Join-Path $OpenSslPrefix 'bin/libssl-3.dll') 'OpenSSL SSL DLL'
$OpenSslCryptoDll = Resolve-File (Join-Path $OpenSslPrefix 'bin/libcrypto-3.dll') 'OpenSSL crypto DLL'
$OpenSslExe = Resolve-File (Join-Path $OpenSslPrefix 'bin/openssl.exe') 'OpenSSL certificate tool'
Assert-Record $OpenSslManifest.outputs.'include/openssl/opensslv.h' $OpenSslHeader 'OpenSSL version header'
Assert-Record $OpenSslManifest.outputs.'lib/libssl.lib' $OpenSslSslLib 'OpenSSL SSL import library'
Assert-Record $OpenSslManifest.outputs.'lib/libcrypto.lib' $OpenSslCryptoLib 'OpenSSL crypto import library'
Assert-Record $OpenSslManifest.outputs.'bin/libssl-3.dll' $OpenSslSslDll 'OpenSSL SSL DLL'
Assert-Record $OpenSslManifest.outputs.'bin/libcrypto-3.dll' $OpenSslCryptoDll 'OpenSSL crypto DLL'
Assert-Record $OpenSslManifest.outputs.'bin/openssl.exe' $OpenSslExe 'OpenSSL certificate tool'
Assert-Win32Image $OpenSslSslDll
Assert-Win32Image $OpenSslCryptoDll
Assert-Win32Image $OpenSslExe
$OpenSslToolVersion = & $OpenSslExe version -a 2>&1 | Out-String
if ($LASTEXITCODE -ne 0 -or $OpenSslToolVersion -notmatch '(?m)^OpenSSL 3\.5\.9\b' -or
    $OpenSslToolVersion.Trim() -cne $OpenSslManifest.versionOutput) {
    throw 'Pinned OpenSSL certificate tool differs from the verified producer version output'
}
$OpenSslToolRecord = [ordered]@{
    path=$OpenSslExe; sha256=Digest $OpenSslExe; bytes=(Get-Item -LiteralPath $OpenSslExe).Length
    versionOutput=$OpenSslToolVersion.Trim()
}

$Zlib = Get-Content -LiteralPath $ZlibManifest -Raw | ConvertFrom-Json
$ZlibKeys = @($Zlib.PSObject.Properties.Name | Sort-Object)
if (@(Compare-Object @('abi','architecture','buildSteps','configuration','library','outputs','runtime','schemaVersion','source','version') $ZlibKeys).Count -or
    $Zlib.schemaVersion -ne 1 -or $Zlib.library -cne 'zlib' -or $Zlib.version -cne '1.3.2' -or
    $Zlib.architecture -cne 'Win32' -or $Zlib.abi -cne 'x86' -or
    $Zlib.configuration -cne 'Win32 shared' -or $Zlib.runtime -cne 'MultiThreadedDLL (/MD)' -or
    $Zlib.buildSteps.configure -cne 'passed' -or $Zlib.buildSteps.compile -cne 'passed' -or
    $Zlib.buildSteps.test -cne 'passed' -or $Zlib.buildSteps.install -cne 'passed') {
    throw 'zlib manifest does not satisfy the reviewed Win32 shared /MD schema'
}
$ZlibSourceKeys = @($Zlib.source.PSObject.Properties.Name | Sort-Object)
if (@(Compare-Object @('archive','bytes','sha256','signingPrimaryFingerprint','url') $ZlibSourceKeys).Count -or
    $Zlib.source.url -cne 'https://github.com/madler/zlib/releases/download/v1.3.2/zlib-1.3.2.tar.gz' -or
    $Zlib.source.archive -cne 'zlib-1.3.2.tar.gz' -or
    $Zlib.source.sha256 -cne 'bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16' -or
    $Zlib.source.bytes -ne 1502830 -or
    $Zlib.source.signingPrimaryFingerprint -cne '5ED46A6721D365587791E2AA783FCD8E58BCAFBA') {
    throw 'zlib manifest does not identify the reviewed zlib 1.3.2 source'
}
$ZlibHeader = Resolve-File (Join-Path $ZlibPrefix 'include/zlib.h') 'zlib public header'
$ZlibConfigHeader = Resolve-File (Join-Path $ZlibPrefix 'include/zconf.h') 'zlib configuration header'
$ZlibImport = Resolve-File (Join-Path $ZlibPrefix 'lib/zlib1.lib') 'zlib import library'
$ZlibDll = Resolve-File (Join-Path $ZlibPrefix 'bin/zlib1.dll') 'zlib DLL'
foreach ($Pair in ([ordered]@{
    'include/zlib.h'=$ZlibHeader; 'include/zconf.h'=$ZlibConfigHeader;
    'lib/zlib1.lib'=$ZlibImport; 'bin/zlib1.dll'=$ZlibDll
}).GetEnumerator()) { Assert-Record $Zlib.outputs.($Pair.Key) $Pair.Value "zlib $($Pair.Key)" }
if (@(Compare-Object @('bin/zlib1.dll','include/zconf.h','include/zlib.h','lib/zlib1.lib') @($Zlib.outputs.PSObject.Properties.Name | Sort-Object)).Count) {
    throw 'zlib output inventory has an unsupported path'
}
Assert-Win32Image $ZlibDll

$BuildRoot = Join-Path $Root "build/windows-curl-$($Lock.version)"
$Downloads = Join-Path $Root 'build/dependency-downloads'
$Archive = Join-Path $Downloads $Lock.archive
$Source = Join-Path $BuildRoot "curl-$($Lock.version)"
$Build = Join-Path $BuildRoot 'build'
$Prefix = Join-Path $BuildRoot 'install'
if (-not $VerifyToolFactsOnly) {
    $null = New-Item -ItemType Directory -Force -Path $Downloads,$BuildRoot,$Evidence
}
if (-not (Test-Path -LiteralPath $Archive -PathType Leaf) -or (Digest $Archive) -cne $Lock.sha256) {
    if ($VerifyToolFactsOnly) { throw 'Verified curl archive unavailable for tool-facts reprobe' }
    Invoke-Checked curl.exe @('--fail','--location','--silent','--show-error','--retry','3','--retry-all-errors',
        '--connect-timeout','20','--max-time','300','--output',$Archive,$Lock.url)
}
if ((Digest $Archive) -cne $Lock.sha256 -or (Get-Item -LiteralPath $Archive).Length -ne $Lock.bytes) {
    throw 'curl source archive digest or size differs from the reviewed lock'
}

$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$Vswhere = Resolve-File $Vswhere 'Visual Studio locator'
$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VisualStudio) { throw 'Licensed MSVC x86 toolchain missing' }
$DumpbinCandidates = @(Get-ChildItem -LiteralPath (Join-Path $VisualStudio 'VC/Tools/MSVC') -Directory |
    Sort-Object Name -Descending | ForEach-Object { Join-Path $_.FullName 'bin/Hostx64/x86/dumpbin.exe' } |
    Where-Object { Test-Path -LiteralPath $_ -PathType Leaf })
if (-not $DumpbinCandidates.Count) { throw 'MSVC Win32 dumpbin missing' }
$Dumpbin = $DumpbinCandidates[0]
$ToolFacts = Resolve-File (Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1') 'Native tool-facts helper'
$CMakeFactsInclude = Resolve-File (Join-Path $PSScriptRoot 'windows-native-tool-facts.cmake') 'Native CMake tool-facts include'
$ImportLayoutInclude = Resolve-File (Join-Path $PSScriptRoot 'windows-curl-import-layout.cmake') 'Curl import layout include'
$Facts = Join-Path $Evidence 'windows-curl-parent-tool-facts.json'
foreach ($Tool in @('cmake.exe','perl.exe')) {
    if (-not (Get-Command $Tool -CommandType Application -ErrorAction SilentlyContinue)) { throw "curl build prerequisite missing: $Tool" }
}
$CMake = (Get-Command cmake.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
$NativePerl = Resolve-File $env:SKAGER_NATIVE_PERL 'Native OpenSSL build Perl'
$TestPerl = Resolve-File $env:SKAGER_CURL_TEST_PERL 'MSYS2 curl test Perl'
$MsysRuntime = Resolve-File (Join-Path (Split-Path $TestPerl -Parent) 'msys-2.0.dll') 'MSYS2 curl test runtime'
$TestPerlOs = & $TestPerl -e 'print $^O'
if ($LASTEXITCODE -ne 0 -or $TestPerlOs -notin @('cygwin','msys') -or $TestPerl -ieq $NativePerl) {
    throw 'Curl test Perl must be a separate MSYS2 POSIX host'
}
$TestPerlRecord = [ordered]@{
    path=$TestPerl; sha256=Digest $TestPerl; bytes=(Get-Item -LiteralPath $TestPerl).Length
    os=$TestPerlOs; runtimePath=$MsysRuntime; runtimeSha256=Digest $MsysRuntime
    runtimeBytes=(Get-Item -LiteralPath $MsysRuntime).Length
}
Invoke-WindowsCurlEnvironment -VisualStudio $VisualStudio -TestPerl $TestPerl -Action {
if ($VerifyToolFactsOnly) {
    $BuiltManifest = Get-Content -LiteralPath (Join-Path $Prefix 'curl-build.json') -Raw | ConvertFrom-Json
    $BuiltHost = $BuiltManifest.buildSteps.testHost
    if ($BuiltHost.path -ine $TestPerlRecord.path -or $BuiltHost.sha256 -cne $TestPerlRecord.sha256 -or
        $BuiltHost.bytes -ne $TestPerlRecord.bytes -or $BuiltHost.os -cne $TestPerlRecord.os -or
        $BuiltHost.runtimePath -ine $TestPerlRecord.runtimePath -or
        $BuiltHost.runtimeSha256 -cne $TestPerlRecord.runtimeSha256 -or
        $BuiltHost.runtimeBytes -ne $TestPerlRecord.runtimeBytes) {
        throw 'Live curl test host differs from the successful producer manifest'
    }
    # Normal curl capture occurs after configure with these dependency DLL
    # directories prepended. Recreate that build PATH before live reprobe.
    $env:PATH = "$(Join-Path $Build 'lib/Release');$(Join-Path $OpenSslPrefix 'bin');$(Join-Path $ZlibPrefix 'bin');$env:PATH"
    & $ToolFacts -Mode Verify -Kind curl-parent -Output $Facts -ProducerScript $ProducerScript `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars (Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat') -Dumpbin $Dumpbin `
        -CMakeCache (Join-Path $Build 'CMakeCache.txt') -CMakeHookFacts (Join-Path $Build 'xnav-native-cmake-tools.txt') `
        -CurlTestPerl $TestPerl
    Write-Output 'Reprobed captured curl build-environment tool facts'
    return
}
$ZlibImports = & $Dumpbin /DEPENDENTS $ZlibDll 2>&1 | Tee-Object -FilePath $NativeLog -Append | Out-String
if ($LASTEXITCODE -ne 0 -or $ZlibImports -notmatch '(?im)^\s*VCRUNTIME140[^\s]*\.dll\s*$' -or
    $ZlibImports -notmatch '(?im)^\s*(?:api-ms-win-crt-[^\s]+|ucrtbase)\.dll\s*$') {
    throw 'zlib DLL does not prove the required modern dynamic MSVC/UCRT runtime'
}
if (Test-Path -LiteralPath $Source) { Remove-Item -LiteralPath $Source -Recurse -Force }
if (Test-Path -LiteralPath $Build) { Remove-Item -LiteralPath $Build -Recurse -Force }
if (Test-Path -LiteralPath $Prefix) { Remove-Item -LiteralPath $Prefix -Recurse -Force }
Add-Content -LiteralPath $NativeLog -Value "curl source extraction begin: $([DateTime]::UtcNow.ToString('o'))" -Encoding UTF8
Invoke-Checked $CMake @('-E','chdir',$BuildRoot,$CMake,'-E','tar','xf',$Archive)
Add-Content -LiteralPath $NativeLog -Value "curl source extraction passed: $([DateTime]::UtcNow.ToString('o'))" -Encoding UTF8
if (-not (Test-Path -LiteralPath (Join-Path $Source 'CMakeLists.txt') -PathType Leaf)) {
    throw 'Verified curl archive did not extract the expected source root'
}
$PatchHelper = Resolve-File (Join-Path $PSScriptRoot 'patch-curl-test-openssl.py') 'Reviewed curl certificate-tool patch'
$GenServ = Resolve-File (Join-Path $Source 'tests/certs/genserv.pl') 'Locked curl certificate generator'
$HostCertConfig = Resolve-File (Join-Path $Source 'tests/certs/test-localhost.prm') 'Locked curl localhost certificate config'
$PatchEvidence = Join-Path $Evidence 'windows-curl-certificate-patch.json'
Invoke-Checked python @($PatchHelper,'--source',$GenServ,'--evidence',$PatchEvidence)
$Patch = Get-Content -LiteralPath $PatchEvidence -Raw | ConvertFrom-Json
if ($Patch.source -cne 'tests/certs/genserv.pl' -or $Patch.state -cne 'patched' -or
    $Patch.beforeSha256 -cne 'd737cbe77e23e275b4fcfcec36e62d49d1d59d9d9fd0013a428b7143ee75c982' -or
    $Patch.afterSha256 -cne '4c176ec6a1556f519d6c0c02d17c40caadb9a542894fedc9f2055b7a48ce9ab3' -or
    $Patch.lockedOriginalSha256 -cne $Patch.beforeSha256 -or
    $Patch.reviewedPatchedSha256 -cne $Patch.afterSha256 -or
    (Digest $GenServ) -cne $Patch.afterSha256) {
    throw 'curl certificate-generator patch does not match reviewed source hashes'
}
$PatchRecord = [ordered]@{
    source=$Patch.source; originalSha256=$Patch.beforeSha256
    patchedSha256=$Patch.afterSha256; helperSha256=Digest $PatchHelper
}

# Probe actual upstream CA generation in a fresh directory before the costly
# curl configure/build. The generator must select this verified openssl.exe,
# and the normal upstream certificate target and TLS tests still run later.
$ProbeDir = Join-Path $BuildRoot ("certificate-probe-$([guid]::NewGuid().ToString('N'))")
$null = New-Item -ItemType Directory -Path $ProbeDir
$OriginalPath = $env:PATH
try {
    $env:PATH = "$(Join-Path $OpenSslPrefix 'bin');$env:PATH"
    $ResolvedTool = (Get-Command openssl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    if ($ResolvedTool -ine $OpenSslExe) { throw 'curl certificate probe resolved a different OpenSSL executable' }
    Push-Location $ProbeDir
    try {
        # MSYS Perl treats backslashes in __FILE__ as ordinary characters.
        # genserv.pl derives its certificate-config directory with dirname().
        # Pass the same slash form used by curl's CMake custom command.
        Invoke-Checked $TestPerl @($GenServ.Replace('\','/'),'test',(Split-Path $HostCertConfig -Leaf))
        $CaCert = Resolve-File (Join-Path $ProbeDir 'test-ca.cacert') 'Generated upstream curl CA certificate'
        $CaKey = Resolve-File (Join-Path $ProbeDir 'test-ca.key') 'Generated upstream curl CA key'
        $HostCert = Resolve-File (Join-Path $ProbeDir 'test-localhost.crt') 'Generated upstream curl host certificate'
        $HostKey = Resolve-File (Join-Path $ProbeDir 'test-localhost.key') 'Generated upstream curl host key'
        foreach ($Generated in @($CaCert,$CaKey,$HostCert,$HostKey)) {
            if ((Get-Item -LiteralPath $Generated).Length -le 0) {
                throw "Upstream curl certificate probe produced an empty output: $Generated"
            }
        }
        Invoke-Checked $OpenSslExe @('x509','-in',$CaCert,'-noout','-subject')
        Invoke-Checked $OpenSslExe @('verify','-CAfile',$CaCert,$HostCert)
        Invoke-Checked $OpenSslExe @('pkey','-in',$HostKey,'-check','-noout')
    } finally { Pop-Location }
} finally {
    $env:PATH = $OriginalPath
    if (Test-Path -LiteralPath $ProbeDir) { Remove-Item -LiteralPath $ProbeDir -Recurse -Force }
}
Add-Content -LiteralPath $NativeLog -Value 'curl upstream certificate generation probe passed' -Encoding UTF8

$Configure = @('-S',$Source,'-B',$Build,'-G','Visual Studio 17 2022','-A','Win32',
    "-DCMAKE_INSTALL_PREFIX=$Prefix","-DCMAKE_PROJECT_INCLUDE=$($CMakeFactsInclude.Replace('\','/'));$($ImportLayoutInclude.Replace('\','/'))",
    "-DPERL_EXECUTABLE:FILEPATH=$TestPerl",
    '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL',
    '-DBUILD_SHARED_LIBS=ON','-DBUILD_STATIC_LIBS=OFF','-DBUILD_CURL_EXE=ON','-DBUILD_TESTING=ON',
    '-DIMPORT_LIB_SUFFIX:STRING=',
    '-DBUILD_EXAMPLES=OFF','-DBUILD_LIBCURL_DOCS=OFF','-DBUILD_MISC_DOCS=OFF',
    '-DCURL_USE_OPENSSL=ON','-DCURL_USE_SCHANNEL=OFF','-DCURL_STATIC_CRT=OFF','-DCURL_USE_CMAKECONFIG=OFF',
    "-DOPENSSL_ROOT_DIR=$OpenSslPrefix","-DOPENSSL_INCLUDE_DIR=$OpenSslPrefix/include",
    "-DOPENSSL_SSL_LIBRARY=$OpenSslSslLib","-DOPENSSL_CRYPTO_LIBRARY=$OpenSslCryptoLib",
    '-DCURL_ZLIB=ON',"-DZLIB_INCLUDE_DIR=$ZlibPrefix/include","-DZLIB_LIBRARY=$ZlibImport",
    '-DCURL_USE_LIBPSL=OFF','-DCURL_USE_LIBSSH2=OFF','-DCURL_USE_LIBSSH=OFF','-DUSE_NGHTTP2=OFF',
    '-DCURL_BROTLI=OFF','-DCURL_ZSTD=OFF','-DENABLE_ARES=OFF','-DCURL_USE_GSSAPI=OFF',
    '-DUSE_WIN32_IDN=ON','-DCURL_DISABLE_FORM_API=OFF','-DHTTP_ONLY=OFF',
    '-DCURL_CA_BUNDLE=none','-DCURL_CA_PATH=none','-DCURL_CA_FALLBACK=OFF','-DCURL_DISABLE_CA_SEARCH=ON','-DCURL_CA_SEARCH_SAFE=OFF')
Invoke-Checked cmake.exe $Configure
$ImportLayoutPath = Resolve-File (Join-Path $Build 'xnav-curl-import-Release.txt') 'Generated curl import layout'
$ConfiguredImport = [IO.File]::ReadAllText($ImportLayoutPath).Trim()
$ExpectedBuildImport = [IO.Path]::GetFullPath((Join-Path $Build 'lib/Release/libcurl.lib'))
if ([IO.Path]::GetFullPath($ConfiguredImport) -ine $ExpectedBuildImport) {
    throw "Configured curl import path differs from the producer contract: $ConfiguredImport"
}
Copy-Item -LiteralPath $ImportLayoutPath -Destination (Join-Path $Evidence 'windows-curl-import-Release.txt') -Force
$CacheText = Get-Content -LiteralPath (Join-Path $Build 'CMakeCache.txt') -Raw
if ($CacheText -notmatch '(?m)^PERL_EXECUTABLE:FILEPATH=(.+)$' -or
    (Resolve-Path -LiteralPath $Matches[1].Trim()).Path -ine $TestPerl) {
    throw 'Curl CMake did not bind the selected MSYS2 Perl test host'
}
foreach ($RequiredPath in @((Join-Path $OpenSslPrefix 'include'),$OpenSslSslLib,$OpenSslCryptoLib,(Join-Path $ZlibPrefix 'include'),$ZlibImport)) {
    $CmakePath = $RequiredPath.Replace('\','/')
    if ($CacheText.Replace('\','/') -notlike "*$CmakePath*") { throw "CMake did not bind the explicit dependency path: $RequiredPath" }
}
$env:PATH = "$(Join-Path $Build 'lib/Release');$(Join-Path $OpenSslPrefix 'bin');$(Join-Path $ZlibPrefix 'bin');$env:PATH"
& $ToolFacts -Mode Capture -Kind curl-parent -Output $Facts -ProducerScript $ProducerScript `
    -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars (Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat') -Dumpbin $Dumpbin `
    -CMakeCache (Join-Path $Build 'CMakeCache.txt') -CMakeHookFacts (Join-Path $Build 'xnav-native-cmake-tools.txt') `
    -CurlTestPerl $TestPerl
Invoke-WindowsCurlSourceChecks -TestPerl $TestPerl -Source $Source -Build $Build `
    -Evidence (Join-Path $Evidence 'windows-curl-source-preflight') | Tee-Object -FilePath $NativeLog -Append
Invoke-Checked cmake.exe @('--build',$Build,'--config','Release','--parallel','2')
# Do not spend the upstream suite on a producer whose actual linker output
# cannot satisfy the declared install/cache contract. No rename or fallback.
$BuiltImport = Resolve-File $ExpectedBuildImport 'Built curl import library before upstream tests'
if ((Get-Item -LiteralPath $BuiltImport).Length -le 0) { throw 'Built curl import library is empty' }
Invoke-Checked cmake.exe @('--build',$Build,'--config','Release','--target','tests','--parallel','2')
$TestSummary = $null
foreach ($Line in [IO.File]::ReadLines($NativeLog)) {
    $Match = [regex]::Match($Line,'^\s*(?:\d+>)?\s*TESTDONE: (\d+) tests out of (\d+) reported OK:')
    if ($Match.Success) { $TestSummary = $Match }
}
if (-not $TestSummary) { throw 'curl upstream tests produced no retained execution summary' }
$TestsPassed = [int]$TestSummary.Groups[1].Value
$TestsReported = [int]$TestSummary.Groups[2].Value
if ($TestsReported -le 0 -or $TestsPassed -ne $TestsReported) {
    throw "curl upstream tests did not execute and pass a nonzero set: $TestsPassed/$TestsReported"
}
Invoke-Checked cmake.exe @('--install',$Build,'--config','Release')
& $ToolFacts -Mode Verify -Kind curl-parent -Output $Facts -ProducerScript $ProducerScript `
    -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars (Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat') -Dumpbin $Dumpbin `
    -CMakeCache (Join-Path $Build 'CMakeCache.txt') -CMakeHookFacts (Join-Path $Build 'xnav-native-cmake-tools.txt') `
    -CurlTestPerl $TestPerl

$CurlDll = Resolve-File (Join-Path $Prefix 'bin/libcurl.dll') 'curl DLL'
$CurlImport = Resolve-File (Join-Path $Prefix 'lib/libcurl.lib') 'curl import library'
$CurlExe = Resolve-File (Join-Path $Prefix 'bin/curl.exe') 'curl version probe'
Assert-Win32Image $CurlDll
$Project = Resolve-File (Join-Path $Build 'lib/libcurl_shared.vcxproj') 'generated libcurl MSBuild project'
$ProjectXml = [xml](Get-Content -LiteralPath $Project -Raw)
$ReleaseGroups = @($ProjectXml.Project.ItemDefinitionGroup | Where-Object { $_.Condition -match "Release\|Win32" })
if ($ReleaseGroups.Count -ne 1 -or $ReleaseGroups[0].ClCompile.RuntimeLibrary -cne 'MultiThreadedDLL') {
    throw 'Generated libcurl Release|Win32 project does not use the required dynamic /MD runtime'
}
$env:PATH = "$(Join-Path $Prefix 'bin');$(Join-Path $OpenSslPrefix 'bin');$(Join-Path $ZlibPrefix 'bin');$env:PATH"
$VersionOutput = & $CurlExe -V 2>&1 | Tee-Object -FilePath $NativeLog -Append | Out-String
if ($LASTEXITCODE -ne 0 -or $VersionOutput -notmatch 'curl 8\.22\.0' -or
    $VersionOutput -notmatch 'libcurl/8\.22\.0' -or $VersionOutput -notmatch 'OpenSSL/3\.5\.9' -or
    $VersionOutput -notmatch '(?m)^Protocols:.*\bhttp\b.*\bhttps\b' -or
    $VersionOutput -notmatch '(?m)^Protocols:.*\bftp\b.*\bftps\b' -or
    $VersionOutput -notmatch '(?m)^Protocols:.*\btelnet\b') {
    throw 'Built curl version, TLS backend, or required protocol set differs'
}
$Imports = & $Dumpbin /DEPENDENTS $CurlDll 2>&1 | Tee-Object -FilePath $NativeLog -Append | Out-String
if ($LASTEXITCODE -ne 0 -or $Imports -notmatch '(?im)^\s*libssl-3\.dll\s*$' -or
    $Imports -notmatch '(?im)^\s*libcrypto-3\.dll\s*$' -or $Imports -notmatch '(?im)^\s*zlib1\.dll\s*$' -or
    $Imports -notmatch '(?im)^\s*VCRUNTIME140[^\s]*\.dll\s*$' -or
    $Imports -notmatch '(?im)^\s*(?:api-ms-win-crt-[^\s]+|ucrtbase)\.dll\s*$' -or
    $Imports -match '(?i)ssleay32\.dll|libeay32\.dll') {
    throw 'Built libcurl dependency closure does not match OpenSSL 3.5.9 and reviewed zlib'
}

$Expected = [ordered]@{'bin/libcurl.dll'=$CurlDll; 'lib/libcurl.lib'=$CurlImport}
$HeaderRoot = Join-Path $Prefix 'include/curl'
$Headers = @(Get-ChildItem -LiteralPath $HeaderRoot -File -Recurse | Sort-Object FullName)
if (-not $Headers.Count) { throw 'Installed curl public header set is empty' }
foreach ($Header in $Headers) {
    $Relative = $Header.FullName.Substring($Prefix.Length + 1).Replace([IO.Path]::DirectorySeparatorChar,'/')
    $Expected[$Relative] = $Header.FullName
}
$Outputs = [ordered]@{}
foreach ($Pair in $Expected.GetEnumerator()) { $Outputs[$Pair.Key] = FileRecord $Pair.Value }

$Cache = Join-Path $IntegrationSource 'cache/buildwin'
$CacheHeaders = Join-Path $Cache 'include/curl'
if (Test-Path -LiteralPath $CacheHeaders) { Remove-Item -LiteralPath $CacheHeaders -Recurse -Force }
$null = New-Item -ItemType Directory -Force -Path $CacheHeaders
Copy-Item -Path (Join-Path $HeaderRoot '*') -Destination $CacheHeaders -Recurse -Force
Copy-Item -LiteralPath $CurlImport -Destination (Join-Path $Cache 'libcurl.lib') -Force
Copy-Item -LiteralPath $CurlDll -Destination (Join-Path $Cache 'libcurl.dll') -Force
$Mappings = [ordered]@{}
foreach ($Pair in $Expected.GetEnumerator()) {
    $CacheName = if ($Pair.Key -ceq 'bin/libcurl.dll') { 'libcurl.dll' } elseif ($Pair.Key -ceq 'lib/libcurl.lib') { 'libcurl.lib' } else { $Pair.Key }
    $Destination = Resolve-File (Join-Path $Cache $CacheName) 'curl cache output'
    if ((Digest $Destination) -cne $Outputs[$Pair.Key].sha256) { throw "curl cache mapping differs: $CacheName" }
    $Mappings[$CacheName] = [ordered]@{source=$Pair.Key; sha256=Digest $Destination; bytes=(Get-Item $Destination).Length}
}

$Manifest = [ordered]@{
    schemaVersion=1; library='curl'; version=$Lock.version; configuration=$Lock.configuration
    architecture='Win32'; abi='x86'; runtime='MultiThreadedDLL (/MD)'
    source=[ordered]@{url=$Lock.url; archive=$Lock.archive; sha256=$Lock.sha256; bytes=$Lock.bytes; signingPrimaryFingerprint=$Lock.signingPrimaryFingerprint}
    dependencies=[ordered]@{
        openssl=[ordered]@{version=$OpenSslManifest.version; manifestSha256=Digest $OpenSslManifestPath; prefix=$OpenSslPrefix}
        zlib=[ordered]@{version=$Zlib.version; manifestSha256=Digest $ZlibManifest; prefix=$ZlibPrefix}
    }
    options=$Configure; buildSteps=[ordered]@{configure='passed'; compile='passed'; test='passed'; install='passed'; testTarget='tests'; testsPassed=$TestsPassed; testsReported=$TestsReported; testHost=$TestPerlRecord; certificatePatch=$PatchRecord; certificateTool=$OpenSslToolRecord; certificateProbe='passed'; log='evidence/local/windows-curl-native-output.log'; logSha256=Digest $NativeLog}
    versionOutput=$VersionOutput.Trim(); importOutput=$Imports.Trim(); outputs=$Outputs; cacheBuildwin=$Mappings
}
$Json = $Manifest | ConvertTo-Json -Depth 10
$Json | Set-Content -LiteralPath (Join-Path $Prefix 'curl-build.json') -Encoding UTF8
$Json | Set-Content -LiteralPath (Join-Path $Cache 'curl-build.json') -Encoding UTF8
$Json | Set-Content -LiteralPath (Join-Path $Evidence 'windows-curl-build.json') -Encoding UTF8
Write-Output "Built and verified curl $($Lock.version) with OpenSSL $($Lock.opensslVersion) for Win32/x86"
}
