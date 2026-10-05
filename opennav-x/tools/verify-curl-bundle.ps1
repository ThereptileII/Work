# Consumer-only probe. Original producer scripts, native receipts and manifests
# remain unchanged; no dependency configure/build/test/install happens here.
param(
    [Parameter(Mandatory=$true)][string]$DependencyBundle,
    [Parameter(Mandatory=$true)][string]$DependencyBundleProvenance,
    [Parameter(Mandatory=$true)][string]$Python,
    [string]$RuntimeEvidenceDirectory = ''
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Root = Split-Path $PSScriptRoot -Parent
$Evidence = Join-Path $Root 'evidence/local'
# Only fresh consumer observations may move; original tool-fact reads below
# remain bound to evidence/local and the authenticated producer inventory.
$RuntimeEvidence = if ($RuntimeEvidenceDirectory) { $RuntimeEvidenceDirectory } else { $Evidence }
$RuntimeReport = Join-Path $RuntimeEvidence ("openssl-consumer-runtime-" + [guid]::NewGuid().ToString('N') + '.json')
if (Test-Path -LiteralPath $RuntimeReport) { throw 'Consumer runtime observation must be fresh' }
$OpenSslPrefix = Join-Path $Root 'build/windows-openssl-3.5.9/install'
$ZlibPrefix = Join-Path $Root 'build/windows-zlib-1.3.2/install'
$Build = Join-Path $Root 'build/windows-curl-8.22.0/build'
$Prefix = Join-Path $Root 'build/windows-curl-8.22.0/install'
# The original producer remains the authority for the captured tool facts.
$ProducerScript = Join-Path $PSScriptRoot 'build-curl-windows.ps1'
. (Join-Path $PSScriptRoot 'windows-curl-environment.ps1')
function Checked-Python([string[]]$Arguments) {
    & $Python @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Consumer dependency verification failed: $LASTEXITCODE" }
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
$VerificationArguments = @((Join-Path $PSScriptRoot 'windows_dependency_bundle.py'),
    'verify-restored', '--root', $Root, '--bundle', $DependencyBundle,
    '--provenance', $DependencyBundleProvenance)
# This verifies the authenticated complete SDK, original manifests, PE ABI,
# source archives, successful producer tests and every original file hash.
Checked-Python $VerificationArguments
Checked-Python @((Join-Path $PSScriptRoot 'verify-openssl-runtime.py'),
    '--manifest', (Join-Path $OpenSslPrefix 'openssl-build.json'),
    '--executable', (Join-Path $OpenSslPrefix 'bin/openssl.exe'),
    '--output', $RuntimeReport)

# Keep tool selection and environment ordering identical to the original
# build-curl-windows.ps1 VerifyToolFactsOnly branch, including its parent PATH.
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$Vswhere = Resolve-File $Vswhere 'Visual Studio locator'
$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if ($LASTEXITCODE -ne 0 -or -not $VisualStudio) { throw 'Licensed MSVC x86 toolchain missing' }
$DumpbinCandidates = @(Get-ChildItem -LiteralPath (Join-Path $VisualStudio 'VC/Tools/MSVC') -Directory |
    Sort-Object Name -Descending | ForEach-Object { Join-Path $_.FullName 'bin/Hostx64/x86/dumpbin.exe' } |
    Where-Object { Test-Path -LiteralPath $_ -PathType Leaf })
if (-not $DumpbinCandidates.Count) { throw 'MSVC Win32 dumpbin missing' }
$Dumpbin = $DumpbinCandidates[0]
$ToolFacts = Resolve-File (Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1') 'Native tool-facts helper'
$Facts = Join-Path $Evidence 'windows-curl-parent-tool-facts.json'
foreach ($Tool in @('cmake.exe','perl.exe')) {
    if (-not (Get-Command $Tool -CommandType Application -ErrorAction SilentlyContinue)) { throw "curl build prerequisite missing: $Tool" }
}
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
    $BuiltManifest = Get-Content -LiteralPath (Join-Path $Prefix 'curl-build.json') -Raw | ConvertFrom-Json
    $BuiltHost = $BuiltManifest.buildSteps.testHost
    if ($BuiltHost.path -ine $TestPerlRecord.path -or $BuiltHost.sha256 -cne $TestPerlRecord.sha256 -or
        $BuiltHost.bytes -ne $TestPerlRecord.bytes -or $BuiltHost.os -cne $TestPerlRecord.os -or
        $BuiltHost.runtimePath -ine $TestPerlRecord.runtimePath -or
        $BuiltHost.runtimeSha256 -cne $TestPerlRecord.runtimeSha256 -or
        $BuiltHost.runtimeBytes -ne $TestPerlRecord.runtimeBytes) {
        throw 'Live curl test host differs from the successful producer manifest'
    }
    $env:PATH = "$(Join-Path $Build 'lib/Release');$(Join-Path $OpenSslPrefix 'bin');$(Join-Path $ZlibPrefix 'bin');$env:PATH"
    & $ToolFacts -Mode Verify -Kind curl-parent -Output $Facts -ProducerScript $ProducerScript `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars (Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat') -Dumpbin $Dumpbin `
        -CMakeCache (Join-Path $Build 'CMakeCache.txt') -CMakeHookFacts (Join-Path $Build 'xnav-native-cmake-tools.txt') `
        -CurlTestPerl $TestPerl
}
Checked-Python $VerificationArguments
Write-Output 'Reprobed original curl producer tool facts and separately verified consumer OpenSSL runtime'
