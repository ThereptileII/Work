param(
    [Parameter(Mandatory=$true)][string]$TestPerl,
    [Parameter(Mandatory=$true)][string]$NativePerl,
    [Parameter(Mandatory=$true)][string]$Evidence
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'Curl test-host binding check requires native Windows'
}
$Root = Split-Path $PSScriptRoot -Parent
$TestPerl = (Resolve-Path -LiteralPath $TestPerl).Path
$NativePerl = (Resolve-Path -LiteralPath $NativePerl).Path
if ($TestPerl -ieq $NativePerl) { throw 'Native and curl test Perl must differ' }
$Evidence = [IO.Path]::GetFullPath($Evidence)
$null = New-Item -ItemType Directory -Force -Path $Evidence
$Source = Join-Path $Evidence 'cmake-source'
$Build = Join-Path $Evidence 'cmake-build'
$null = New-Item -ItemType Directory -Force -Path $Source
[IO.File]::WriteAllText((Join-Path $Source 'CMakeLists.txt'), @'
cmake_minimum_required(VERSION 3.25)
project(curl_test_host_binding C)
find_package(Perl REQUIRED)
add_library(libcurl_shared SHARED marker.c)
'@)
[IO.File]::WriteAllText((Join-Path $Source 'marker.c'), 'int curl_test_host_marker(void) { return 1; }')
$Hook = Join-Path $PSScriptRoot 'windows-native-tool-facts.cmake'
& cmake.exe -S $Source -B $Build -G 'Visual Studio 17 2022' -A Win32 `
    "-DCMAKE_PROJECT_INCLUDE=$Hook" "-DPERL_EXECUTABLE:FILEPATH=$TestPerl"
if ($LASTEXITCODE -ne 0) { throw 'Native curl test-host CMake fixture failed to configure' }
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if ($LASTEXITCODE -ne 0 -or -not $VisualStudio) { throw 'Visual Studio x86 instance missing' }
$Dumpbin = @(Get-ChildItem -LiteralPath (Join-Path $VisualStudio 'VC/Tools/MSVC') -Directory |
    Sort-Object Name -Descending | ForEach-Object { Join-Path $_.FullName 'bin/Hostx64/x86/dumpbin.exe' } |
    Where-Object { Test-Path -LiteralPath $_ -PathType Leaf } | Select-Object -First 1)[0]
if (-not $Dumpbin) { throw 'Win32 dumpbin missing' }
$Facts = Join-Path $Evidence 'curl-test-host-facts.json'
$ToolFacts = Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1'
$BeforePath = $env:PATH
try {
    $env:PATH = "$(Split-Path $NativePerl -Parent);$(Split-Path $TestPerl -Parent);$env:PATH"
    $Arguments = @{
        Kind='curl-parent'; Output=$Facts; ProducerScript=$PSCommandPath
        Vswhere=$Vswhere; VisualStudio=$VisualStudio; Dumpbin=$Dumpbin
        CMakeCache=(Join-Path $Build 'CMakeCache.txt')
        CMakeHookFacts=(Join-Path $Build 'xnav-native-cmake-tools.txt')
    }
    & $ToolFacts -Mode Capture @Arguments -CurlTestPerl $TestPerl
    & $ToolFacts -Mode Verify @Arguments -CurlTestPerl $TestPerl
    $Rejected = $false
    try { & $ToolFacts -Mode Verify @Arguments -CurlTestPerl $NativePerl }
    catch { $Rejected = $_.Exception.Message -match 'different test Perl|MSYS2 curl test runtime missing' }
    if (-not $Rejected) { throw 'Wrong Perl did not fail the test-host identity check' }
    $CachePath = Join-Path $Build 'CMakeCache.txt'
    $OriginalCache = [IO.File]::ReadAllBytes($CachePath)
    try {
        $CacheText = [IO.File]::ReadAllText($CachePath)
        $ChangedCache = [regex]::Replace($CacheText, '(?m)^PERL_EXECUTABLE:FILEPATH=.*$',
            [System.Text.RegularExpressions.MatchEvaluator]{ param($Match) "PERL_EXECUTABLE:FILEPATH=$NativePerl" })
        if ($ChangedCache -ceq $CacheText) { throw 'CMake test Perl field was not changed' }
        [IO.File]::WriteAllText($CachePath, $ChangedCache)
        $Rejected = $false
        try { & $ToolFacts -Mode Verify @Arguments -CurlTestPerl $TestPerl }
        catch { $Rejected = $_.Exception.Message -match 'different test Perl' }
        if (-not $Rejected) { throw 'Mismatched CMake Perl was not rejected' }
    } finally { [IO.File]::WriteAllBytes($CachePath, $OriginalCache) }
    & $ToolFacts -Mode Verify @Arguments -CurlTestPerl $TestPerl
} finally { $env:PATH = $BeforePath }
Write-Output "Native curl test-host CMake binding passed; evidence=$Facts"
