# Focused native checks for the production environment wrapper, under PS 5.1/7.
param(
    [Parameter(Mandatory=$true)][string]$TestPerl,
    [Parameter(Mandatory=$true)][string]$Evidence
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Requires native Windows' }
$Helper = Join-Path $PSScriptRoot 'windows-curl-environment.ps1'
. $Helper
$TestPerl = (Resolve-Path -LiteralPath $TestPerl).Path
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if ($LASTEXITCODE -ne 0 -or -not $VisualStudio) { throw 'Visual Studio x86 tools unavailable' }
$Evidence = [IO.Path]::GetFullPath($Evidence)
$null = New-Item -ItemType Directory -Force -Path $Evidence
function SnapshotEnvironment {
    $Snapshot = @{}
    Get-ChildItem Env: | ForEach-Object { $Snapshot[$_.Name] = $_.Value }
    return $Snapshot
}
function AssertRestored($Before) {
    $After = SnapshotEnvironment
    if ($After.Count -ne $Before.Count) { throw 'Wrapper changed the number of process environment variables' }
    foreach ($Name in $Before.Keys) {
        if (-not $After.ContainsKey($Name) -or $After[$Name] -cne $Before[$Name]) {
            throw "Wrapper did not restore environment variable $Name"
        }
    }
}
function CompilerEnvironmentIdentity {
    if ($env:MSYS2_ARG_CONV_EXCL -cne '/D') { throw 'Required exact /D conversion exclusion absent' }
    if ($env:VSCMD_ARG_TGT_ARCH -cne 'x86' -or -not $env:INCLUDE -or -not $env:LIB) { throw 'Incomplete x86 compiler environment' }
    $Perl = (Get-Command perl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    if ($Perl -ine $TestPerl) { throw 'Wrapper resolved a different Perl' }
    $Compiler = (Get-Command cl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    $Identity = [ordered]@{perl=$Perl; compiler=$Compiler; path=$env:PATH; include=$env:INCLUDE; lib=$env:LIB;
        libpath=$env:LIBPATH; targetArchitecture=$env:VSCMD_ARG_TGT_ARCH; windowsSdkVersion=$env:WindowsSDKVersion;
        vcToolsVersion=$env:VCToolsVersion; msys2ArgConvExcl=$env:MSYS2_ARG_CONV_EXCL}
    $Bytes = [Text.Encoding]::UTF8.GetBytes(($Identity | ConvertTo-Json -Compress))
    $Hash = [Security.Cryptography.SHA256]::Create()
    try { $Digest = ([BitConverter]::ToString($Hash.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant() }
    finally { $Hash.Dispose() }
    # Only publish identity hashes and tool paths, never the process environment.
    return @{sha256=$Digest; compiler=$Compiler; perl=$Perl; targetArchitecture=$env:VSCMD_ARG_TGT_ARCH;
        windowsSdkVersion=$env:WindowsSDKVersion; vcToolsVersion=$env:VCToolsVersion; msys2ArgConvExcl=$env:MSYS2_ARG_CONV_EXCL}
}
$Original = SnapshotEnvironment
$Report = [ordered]@{schemaVersion=1; powershellVersion=$PSVersionTable.PSVersion.ToString();
    helperSha256=(Get-FileHash -LiteralPath $Helper -Algorithm SHA256).Hash.ToLowerInvariant();
    successRestored=$false; repeatedIdentityMatched=$false; throwRestored=$false; passed=$false}
try {
    # A caller-specific value must survive both success and failure, while the
    # action receives the reviewed, exact native-preprocessor setting.
    $env:MSYS2_ARG_CONV_EXCL = '/xnav-caller-sentinel'
    $Before = SnapshotEnvironment
    $script:FirstIdentity = $null
    Invoke-WindowsCurlEnvironment -VisualStudio $VisualStudio -TestPerl $TestPerl -Action {
        $script:FirstIdentity = CompilerEnvironmentIdentity
    }
    AssertRestored $Before
    $Report.successRestored = $true
    $script:SecondIdentity = $null
    Invoke-WindowsCurlEnvironment -VisualStudio $VisualStudio -TestPerl $TestPerl -Action {
        $script:SecondIdentity = CompilerEnvironmentIdentity
    }
    AssertRestored $Before
    if (-not $script:FirstIdentity -or -not $script:SecondIdentity -or
        $script:FirstIdentity.sha256 -cne $script:SecondIdentity.sha256) { throw 'Reinitialization changed the selected compiler environment' }
    $Report.repeatedIdentityMatched = $true
    $Report.compilerEnvironment = $script:SecondIdentity
    $ExpectedThrow = $false
    try {
        Invoke-WindowsCurlEnvironment -VisualStudio $VisualStudio -TestPerl $TestPerl -Action {
            $null = CompilerEnvironmentIdentity
            $env:PATH = 'xnav-action-mutated-path'
            throw 'xnav-expected-action-failure'
        }
    } catch { $ExpectedThrow = $_.Exception.Message -eq 'xnav-expected-action-failure' }
    if (-not $ExpectedThrow) { throw 'Wrapper did not preserve the action failure' }
    AssertRestored $Before
    $Report.throwRestored = $true
    $Report.passed = $true
} catch {
    $Report.error = $_.Exception.Message
    throw
} finally {
    Get-ChildItem Env: | Where-Object { -not $Original.ContainsKey($_.Name) } | ForEach-Object { [Environment]::SetEnvironmentVariable($_.Name, $null, 'Process') }
    foreach ($Name in $Original.Keys) { [Environment]::SetEnvironmentVariable($Name, $Original[$Name], 'Process') }
    $Json = $Report | ConvertTo-Json -Depth 8
    $Json | Set-Content -LiteralPath (Join-Path $Evidence 'environment-wrapper.json') -Encoding UTF8
    Write-Output $Json
}
