param(
    [Parameter(Mandatory=$true)][string]$Evidence,
    [switch]$Child
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'Native tool-facts contract test requires Windows'
}
$Evidence = [IO.Path]::GetFullPath($Evidence)
$null = New-Item -ItemType Directory -Path $Evidence -Force
$Root = Split-Path $PSScriptRoot -Parent
$Helper = Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1'
$Hook = Join-Path $PSScriptRoot 'windows-native-tool-facts.cmake'
$Producer = Join-Path $PSScriptRoot 'build-zlib-windows.ps1'
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$Encoding = New-Object Text.UTF8Encoding($false)
$Stages = Join-Path $Evidence 'stages.jsonl'
function Stage([string]$Name,[string]$State,[object]$Details=@{}) {
    $Record = [ordered]@{stage=$Name;state=$State;utc=[DateTime]::UtcNow.ToString('o');details=$Details}
    [IO.File]::AppendAllText($Stages,(($Record | ConvertTo-Json -Depth 5 -Compress) + "`n"),$Encoding)
}
function RequireFailure([scriptblock]$Action,[string]$Expected,[string]$Label) {
    $Rejected = $false
    try { & $Action } catch {
        if ($_.Exception.Message -notlike "*$Expected*") { throw }
        $Rejected = $true
    }
    if (-not $Rejected) { throw "Expected rejection was absent: $Label" }
    Stage $Label 'rejected' @{reason=$Expected}
}

$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if ($LASTEXITCODE -ne 0 -or @($VisualStudio).Count -ne 1 -or -not $VisualStudio) {
    throw 'Visual Studio 2022 x86 toolchain selection failed'
}
$VcVars = Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat'
if (-not (Test-Path -LiteralPath $VcVars -PathType Leaf)) { throw 'vcvarsall x86 initializer missing' }
$Source = Join-Path $Evidence 'source'
$Build = Join-Path $Evidence 'build'
$Facts = Join-Path $Evidence 'zlib-child-tool-facts.json'
$HookFacts = Join-Path $Build 'xnav-native-cmake-tools.txt'
$Cache = Join-Path $Build 'CMakeCache.txt'

if ($Child) {
    if ($env:VSCMD_ARG_TGT_ARCH -cne 'x86') { throw 'Contract child did not receive vcvarsall x86' }
    Stage 'child' 'begin' @{architecture=$env:VSCMD_ARG_TGT_ARCH}
    $CMake = (Get-Command cmake.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    & $CMake -S $Source -B $Build -G 'Visual Studio 17 2022' -A Win32 `
        '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL' "-DCMAKE_PROJECT_INCLUDE=$($Hook.Replace('\','/'))"
    if ($LASTEXITCODE -ne 0) { throw 'Disposable Win32 CMake configure failed' }
    if (-not (Test-Path -LiteralPath (Join-Path $Build 'zlib.vcxproj') -PathType Leaf)) {
        throw 'Disposable Win32 zlib project missing'
    }
    Stage 'cmake' 'generated' @{cache=$Cache;project='zlib.vcxproj';hookFacts=$HookFacts}
    & $Helper -Mode Capture -Kind zlib-child -Output $Facts -ProducerScript $Producer `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars $VcVars `
        -CMakeCache $Cache -CMakeHookFacts $HookFacts
    & $Helper -Mode Verify -Kind zlib-child -Output $Facts -ProducerScript $Producer `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars $VcVars `
        -CMakeCache $Cache -CMakeHookFacts $HookFacts
    Stage 'child-facts' 'capture-verify-passed'

    $PriorInclude = $env:INCLUDE
    try {
        $env:INCLUDE = "$PriorInclude;XNAV_TOOL_FACTS_NEGATIVE_CONTROL"
        RequireFailure {
            & $Helper -Mode Verify -Kind zlib-child -Output $Facts -ProducerScript $Producer `
                -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars $VcVars `
                -CMakeCache $Cache -CMakeHookFacts $HookFacts
        } 'Native tool facts changed: zlib-child' 'changed-x86-environment'
    } finally { $env:INCLUDE = $PriorInclude }

    $OriginalHook = [IO.File]::ReadAllBytes($HookFacts)
    try {
        $HookText = [IO.File]::ReadAllText($HookFacts)
        $ToolsetPattern = '(?m)^CMAKE_VS_PLATFORM_TOOLSET=v143(?=\r?$)'
        if ([regex]::Matches($HookText,$ToolsetPattern).Count -ne 1) {
            throw 'Observed v143 toolset fact missing'
        }
        $Tampered = [regex]::Replace($HookText,$ToolsetPattern,
            'CMAKE_VS_PLATFORM_TOOLSET=v999')
        [IO.File]::WriteAllText($HookFacts,$Tampered,$Encoding)
        RequireFailure {
            & $Helper -Mode Verify -Kind zlib-child -Output $Facts -ProducerScript $Producer `
                -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars $VcVars `
                -CMakeCache $Cache -CMakeHookFacts $HookFacts
        } 'Generated CMake tool facts lack exact Win32 SDK/toolset identity' 'changed-generated-toolset'
    } finally { [IO.File]::WriteAllBytes($HookFacts,$OriginalHook) }
    & $Helper -Mode Verify -Kind zlib-child -Output $Facts -ProducerScript $Producer `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -VcVars $VcVars `
        -CMakeCache $Cache -CMakeHookFacts $HookFacts
    Stage 'child-facts' 'restored-verify-passed'
    [IO.File]::WriteAllText((Join-Path $Evidence 'child-result.json'),
        (@{status='passed';kind='zlib-child';testOnly=$true} | ConvertTo-Json -Compress),$Encoding)
    return
}

$Status = 'failed'
$Failure = $null
try {
    Stage 'contract' 'begin' @{testOnly=$true;sourceCommit=(git -C $Root rev-parse HEAD)}
    $ChildResult = Join-Path $Evidence 'child-result.json'
    if (Test-Path -LiteralPath $ChildResult) { Remove-Item -LiteralPath $ChildResult -Force }
    $null = New-Item -ItemType Directory -Path $Source -Force
    [IO.File]::WriteAllText((Join-Path $Source 'CMakeLists.txt'),
        "cmake_minimum_required(VERSION 3.20)`nproject(native-tool-facts-smoke LANGUAGES C)`nadd_library(zlib SHARED stub.c)`n",$Encoding)
    [IO.File]::WriteAllText((Join-Path $Source 'stub.c'),"int xnav_tool_facts_smoke(void) { return 1; }`n",$Encoding)
    $RunChild = Join-Path $Evidence 'run-child.cmd'
    foreach ($Path in @($VcVars,$PSCommandPath,$Evidence)) {
        if ($Path.Contains('%') -or $Path.Contains('"') -or $Path.Contains([char]10) -or $Path.Contains([char]13)) {
            throw 'Unsupported character in disposable native contract path'
        }
    }
    [IO.File]::WriteAllLines($RunChild,@(
        '@echo off','setlocal DisableDelayedExpansion',
        "call `"$VcVars`" x86 || exit /b 1",
        "powershell.exe -NoProfile -ExecutionPolicy Bypass -File `"$PSCommandPath`" -Evidence `"$Evidence`" -Child || exit /b 1"
    ),$Encoding)
    Stage 'child' 'before-start' @{batch=$RunChild}
    & cmd.exe @('/d','/s','/c',"`"$RunChild`"") 2>&1 | Tee-Object -FilePath (Join-Path $Evidence 'child-output.log')
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath (Join-Path $Evidence 'child-result.json') -PathType Leaf)) {
        throw 'Disposable x86 child contract failed'
    }
    Stage 'child' 'passed'

    $Dumpbin = Get-ChildItem -Path (Join-Path $VisualStudio 'VC/Tools/MSVC') -Filter dumpbin.exe -Recurse -File |
        Where-Object { $_.FullName -match '\\Host(?:x64|x86)\\x86\\dumpbin\.exe$' } | Select-Object -First 1
    if (-not $Dumpbin) { throw 'Parent x86 dumpbin selection failed' }
    $ParentFacts = Join-Path $Evidence 'zlib-parent-tool-facts.json'
    & $Helper -Mode Capture -Kind zlib-parent -Output $ParentFacts -ProducerScript $Producer `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -Dumpbin $Dumpbin.FullName
    & $Helper -Mode Verify -Kind zlib-parent -Output $ParentFacts -ProducerScript $Producer `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -Dumpbin $Dumpbin.FullName
    Stage 'parent-facts' 'capture-verify-passed'
    $PriorInclude = $env:INCLUDE
    try {
        $env:INCLUDE = "$PriorInclude;XNAV_TOOL_FACTS_NEGATIVE_CONTROL"
        RequireFailure {
            & $Helper -Mode Verify -Kind zlib-parent -Output $ParentFacts -ProducerScript $Producer `
                -Vswhere $Vswhere -VisualStudio $VisualStudio -Dumpbin $Dumpbin.FullName
        } 'Native tool facts changed: zlib-parent' 'changed-parent-environment'
    } finally { $env:INCLUDE = $PriorInclude }
    & $Helper -Mode Verify -Kind zlib-parent -Output $ParentFacts -ProducerScript $Producer `
        -Vswhere $Vswhere -VisualStudio $VisualStudio -Dumpbin $Dumpbin.FullName
    Stage 'parent-facts' 'restored-verify-passed'
    $Status = 'passed'
} catch {
    $Failure = $_.Exception.Message
    Stage 'contract' 'failed' @{reason=$Failure}
} finally {
    [IO.File]::WriteAllText((Join-Path $Evidence 'summary.json'),
        ([ordered]@{schemaVersion=1;status=$Status;failure=$Failure;testOnly=$true;
            childKind='zlib-child';parentKind='zlib-parent';sourceCommit=(git -C $Root rev-parse HEAD);
            stages='stages.jsonl'} | ConvertTo-Json -Depth 4),$Encoding)
}
if ($Status -ne 'passed') { throw "Native tool-facts contract failed; evidence=$Evidence; reason=$Failure" }
Write-Output "Disposable Win32 tool-facts contract passed; evidence=$Evidence"
