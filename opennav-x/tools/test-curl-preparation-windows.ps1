param(
    [Parameter(Mandatory=$true)][string]$Evidence
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'Curl preparation reproduction requires native Windows'
}
$Root = Split-Path $PSScriptRoot -Parent
$LockPath = Join-Path $PSScriptRoot 'windows-curl.lock.json'
$Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
if ($Lock.version -cne '8.22.0' -or $Lock.archive -cne 'curl-8.22.0.tar.xz' -or
    $Lock.sha256 -cne 'f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7' -or
    $Lock.bytes -ne 2953092) {
    throw 'Curl preparation reproduction requires the reviewed 8.22.0 archive lock'
}
$Evidence = [IO.Path]::GetFullPath($Evidence)
$null = New-Item -ItemType Directory -Force -Path $Evidence
$Stages = Join-Path $Evidence 'stages.jsonl'
$Summary = Join-Path $Evidence 'summary.json'
$Encoding = New-Object Text.UTF8Encoding($false)
if (Test-Path -LiteralPath $Stages) { Remove-Item -LiteralPath $Stages -Force }

function Digest([string]$Path) {
    (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}
function Stage([string]$Name,[string]$Event,[object]$Details=@{}) {
    $Record = [ordered]@{stage=$Name;event=$Event;utc=[DateTime]::UtcNow.ToString('o');details=$Details}
    [IO.File]::AppendAllText($Stages,(($Record | ConvertTo-Json -Depth 6 -Compress) + "`n"),$Encoding)
}
function SelectedTool([string]$Name) {
    $Command = Get-Command $Name -CommandType Application -ErrorAction Stop | Select-Object -First 1
    if (-not $Command -or -not (Test-Path -LiteralPath $Command.Source -PathType Leaf)) {
        throw "Selected Windows tool is unavailable: $Name"
    }
    $Path = (Resolve-Path -LiteralPath $Command.Source).Path
    [ordered]@{path=$Path;sha256=Digest $Path;bytes=(Get-Item -LiteralPath $Path).Length}
}
function Quote([string]$Value) {
    if ($Value.Contains('"') -or $Value.Contains([char]10) -or $Value.Contains([char]13)) {
        throw 'Unsafe character in a disposable preparation path'
    }
    '"' + $Value + '"'
}
function RunBounded([string]$Name,[string]$Program,[string[]]$Arguments,[int]$TimeoutSeconds,
                    [string]$WorkingDirectory='') {
    if ($TimeoutSeconds -lt 1 -or $TimeoutSeconds -gt 300) { throw 'Unsupported process deadline' }
    $StartInfo = New-Object Diagnostics.ProcessStartInfo
    $StartInfo.FileName = $Program
    $StartInfo.Arguments = ($Arguments -join ' ')
    if ($WorkingDirectory) { $StartInfo.WorkingDirectory = $WorkingDirectory }
    $StartInfo.UseShellExecute = $false
    $StartInfo.CreateNoWindow = $true
    $StartInfo.RedirectStandardOutput = $true
    $StartInfo.RedirectStandardError = $true
    $Process = New-Object Diagnostics.Process
    $Process.StartInfo = $StartInfo
    Stage $Name 'before-start' @{program=$Program;arguments=$Arguments;deadlineSeconds=$TimeoutSeconds}
    $Clock = [Diagnostics.Stopwatch]::StartNew()
    try {
        if (-not $Process.Start()) { throw "Could not start $Name" }
        Stage $Name 'started' @{pid=$Process.Id}
        $StdoutTask = $Process.StandardOutput.ReadToEndAsync()
        $StderrTask = $Process.StandardError.ReadToEndAsync()
        $TimedOut = -not $Process.WaitForExit($TimeoutSeconds * 1000)
        if ($TimedOut) {
            Stage $Name 'deadline' @{pid=$Process.Id;elapsedMs=$Clock.ElapsedMilliseconds}
            # Kill only this disposable child; no machine-wide process cleanup.
            $Process.Kill()
            if (-not $Process.WaitForExit(10000)) { throw "Owned $Name process did not exit after kill" }
        }
        $Stdout = $StdoutTask.GetAwaiter().GetResult()
        $Stderr = $StderrTask.GetAwaiter().GetResult()
        if ($Stdout.Length -gt 262144 -or $Stderr.Length -gt 262144) {
            throw "$Name output exceeds bounded evidence size"
        }
        [IO.File]::WriteAllText((Join-Path $Evidence "$Name.stdout.txt"),$Stdout,$Encoding)
        [IO.File]::WriteAllText((Join-Path $Evidence "$Name.stderr.txt"),$Stderr,$Encoding)
        $Result = [ordered]@{exitCode=$Process.ExitCode;timedOut=$TimedOut;
            elapsedMs=$Clock.ElapsedMilliseconds;pid=$Process.Id}
        Stage $Name 'finished' $Result
        return $Result
    } finally {
        $Clock.Stop()
        $Process.Dispose()
    }
}

$Status = 'failed'
$Failure = $null
$Results = [ordered]@{}
try {
    Stage 'context' 'begin' @{scriptSha256=Digest $PSCommandPath;lockSha256=Digest $LockPath;
        osVersion=[Environment]::OSVersion.VersionString;processorArchitecture=$env:PROCESSOR_ARCHITECTURE}
    # In the integrated producer chain, OpenSSL ran earlier in this same
    # PowerShell process and prepended these existing host-tool directories.
    # Reproduce that lookup order without installing or running OpenSSL.
    $Prepended = @()
    foreach ($Directory in @('C:\Program Files\NASM','C:\Strawberry\perl\bin')) {
        if (Test-Path -LiteralPath $Directory -PathType Container) {
            $env:PATH = "$Directory;$env:PATH"
            $Prepended += $Directory
        }
    }
    $Results.pathPrepend = $Prepended
    Stage 'path' 'producer-order' @{prepended=$Prepended}
    $Tar = SelectedTool 'tar.exe'
    $Curl = SelectedTool 'curl.exe'
    $CMake = SelectedTool 'cmake.exe'
    $Results.tools=[ordered]@{tar=$Tar;curl=$Curl;cmake=$CMake}
    Stage 'tools' 'selected' @{tar=$Tar;curl=$Curl;cmake=$CMake}
    $TarVersion = RunBounded 'tar-version' $Tar.path @('--version') 15
    if ($TarVersion.timedOut -or $TarVersion.exitCode -ne 0) { throw 'Selected tar.exe version probe failed' }
    $ArchiveDir = Join-Path $Root 'build/dependency-downloads'
    $null = New-Item -ItemType Directory -Force -Path $ArchiveDir
    $Archive = Join-Path $ArchiveDir $Lock.archive
    if (-not (Test-Path -LiteralPath $Archive -PathType Leaf) -or
        (Get-Item -LiteralPath $Archive).Length -ne $Lock.bytes -or
        (Digest $Archive) -cne $Lock.sha256) {
        $Download = RunBounded 'curl-download' $Curl.path @('--fail','--location','--silent',
            '--show-error','--retry','3','--retry-all-errors','--connect-timeout','20',
            '--max-time','300','--output',(Quote $Archive),(Quote $Lock.url)) 300
        if ($Download.timedOut -or $Download.exitCode -ne 0) { throw 'Locked curl archive download failed' }
    }
    $Observed = [ordered]@{path=$Archive;bytes=(Get-Item -LiteralPath $Archive).Length;
        sha256=Digest $Archive}
    $Results.archive=$Observed
    if ($Observed.bytes -ne $Lock.bytes -or $Observed.sha256 -cne $Lock.sha256) {
        throw 'Downloaded curl archive differs from the reviewed lock'
    }
    Stage 'archive' 'verified' $Observed
    $Extraction = Join-Path $Evidence 'tar-extraction'
    if (Test-Path -LiteralPath $Extraction) { Remove-Item -LiteralPath $Extraction -Recurse -Force }
    $null = New-Item -ItemType Directory -Force -Path $Extraction
    $TarResult = RunBounded 'tar-extract' $Tar.path @('-xf',(Quote $Archive),'-C',(Quote $Extraction)) 120
    $Results.tarExtraction=$TarResult
    if ($TarResult.timedOut -or $TarResult.exitCode -ne 0) {
        # A second, separate directory is diagnostic only. It cannot make the
        # primary tar reproduction pass or alter the producer selection.
        $Comparison = Join-Path $Evidence 'cmake-extraction'
        if (Test-Path -LiteralPath $Comparison) { Remove-Item -LiteralPath $Comparison -Recurse -Force }
        $null = New-Item -ItemType Directory -Force -Path $Comparison
        $Results.cmakeTarComparison = RunBounded 'cmake-tar-comparison' $CMake.path @('-E','tar','xf',
            (Quote $Archive)) 120 $Comparison
        throw 'Selected tar.exe did not extract the locked curl archive'
    }
    $Source = Join-Path $Extraction "curl-$($Lock.version)"
    $CMakeLists = Join-Path $Source 'CMakeLists.txt'
    if (-not (Test-Path -LiteralPath $CMakeLists -PathType Leaf)) {
        throw 'Tar exited successfully without the expected curl source root'
    }
    Stage 'source' 'ready' @{cmakeListsSha256=Digest $CMakeLists;
        sourceRoot=$Source;fileCount=@(Get-ChildItem -LiteralPath $Source -File -Recurse).Count}
    $CMakeVersion = RunBounded 'cmake-version' $CMake.path @('--version') 15
    if ($CMakeVersion.timedOut -or $CMakeVersion.exitCode -ne 0) {
        throw 'CMake was unavailable after source preparation'
    }
    Stage 'cmake-boundary' 'reached' @{firstSourceFile=$CMakeLists}
    $Status = 'passed'
} catch {
    $Failure = $_.Exception.Message
    Stage 'reproduction' 'failed' @{reason=$Failure}
} finally {
    $Report = [ordered]@{schemaVersion=1;status=$Status;failure=$Failure;
        sourceCommit=(git -C $Root rev-parse HEAD);results=$Results;
        stages='stages.jsonl'}
    [IO.File]::WriteAllText($Summary,($Report | ConvertTo-Json -Depth 10),$Encoding)
}
if ($Status -ne 'passed') { throw "Curl preparation reproduction failed; evidence=$Summary; reason=$Failure" }
Write-Output "Curl preparation reached CMake boundary; evidence=$Summary"
