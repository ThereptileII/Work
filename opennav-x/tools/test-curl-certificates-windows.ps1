param([Parameter(Mandatory=$true)][string]$Evidence)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'Curl certificate probe requires native Windows'
}

$Root = Split-Path $PSScriptRoot -Parent
$LockPath = Join-Path $PSScriptRoot 'windows-curl.lock.json'
$Patch = Join-Path $PSScriptRoot 'patch-curl-test-openssl.py'
$Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
if ($Lock.version -cne '8.22.0' -or $Lock.archive -cne 'curl-8.22.0.tar.xz' -or
    $Lock.bytes -ne 2953092 -or
    $Lock.sha256 -cne 'f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7') {
    throw 'Curl certificate probe requires the reviewed source archive lock'
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
        throw "Selected host tool missing: $Name"
    }
    $Path = (Resolve-Path -LiteralPath $Command.Source).Path
    [ordered]@{path=$Path;sha256=Digest $Path;bytes=(Get-Item -LiteralPath $Path).Length}
}
function Quote([string]$Value) {
    if ($Value.Contains('"') -or $Value.Contains([char]10) -or $Value.Contains([char]13)) {
        throw 'Unsupported character in disposable probe argument'
    }
    '"' + $Value + '"'
}
function RunBounded([string]$Name,[string]$Program,[string[]]$Arguments,[int]$TimeoutSeconds,
                    [string]$WorkingDirectory='') {
    if ($TimeoutSeconds -lt 1 -or $TimeoutSeconds -gt 300) { throw 'Unsupported process deadline' }
    # Let cmd redirect child handles directly to files. Buffered ReadToEndAsync
    # loses all partial evidence when a descendant keeps a pipe open.
    # Raw output is excluded by the workflow's top-level artifact globs.
    $RawDirectory = Join-Path $Evidence 'raw-output'
    $null = New-Item -ItemType Directory -Force -Path $RawDirectory
    $StdoutPath = Join-Path $RawDirectory "$Name.stdout.txt"
    $StderrPath = Join-Path $RawDirectory "$Name.stderr.txt"
    $CommandPath = Join-Path $Evidence "$Name.cmd"
    foreach ($Value in @($Program,$CommandPath,$StdoutPath,$StderrPath) + $Arguments) {
        if ($Value -match '[%!?&|<>\r\n]') { throw 'Unsupported disposable command character' }
    }
    $Command = '@echo off' + "`r`n" + (Quote $Program) + ' ' + ($Arguments -join ' ') +
        ' 1>' + (Quote $StdoutPath) + ' 2>' + (Quote $StderrPath) + "`r`nexit /b %errorlevel%`r`n"
    [IO.File]::WriteAllText($CommandPath,$Command,$Encoding)
    $StartInfo = New-Object Diagnostics.ProcessStartInfo
    $StartInfo.FileName = Join-Path $env:SystemRoot 'System32/cmd.exe'
    $StartInfo.Arguments = '/d /s /c ""' + $CommandPath + '""'
    if ($WorkingDirectory) { $StartInfo.WorkingDirectory = $WorkingDirectory }
    $StartInfo.UseShellExecute = $false
    $StartInfo.CreateNoWindow = $true
    $Process = New-Object Diagnostics.Process
    $Process.StartInfo = $StartInfo
    Stage $Name 'before-start' @{program=$Program;arguments=$Arguments;deadlineSeconds=$TimeoutSeconds}
    $Clock = [Diagnostics.Stopwatch]::StartNew()
    try {
        if (-not $Process.Start()) { throw "Could not start $Name" }
        Stage $Name 'started' @{pid=$Process.Id}
        $TimedOut = -not $Process.WaitForExit($TimeoutSeconds * 1000)
        if ($TimedOut) {
            Stage $Name 'deadline' @{pid=$Process.Id;elapsedMs=$Clock.ElapsedMilliseconds}
            # Capture only this disposable invocation and its descendants, never
            # unrelated runner processes or environment/credentials.
            try {
                $All = @(Get-CimInstance Win32_Process -OperationTimeoutSec 5 -ErrorAction Stop)
                $Owned = @([uint32]$Process.Id)
                $Records = @()
                for ($Depth = 0; $Depth -lt 16; $Depth++) {
                    $Children = @($All | Where-Object {
                        $_.ParentProcessId -in $Owned -and $_.ProcessId -notin $Owned
                    })
                    if (-not $Children.Count) { break }
                    $Owned += @($Children | ForEach-Object { [uint32]$_.ProcessId })
                    if ($Owned.Count -gt 128) { throw 'Owned process tree exceeds evidence bound' }
                }
                $Records = @($All | Where-Object { $_.ProcessId -in $Owned } |
                    Select-Object ProcessId,ParentProcessId,Name,ExecutablePath,CommandLine)
                Stage $Name 'owned-processes-at-deadline' @{processes=$Records}
            } catch { Stage $Name 'process-snapshot-failed' @{reason=$_.Exception.Message} }
            $Process.Kill($true)
            if (-not $Process.WaitForExit(10000)) { throw "Owned $Name process did not exit after kill" }
        }
        # File redirection has no parent pipe-drain dependency. Even on timeout
        # the bytes already written remain available to the artifact uploader.
        foreach ($OutputPath in @($StdoutPath,$StderrPath)) {
            if ((Get-Item -LiteralPath $OutputPath).Length -gt 262144) {
                throw "$Name output exceeds bounded evidence size"
            }
        }
        $Stdout = [IO.File]::ReadAllText($StdoutPath)
        $Stderr = [IO.File]::ReadAllText($StderrPath)
        if ($Stdout.Length -gt 262144 -or $Stderr.Length -gt 262144) {
            throw "$Name output exceeds bounded evidence size"
        }
        # genserv prints the complete PATH on its expected failure. Keep the
        # failure marker, while omitting unrelated runner environment paths.
        $RetainedOut = [regex]::Replace($Stdout,'(?m)^PATH used: .*$', 'PATH used: [redacted]')
        [IO.File]::WriteAllText((Join-Path $Evidence "$Name.stdout.txt"),$RetainedOut,$Encoding)
        [IO.File]::WriteAllText((Join-Path $Evidence "$Name.stderr.txt"),$Stderr,$Encoding)
        $Result = [ordered]@{exitCode=$Process.ExitCode;timedOut=$TimedOut;
            elapsedMs=$Clock.ElapsedMilliseconds;pid=$Process.Id;stdout=$Stdout;stderr=$Stderr}
        Stage $Name 'finished' @{exitCode=$Result.exitCode;timedOut=$TimedOut;elapsedMs=$Result.elapsedMs;pid=$Result.pid}
        return $Result
    } finally {
        $Clock.Stop()
        $Process.Dispose()
        # Preserve bounded, sanitized partial output even when process cleanup
        # itself fails. Never upload raw PATH output from the expected failure.
        foreach ($StreamName in @('stdout','stderr')) {
            $RawPath = Join-Path $RawDirectory "$Name.$StreamName.txt"
            if ((Test-Path -LiteralPath $RawPath -PathType Leaf) -and
                (Get-Item -LiteralPath $RawPath).Length -le 262144) {
                $Retained = [regex]::Replace([IO.File]::ReadAllText($RawPath),
                    '(?m)^PATH used: .*$', 'PATH used: [redacted]')
                [IO.File]::WriteAllText((Join-Path $Evidence "$Name.$StreamName.txt"),$Retained,$Encoding)
            }
        }
    }
}
function RequireSuccess([object]$Result,[string]$Name) {
    if ($Result.timedOut -or $Result.exitCode -ne 0) { throw "$Name did not finish successfully" }
}
function RecordFile([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "Generated certificate file missing: $Path" }
    [ordered]@{bytes=(Get-Item -LiteralPath $Path).Length;sha256=Digest $Path}
}

$Status = 'failed'
$Failure = $null
$Results = [ordered]@{}
try {
    Stage 'context' 'begin' @{sourceCommit=(git -C $Root rev-parse HEAD);
        githubSha=$env:GITHUB_SHA;scriptSha256=Digest $PSCommandPath;
        lockSha256=Digest $LockPath;patchHelperSha256=Digest $Patch}
    if ($env:GITHUB_SHA -and (git -C $Root rev-parse HEAD) -cne $env:GITHUB_SHA) {
        throw 'Checked-out source differs from GITHUB_SHA'
    }
    $Curl = SelectedTool 'curl.exe'
    $CMake = SelectedTool 'cmake.exe'
    $Perl = SelectedTool 'perl.exe'
    $OpenSsl = SelectedTool 'openssl.exe'
    $Python = SelectedTool 'python.exe'
    $Results.tools = [ordered]@{curl=$Curl;cmake=$CMake;perl=$Perl;openssl=$OpenSsl;python=$Python}
    Stage 'tools' 'selected' $Results.tools
    $OpenSslDir = Split-Path $OpenSsl.path -Parent
    $env:PATH = "$OpenSslDir;$env:PATH"
    if ((Get-Command openssl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source -cne $OpenSsl.path) {
        throw 'Host openssl.exe is not first on PATH'
    }
    foreach ($Directory in ($env:PATH -split ';')) {
        if ($Directory -and (Test-Path -LiteralPath (Join-Path $Directory 'openssl') -PathType Leaf)) {
            throw 'A literal openssl filename prevents isolating the upstream lookup defect'
        }
    }
    $Version = RunBounded 'host-openssl-version' $OpenSsl.path @('version','-a') 20
    RequireSuccess $Version 'host openssl.exe version'
    if ($Version.stdout -notmatch '(?m)^OpenSSL\s+\d+\.\d+') { throw 'Host OpenSSL version response is unrecognized' }
    $Results.hostOpenSslVersion = ($Version.stdout -split '\r?\n')[0]

    $Archive = Join-Path $Evidence $Lock.archive
    if (-not (Test-Path -LiteralPath $Archive -PathType Leaf) -or
        (Get-Item -LiteralPath $Archive).Length -ne $Lock.bytes -or (Digest $Archive) -cne $Lock.sha256) {
        $Download = RunBounded 'curl-download' $Curl.path @('--fail','--location','--silent',
            '--show-error','--retry','3','--retry-all-errors','--connect-timeout','20',
            '--max-time','300','--output',(Quote $Archive),(Quote $Lock.url)) 300
        RequireSuccess $Download 'locked curl download'
    }
    $Results.archive = RecordFile $Archive
    if ($Results.archive.bytes -ne $Lock.bytes -or $Results.archive.sha256 -cne $Lock.sha256) {
        throw 'Curl archive differs from reviewed lock'
    }
    Stage 'archive' 'verified' $Results.archive
    $Work = Join-Path $Evidence ('disposable-' + [guid]::NewGuid().ToString('N'))
    $null = New-Item -ItemType Directory -Path $Work
    $Extract = Join-Path $Work 'extracted'
    $null = New-Item -ItemType Directory -Force -Path $Extract
    $ExtractResult = RunBounded 'cmake-extract' $CMake.path @('-E','chdir',(Quote $Extract),
        (Quote $CMake.path),'-E','tar','xf',(Quote $Archive)) 120
    RequireSuccess $ExtractResult 'CMake source extraction'
    $Source = Join-Path $Extract "curl-$($Lock.version)"
    $Genserv = Join-Path $Source 'tests/certs/genserv.pl'
    $OriginalHash = 'd737cbe77e23e275b4fcfcec36e62d49d1d59d9d9fd0013a428b7143ee75c982'
    if (-not (Test-Path -LiteralPath $Genserv -PathType Leaf) -or (Digest $Genserv) -cne $OriginalHash) {
        throw 'Extracted curl certificate generator differs from locked original'
    }
    $Before = Join-Path $Work 'original-output'
    $After = Join-Path $Work 'patched-output'
    $null = New-Item -ItemType Directory -Force -Path $Before,$After
    if (-not (Test-Path -LiteralPath (Join-Path $Source 'tests/certs/test-localhost.prm') -PathType Leaf)) {
        throw 'Locked localhost certificate config is missing'
    }
    $Original = RunBounded 'genserv-original' $Perl.path @((Quote $Genserv),'test','test-localhost.prm') 40 $Before
    if ($Original.timedOut -or $Original.exitCode -eq 0 -or
        $Original.stdout -notmatch '(?m)^PATH used: ' -or
        $Original.stderr -notmatch "Missing or unsupported 'openssl' tool") {
        throw 'Original genserv did not reproduce the exact openssl filename lookup failure'
    }
    foreach ($Name in @('test-ca.cacert','test-ca.key','test-localhost.crt','test-localhost.key')) {
        if (Test-Path -LiteralPath (Join-Path $Before $Name)) {
            throw 'Original genserv created a certificate despite the expected lookup failure'
        }
    }
    Stage 'original-generator' 'expected-lookup-failure' @{exitCode=$Original.exitCode}

    $PatchReceipt = Join-Path $Evidence 'genserv-patch.json'
    $Patched = RunBounded 'patch-genserv' $Python.path @((Quote $Patch),'--source',
        (Quote $Genserv),'--evidence',(Quote $PatchReceipt)) 20
    RequireSuccess $Patched 'exact curl test-source patch'
    $PatchRecord = Get-Content -LiteralPath $PatchReceipt -Raw | ConvertFrom-Json
    if ($PatchRecord.state -cne 'patched' -or $PatchRecord.beforeSha256 -cne $OriginalHash -or
        $PatchRecord.afterSha256 -cne 'a9aac30978a5c6c670aef643a337a7d2922e9d324f49fb663a41df29e6c44e54' -or
        (Digest $Genserv) -cne $PatchRecord.afterSha256) {
        throw 'Curl test-source patch receipt differs from reviewed bytes'
    }
    Stage 'test-source' 'patched' @{beforeSha256=$PatchRecord.beforeSha256;afterSha256=$PatchRecord.afterSha256}
    $Generated = RunBounded 'genserv-patched' $Perl.path @((Quote $Genserv),'test','test-localhost.prm') 180 $After
    RequireSuccess $Generated 'patched curl certificate generation'
    if ($Generated.stdout -notmatch 'CA root generated: test' -or
        $Generated.stdout -notmatch 'Certificate generated: CA=test') {
        throw 'Patched genserv did not report CA and localhost generation'
    }
    $Results.generated = [ordered]@{}
    foreach ($Name in @('test-ca.cacert','test-ca.key','test-localhost.crt','test-localhost.key')) {
        $Results.generated[$Name] = RecordFile (Join-Path $After $Name)
    }
    $Ca = Join-Path $After 'test-ca.cacert'
    $Cert = Join-Path $After 'test-localhost.crt'
    $Key = Join-Path $After 'test-localhost.key'
    foreach ($Check in @(
        @{name='ca-certificate';arguments=@('x509','-in',(Quote $Ca),'-noout','-subject')},
        @{name='localhost-certificate';arguments=@('x509','-in',(Quote $Cert),'-noout','-subject')},
        @{name='localhost-key';arguments=@('pkey','-in',(Quote $Key),'-noout')},
        @{name='localhost-chain';arguments=@('verify','-CAfile',(Quote $Ca),(Quote $Cert))}
    )) {
        $CheckResult = RunBounded $Check.name $OpenSsl.path $Check.arguments 20
        RequireSuccess $CheckResult $Check.name
    }
    Stage 'certificate-validation' 'passed' @{files=@($Results.generated.Keys)}
    $Status = 'passed'
} catch {
    $Failure = $_.Exception.Message
    Stage 'probe' 'failed' @{reason=$Failure}
} finally {
    [IO.File]::WriteAllText($Summary,([ordered]@{schemaVersion=1;status=$Status;failure=$Failure;
        sourceCommit=(git -C $Root rev-parse HEAD);githubSha=$env:GITHUB_SHA;
        testOnlyHostOpenSsl=$true;results=$Results;stages='stages.jsonl'} | ConvertTo-Json -Depth 10),$Encoding)
}
if ($Status -ne 'passed') { throw "Curl certificate probe failed; evidence=$Summary; reason=$Failure" }
Write-Output "Locked curl certificate generator probe passed; evidence=$Summary"
