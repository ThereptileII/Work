param(
    [Parameter(Mandatory)][string]$Package,
    [Parameter(Mandatory)][ValidatePattern('^[0-9a-f]{40}$')][string]$ExpectedCommit,
    [Parameter(Mandatory)][ValidatePattern('^[0-9a-f]{64}$')][string]$ExpectedBuildInfoSha256,
    [Parameter(Mandatory)][string]$Evidence,
    [ValidateSet('stockholm','oresund')][string]$Region = 'stockholm'
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
# This separate tool has no OpenCPN/profile/plugin/hardware dependencies. Never
# use this script to launch the product, change networking, or enable equipment.
$Root = (Resolve-Path -LiteralPath $Package).Path
$InfoPath = Join-Path $Root 'BUILD_INFO.json'
if ((Get-FileHash -LiteralPath $InfoPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $ExpectedBuildInfoSha256) {
    throw 'Probe build information does not match the verified CI artifact'
}
$Info = Get-Content -LiteralPath $InfoPath -Raw | ConvertFrom-Json
if ($Info.commit -ne $ExpectedCommit -or $Info.upstreamCommit -ne '37fd0cddb7334fe489e9f18aa163977a9c5c84f7') {
    throw 'Unexpected probe source revision'
}
$App = Join-Path $Root 'app'
$Expected = @($Info.binaries.PSObject.Properties)
if ($Expected.Count -lt 1 -or $Expected.Count -gt 32) { throw 'Invalid dependency inventory' }
foreach ($File in $Expected) {
    if ($File.Name -notmatch '^app[/\\][A-Za-z0-9_.-]+\.(exe|dll)$' -or $File.Value -notmatch '^[0-9a-f]{64}$') {
        throw 'Invalid dependency path/hash'
    }
    $Path = Join-Path $Root $File.Name
    $Item = Get-Item -LiteralPath $Path
    if (($Item.Attributes -band [IO.FileAttributes]::ReparsePoint) -or
        (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $File.Value) {
        throw 'Probe binary differs from the verified artifact'
    }
}
if (@(Get-ChildItem -LiteralPath $App -Force).Count -ne $Expected.Count) { throw 'Unexpected probe-directory entry' }
$Exe = Join-Path $App 'aisstream_live_probe.exe'
if (Test-Path -LiteralPath $Evidence) { throw 'Use a new private evidence directory' }
$null = New-Item -ItemType Directory -Path $Evidence
$OldPath = $env:PATH
try {
    $env:PATH = Join-Path $env:SystemRoot 'System32'
    $Capabilities = & $Exe --describe | ConvertFrom-Json
    if ($LASTEXITCODE -ne 0 -or $Capabilities.profile_access -ne $false -or
        $Capabilities.marine_equipment -ne $false -or $Capabilities.secret_output -ne $false -or
        $Capabilities.endpoint -ne 'wss://stream.aisstream.io/v0/stream' -or
        $Capabilities.maximum_observation_seconds -ne 45) { throw 'Probe capability mismatch' }
    $Out = Join-Path $Evidence 'aggregate-ais.jsonl'
    $Err = Join-Path $Evidence 'probe-stderr.txt'
    $Process = Start-Process -FilePath $Exe -ArgumentList @('--read-only-live-ais',$Region) -WorkingDirectory $App `
        -PassThru -NoNewWindow -RedirectStandardOutput $Out -RedirectStandardError $Err
    if (-not $Process.WaitForExit(65000)) {
        $Process.Kill() # Only this script's bounded, internet-only child.
        $Process.WaitForExit()
        throw 'Read-only AIS probe exceeded its commissioning deadline'
    }
    $Process.Refresh()
    $Rows = @(Get-Content -LiteralPath $Out | ForEach-Object { $_ | ConvertFrom-Json })
    $Result = @($Rows | Where-Object { $_.event -eq 'result' })
    if ($Result.Count -ne 1) { throw 'Probe did not produce a complete aggregate result' }
    # Only trusted fixed-schema fields leave this private evidence directory.
    $Summary = [ordered]@{commit=$ExpectedCommit;exitCode=$Process.ExitCode;region=$Region;
        subscriptionConfirmed=[bool]$Result[0].subscription_confirmed;
        peakTargetCount=[int]$Result[0].peak_target_count;
        acceptedReports=[int]$Result[0].accepted_reports;rejectedReports=[int]$Result[0].rejected_reports;
        reconnects=[int]$Result[0].reconnects;disabledAndCleared=[bool]$Result[0].disabled_and_cleared;
        chartOrUiAccepted=$false}
    $Summary | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $Evidence 'summary.json') -Encoding UTF8
    $Summary | ConvertTo-Json -Compress
    if ($Process.ExitCode -ne 0) { exit $Process.ExitCode }
} finally { $env:PATH = $OldPath }
