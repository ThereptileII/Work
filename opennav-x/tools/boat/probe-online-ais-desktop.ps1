# Same internet-only bounded probe, dispatched to the already logged-in user's
# credential context. Never starts OpenCPN or changes access/network settings.
[CmdletBinding()]
param(
 [Parameter(Mandatory)][string]$Package,
 [Parameter(Mandatory)][ValidatePattern('^[0-9a-f]{40}$')][string]$ExpectedCommit,
 [Parameter(Mandatory)][ValidatePattern('^[0-9a-f]{64}$')][string]$ExpectedBuildInfoSha256,
 [Parameter(Mandatory)][string]$Evidence,
 [ValidateSet('stockholm','oresund')][string]$Region='stockholm'
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Windows desktop required.'}
foreach($p in @($Package,$Evidence,$PSScriptRoot)) {
 if($p -notmatch '^[A-Za-z]:\\' -or $p.Contains('"') -or $p.Contains("`r") -or $p.Contains("`n")){throw 'Local unambiguous paths required.'}
}
if(Test-Path -LiteralPath $Evidence){throw 'New evidence path required.'}
$sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
$sessions=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {(Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -ceq $sid} | Select-Object -ExpandProperty SessionId -Unique)
if($sessions.Count -ne 1){throw 'Exactly one existing desktop for this account required.'}
$name='OpenNavX-AisProbe-'+[guid]::NewGuid().ToString('N');$task=$null
try {
 $arguments='-NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File "'+(Join-Path $PSScriptRoot 'probe-online-ais.ps1')+'" -Package "'+$Package+'" -ExpectedCommit '+$ExpectedCommit+' -ExpectedBuildInfoSha256 '+$ExpectedBuildInfoSha256+' -Evidence "'+$Evidence+'" -Region '+$Region
 $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument $arguments
 $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
 $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Seconds 90) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
 $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
 Start-ScheduledTask -TaskName $name
 $summary=Join-Path $Evidence 'summary.json';$deadline=[DateTime]::UtcNow.AddSeconds(80)
 while(-not [IO.File]::Exists($summary) -and [DateTime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 250}
 if(-not [IO.File]::Exists($summary)){throw 'Desktop AIS probe did not publish its result; inspect its owned task/evidence.'}
 $result=Get-Content -LiteralPath $summary -Raw|ConvertFrom-Json
 if($result.commit -cne $ExpectedCommit -or -not $result.disabledAndCleared){throw 'Desktop AIS probe result mismatch or provider did not stop.'}
 $result|ConvertTo-Json -Compress
 if($result.exitCode -ne 0){exit $result.exitCode}
} finally {if($task){Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
