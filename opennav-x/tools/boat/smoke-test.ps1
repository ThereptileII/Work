# Read-only application smoke. No route edits, synthetic input, command controls
# or actuator output are exercised. A fresh audited real profile is mandatory.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',[ValidateRange(5,60)][int]$ObserveSeconds=15)
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
Assert-ReadOnlyAudit $config $installed
$directory=New-RunDirectory $Workspace 'smoke'
$record=@{status='running';commit=$installed.ownership.commit;startedUtc=[DateTime]::UtcNow.ToString('o');actuatorCommandsAttempted=0;syntheticInputs=0;chartReview='pending native screenshot review';closedCleanly=$false}
$running=$null
$log=Assert-LocalPath (Join-Path $config.profileDirectory 'opencpn.log')
$beforeMarkers=0
if ([IO.File]::Exists($log)) { $beforeMarkers=([regex]::Matches([IO.File]::ReadAllText($log),'OnInitTimer\.\.\.Finalize Canvases')).Count }
try {
  $running=Invoke-InteractiveJob $Workspace ([pscustomobject]@{action='Launch';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);mode='--xnav'})
  Start-Sleep -Seconds $ObserveSeconds
  $process=Get-Process -Id $running.pid -ErrorAction Stop
  try {
    if (-not $process.Responding) {throw 'XNav window is not responding.'}
    $record.resources=@{workingSetBytes=$process.WorkingSet64;privateBytes=$process.PrivateMemorySize64;handles=$process.HandleCount;cpuSeconds=$process.TotalProcessorTime.TotalSeconds}
  } finally {$process.Dispose()}
  $record.capture=Invoke-InteractiveJob $Workspace ([pscustomobject]@{action='Capture';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);processId=$running.pid;imagePath=(Join-Path $directory 'navigation.png')})
  if (-not [IO.File]::Exists($log)) {throw 'Expected normal-profile log was not created.'}
  # Match known lifecycle messages only, never return raw navigation log lines.
  $text=[IO.File]::ReadAllText($log)
  $record.startupMarkerPresent=([regex]::Matches($text,'OnInitTimer\.\.\.Finalize Canvases')).Count -gt $beforeMarkers
  if (-not $record.startupMarkerPresent) {throw 'Chart startup completion marker missing.'}
  $record.status='captured-review-required'
} catch {$record.status='failed';$record.error=$_.Exception.Message}
finally {
  if ($running) {
    try {
      $closed=Invoke-InteractiveJob $Workspace ([pscustomobject]@{action='Close';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);processId=$running.pid})
      $record.closedCleanly=$closed.exitCode -eq 0
    } catch {$record.status='failed';$record.closeError=$_.Exception.Message}
  }
  Write-Record (Join-Path $directory 'smoke.json') $record
}
$record | ConvertTo-Json -Depth 10
if ($record.status -eq 'failed') {exit 1}
