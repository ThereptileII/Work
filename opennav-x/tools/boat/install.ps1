[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$Setup,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$Sha256,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$ExpectedCommit,
  [ValidateSet('Install','Update','Repair')][string]$Action='Install'
)
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace
$setup=Assert-LocalPath $Setup
if ((Get-Digest $setup) -cne $Sha256) { throw 'Setup hash differs from accepted CI release evidence.' }
if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'Close OpenCPN normally before maintenance.' }
# The durable local cold backup is a separate deliberate operation, not a
# fallback assumption that stock OpenCPN could be downloaded again later.
$recovery=Join-Path $Workspace 'recovery'
$sets=@(Get-ChildItem -LiteralPath $recovery -Directory | Where-Object { $_.Name -notlike '*.partial' } | ForEach-Object {
  $record=Join-Path $_.FullName 'recovery.json'
  if ([IO.File]::Exists($record)) { Read-Record $record }
} | Where-Object { $_.owner -ceq 'OpenNavX.BoatRecovery.1' -and $_.status -ceq 'verified' -and $_.sourceProfile -ieq $config.profileDirectory })
if (-not $sets.Count) { throw 'Verified cold recovery backup required before first deployment.' }
$directory=New-RunDirectory $Workspace $Action.ToLowerInvariant();$report=Join-Path $directory 'installer.json'
$start=New-Object Diagnostics.ProcessStartInfo
$start.FileName=$setup;$start.UseShellExecute=$false
$start.Arguments='/S /ACTION='+$Action+' /OPENCPN="'+$config.stockExecutable+'" /REPORT="'+$report+'"'
$process=[Diagnostics.Process]::Start($start)
try {
  if (-not $process.WaitForExit(240000)) { throw 'Setup is still running; inspect desktop/logs without force termination.' }
  if ($process.ExitCode -ne 0) { throw 'Installer failed; previous generation is retained. Inspect the result report.' }
} finally {$process.Dispose()}
$record=Read-Record $report
if ($record.status -cne 'passed') { throw 'Installer did not report successful validation.' }
$installed=Get-Installed
if ($installed.ownership.commit -cne $ExpectedCommit -or $installed.ownership.version -cne '0.4.0-beta2') { throw 'Installed identity differs from expected Beta 2 CI artifact.' }
Write-Record (Join-Path $directory 'deployment.json') @{status='installed';buildCommit=$ExpectedCommit;setupSha256=$Sha256;version=$installed.ownership.version;stockSha256=(Get-Digest $config.stockExecutable);launchPending=$true;note='No application or hardware command launched. Renew read-only profile/plugin audit before smoke testing.'}
Get-Content -LiteralPath (Join-Path $directory 'deployment.json') -Raw
