[CmdletBinding()]
param(
 [string]$Workspace='C:\XNav',
 [Parameter(Mandatory=$true)][string]$LaunchResult,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchSha256,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedRequestSha256,
 [Parameter(Mandatory=$true)][int]$HelperProcessId,
 [Parameter(Mandatory=$true)][long]$ExpectedHelperStartedUtcTicks
)
. (Join-Path $PSScriptRoot 'ChartHelperShutdown.ps1')
if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Native Windows is required.'}
$context=Read-ChartHelperContext $Workspace $LaunchResult $ExpectedLaunchSha256 $ExpectedRequestSha256 $HelperProcessId $ExpectedHelperStartedUtcTicks
$directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=$context.launch.sid}) 'chart-helper-shutdown'
Add-Type -Path (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs')
$context=Read-ChartHelperContext $Workspace $LaunchResult $ExpectedLaunchSha256 $ExpectedRequestSha256 $HelperProcessId $ExpectedHelperStartedUtcTicks
$intent=@{owner='OpenNavX.ChartHelperShutdown.1';utc=[datetime]::UtcNow.ToString('o');launchResultSha256=$ExpectedLaunchSha256;
 launchRequestSha256=$ExpectedRequestSha256;helper=$context.helper;helperSha256=$script:ChartHelperHash;
 nativeSha256=(Get-Digest (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs'));command=2;bytes=1025;noRetry=$true;forceTermination=$false}
Write-Record (Join-Path $directory 'intent.json') $intent
$null=New-ChartHelperGlobalLocator $Workspace $context.launch.sid $HelperProcessId $ExpectedHelperStartedUtcTicks (Join-Path $directory 'intent.json') 'OpenNavX.ChartHelperShutdown.1'
# One deterministic atomic create-new locator excludes concurrent attempts as
# well as later retries. A crash after it is published requires inspection.
$locator=Join-Path $context.cold ('chart-helper-shutdown-'+$HelperProcessId+'-'+$ExpectedHelperStartedUtcTicks+'.json')
Write-Record $locator @{owner='OpenNavX.ChartHelperShutdown.1';intent=(Join-Path $directory 'intent.json');intentSha256=(Get-Digest (Join-Path $directory 'intent.json'))}
$result=[OpenNavX.ChartHelperShutdownNative]::Shutdown($HelperProcessId,$ExpectedHelperStartedUtcTicks,[int]$context.launch.pid,[int]$context.launch.sessionId,$context.helperPath)
Write-Record (Join-Path $directory 'result.json') $result
$result|ConvertTo-Json -Depth 5
if(-not $result.Succeeded){throw 'Chart-helper shutdown is not proven complete. Inspect the retained result; no retry or forced termination.'}
# This proves only the exact local helper exit. A new full cold InspectRestore
# remains mandatory; neither profile approval nor plugin restoration is implied.
