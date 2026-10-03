[CmdletBinding()]
param(
 [string]$Workspace='C:\XNav',
 [Parameter(Mandatory=$true)][string]$Record,
 [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedRecordSha256,
 [Parameter(Mandatory=$true)][int]$HelperProcessId,
 [Parameter(Mandatory=$true)][int]$ParentProcessId,
 [Parameter(Mandatory=$true)][long]$ExpectedHelperStartedUtcTicks
)
. (Join-Path $PSScriptRoot 'OrphanChartHelper.ps1')
if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Native Windows is required.'}
$context=Read-OrphanChartHelper $Workspace $Record $ExpectedRecordSha256 $HelperProcessId $ParentProcessId $ExpectedHelperStartedUtcTicks
$directory=New-PreparationDirectory $context.context 'orphan-chart-helper-shutdown'
Add-Type -Path (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs')
$context=Read-OrphanChartHelper $Workspace $Record $ExpectedRecordSha256 $HelperProcessId $ParentProcessId $ExpectedHelperStartedUtcTicks
$intent=Join-Path $directory 'intent.json'
Write-Record $intent @{owner='OpenNavX.OrphanChartHelperShutdown.1';utc=[datetime]::UtcNow.ToString('o');recordSha256=$ExpectedRecordSha256;
 helper=$context.helper;helperSha256=$script:ChartHelperHash;nativeSha256=(Get-Digest (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs'));
 launchProvenance='Unrecorded parent has exited; no launch or restore permission implied';command=2;bytes=1025;noRetry=$true;forceTermination=$false}
$null=New-ChartHelperGlobalLocator $Workspace $context.context.sid $HelperProcessId $ExpectedHelperStartedUtcTicks $intent 'OpenNavX.OrphanChartHelperShutdown.1'
# Shared locator with the older launch-bound path: exactly one attempt per process.
$locator=Join-Path $context.cold ('chart-helper-shutdown-'+$HelperProcessId+'-'+$ExpectedHelperStartedUtcTicks+'.json')
Write-Record $locator @{owner='OpenNavX.OrphanChartHelperShutdown.1';intent=$intent;intentSha256=(Get-Digest $intent)}
$result=[OpenNavX.ChartHelperShutdownNative]::Shutdown($HelperProcessId,$ExpectedHelperStartedUtcTicks,$ParentProcessId,[int]$context.helper.sessionId,$context.helper.path)
Write-Record (Join-Path $directory 'result.json') $result
$result|ConvertTo-Json -Depth 5
if(-not $result.Succeeded){throw 'Orphan helper exit is not proven; inspect retained result. No retry or forced termination.'}
# A separate cold InspectRestore remains mandatory. No profile/plugins changed.
