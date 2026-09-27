# Read-only startup evidence from the exact running official stock process.
# No task dispatch, window API, profile write, input, plugin or equipment command.
[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$LaunchResult,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchSha256,
  [ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchRequestSha256,
  [Parameter(Mandatory=$true)][string]$BaselineLog,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedBaselineSha256
)
. (Join-Path $PSScriptRoot 'Common.ps1')
. (Join-Path $PSScriptRoot 'RendererLog.ps1')
function Read-RendererRecord([string]$Path,[string]$ExpectedHash) {
  $bytes=Read-StartupLogBytes (Assert-LocalPath $Path)
  if ($bytes.Length -eq 0 -or (Get-RendererBytesHash $bytes) -cne $ExpectedHash) { throw 'RECEIPT_HASH_MISMATCH' }
  return ((New-Object Text.UTF8Encoding($false,$true)).GetString($bytes)|ConvertFrom-Json)
}
$process=$null
try {
  if ([Environment]::OSVersion.Platform -ne 'Win32NT') { throw 'NATIVE_WINDOWS_OBSERVER_REQUIRED' }
  $workspace=Assert-LocalPath $Workspace;$path=Assert-LocalPath $LaunchResult;$directory=[IO.Path]::GetDirectoryName($path)
  if ([IO.Path]::GetFileName($path) -cne 'result.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $workspace 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-launchstock-[a-f0-9]{8}$') { throw 'UNOWNED_LAUNCH_RECEIPT' }
  $launch=Read-RendererRecord $path $ExpectedLaunchSha256
  $requestPath=Join-Path $directory 'request.json';$requestBytes=Read-StartupLogBytes $requestPath
  $requestHash=Get-RendererBytesHash $requestBytes
  if ($ExpectedLaunchRequestSha256 -and $requestHash -cne $ExpectedLaunchRequestSha256) { throw 'REQUEST_HASH_MISMATCH' }
  $request=(New-Object Text.UTF8Encoding($false,$true)).GetString($requestBytes)|ConvertFrom-Json
  $config=Get-Target $workspace
  $profile=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('CommonApplicationData')) 'opencpn')
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $start=Assert-RendererReceipt $launch $request $config $workspace $path $profile $sid (Get-Digest (Join-Path $workspace 'boat-target.json')) ([datetime]::UtcNow)
  $process=Get-Process -Id $launch.pid -ErrorAction Stop;Assert-RendererProcess $process $launch $config.stockExecutable
  $open=@(Get-Process -Name opencpn -ErrorAction SilentlyContinue)
  try { if ($open.Count -ne 1 -or $open[0].Id -ne $launch.pid) { throw 'AMBIGUOUS_OPENCPN_WRITERS' } }
  finally { foreach($item in $open){$item.Dispose()} }
  $baselinePath=Assert-LocalPath $BaselineLog
  $currentPath=Assert-LocalPath (Join-Path $profile 'opencpn.log');$rotatedPath=Assert-LocalPath (Join-Path $profile 'opencpn.log.log')
  if ($baselinePath -ieq $currentPath -or $baselinePath -ieq $rotatedPath -or -not [IO.File]::Exists($baselinePath)) { throw 'EXPLICIT_COLD_LOG_COPY_REQUIRED' }
  $baseline=Read-StartupLogBytes $baselinePath;$current=Read-StartupLogBytes $currentPath
  $rotated=if([IO.File]::Exists($rotatedPath)){Read-StartupLogBytes $rotatedPath}else{$null}
  $observed=[datetime]::UtcNow
  $process.Refresh();Assert-RendererProcess $process $launch $config.stockExecutable
  if ((Get-Digest $path) -cne $ExpectedLaunchSha256 -or (Get-Digest $requestPath) -cne $requestHash -or
      (Get-Digest $config.stockExecutable) -cne $launch.executableSha256) { throw 'OBSERVATION_PROOF_CHANGED' }
  $result=Get-RendererLogObservation -Baseline $baseline -Current $current -Rotated $rotated -BaselineSha256 $ExpectedBaselineSha256 -ProcessStartedUtc $start -ObservedUtc $observed -TimeZone ([TimeZoneInfo]::Local)
  $result | Add-Member -NotePropertyName processId -NotePropertyValue $launch.pid
  $result | Add-Member -NotePropertyName executableSha256 -NotePropertyValue $launch.executableSha256
  $result | Add-Member -NotePropertyName launchResultSha256 -NotePropertyValue $ExpectedLaunchSha256
  $result | Add-Member -NotePropertyName launchRequestSha256 -NotePropertyValue $requestHash
  $result | Add-Member -NotePropertyName launchRequestHashExplicitlyPinned -NotePropertyValue ([bool]$ExpectedLaunchRequestSha256)
} catch {
  $result=[pscustomobject](New-RendererObservation);$code=$_.Exception.Message
  $result.reason=if($code -cmatch '^[A-Z][A-Z0-9_]{2,64}$'){$code}else{'OBSERVER_PROOF_OR_READ_FAILED'}
} finally { if($process){$process.Dispose()} }
# Metadata only: the operator may redirect this JSON to the private evidence
# directory. No profile/log path, raw log line, SID or unrelated process is output.
$result | ConvertTo-Json -Depth 6
