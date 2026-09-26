# Retire an explicitly identified old portable distribution without deleting its
# charts, profile, logs or user additions. The whole directory becomes a local
# recovery archive; it is no longer a Desktop/Downloads runnable installation.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][string]$Directory,[Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedManifestSha256)
. (Join-Path $PSScriptRoot 'Common.ps1')
$source=Assert-LocalPath $Directory;$root=Assert-LocalPath $Workspace
if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) {throw 'Close every OpenCPN mode normally before retiring a portable build.'}
if ($source -ieq $root -or $source.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase) -or $root.StartsWith($source+'\',[StringComparison]::OrdinalIgnoreCase)) {throw 'Source must be a separate old portable directory outside the recovery workspace.'}
$manifest=Join-Path $source 'FILE_SHA256.json'
if ((Get-Digest $manifest) -cne $ExpectedManifestSha256) {throw 'Portable inventory does not match the accepted old release supplied for this operation.'}
if (-not [IO.File]::Exists((Join-Path $source 'app\OPENNAV_PORTABLE_PREVIEW'))) {throw 'Not a recognized isolated portable OpenNav release.'}
foreach ($item in Get-ChildItem -LiteralPath $source -Force -Recurse) {
  if ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) {throw 'Portable directory contains redirected paths; inspect manually and preserve it.'}
}
$recovery=Assert-LocalPath (Join-Path $root 'recovery')
if ([IO.Path]::GetPathRoot($source) -ine [IO.Path]::GetPathRoot($recovery)) {throw 'Retirement requires a same-volume atomic move; preserve the source on other volumes.'}
$null=New-Item -ItemType Directory -Path $recovery -Force
$target=Join-Path $recovery ('retired-portable-'+[DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8))
$identity=[IO.Path]::GetFileName($target)
$journal=Join-Path $recovery ($identity+'.planned.json')
$completion=Join-Path $recovery ($identity+'.json')
$record=@{status='planned';utc=[DateTime]::UtcNow.ToString('o');originalDirectory=$source;recoveryDirectory=$target;manifestSha256=$ExpectedManifestSha256;userDataDeleted=$false;restore='Move the entire directory back only when no OpenCPN process is running.'}
# Publish the recovery locator before the atomic move. A power interruption or
# failure writing the completion report must never strand an unrecorded archive.
Write-Record $journal $record
if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) {throw 'OpenCPN started during retirement preparation; source retained.'}
if ((Get-Digest $manifest) -cne $ExpectedManifestSha256) {throw 'Portable ownership changed during retirement preparation; source retained.'}
[IO.Directory]::Move($source,$target)
$record.status='retired'
$record.completionRecord=$completion
try {
  if ((Get-Digest (Join-Path $target 'FILE_SHA256.json')) -cne $ExpectedManifestSha256 -or [IO.Directory]::Exists($source)) {throw 'Post-move archive identity mismatch.'}
  Write-Record $completion $record
} catch {
  throw ('Portable folder is in recovery; no data was deleted. Completion verification/report needs attention. Read the durable locator: '+$journal+'. Error: '+$_.Exception.Message)
}
$record | ConvertTo-Json
