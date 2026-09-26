# Archive a byte-verified obsolete OpenNav download without deleting recovery
# material or touching any installed executable or navigation profile.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',
      [Parameter(Mandatory=$true)][string]$File,
      [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedSha256)
. (Join-Path $PSScriptRoot 'Common.ps1')
$source=Assert-LocalPath $File
$root=Assert-LocalPath $Workspace
if ([IO.Path]::GetExtension($source) -ine '.zip' -or
    [IO.Path]::GetFileName($source) -cnotmatch '^OpenNavX-[A-Za-z0-9_-]+( \([0-9]+\))?\.zip$') {
  throw 'Only an explicitly identified OpenNav ZIP download can be archived.'
}
if ($source.StartsWith($root+'\',[StringComparison]::OrdinalIgnoreCase)) {
  throw 'Existing recovery files must not be retired again.'
}
if ((Get-Digest $source) -cne $ExpectedSha256) {throw 'Download differs from the accepted release hash; source retained.'}
$recovery=Assert-LocalPath (Join-Path $root 'recovery')
if ([IO.Path]::GetPathRoot($source) -ine [IO.Path]::GetPathRoot($recovery)) {
  throw 'Retirement requires a same-volume atomic move.'
}
$null=New-Item -ItemType Directory -Path $recovery -Force
$identity='retired-download-'+[DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')+'-'+[guid]::NewGuid().ToString('N').Substring(0,8)
$target=Join-Path $recovery ($identity+'.zip')
$journal=Join-Path $recovery ($identity+'.planned.json')
$completion=Join-Path $recovery ($identity+'.json')
$record=@{status='planned';utc=[DateTime]::UtcNow.ToString('o');originalFile=$source;
  recoveryFile=$target;sha256=$ExpectedSha256;bytes=(Get-Item -LiteralPath $source).Length;
  userDataDeleted=$false;restore='Move this ZIP back to its recorded original path.'}
Write-Record $journal $record
if ((Get-Digest $source) -cne $ExpectedSha256) {throw 'Download changed during preparation; source retained.'}
[IO.File]::Move($source,$target)
$record.status='retired'
try {
  if ((Get-Digest $target) -cne $ExpectedSha256 -or [IO.File]::Exists($source)) {
    throw 'Post-move identity mismatch.'
  }
  Write-Record $completion $record
} catch {
  throw ('Archive retained without deletion; inspect durable recovery locator '+$journal+'. '+$_.Exception.Message)
}
$record | ConvertTo-Json
