# Archive a byte-verified obsolete OpenNav download without deleting recovery
# material or touching any installed executable or navigation profile.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',
      [Parameter(Mandatory=$true)][string]$File,
      [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedSha256,
      [switch]$Beta1Setup)
. (Join-Path $PSScriptRoot 'Common.ps1')
. (Join-Path $PSScriptRoot 'RetirementPolicy.ps1')
$source=Assert-LocalPath $File
$root=Assert-LocalPath $Workspace
$kind=Get-DownloadRetirementKind ([IO.Path]::GetFileName($source)) $ExpectedSha256 $Beta1Setup.IsPresent
if (-not [IO.File]::Exists($source)) { throw 'Expected one existing regular release file.' }
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
$extension=if ($kind -ceq 'beta1-setup') { '.exe' } else { '.zip' }
$target=Assert-LocalPath (Join-Path $recovery ($identity+$extension))
$journal=Join-Path $recovery ($identity+'.planned.json')
$completion=Join-Path $recovery ($identity+'.json')
foreach ($path in @($target,$journal,$completion)) {
  if (Test-Path -LiteralPath $path) { throw 'Retirement destination already exists; source retained.' }
}
$record=@{status='planned';utc=[DateTime]::UtcNow.ToString('o');originalFile=$source;
  recoveryFile=$target;sha256=$ExpectedSha256;bytes=(Get-Item -LiteralPath $source).Length;kind=$kind;
  userDataDeleted=$false;restore='Move this exact release file back to its recorded original path.'}
Write-Record $journal $record
if ((Get-Digest $source) -cne $ExpectedSha256) {throw 'Download changed during preparation; source retained.'}
$null=Assert-LocalPath $target
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
