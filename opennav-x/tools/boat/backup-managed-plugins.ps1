# Inventory/copy only. No plugin moves, configuration edits or application launch.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',[string]$ProfileCandidate)
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$context=Get-PreparationContext $Workspace
$ini=Join-Path $context.profile 'opencpn.ini'
if ($ProfileCandidate) {
  Assert-AuthorizedRecovery $ini $ProfileCandidate
  $profileForRoots=Assert-LocalPath $ProfileCandidate
} else { $profileForRoots=$ini }
$values=Read-ProfileForAudit $profileForRoots
if ($values['Directories/pluginInstallDir']) { throw 'Custom plugin roots require a separate source/path audit.' }
$profileHash=Get-Digest $profileForRoots
$sources=[ordered]@{
  managed=$context.managed
  application=(Join-Path $context.application 'plugins')
  metadataAndCache=(Join-Path $context.profile 'plugins')
}
$snapshots=[ordered]@{}
foreach ($name in $sources.Keys) { $snapshots[$name]=Get-PreparationTree $sources[$name] }
$catalog=Join-Path $context.profile 'ocpn-plugins.xml'
$catalogRecord=if ([IO.File]::Exists($catalog)) { @{path=$catalog;bytes=(Get-Item -LiteralPath $catalog).Length;sha256=(Get-Digest $catalog)} } else { $null }
$size=[long]0
foreach ($snapshot in $snapshots.Values) { foreach ($entry in $snapshot.entries) { $size+=$entry.bytes } }
if ($catalogRecord) { $size+=$catalogRecord.bytes }
$drive=New-Object IO.DriveInfo([IO.Path]::GetPathRoot($context.workspace))
if ($drive.AvailableFreeSpace -lt $size+268435456) { throw 'Insufficient space for complete plugin recovery and margin.' }
$directory=New-PreparationDirectory $context 'managed-plugin-backup'
$stage=Join-Path $directory 'payload.partial';$target=Join-Path $directory 'payload'
$record=Join-Path $directory 'prepared.json'
Write-Record $record @{schema=1;owner='OpenNavX.ManagedPluginBackup.1';status='prepared';createdUtc=[DateTime]::UtcNow.ToString('o');context=$context;profileForRoots=$profileForRoots;profileSha256=$profileHash;trees=$snapshots;catalog=$catalogRecord;bytes=$size;privacy='Private local recovery; no upload or public paths.'}
try {
  $null=New-Item -ItemType Directory -Path $stage
  foreach ($name in $sources.Keys) {
    Assert-PreparationClosed @($context.application,$context.managed)
    Copy-PreparationTree $snapshots[$name] (Join-Path $stage $name)
  }
  if ($catalogRecord) { Copy-PreparationFile $catalog (Join-Path $stage 'ocpn-plugins.xml') $catalogRecord.sha256 $catalogRecord.bytes }
  foreach ($snapshot in $snapshots.Values) { Assert-PreparationTree $snapshot }
  if ((Get-Digest $profileForRoots) -cne $profileHash -or ($catalogRecord -and (Get-Digest $catalog) -cne $catalogRecord.sha256) -or (-not $catalogRecord -and (Test-Path -LiteralPath $catalog))) { throw 'Profile/catalog changed during backup.' }
  $current=Get-PreparationContext $Workspace
  if ($current.sid -cne $context.sid -or $current.session -ne $context.session -or $current.managed -ine $context.managed) { throw 'Interactive identity changed during backup.' }
  [IO.Directory]::Move($stage,$target)
  $verified=Join-Path $directory 'verified.json'
  Write-Record $verified @{schema=1;owner='OpenNavX.ManagedPluginBackup.1';status='verified';preparedSha256=(Get-Digest $record);payload=$target;bytes=$size;sourceUnchanged=$true;applicationLaunched=$false;pluginMoved=$false;privacy='Private local recovery only.'}
  [pscustomobject]@{status='verified';record=$record;recordSha256=(Get-Digest $record);verification=$verified;verificationSha256=(Get-Digest $verified);bytes=$size;applicationLaunched=$false;pluginMoved=$false} | ConvertTo-Json
} catch {
  Write-Record (Join-Path $directory 'failed.json') @{status='failed';error=$_.Exception.Message;sourceModified=$false;partialRetained=$true;applicationLaunched=$false}
  throw
}
