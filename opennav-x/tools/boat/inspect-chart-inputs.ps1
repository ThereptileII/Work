# Read-only availability check. Does not read chart contents or change a profile.
[CmdletBinding()]
param([string]$Workspace='C:\XNav')
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$context=Get-PreparationContext $Workspace
$ini=Join-Path $context.profile 'opencpn.ini'
$before=Get-Digest $ini
$values=Read-ProfileForAudit $ini
$items=@()
foreach ($key in @($values.Keys | Where-Object {$_ -like 'ChartDirectories/*'} | Sort-Object)) {
  # Pinned MyConfig::LoadChartDirArray uses the portion before the first ^.
  $directory=([string]$values[$key]).Split('^')[0]
  $state='unresolved'
  if ($directory -match '^[A-Za-z]:\\') {
    $state=if ([IO.Directory]::Exists($directory)) {'available'} else {'missing'}
  }
  $items+=@{index=$items.Count+1;state=$state}
}
if ((Get-Digest $ini) -cne $before) { throw 'Profile changed during availability inspection.' }
# Pinned newPrivateFileName selects the historical Windows spelling.
$database=Join-Path $context.profile 'CHRTLIST.DAT'
[pscustomobject]@{
  schema='OpenNavX.ChartInputAvailability.1';utc=[DateTime]::UtcNow.ToString('o')
  profileSha256=$before;configuredDirectoryCount=$items.Count;directories=$items
  chartDatabasePresent=[IO.File]::Exists($database)
  chartDatabaseBytes=$(if ([IO.File]::Exists($database)) {(Get-Item -LiteralPath $database).Length} else {$null})
  chartContentsRead=$false;applicationLaunched=$false
  note='Directory availability only; native chart rendering and chart licensing remain unverified.'
} | ConvertTo-Json -Depth 5
