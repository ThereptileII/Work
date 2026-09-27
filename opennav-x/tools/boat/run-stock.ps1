[CmdletBinding()]
param([string]$Workspace='C:\XNav')
. (Join-Path $PSScriptRoot 'StockReview.ps1')
$config=Get-StockTarget $Workspace
$null=Assert-StockLaunchAudit $config $Workspace
$job=[pscustomobject]@{action='LaunchStock';mode='StockLegacy';arguments='';executable=$config.stockExecutable;
  executableSha256=(Get-Digest $config.stockExecutable);targetSha256=(Get-Digest (Join-Path $Workspace 'boat-target.json'))}
Invoke-InteractiveJob $Workspace $job | ConvertTo-Json -Depth 8
