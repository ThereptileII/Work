# Creates deployment metadata only; it never authorizes a launch.
[CmdletBinding()]
param([string]$Workspace='C:\XNav')
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$context=Get-PreparationContext $Workspace
$path=Join-Path $context.workspace 'boat-target.json'
if (Test-Path -LiteralPath $path) { throw 'Target metadata already exists; inspect it instead of overwriting.' }
$target=@{
  schema=1;owner='OpenNavX.BoatTarget.1'
  stockExecutable=$context.executable;profileDirectory=$context.profile
  readOnlyAudit=@{
    reviewedUtc=$null;buildCommit=$null;profileIniSha256=$null
    connectionsOutputDisabled=$false;pluginOutputsReviewed=$false
    noActiveRouteOutput=$false;pluginFiles=@();commissioning=$null
  }
}
Write-Record $path $target
$null=Get-Target $context.workspace
[pscustomobject]@{
  status='initialized';targetSha256=(Get-Digest $path)
  stockSha256=(Get-Digest $context.executable)
  applicationLaunched=$false;launchAuthorized=$false
  note='Exact supported installation recorded. Separate commissioning and source audit remain required.'
} | ConvertTo-Json
