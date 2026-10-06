# Separate opt-in transaction; no steering commands or plugin restoration here.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][ValidateSet('Prepare','Apply','Launch','Inspect','Close','Rollback')][string]$Action,
  [Parameter(Mandatory=$true)][string]$Workspace,
  [string]$ExpectedCommit,[string]$ExpectedGeneration,[string]$ExpectedExecutableSha256,[string]$ExpectedOwnershipSha256,[string]$ExpectedPackageSha256,
  [string]$Qualification,[string]$ExpectedQualificationSha256,
  [string]$Record,[string]$ExpectedRecordSha256,
  [string]$Inspection,[string]$ExpectedInspectionSha256,[string]$ReviewedCurrentIniSha256
)
. (Join-Path $PSScriptRoot 'ManualPilotCommissioning.ps1')
if($Action -ceq 'Prepare') {
  $candidate=[pscustomobject]@{commit=$ExpectedCommit;generation=$ExpectedGeneration;executableSha256=$ExpectedExecutableSha256;ownershipSha256=$ExpectedOwnershipSha256;packageSha256=$ExpectedPackageSha256}
  $result=New-ManualPilot $Workspace $candidate $Qualification $ExpectedQualificationSha256
}elseif($Action -ceq 'Apply') {$result=Apply-ManualPilot $Workspace $Record $ExpectedRecordSha256}
elseif($Action -ceq 'Launch') {$result=Invoke-ManualPilotJob $Workspace $Record $ExpectedRecordSha256 $true}
elseif($Action -ceq 'Inspect') {$result=Inspect-ManualPilot $Workspace $Record $ExpectedRecordSha256}
elseif($Action -ceq 'Close') {$result=Invoke-ManualPilotJob $Workspace $Record $ExpectedRecordSha256 $false}
else {$result=Rollback-ManualPilot $Workspace $Record $ExpectedRecordSha256 $Inspection $ExpectedInspectionSha256 $ReviewedCurrentIniSha256}
$result | ConvertTo-Json -Depth 8
