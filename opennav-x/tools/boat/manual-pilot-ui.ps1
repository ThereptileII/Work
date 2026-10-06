# One explicitly chosen native product action. No automatic command sequences.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string]$Workspace,
  [Parameter(Mandatory=$true)][string]$Record,
  [Parameter(Mandatory=$true)][string]$ExpectedRecordSha256,
  [Parameter(Mandatory=$true)][string]$Action,
  [Parameter(Mandatory=$true)][string]$Nonce,
  [string]$Value='', [string]$ExpectedEpoch='', [string]$ExpectedCommandId=''
)
. (Join-Path $PSScriptRoot 'ManualPilotUi.ps1')
Invoke-ManualPilotUiJob $Workspace $Record $ExpectedRecordSha256 $Action $Value $Nonce $ExpectedEpoch $ExpectedCommandId | ConvertTo-Json -Depth 8
