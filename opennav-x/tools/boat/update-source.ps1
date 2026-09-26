[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$Commit)
. (Join-Path $PSScriptRoot 'SourceCheckout.ps1')
$null=Get-Target $Workspace
$origin='https://github.com/ThereptileII/Work.git'
Update-SourceCheckout $Workspace $Commit $origin | ConvertTo-Json
