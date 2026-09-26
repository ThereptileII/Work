[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][int]$ProcessId)
. (Join-Path $PSScriptRoot 'Common.ps1')
$installed=Get-Installed
Invoke-InteractiveJob $Workspace ([pscustomobject]@{action='Close';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);processId=$ProcessId}) | ConvertTo-Json -Depth 8
