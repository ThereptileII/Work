[CmdletBinding()]
param([string]$Workspace='C:\XNav')
& (Join-Path $PSScriptRoot 'maintain.ps1') -Workspace $Workspace -Action 'Repair'
