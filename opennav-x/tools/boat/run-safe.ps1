[CmdletBinding()]
param([string]$Workspace='C:\XNav')
& (Join-Path $PSScriptRoot 'run-mode.ps1') -Workspace $Workspace -Mode 'Safe'
