[CmdletBinding()]
param([string]$Workspace='C:\XNav',[switch]$UseStartupLauncher)
& (Join-Path $PSScriptRoot 'run-mode.ps1') -Workspace $Workspace -Mode 'XNav' -UseStartupLauncher:$UseStartupLauncher
