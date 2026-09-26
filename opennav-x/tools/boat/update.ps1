[CmdletBinding()]
param([string]$Workspace='C:\XNav',[Parameter(Mandatory=$true)][string]$Setup,[Parameter(Mandatory=$true)][string]$Sha256,[Parameter(Mandatory=$true)][string]$ExpectedCommit)
& (Join-Path $PSScriptRoot 'install.ps1') -Workspace $Workspace -Setup $Setup -Sha256 $Sha256 -ExpectedCommit $ExpectedCommit -Action Update
