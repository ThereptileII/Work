[CmdletBinding()]
param([string]$Workspace='C:\XNav',[ValidateSet('XNav','Legacy','Safe')][string]$Mode='XNav')
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
Assert-ReadOnlyAudit $config $installed
$flag=@{XNav='--xnav';Legacy='--legacy';Safe='--safe-mode'}[$Mode]
Invoke-InteractiveJob $Workspace ([pscustomobject]@{action='Launch';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);mode=$flag}) | ConvertTo-Json -Depth 8
