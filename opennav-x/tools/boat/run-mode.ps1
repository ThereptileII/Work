[CmdletBinding()]
param([string]$Workspace='C:\XNav',[ValidateSet('XNav','Legacy','Safe')][string]$Mode='XNav',[string]$RestartSessionRecord,[string]$RestartSessionSha256)
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
$launchEnvironment=Assert-ReadOnlyAudit $config $installed $Workspace
$flag=@{XNav='--xnav';Legacy='--legacy';Safe='--safe-mode'}[$Mode]
$job=[pscustomobject]@{action='Launch';executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);mode=$flag}
if($PSBoundParameters.ContainsKey('RestartSessionRecord') -or $PSBoundParameters.ContainsKey('RestartSessionSha256')) {
  $job|Add-Member restartSessionRecord $RestartSessionRecord
  $job|Add-Member restartSessionSha256 $RestartSessionSha256
  $null=Get-OptionalRestartBinding $job $installed $config $launchEnvironment
}
Invoke-InteractiveJob $Workspace $job | ConvertTo-Json -Depth 8
