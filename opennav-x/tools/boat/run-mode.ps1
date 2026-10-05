[CmdletBinding()]
param([string]$Workspace='C:\XNav',[ValidateSet('XNav','Legacy','Safe')][string]$Mode='XNav',[string]$RestartSessionRecord,[string]$RestartSessionSha256,[switch]$UseStartupLauncher)
. (Join-Path $PSScriptRoot 'Common.ps1')
$config=Get-Target $Workspace;$installed=Get-Installed
$launchEnvironment=Assert-ReadOnlyAudit $config $installed $Workspace
if($UseStartupLauncher -and ($Mode -cne 'XNav' -or $PSBoundParameters.ContainsKey('RestartSessionRecord') -or $PSBoundParameters.ContainsKey('RestartSessionSha256'))){throw 'Startup launcher opt-in permits XNav bootstrap only, without restart commissioning.'}
if($UseStartupLauncher){. (Join-Path $PSScriptRoot 'StartupLauncher.ps1');$null=Get-StartupLauncherContext $installed}
$flag=@{XNav='--xnav';Legacy='--legacy';Safe='--safe-mode'}[$Mode]
$job=[pscustomobject]@{action=$(if($UseStartupLauncher){'LaunchStartup'}else{'Launch'});executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);mode=$flag}
if($PSBoundParameters.ContainsKey('RestartSessionRecord') -or $PSBoundParameters.ContainsKey('RestartSessionSha256')) {
  $job|Add-Member restartSessionRecord $RestartSessionRecord
  $job|Add-Member restartSessionSha256 $RestartSessionSha256
  $null=Get-OptionalRestartBinding $job $installed $config $launchEnvironment
}
Invoke-InteractiveJob $Workspace $job -TimeoutSeconds $(if($UseStartupLauncher){150}else{90}) | ConvertTo-Json -Depth 8
