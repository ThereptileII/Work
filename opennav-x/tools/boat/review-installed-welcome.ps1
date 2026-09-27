[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$LaunchResult,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchSha256,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$ExpectedCommit,
  [Parameter(Mandatory=$true)][ValidateSet('InspectWelcome','AcknowledgeWelcome')][string]$Action,
  [string]$WelcomeInspection,
  [ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedWelcomeInspectionSha256
)
. (Join-Path $PSScriptRoot 'InstalledWelcome.ps1')
$installed=Get-Installed;$path=Assert-LocalPath $LaunchResult;$launch=Read-Record $path
$job=[pscustomobject]@{action='ReviewInstalledWelcome';reviewAction=$Action;workspace=(Assert-LocalPath $Workspace);executable=$installed.executable;
  executableSha256=(Get-Digest $installed.executable);processId=$launch.pid;buildCommit=$ExpectedCommit;generation=$installed.state.current;
  launchResult=$path;launchResultSha256=$ExpectedLaunchSha256;launchRequestSha256=(Get-Digest (Join-Path ([IO.Path]::GetDirectoryName($path)) 'request.json'));
  helperFiles=@($script:InstalledWelcomeFiles | ForEach-Object { @{name=$_;sha256=(Get-Digest (Join-Path $PSScriptRoot $_))} })}
if ($Action -ceq 'AcknowledgeWelcome') {
  if (-not $WelcomeInspection -or -not $ExpectedWelcomeInspectionSha256) { throw 'Inspect/review this warning first, then provide its exact record/hash.' }
  $job|Add-Member welcomeInspection (Assert-LocalPath $WelcomeInspection);$job|Add-Member welcomeInspectionSha256 $ExpectedWelcomeInspectionSha256
} elseif ($WelcomeInspection -or $ExpectedWelcomeInspectionSha256) { throw 'Inspection evidence belongs only to AcknowledgeWelcome.' }
$null=Read-InstalledWelcome $job
$directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}) 'installed-welcome'
$job|Add-Member evidenceDirectory $directory;$job.PSObject.Properties.Remove('workspace')
$result=Invoke-InteractiveJob $Workspace $job
Write-Record (Join-Path $directory 'review.json') $result
$result | ConvertTo-Json -Depth 12
