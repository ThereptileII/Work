[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$LaunchResult,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchSha256,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{40}$')][string]$ExpectedCommit,
  [Parameter(Mandatory=$true)][string]$Action
)
. (Join-Path $PSScriptRoot 'ReviewWindow.ps1')
if($Action -cnotin (Get-WindowReviewActions)){throw 'Choose one documented display-only review action.'}
$installed=Get-Installed;$launchPath=Assert-LocalPath $LaunchResult
$launch=Read-Record $launchPath
$job=[pscustomobject]@{action='ReviewWindow';reviewAction=$Action;executable=$installed.executable;executableSha256=(Get-Digest $installed.executable);
  processId=$launch.pid;buildCommit=$ExpectedCommit;generation=$installed.state.current;launchResult=$launchPath;launchResultSha256=$ExpectedLaunchSha256;
  reviewHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'ReviewWindow.ps1'));nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'ReviewWindowNative.cs'));
  launchRequestSha256=(Get-Digest (Join-Path ([IO.Path]::GetDirectoryName($launchPath)) 'request.json'));workspace=(Assert-LocalPath $Workspace)}
$null=Read-WindowReview $job
# Screenshots can contain private chart positions. Give only this SID, SYSTEM
# and administrators access to the new evidence folder; originals are untouched.
. (Join-Path $PSScriptRoot 'Preparation.ps1')
$directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}) 'window-review'
$job | Add-Member -NotePropertyName evidenceDirectory -NotePropertyValue $directory
# Common's dispatcher supplies the same workspace to its durable request.
$job.PSObject.Properties.Remove('workspace')
$result=Invoke-InteractiveJob $Workspace $job
Write-Record (Join-Path $directory 'review.json') $result
$result | ConvertTo-Json -Depth 8
