[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$LaunchResult,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchSha256,
  [Parameter(Mandatory=$true)][ValidateSet('Capture','Resize1280x800','ZoomOut','Close','InspectWelcome','FocusWelcome','AcknowledgeWelcome')][string]$Action,
  [string]$WelcomeInspection,
  [ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedWelcomeInspectionSha256
)
. (Join-Path $PSScriptRoot 'StockReview.ps1')
$config=Get-StockTarget $Workspace;$path=Assert-LocalPath $LaunchResult;$launch=Read-Record $path
$job=[pscustomobject]@{action='ReviewStock';reviewAction=$Action;workspace=(Assert-LocalPath $Workspace);executable=$config.stockExecutable;
  executableSha256=(Get-Digest $config.stockExecutable);processId=$launch.pid;launchResult=$path;launchResultSha256=$ExpectedLaunchSha256;
  launchRequestSha256=(Get-Digest (Join-Path ([IO.Path]::GetDirectoryName($path)) 'request.json'));
  reviewHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'StockReview.ps1'));nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'StockReviewNative.cs'))}
if ($Action -cin @('InspectWelcome','FocusWelcome','AcknowledgeWelcome')) {
  $job | Add-Member -NotePropertyName welcomeHelperSha256 -NotePropertyValue (Get-Digest (Join-Path $PSScriptRoot 'StockWelcome.ps1'))
  $job | Add-Member -NotePropertyName welcomeNativeSha256 -NotePropertyValue (Get-Digest (Join-Path $PSScriptRoot 'StockWelcomeNative.cs'))
}
if ($Action -ceq 'AcknowledgeWelcome') {
  if (-not $WelcomeInspection -or -not $ExpectedWelcomeInspectionSha256) { throw 'Capture/review the warning first, then supply that inspection record and exact hash.' }
  $job | Add-Member -NotePropertyName welcomeInspection -NotePropertyValue (Assert-LocalPath $WelcomeInspection)
  $job | Add-Member -NotePropertyName welcomeInspectionSha256 -NotePropertyValue $ExpectedWelcomeInspectionSha256
} elseif ($WelcomeInspection -or $ExpectedWelcomeInspectionSha256) { throw 'Warning acknowledgement evidence belongs only to AcknowledgeWelcome.' }
$null=Read-StockReview $job
$directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}) 'stock-review'
$job | Add-Member -NotePropertyName evidenceDirectory -NotePropertyValue $directory
$job.PSObject.Properties.Remove('workspace')
$result=Invoke-InteractiveJob $Workspace $job
Write-Record (Join-Path $directory 'review.json') $result
$result | ConvertTo-Json -Depth 8
