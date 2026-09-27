[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$LaunchResult,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedLaunchSha256,
  [Parameter(Mandatory=$true)][ValidateSet('Capture','Resize1280x800','Close')][string]$Action
)
. (Join-Path $PSScriptRoot 'StockReview.ps1')
$config=Get-StockTarget $Workspace;$path=Assert-LocalPath $LaunchResult;$launch=Read-Record $path
$job=[pscustomobject]@{action='ReviewStock';reviewAction=$Action;workspace=(Assert-LocalPath $Workspace);executable=$config.stockExecutable;
  executableSha256=(Get-Digest $config.stockExecutable);processId=$launch.pid;launchResult=$path;launchResultSha256=$ExpectedLaunchSha256;
  launchRequestSha256=(Get-Digest (Join-Path ([IO.Path]::GetDirectoryName($path)) 'request.json'));
  reviewHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'StockReview.ps1'));nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'StockReviewNative.cs'))}
$null=Read-StockReview $job
$directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}) 'stock-review'
$job | Add-Member -NotePropertyName evidenceDirectory -NotePropertyValue $directory
$job.PSObject.Properties.Remove('workspace')
$result=Invoke-InteractiveJob $Workspace $job
Write-Record (Join-Path $directory 'review.json') $result
$result | ConvertTo-Json -Depth 8
