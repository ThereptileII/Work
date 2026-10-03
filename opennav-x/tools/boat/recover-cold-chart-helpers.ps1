# Capture an existing stock-helper set; close it only with an independent,
# caller-supplied exact-set review. No application/profile/plugin operation.
[CmdletBinding()]
param(
 [ValidateSet('Capture','Close')][string]$Action='Capture',
 [string]$Workspace='C:\XNav',
 [string]$CaptureRecord,[string]$ExpectedCaptureSha256,
 [string]$Review,[string]$ExpectedReviewSha256
)
. (Join-Path $PSScriptRoot 'ColdChartHelper.ps1')
if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Native Windows is required.'}
$context=Get-CommissioningIdentityContext $Workspace
if(Test-Path -LiteralPath (Join-Path $context.workspace 'commissioning-active.json')){throw 'An active commissioning transaction requires its existing recovery path.'}
$profile=Get-PreparationTree $context.profile
if(-not $profile.exists -or -not (Test-Path -LiteralPath (Join-Path $context.profile 'opencpn.ini'))){throw 'Existing normal profile is required.'}
$trees=@(Get-CommissioningTrees $context.pluginRoots)
$ini=Join-Path $context.profile 'opencpn.ini'
$iniAcl=(Get-Acl -LiteralPath $ini).Sddl
$helpers=@(Get-ColdHelperSet $context)
if($Action -ceq 'Capture'){
  if($CaptureRecord -or $ExpectedCaptureSha256 -or $Review -or $ExpectedReviewSha256){throw 'Capture accepts no review or prior capture.'}
  $directory=New-PreparationDirectory $context 'cold-chart-helper'
  Write-Record (Join-Path $directory 'capture-planned.json') @{owner=$script:ColdHelperOwner;createdUtc=[datetime]::UtcNow.ToString('o');helpers=$helpers;noRetry=$true}
  Copy-PreparationTree $profile (Join-Path $directory 'profile-backup')
  Assert-PreparationTree $profile
  foreach($tree in $trees){Assert-PreparationTree $tree}
  Assert-PreparationAcl $iniAcl (Get-Acl -LiteralPath $ini).Sddl
  Assert-CommissioningContext $context (Get-CommissioningIdentityContext $Workspace)
  Assert-ColdHelperRemaining $helpers $context @()
  Assert-ColdPrivateEvidence $directory $context.sid
  $record=Join-Path $directory 'capture.json'
  Write-Record $record @{schema=1;owner=$script:ColdHelperOwner;status='captured';createdUtc=[datetime]::UtcNow.ToString('o');
    context=$context;helpers=$helpers;profile=$profile;pluginTrees=$trees;iniAcl=$iniAcl;
    profileChanged=$false;applicationLaunched=$false;launchPermission=$false}
  [pscustomobject]@{status='captured-review-required';captureRecord=$record;captureSha256=(Get-Digest $record);
    helpers=$helpers;profileChanged=$false;applicationLaunched=$false}|ConvertTo-Json -Depth 8
  return
}
if($ExpectedCaptureSha256 -cnotmatch '^[a-f0-9]{64}$' -or $ExpectedReviewSha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact capture and independent review hashes required.'}
$record=Assert-LocalPath $CaptureRecord;$directory=[IO.Path]::GetDirectoryName($record)
if([IO.Path]::GetFileName($record) -cne 'capture.json' -or
   [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $context.workspace 'runs') -or
   [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-chart-helper-[a-f0-9]{8}$' -or
   (Get-Digest $record) -cne $ExpectedCaptureSha256){throw 'Exact private cold-helper capture required.'}
$capture=Read-Record $record
Assert-ColdPrivateEvidence $directory $context.sid
Assert-ColdHelperCapture $capture $context
$reviewPath=Assert-LocalPath $Review
if((Get-Digest $reviewPath) -cne $ExpectedReviewSha256){throw 'Independent review changed.'}
$approval=Read-Record $reviewPath
Assert-ColdHelperReview $approval $capture $ExpectedCaptureSha256
if(Test-Path -LiteralPath (Join-Path $directory 'review.json')){throw 'Cold helper closure was already started; no retry.'}
Copy-PreparationFile $reviewPath (Join-Path $directory 'review.json') $ExpectedReviewSha256 (Get-Item -LiteralPath $reviewPath).Length
Add-Type -Path (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs')
$completed=New-Object 'Collections.Generic.List[int]'
foreach($helper in @($capture.helpers)){
  try {
    if(Test-Path -LiteralPath (Join-Path $context.workspace 'commissioning-active.json')){throw 'Commissioning became active during cold helper recovery.'}
    Assert-CommissioningContext $capture.context (Get-CommissioningIdentityContext $Workspace)
    Assert-PreparationTree $capture.profile
    foreach($tree in $capture.pluginTrees){Assert-PreparationTree $tree}
    Assert-PreparationAcl $capture.iniAcl (Get-Acl -LiteralPath $ini).Sddl
    $backup=Get-PreparationTree (Join-Path $directory 'profile-backup')
    if(($backup.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($capture.profile.entries|ConvertTo-Json -Depth 8 -Compress)){throw 'Private profile copy changed.'}
    Assert-ColdPrivateEvidence $directory $context.sid
    Assert-ColdHelperRemaining $capture.helpers $context $completed.ToArray()
    $intent=Join-Path $directory ('intent-'+$helper.pid+'.json')
    Write-Record $intent @{owner=$script:ColdHelperOwner;captureSha256=$ExpectedCaptureSha256;reviewSha256=$ExpectedReviewSha256;
      helper=$helper;nativeSha256=(Get-Digest (Join-Path $PSScriptRoot 'ChartHelperShutdownNative.cs'));
      command=2;bytes=1025;noRetry=$true;forceTermination=$false;utc=[datetime]::UtcNow.ToString('o')}
    $ticks=([datetime]($helper.startedUtc)).ToUniversalTime().Ticks
    $null=New-ChartHelperGlobalLocator $context.workspace $context.sid ([int]$helper.pid) $ticks $intent $script:ColdHelperOwner
    $result=[OpenNavX.ChartHelperShutdownNative]::Shutdown([int]$helper.pid,$ticks,[int]$helper.parentPid,[int]$helper.sessionId,[string]$helper.path)
    Write-Record (Join-Path $directory ('result-'+$helper.pid+'.json')) $result
    if(-not $result.Succeeded){throw 'Native helper exit is not proven; no retry.'}
    $completed.Add([int]$helper.pid)
  } catch {
    Write-Record (Join-Path $directory ('failure-'+$helper.pid+'.json')) @{owner=$script:ColdHelperOwner;pid=$helper.pid;
      completed=@($completed.ToArray());failure=$_.Exception.Message;utc=[datetime]::UtcNow.ToString('o');noRetry=$true}
    throw
  }
}
Assert-ColdHelperRemaining $capture.helpers $context $completed.ToArray()
if(Test-Path -LiteralPath (Join-Path $context.workspace 'commissioning-active.json')){throw 'Commissioning became active during cold helper recovery.'}
Assert-CommissioningContext $capture.context (Get-CommissioningIdentityContext $Workspace)
Assert-PreparationTree $capture.profile
foreach($tree in $capture.pluginTrees){Assert-PreparationTree $tree}
Assert-PreparationAcl $capture.iniAcl (Get-Acl -LiteralPath $ini).Sddl
Assert-ColdPrivateEvidence $directory $context.sid
$backup=Get-PreparationTree (Join-Path $directory 'profile-backup')
if(($backup.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($capture.profile.entries|ConvertTo-Json -Depth 8 -Compress)){throw 'Private profile copy changed before closure publication.'}
Write-Record (Join-Path $directory 'closed.json') @{owner=$script:ColdHelperOwner;status='closed';captureSha256=$ExpectedCaptureSha256;
  reviewSha256=$ExpectedReviewSha256;completed=@($completed.ToArray());utc=[datetime]::UtcNow.ToString('o');
  profileChanged=$false;applicationLaunched=$false;launchPermission=$false}
[pscustomobject]@{status='closed-cold-capture-required';evidence=(Join-Path $directory 'closed.json');
  completed=@($completed.ToArray());profileChanged=$false;applicationLaunched=$false}|ConvertTo-Json -Depth 5
