# Preserve a closed, pre-existing user profile for explicit later commissioning.
# Both phases write only private workspace evidence. Neither changes the live
# profile, plugins, application or connection state.
[CmdletBinding()]
param(
  [ValidateSet('Capture','Complete')][string]$Action='Capture',
  [string]$Workspace='C:\XNav',
  [string]$PredecessorRecord,[string]$ExpectedPredecessorSha256,
  [string]$CaptureRecord,[string]$ExpectedCaptureSha256,
  [string]$Review,[string]$ExpectedReviewSha256
)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')

$context=Get-CommissioningContext $Workspace
$ini=Join-Path $context.profile 'opencpn.ini'
if(Test-Path -LiteralPath (Join-Path $context.workspace 'commissioning-active.json')){throw 'Complete or inspect the active commissioning transaction first.'}
$live=Get-PreparationTree $context.profile
if(-not $live.exists){throw 'Actual normal profile is missing.'}
$initialIni=Get-Item -LiteralPath $ini -Force
$initialLastWriteTicks=$initialIni.LastWriteTimeUtc.Ticks
$initialCreationTicks=$initialIni.CreationTimeUtc.Ticks
$originalAcl=(Get-Acl -LiteralPath $ini).Sddl
$values=Read-ProfileForAudit $ini
if($values['Directories/pluginInstallDir']){throw 'Custom plugin loader requires a separate path review.'}
$input=Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($ini))
$trees=@(Get-CommissioningTrees $context.pluginRoots)
foreach($tree in $trees){Assert-PreparationTree $tree}

if($Action -ceq 'Capture') {
  if($CaptureRecord -or $ExpectedCaptureSha256 -or $Review -or $ExpectedReviewSha256){throw 'Capture does not accept later review arguments.'}
  if(-not $PredecessorRecord -or $ExpectedPredecessorSha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'An exact completed predecessor is required.'}
  $prior=Read-CommissioningBaseline $context.workspace $PredecessorRecord $ExpectedPredecessorSha256
  if(-not $prior.record -or $prior.sid -cne $context.sid -or $prior.profile -ine $context.profile){throw 'Predecessor must belong to the same actual account/profile.'}
  if((Get-Digest $ini) -cne $live.entries.Where({$_.path -ceq 'opencpn.ini'})[0].sha256){throw 'Profile changed during initial inventory.'}
  $directory=New-PreparationDirectory $context 'cold-baseline'
  $planned=Join-Path $directory 'capture-planned.json'
  Write-Record $planned @{schema=1;owner=$script:ColdCaptureOwner;status='planned';createdUtc=[datetime]::UtcNow.ToString('o');
    predecessorRecord=$prior.record;predecessorSha256=$prior.recordSha256;profileSha256=(Get-Digest $ini);profileBytes=(Get-Item -LiteralPath $ini).Length;
    provenance='pre-existing-current-user-state;origin-unverified';profileChanged=$false;applicationLaunched=$false}
  $backup=Join-Path $directory 'profile-backup'
  Copy-PreparationTree $live $backup
  $old=Join-Path $directory 'predecessor.ini'
  # The completed predecessor reader verifies its immutable bytes. Copy those
  # bytes into this private review pair without assuming an earlier session.
  $priorIni=if([IO.Path]::GetFileName($prior.record) -ceq 'completed-cold-baseline.json') {
    Join-Path ([IO.Path]::GetDirectoryName($prior.record)) 'profile-backup\opencpn.ini'
  } else {Join-Path ([IO.Path]::GetDirectoryName($prior.record)) 'baseline.ini'}
  Copy-PreparationFile $priorIni $old $prior.sha256 $prior.bytes
  $saved=Join-Path $backup 'opencpn.ini'
  if((Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($saved)))) -cne (Get-CommissioningHash $input)){
    throw 'Cold copied bytes differ from the proposed one-byte input transform.'
  }
  $inputPath=Join-Path $directory 'input-only.ini'
  $stream=New-Object IO.FileStream($inputPath,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
  try{$stream.Write($input,0,$input.Length);$stream.Flush($true)}finally{$stream.Dispose()}
  Assert-InputOnlyProfile (Read-ProfileForAudit $inputPath)
  Assert-PreparationTree $live
  foreach($tree in $trees){Assert-PreparationTree $tree}
  Assert-PreparationAcl $originalAcl (Get-Acl -LiteralPath $ini).Sddl
  Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
  if(Test-Path -LiteralPath (Join-Path $context.workspace 'commissioning-active.json')){throw 'Commissioning started during cold capture.'}
  if((Get-Digest $ini) -cne (Get-Digest $saved)){throw 'Profile changed before capture publication.'}
  $finalIni=Get-Item -LiteralPath $ini -Force
  if($finalIni.LastWriteTimeUtc.Ticks -ne $initialLastWriteTicks -or $finalIni.CreationTimeUtc.Ticks -ne $initialCreationTicks){throw 'Profile metadata changed during cold capture.'}
  $record=Join-Path $directory 'capture.json'
  Assert-ColdPrivateEvidence $directory $context.sid
  Write-Record $record @{schema=1;owner=$script:ColdCaptureOwner;status='captured';createdUtc=[datetime]::UtcNow.ToString('o');
    predecessorRecord=$prior.record;predecessorSha256=$prior.recordSha256;context=$context;profileTree=$live;pluginTrees=$trees;
    profileSha256=(Get-Digest $saved);profileBytes=(Get-Item -LiteralPath $saved).Length;profileAcl=$originalAcl;
    sourceIniLastWriteTicks=$initialLastWriteTicks;sourceIniCreationTicks=$initialCreationTicks;
    inputSha256=(Get-Digest $inputPath);stockSha256=(Get-Digest $context.executable);
    provenance='pre-existing-current-user-state;origin-unverified';
    profileChanged=$false;applicationLaunched=$false;launchPermission=$false}
  [pscustomobject]@{status='captured-review-required';captureRecord=$record;captureSha256=(Get-Digest $record);
    profileSha256=(Get-Digest $saved);predecessorSha256=$prior.sha256;applicationLaunched=$false;profileChanged=$false} | ConvertTo-Json
  return
}

if($PredecessorRecord -or $ExpectedPredecessorSha256){throw 'Complete uses only the captured predecessor binding.'}
if($ExpectedCaptureSha256 -cnotmatch '^[a-f0-9]{64}$' -or $ExpectedReviewSha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact capture and independent review hashes required.'}
$capturePath=Assert-LocalPath $CaptureRecord;$directory=[IO.Path]::GetDirectoryName($capturePath)
if([IO.Path]::GetFileName($capturePath) -cne 'capture.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $context.workspace 'runs') -or
   [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-baseline-[a-f0-9]{8}$' -or
   (Get-Digest $capturePath) -cne $ExpectedCaptureSha256){throw 'Expected immutable private cold capture.'}
$captured=Read-Record $capturePath
if($captured.schema -ne 1 -or $captured.owner -cne $script:ColdCaptureOwner -or $captured.status -cne 'captured' -or
   $captured.provenance -cne 'pre-existing-current-user-state;origin-unverified' -or $captured.launchPermission -isnot [bool] -or $captured.launchPermission){throw 'Capture is incomplete or claims launch authority.'}
if((Assert-LocalPath $captured.context.workspace) -ine $context.workspace -or
   (Assert-LocalPath $captured.profileTree.root) -ine $context.profile -or
   $captured.profileTree.exists -isnot [bool] -or -not $captured.profileTree.exists){throw 'Capture belongs to another workspace or profile.'}
Assert-ColdPrivateEvidence $directory $context.sid
Assert-CommissioningContext $captured.context $context
$prior=Read-CommissioningBaseline $context.workspace $captured.predecessorRecord $captured.predecessorSha256
if($prior.sid -cne $context.sid -or $prior.profile -ine $context.profile){throw 'Predecessor identity changed.'}
$saved=Join-Path $directory 'profile-backup\opencpn.ini';$before=Join-Path $directory 'predecessor.ini'
if((Get-Digest $before) -cne $prior.sha256 -or (Get-Item -LiteralPath $before).Length -ne $prior.bytes -or
   (Get-Digest $saved) -cne $captured.profileSha256 -or (Get-Item -LiteralPath $saved).Length -ne $captured.profileBytes -or
   (Get-Digest $ini) -cne $captured.profileSha256){throw 'Reviewed predecessor/current bytes changed.'}
if((Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($saved)))) -cne $captured.inputSha256 -or
   (Get-Digest (Join-Path $directory 'input-only.ini')) -cne $captured.inputSha256){throw 'Copied profile does not match one-byte input transform.'}
Assert-PreparationTree $captured.profileTree
foreach($tree in $captured.pluginTrees){Assert-PreparationTree $tree}
$backupTree=Get-PreparationTree (Join-Path $directory 'profile-backup')
if(($backupTree.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($captured.profileTree.entries|ConvertTo-Json -Depth 8 -Compress)){throw 'Cold backup differs from complete original profile.'}
Assert-PreparationAcl $captured.profileAcl (Get-Acl -LiteralPath $ini).Sddl
$liveIni=Get-Item -LiteralPath $ini -Force
if($liveIni.LastWriteTimeUtc.Ticks -ne $captured.sourceIniLastWriteTicks -or
   $liveIni.CreationTimeUtc.Ticks -ne $captured.sourceIniCreationTicks){throw 'Current profile metadata changed since capture.'}
$reviewPath=Assert-LocalPath $Review
if((Get-Digest $reviewPath) -cne $ExpectedReviewSha256){throw 'Independent exact-key review changed.'}
$reviewData=Read-Record $reviewPath
if($reviewData.captureSha256 -cne $ExpectedCaptureSha256 -or $reviewData.predecessorRecordSha256 -cne $captured.predecessorSha256){throw 'Review belongs to another capture or predecessor.'}
$changes=@(Assert-ColdBaselineDelta $before $saved $reviewData)
$reviewCopy=Join-Path $directory 'review.json'
if(Test-Path -LiteralPath $reviewCopy){throw 'Existing review copy requires a fresh cold capture; do not overwrite partial evidence.'}
Copy-PreparationFile $reviewPath $reviewCopy $ExpectedReviewSha256 (Get-Item -LiteralPath $reviewPath).Length
Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
Assert-PreparationTree $captured.profileTree
foreach($tree in $captured.pluginTrees){Assert-PreparationTree $tree}
Assert-PreparationAcl $captured.profileAcl (Get-Acl -LiteralPath $ini).Sddl
if((Get-Item -LiteralPath $ini -Force).LastWriteTimeUtc.Ticks -ne $captured.sourceIniLastWriteTicks){throw 'Profile metadata changed before completion.'}
if((Get-Digest $ini) -cne $captured.profileSha256){throw 'Current profile changed before completion.'}
if(Test-Path -LiteralPath (Join-Path $context.workspace 'commissioning-active.json')){throw 'Commissioning started during cold completion.'}
Assert-ColdPrivateEvidence $directory $context.sid
$complete=Join-Path $directory 'completed-cold-baseline.json'
if(Test-Path -LiteralPath $complete){throw 'A completed cold baseline already exists.'}
Write-Record $complete @{schema=1;owner=$script:ColdBaselineOwner;status='completed';createdUtc=[datetime]::UtcNow.ToString('o');
  captureRecord=$capturePath;captureSha256=$ExpectedCaptureSha256;reviewSha256=$ExpectedReviewSha256;
  predecessorRecord=$captured.predecessorRecord;predecessorSha256=$captured.predecessorSha256;
  baselineSha256=$captured.profileSha256;baselineBytes=$captured.profileBytes;changedKeys=$changes.Count;
  provenance='pre-existing-current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;
  profileChanged=$false;applicationLaunched=$false}
[pscustomobject]@{status='completed-preservation-only';baselineRecord=$complete;baselineRecordSha256=(Get-Digest $complete);
  profileSha256=$captured.profileSha256;launchPermission=$false;profileChanged=$false;applicationLaunched=$false} | ConvertTo-Json
