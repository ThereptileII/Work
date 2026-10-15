# Explicit cold reconciliation of an old active transaction and later user install.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][ValidateSet('Inspect','Plan','Commit')][string]$Action,
  [string]$Workspace='C:\XNav',[string]$Record,[string]$ExpectedRecordSha256,
  [string]$Inspection,[string]$ExpectedInspectionSha256,[string]$PreservationReview,[string]$ExpectedReviewSha256,
  [string]$Plan,[string]$ExpectedPlanSha256
)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
Assert-NoManualPilotChild $Workspace
$context=Get-CommissioningContext $Workspace
if($Action -ceq 'Inspect') {$proof=Read-ReconciliationParent $context.workspace $Record $ExpectedRecordSha256}
elseif($Action -ceq 'Plan') {
  $inspectionPath=Assert-LocalPath $Inspection
  if($ExpectedInspectionSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $inspectionPath) -cne $ExpectedInspectionSha256){throw 'Exact inspection required.'}
  $observed=Read-Record $inspectionPath;$proof=Read-ReconciliationParent $context.workspace $observed.parentPrepared $observed.parentPreparedSha256
} else {$selected=Read-ReconciliationPlan $context.workspace $Plan $ExpectedPlanSha256;$proof=$selected.proof}
$profileLock=Open-CommissioningProfileLock $context.profile;$transactionLock=$null
try {
  $transactionLock=Open-CommissioningRestoreLock $proof.directory
  Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
  Assert-NoManualPilotChild $Workspace;Assert-ReconciliationLiveIdentity $proof $context
  $intentPath=Join-Path $proof.directory 'restore-reconciliation-intent.json'
  if($Action -cne 'Commit' -and @(Get-ChildItem -LiteralPath $proof.directory -Filter 'restore*.json' -Force|Where-Object {$_.Name -notlike 'restore-inspection-*'}).Count){throw 'Restoration already started; resume its exact pinned choice.'}
  if($Action -cne 'Commit'){Assert-ReconciliationActive $proof ''}
  $ini=Join-Path $context.profile 'opencpn.ini'
  if($Action -ceq 'Inspect') {
    $directory=New-PreparationDirectory $context 'cold-reconciliation'
    Assert-ReconciliationMoves $proof $directory
    $snapshot=Get-PreparationTree $context.profile
    # Only current-generation plugins are new roots. Shared/old roots remain
    # governed by the original exact inventory and each original quarantine.
    $currentTrees=@(Get-CommissioningTrees @($context.pluginRoots|Where-Object {$_ -cnotin $proof.parent.context.pluginRoots}))
    $acls=Get-SessionPreservationAcls (@($snapshot)+@($proof.inventory.trees)+$currentTrees) $proof.parent.quarantine
    Copy-PreparationTree $snapshot (Join-Path $directory 'profile-backup')
    $bytes=Get-ReconciliationProfileBytes $proof $ini
    $normalized=Join-Path $directory 'normalized-input.ini'
    $stream=New-Object IO.FileStream($normalized,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try{$stream.Write($bytes.input,0,$bytes.input.Length);$stream.Flush($true)}finally{$stream.Dispose()}
    Assert-ReconciliationMoves $proof $directory;foreach($tree in $currentTrees){Assert-PreparationTree $tree}
    Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
    $output=Join-Path $directory 'inspection.json'
    Write-Record $output @{schema=1;owner=$script:ReconciliationOwner;status='inspected';parentPrepared=$proof.record;parentPreparedSha256=$proof.sha256;
      currentContext=$context;activeMarkerSha256=(Get-Digest (Join-Path $context.workspace 'commissioning-active.json'));
      profileTree=$snapshot;currentPluginTrees=$currentTrees;acls=$acls;currentIniSha256=(Get-Digest $ini);
      connectionDisposition=$bytes.disposition;diff=@(Get-CommissioningIniDiff (Join-Path $proof.directory 'input-only.ini') $ini);applicationLaunched=$false}
    [pscustomobject]@{status='review-required';inspection=$output;inspectionSha256=(Get-Digest $output);profileChanged=$false;applicationLaunched=$false}|ConvertTo-Json
    return
  }
  if($Action -ceq 'Plan') {
    $directory=[IO.Path]::GetDirectoryName($inspectionPath)
    if([IO.Path]::GetFileName($inspectionPath) -cne 'inspection.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $context.workspace 'runs') -or
       [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-reconciliation-[a-f0-9]{8}$' -or
       $observed.owner -cne $script:ReconciliationOwner -or $observed.status -cne 'inspected'){throw 'Owned private reconciliation inspection required.'}
    Assert-ColdPrivateEvidence $directory $context.sid
    if($ExpectedReviewSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $PreservationReview) -cne $ExpectedReviewSha256){throw 'Exact independent preservation review required.'}
    $review=Read-Record $PreservationReview
    if($review.inspectionSha256 -cne $ExpectedInspectionSha256){throw 'Review belongs to another inspection.'}
    $saved=Join-Path $directory 'profile-backup/opencpn.ini';$at=[datetime]::UtcNow
    $changes=@(Assert-ReconciliationReview $proof $saved (Join-Path $directory 'normalized-input.ini') $review $at)
    $baseline=Join-Path $directory 'baseline.ini';$bytes=(Get-ReconciliationProfileBytes $proof $saved).baseline
    $stream=New-Object IO.FileStream($baseline,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try{$stream.Write($bytes,0,$bytes.Length);$stream.Flush($true)}finally{$stream.Dispose()}
    Copy-PreparationFile $PreservationReview (Join-Path $directory 'preservation-review.json') $ExpectedReviewSha256 (Get-Item -LiteralPath $PreservationReview).Length
    $output=Join-Path $directory 'plan.json'
    Write-Record $output @{schema=1;owner=$script:ReconciliationOwner;status='planned';createdUtc=$at.ToString('o');parentPrepared=$proof.record;parentPreparedSha256=$proof.sha256;
      inspectionSha256=$ExpectedInspectionSha256;reviewSha256=$ExpectedReviewSha256;baselineSha256=(Get-Digest $baseline);baselineBytes=$bytes.Length;changedKeys=$changes.Count;
      provenance='explicit-cross-generation-reconciliation;current-user-state-origin-unverified';launchPermission=$false}
    $selected=Read-ReconciliationPlan $context.workspace $output (Get-Digest $output)
    Assert-ReconciliationPlanLive $selected $context
    [pscustomobject]@{status='planned-only';plan=$output;planSha256=$selected.sha256;profileChanged=$false;applicationLaunched=$false}|ConvertTo-Json
    return
  }
  $resume=Test-Path -LiteralPath $intentPath
  if($resume) {
    $intent=Read-Record $intentPath
    if($intent.owner -cne $script:ReconciliationOwner -or $intent.parentPreparedSha256 -cne $proof.sha256 -or
       $intent.plan -ine $selected.path -or $intent.planSha256 -cne $selected.sha256){throw 'Reconciliation already has another durable plan; no substitution.'}
  } elseif(@(Get-ChildItem -LiteralPath $proof.directory -Filter 'restore*.json' -Force|Where-Object {$_.Name -notlike 'restore-inspection-*'}).Count){throw 'Another restoration has begun.'}
  $marker=Join-Path $context.workspace 'commissioning-active.json'
  if(-not(Test-Path -LiteralPath $marker)){
    # Marker removal is the final durable effect. An exact completed transaction
    # can return its existing receipt without writing or restoring anything.
    if(-not $resume){throw 'No active transaction or pinned reconciliation intent.'}
    $baselineRecord=Join-Path $selected.directory 'reconciled-baseline.json'
    $null=Read-CommissioningBaseline $context.workspace $baselineRecord (Get-Digest $baselineRecord)
    Assert-ReconciliationPlanLive $selected $context -Resume -Completed
    Assert-CommissioningTrees $proof.inventory.trees $proof.parent.quarantine
    Get-ReconciliationReceipt $selected|ConvertTo-Json
    return
  }
  Assert-ReconciliationPlanLive $selected $context -Resume:$resume
  if(-not $resume){Write-Record $intentPath @{owner=$script:ReconciliationOwner;plan=$selected.path;planSha256=$selected.sha256;parentPreparedSha256=$proof.sha256;applicationLaunched=$false}}
  Assert-ReconciliationPlanLive $selected (Get-CommissioningContext $Workspace) -Resume
  if((Get-Digest $ini) -cne $selected.value.baselineSha256) {
    Publish-ReconciliationProfile $selected $context
  }
  $index=0
  foreach($item in @($proof.parent.quarantine)) {
    Assert-ReconciliationPlanLive $selected (Get-CommissioningContext $Workspace) -Resume
    $journal=Join-Path $selected.directory ('plugin-'+$index+'.json');$index++
    if([IO.File]::Exists($item.destination)) {
      if(-not(Test-Path -LiteralPath $journal)){Write-Record $journal @{owner=$script:ReconciliationOwner;parentPreparedSha256=$proof.sha256;source=$item.destination;destination=$item.path;sha256=$item.sha256}}
      Restore-ReconciliationPlugin $item
    }
  }
  Assert-ReconciliationPlanLive $selected (Get-CommissioningContext $Workspace) -Resume
  Assert-CommissioningTrees $proof.inventory.trees $proof.parent.quarantine
  $completion=Join-Path $selected.directory 'completion.json'
  if(-not(Test-Path -LiteralPath $completion)){Write-Record $completion @{schema=1;owner=$script:ReconciliationOwner;status='reconciled';planSha256=$selected.sha256;parentPreparedSha256=$proof.sha256;
    currentContext=$context;profileSha256=$selected.value.baselineSha256;originalPathsRestored=$true;otherProfileFilesPreserved=$true;noPluginTransplant=$true;doNotAutoLaunch=$true;applicationLaunched=$false}}
  $baselineRecord=Join-Path $selected.directory 'reconciled-baseline.json'
  if(-not(Test-Path -LiteralPath $baselineRecord)){Write-Record $baselineRecord @{schema=1;owner=$script:ReconciliationOwner;status='reconciled';planSha256=$selected.sha256;completionSha256=(Get-Digest $completion);
    baselineSha256=$selected.value.baselineSha256;baselineBytes=$selected.value.baselineBytes;provenance=$selected.value.provenance;launchPermission=$false}}
  $null=Read-CommissioningBaseline $context.workspace $baselineRecord (Get-Digest $baselineRecord)
  Assert-ReconciliationPlanLive $selected (Get-CommissioningContext $Workspace) -Resume
  # Remove only the still-identical active marker after verified durable effects.
  Remove-ReconciliationActive $selected
  Get-ReconciliationReceipt $selected|ConvertTo-Json
} finally {if($transactionLock){$transactionLock.Dispose()};$profileLock.Dispose()}
