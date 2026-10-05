# Preserve substantive current settings under explicit independent review.
# No profile/DLL mutation, launch permission or automatic-migration assertion.
[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$Record,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedRecordSha256,
  [Parameter(Mandatory=$true)][string]$Inspection,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedInspectionSha256,
  [Parameter(Mandatory=$true)][string]$PreservationReview,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedReviewSha256
)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$context=Get-CommissioningContext $Workspace;$record=Assert-LocalPath $Record;$parentDir=[IO.Path]::GetDirectoryName($record)
if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($parentDir) -ine (Join-Path $context.workspace 'runs') -or
   (Get-Digest $record) -cne $ExpectedRecordSha256){throw 'Expected exact original commissioning transaction.'}
$prepared=Read-SessionPreservationParent $record $ExpectedRecordSha256;$null=Get-PreparedCommissioningBaseline $prepared $parentDir $context.workspace
Assert-CommissioningContext $prepared.context $context
$lock=Open-CommissioningRestoreLock $parentDir
try {
$active=Read-Record (Join-Path $context.workspace 'commissioning-active.json')
if($active.owner -cne $script:CommissioningOwner -or $active.record -ine $record -or $active.recordSha256 -cne $ExpectedRecordSha256){throw 'Parent commissioning must remain active until its controlled restoration.'}
if(@(Get-ChildItem -LiteralPath $parentDir -Filter 'restore*.json' -Force | Where-Object {$_.Name -notlike 'restore-inspection-*'}).Count){throw 'Prepare preservation before restoration starts; incomplete restoration needs its exact existing proposal.'}
$inspectionPath=Assert-LocalPath $Inspection
if([IO.Path]::GetDirectoryName($inspectionPath) -ine $parentDir -or (Get-Digest $inspectionPath) -cne $ExpectedInspectionSha256){throw 'Exact closed-session inspection required.'}
$inspected=Read-Record $inspectionPath;$ini=Join-Path $context.profile 'opencpn.ini'
if($inspected.owner -cne 'OpenNavX.ReadOnlyCommissioning.RestoreInspection.1' -or $inspected.recordSha256 -cne $ExpectedRecordSha256 -or
   (Get-Digest $ini) -cne $inspected.currentIniSha256 -or (Get-Digest $inspected.savedIni) -cne $inspected.currentIniSha256){throw 'Current closed profile changed since inspection.'}
Assert-CommissioningContext $inspected.context $context;Assert-PreparationTree $inspected.profileBeforeRestore
Assert-PreparationAcl $inspected.currentAcl (Get-Acl -LiteralPath $ini).Sddl
$inventoryPath=Join-Path $parentDir 'inventory.json'
if((Get-Digest $inventoryPath) -cne $prepared.inventorySha256){throw 'Original complete plugin inventory changed.'}
$inventory=Read-Record $inventoryPath;Assert-CommissioningInventory $inventory $context
Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
if((Get-Digest $PreservationReview) -cne $ExpectedReviewSha256){throw 'Exact independent per-key preservation review required.'}
$review=Read-Record $PreservationReview
if($review.parentPreparedSha256 -cne $ExpectedRecordSha256 -or $review.inspectionSha256 -cne $ExpectedInspectionSha256){throw 'Preservation review belongs to another transaction/inspection.'}
$resourceProof=if($inspected.PSObject.Properties['resourceProof']){$inspected.resourceProof}else{$null}
$default=Assert-CommissioningResourceProof $prepared $resourceProof
if ($resourceProof) {
  # Verify the current owned stock locator again before freezing its historical
  # proof in this immutable preservation lineage. Later updates may retire a generation.
  $currentProof=Get-CommissioningResourceProof $prepared
  if (($currentProof | ConvertTo-Json -Compress) -cne ($resourceProof | ConvertTo-Json -Compress)) {
    throw 'Installed resource evidence changed since cold inspection.'
  }
}
$wmmProof=if($inspected.PSObject.Properties['wmmResourceProof']){$inspected.wmmResourceProof}else{$null}
$wmmProof=Assert-CommissioningWmmResourceProof $prepared $wmmProof
Assert-CommissioningWmmLiveProof $prepared $wmmProof
$changes=@(Assert-SessionPreservationReview (Join-Path $parentDir 'input-only.ini') $inspected.savedIni $review ([datetime]::UtcNow) $default $wmmProof)
$bytes=Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($inspected.savedIni))
$acls=Get-SessionPreservationAcls (@($inspected.profileBeforeRestore)+@($inventory.trees)) $prepared.quarantine
$directory=New-PreparationDirectory $context 'session-preservation'
Copy-PreparationFile $inspected.savedIni (Join-Path $directory 'post-session.ini') $inspected.currentIniSha256 (Get-Item -LiteralPath $inspected.savedIni).Length
Copy-PreparationFile $PreservationReview (Join-Path $directory 'preservation-review.json') $ExpectedReviewSha256 (Get-Item -LiteralPath $PreservationReview).Length
Copy-PreparationTree $inspected.profileBeforeRestore (Join-Path $directory 'profile-backup')
$baseline=Join-Path $directory 'baseline.ini'
$stream=New-Object IO.FileStream($baseline,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
try{$stream.Write($bytes,0,$bytes.Length);$stream.Flush($true)}finally{$stream.Dispose()}
Assert-PreparationTree $inspected.profileBeforeRestore;Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
Assert-SessionPreservationAcls $acls (Get-SessionPreservationAcls (@($inspected.profileBeforeRestore)+@($inventory.trees)) $prepared.quarantine) $ini
Assert-CommissioningWmmLiveProof $prepared $wmmProof
$proposal=Join-Path $directory 'proposal.json'
Write-Record $proposal @{schema=1;owner=$script:SessionPreservationOwner;status='proposed';createdUtc=[datetime]::UtcNow.ToString('o');
  parentPrepared=$record;parentPreparedSha256=$ExpectedRecordSha256;inspection=$inspectionPath;inspectionSha256=$ExpectedInspectionSha256;
  currentIniSha256=$inspected.currentIniSha256;reviewSha256=$ExpectedReviewSha256;baselineSha256=(Get-Digest $baseline);baselineBytes=$bytes.Length;
  provenance='current-user-state;origin-unverified';launchPermission=$false;acls=$acls;
  changedKeys=$changes.Count;onlyReversedConnectionByte=$true;applicationLaunched=$false;profileChanged=$false;sourceReviewStillRequired=$true}
$proof=Read-SessionPreservationProposal $context.workspace $proposal (Get-Digest $proposal) $record $ExpectedRecordSha256
Assert-SessionPreservationLiveState $proof (Get-CommissioningContext $Workspace)
[pscustomobject]@{status='proposed-only';proposal=$proposal;proposalSha256=(Get-Digest $proposal);baselineSha256=(Get-Digest $baseline);
  applicationLaunched=$false;profileChanged=$false;usableForNewCommissioning=$false;next='Explicit Restore with this preservation proposal; fresh Inventory and independent source/launch audit remain required.'} | ConvertTo-Json

} finally {$lock.Dispose()}
