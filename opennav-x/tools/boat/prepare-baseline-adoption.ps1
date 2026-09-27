# Prepare a separately reviewed migration target only. No profile/DLL mutation,
# launch, source approval or change to the fixed recovered-root baseline.
[CmdletBinding()]
param(
  [string]$Workspace='C:\XNav',
  [Parameter(Mandatory=$true)][string]$Record,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedRecordSha256,
  [Parameter(Mandatory=$true)][string]$Inspection,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedInspectionSha256,
  [Parameter(Mandatory=$true)][string]$MigrationReview,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedReviewSha256
)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$context=Get-CommissioningContext $Workspace;$record=Assert-LocalPath $Record;$parentDir=[IO.Path]::GetDirectoryName($record)
if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($parentDir) -ine (Join-Path $context.workspace 'runs') -or
   (Get-Digest $record) -cne $ExpectedRecordSha256){throw 'Expected exact original commissioning transaction.'}
$prepared=Read-Record $record;$null=Get-PreparedCommissioningBaseline $prepared $parentDir $context.workspace
Assert-CommissioningContext $prepared.context $context
$active=Read-Record (Join-Path $context.workspace 'commissioning-active.json')
if($active.owner -cne $script:CommissioningOwner -or $active.record -ine $record -or $active.recordSha256 -cne $ExpectedRecordSha256){throw 'Parent commissioning must remain active until its controlled restoration.'}
if(@(Get-ChildItem -LiteralPath $parentDir -Filter 'restore*.json' -Force | Where-Object {$_.Name -notlike 'restore-inspection-*'}).Count){throw 'Begin adoption before restoration starts; incomplete restoration needs its existing proposal.'}
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
if((Get-Digest $MigrationReview) -cne $ExpectedReviewSha256){throw 'Exact independent per-key migration review required.'}
$review=Read-Record $MigrationReview
if($review.parentPreparedSha256 -cne $ExpectedRecordSha256 -or $review.inspectionSha256 -cne $ExpectedInspectionSha256){throw 'Migration review belongs to another transaction/inspection.'}
$resourceProof=if($inspected.PSObject.Properties['resourceProof']){$inspected.resourceProof}else{$null}
$default=Assert-CommissioningResourceProof $prepared $resourceProof
if ($resourceProof) {
  # Verify the current owned stock locator again before freezing its historical
  # proof in this immutable adoption lineage. Later updates may retire a generation.
  $currentProof=Get-CommissioningResourceProof $prepared
  if (($currentProof | ConvertTo-Json -Compress) -cne ($resourceProof | ConvertTo-Json -Compress)) {
    throw 'Installed resource evidence changed since cold inspection.'
  }
}
$changes=@(Assert-CommissioningMigrationReview (Join-Path $parentDir 'input-only.ini') $inspected.savedIni $review ([datetime]::UtcNow) $default)
$bytes=Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($inspected.savedIni))
$directory=New-PreparationDirectory $context 'baseline-adoption'
Copy-PreparationFile $inspected.savedIni (Join-Path $directory 'post-session.ini') $inspected.currentIniSha256 (Get-Item -LiteralPath $inspected.savedIni).Length
Copy-PreparationFile $MigrationReview (Join-Path $directory 'migration-review.json') $ExpectedReviewSha256 (Get-Item -LiteralPath $MigrationReview).Length
$baseline=Join-Path $directory 'baseline.ini'
$stream=New-Object IO.FileStream($baseline,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
try{$stream.Write($bytes,0,$bytes.Length);$stream.Flush($true)}finally{$stream.Dispose()}
Assert-PreparationTree $inspected.profileBeforeRestore;Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
$proposal=Join-Path $directory 'proposal.json'
Write-Record $proposal @{schema=1;owner=$script:CommissioningBaselineOwner;status='proposed';createdUtc=[datetime]::UtcNow.ToString('o');
  parentPrepared=$record;parentPreparedSha256=$ExpectedRecordSha256;inspection=$inspectionPath;inspectionSha256=$ExpectedInspectionSha256;
  currentIniSha256=$inspected.currentIniSha256;reviewSha256=$ExpectedReviewSha256;baselineSha256=(Get-Digest $baseline);baselineBytes=$bytes.Length;
  changedKeys=$changes.Count;onlyReversedConnectionByte=$true;applicationLaunched=$false;profileChanged=$false;sourceReviewStillRequired=$true}
$null=Read-CommissioningAdoptionProposal $context.workspace $proposal (Get-Digest $proposal) $record $ExpectedRecordSha256
[pscustomobject]@{status='proposed-only';proposal=$proposal;proposalSha256=(Get-Digest $proposal);baselineSha256=(Get-Digest $baseline);
  applicationLaunched=$false;profileChanged=$false;usableForNewCommissioning=$false;next='Explicit Restore with this adoption proposal, then fresh Inventory/source review.'} | ConvertTo-Json
