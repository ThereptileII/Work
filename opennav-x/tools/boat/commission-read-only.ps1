# Explicit local preparation only. This tool never starts OpenCPN or sends data.
[CmdletBinding()]
param(
  [ValidateSet('Inventory','Prepare','Apply','InspectRestore','Restore')][string]$Action='Inventory',
  [string]$Workspace='C:\XNav',
  [string]$Plan,[string]$ExpectedPlanSha256,
  [string]$Record,[string]$ExpectedRecordSha256,
  [string]$Inspection,[string]$ExpectedInspectionSha256,
  [string]$ReviewedCurrentIniSha256
)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$context=Get-CommissioningContext $Workspace
$ini=Join-Path $context.profile 'opencpn.ini'
$active=Join-Path $context.workspace 'commissioning-active.json'
function Read-PinnedCommissioningRecord([string]$Path,[string]$Hash,[string]$Owner) {
  if ($Hash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Path) -cne $Hash) { throw 'Exact record hash is required.' }
  $value=Read-Record $Path
  if ($value.schema -ne 1 -or $value.owner -cne $Owner) { throw 'Unrecognized commissioning record.' }
  return $value
}
function Assert-RecordDirectory([string]$Path) {
  $path=Assert-LocalPath $Path
  if (-not $path.StartsWith((Join-Path $context.workspace 'runs')+'\',[StringComparison]::OrdinalIgnoreCase)) { throw 'Private record must be below this workspace runs directory.' }
  return [IO.Path]::GetDirectoryName($path)
}
function Save-CommissioningBytes([string]$Path,[byte[]]$Bytes) {
  $path=Assert-LocalPath $Path
  $stream=New-Object IO.FileStream($path,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
  try { $stream.Write($Bytes,0,$Bytes.Length);$stream.Flush($true) } finally { $stream.Dispose() }
}
function Assert-ClosedCommissioning {
  Assert-PreparationClosed (@($context.application,$context.managed)+@($context.pluginRoots))
}
if ($Action -cin @('Inventory','Prepare')) {
  if (Test-Path -LiteralPath $active) { throw 'An existing commissioning transaction must be inspected/restored first.' }
  if ((Get-Digest $ini) -cne $script:CommissioningBaseline) { throw 'The separately recovered exact working INI is required before commissioning.' }
  $values=Read-ProfileForAudit $ini
  if ($values['Directories/pluginInstallDir']) { throw 'Custom plugin paths require a separate review.' }
}
if ($Action -ceq 'Inventory') {
  $trees=@(Get-CommissioningTrees $context.pluginRoots)
  $directory=New-PreparationDirectory $context 'commissioning-inventory'
  $output=Join-Path $directory 'inventory.json'
  foreach ($tree in $trees) { Assert-PreparationTree $tree }
  Assert-ClosedCommissioning
  Write-Record $output @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Inventory.1';createdUtc=[DateTime]::UtcNow.ToString('o');context=$context;profileSha256=(Get-Digest $ini);trees=$trees;plugins=@(Get-CommissioningCandidates $trees);privacy='Private local review; no upload.'}
  [pscustomobject]@{status='inventory';record=$output;recordSha256=(Get-Digest $output);applicationLaunched=$false;profileModified=$false} | ConvertTo-Json
  return
}
if ($Action -ceq 'Prepare') {
  $planData=Read-PinnedCommissioningRecord $Plan $ExpectedPlanSha256 'OpenNavX.ReadOnlyCommissioning.Plan.1'
  $reviewed=[DateTime]::Parse($planData.reviewedUtc).ToUniversalTime()
  if ($reviewed -gt [DateTime]::UtcNow -or ([DateTime]::UtcNow-$reviewed).TotalHours -gt 24) { throw 'Operator source review is missing or expired.' }
  $inventory=Read-PinnedCommissioningRecord $planData.inventoryPath $planData.inventorySha256 'OpenNavX.ReadOnlyCommissioning.Inventory.1'
  Assert-CommissioningInventory $inventory $context
  if ($inventory.profileSha256 -cne $script:CommissioningBaseline) { throw 'Inventory did not cover the working recovered profile.' }
  foreach ($tree in $inventory.trees) { Assert-PreparationTree $tree }
  $candidates=@(Get-CommissioningCandidates $inventory.trees)
  Assert-CommissioningReview $candidates $planData.plugins
  $before=Get-PreparationTree $context.profile
  $inputBytes=Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($ini))
  $directory=New-PreparationDirectory $context 'read-only-commissioning'
  $quarantineDirectory=Join-Path $directory 'quarantine'
  Assert-CommissioningQuarantine $quarantineDirectory $context.pluginRoots $context.launchEnvironment.path
  $null=New-Item -ItemType Directory -Path $quarantineDirectory
  $original=Join-Path $directory 'baseline.ini';$inputProfile=Join-Path $directory 'input-only.ini'
  Copy-PreparationFile $ini $original $script:CommissioningBaseline 21380
  Save-CommissioningBytes $inputProfile $inputBytes
  Assert-InputOnlyProfile (Read-ProfileForAudit $inputProfile)
  Copy-PreparationFile $Plan (Join-Path $directory 'review-plan.json') $ExpectedPlanSha256 (Get-Item -LiteralPath $Plan).Length
  Copy-PreparationFile $planData.inventoryPath (Join-Path $directory 'inventory.json') $planData.inventorySha256 (Get-Item -LiteralPath $planData.inventoryPath).Length
  $moves=New-Object 'Collections.Generic.List[object]';$evidence=New-Object 'Collections.Generic.List[object]';$index=0
  foreach ($decision in @($planData.plugins)) {
    $index++;$savedEvidence=Join-Path $directory ('review-'+$index+'.txt')
    Copy-PreparationFile $decision.evidencePath $savedEvidence $decision.evidenceSha256 (Get-Item -LiteralPath $decision.evidencePath).Length
    $evidence.Add([pscustomobject]@{path=$savedEvidence;sha256=$decision.evidenceSha256})
    if ($decision.decision -ceq 'quarantine') {
      $candidate=@($candidates | Where-Object {$_.path -ieq $decision.path})[0]
      if ([IO.Path]::GetPathRoot($candidate.path) -ine [IO.Path]::GetPathRoot($quarantineDirectory)) { throw 'Quarantine requires an atomic move on the same volume.' }
      $backup=Join-Path $directory ('plugin-backup-'+$index+'.bin')
      Copy-PreparationFile $candidate.path $backup $candidate.sha256 $candidate.bytes
      $moves.Add([pscustomobject]@{path=$candidate.path;sha256=$candidate.sha256;bytes=$candidate.bytes;backup=$backup;destination=(Join-Path $quarantineDirectory ('plugin-'+$index+'.bin'));acl=(Get-Acl -LiteralPath $candidate.path).Sddl})
    }
  }
  Assert-PreparationTree $before
  foreach ($tree in $inventory.trees) { Assert-PreparationTree $tree }
  Assert-CommissioningContext $context (Get-CommissioningContext $Workspace)
  $prepared=Join-Path $directory 'prepared.json'
  Write-Record $prepared @{schema=1;owner=$script:CommissioningOwner;status='prepared';createdUtc=[DateTime]::UtcNow.ToString('o');context=$context;planSha256=$ExpectedPlanSha256;inventorySha256=$planData.inventorySha256;evidence=$evidence.ToArray();baselineSha256=$script:CommissioningBaseline;inputSha256=(Get-Digest $inputProfile);profileBefore=$before;originalAcl=(Get-Acl -LiteralPath $ini).Sddl;quarantine=$moves.ToArray();gatewayManagementTraffic='OpenCPN serial driver management writes still occur; no actuator data commands authorized.';applicationLaunched=$false}
  [pscustomobject]@{status='prepared';record=$prepared;recordSha256=(Get-Digest $prepared);quarantineCount=$moves.Count;profileModified=$false;applicationLaunched=$false} | ConvertTo-Json
  return
}
$recordPath=Assert-LocalPath $Record
$directory=Assert-RecordDirectory $recordPath
$prepared=Read-PinnedCommissioningRecord $recordPath $ExpectedRecordSha256 $script:CommissioningOwner
if ($prepared.status -cne 'prepared' -or $prepared.baselineSha256 -cne $script:CommissioningBaseline) { throw 'Unrecognized prepared transaction.' }
Assert-CommissioningContext $prepared.context $context
$planData=Read-PinnedCommissioningRecord (Join-Path $directory 'review-plan.json') $prepared.planSha256 'OpenNavX.ReadOnlyCommissioning.Plan.1'
$inventory=Read-PinnedCommissioningRecord (Join-Path $directory 'inventory.json') $prepared.inventorySha256 'OpenNavX.ReadOnlyCommissioning.Inventory.1'
Assert-CommissioningInventory $inventory $context
$original=Join-Path $directory 'baseline.ini';$inputProfile=Join-Path $directory 'input-only.ini'
if ((Get-Digest $original) -cne $script:CommissioningBaseline -or (Get-Digest $inputProfile) -cne $prepared.inputSha256 -or
    (Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($original)))) -cne $prepared.inputSha256) { throw 'Prepared exact-byte profile copies changed.' }
foreach ($item in @($prepared.evidence)) { if ((Get-Digest $item.path) -cne $item.sha256) { throw 'Saved source review evidence changed.' } }
foreach ($item in @($prepared.quarantine)) {
  if (-not $item.destination.StartsWith((Join-Path $directory 'quarantine')+'\',[StringComparison]::OrdinalIgnoreCase) -or
      [IO.Path]::GetFileName($item.path) -notlike '*_pi.dll' -or (Get-Digest $item.backup) -cne $item.sha256) { throw 'Quarantine ownership or saved original changed.' }
}
$plannedMoves=@($planData.plugins | Where-Object {$_.decision -ceq 'quarantine'})
if (@($prepared.quarantine).Count -ne $plannedMoves.Count) { throw 'Prepared quarantine differs from the operator review plan.' }
$movePaths=@{}
foreach ($item in @($prepared.quarantine)) {
  $match=@($plannedMoves | Where-Object {$_.path -ieq $item.path -and $_.sha256 -ceq $item.sha256})
  if ($match.Count -ne 1 -or $movePaths.ContainsKey($item.path)) { throw 'Unexpected or duplicate quarantine file.' }
  $movePaths[$item.path]=$true
}
Assert-CommissioningQuarantine (Join-Path $directory 'quarantine') $context.pluginRoots $context.launchEnvironment.path
if ($Action -ceq 'Apply') {
  if (Test-Path -LiteralPath $active) { throw 'Commissioning already active; inspect or restore, never replay Apply.' }
  $reviewed=[DateTime]::Parse($planData.reviewedUtc).ToUniversalTime()
  if ($reviewed -gt [DateTime]::UtcNow -or ([DateTime]::UtcNow-$reviewed).TotalHours -gt 24) { throw 'Operator source review expired before Apply.' }
  Assert-PreparationTree $prepared.profileBefore
  Assert-PreparationAcl $prepared.originalAcl (Get-Acl -LiteralPath $ini).Sddl
  Assert-CommissioningTrees $inventory.trees $prepared.quarantine
  Write-Record $active @{schema=1;owner=$script:CommissioningOwner;record=$recordPath;recordSha256=$ExpectedRecordSha256}
  $index=0
  foreach ($item in @($prepared.quarantine)) {
    $index++;Assert-ClosedCommissioning
    Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
    Assert-PreparationAcl $item.acl (Get-Acl -LiteralPath $item.path).Sddl
    Write-Record (Join-Path $directory ('move-'+$index+'-intent.json')) @{schema=1;owner=$script:CommissioningOwner;recordSha256=$ExpectedRecordSha256;source=$item.path;destination=$item.destination;sha256=$item.sha256}
    Assert-ClosedCommissioning
    [IO.File]::Move($item.path,$item.destination)
    if ((Get-Digest $item.destination) -cne $item.sha256) { throw 'Quarantine postcondition failed; inspect transaction.' }
    Assert-PreparationAcl $item.acl (Get-Acl -LiteralPath $item.destination).Sddl
  }
  Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
  Assert-PreparationTree $prepared.profileBefore
  $apply=Join-Path $directory 'input-only-intent.json'
  Write-Record $apply @{schema=1;owner=$script:CommissioningOwner;recordSha256=$ExpectedRecordSha256;beforeSha256=$script:CommissioningBaseline;afterSha256=$prepared.inputSha256}
  Assert-ClosedCommissioning
  Publish-PreparedProfile $ini $inputProfile $script:CommissioningBaseline $prepared.inputSha256 21380 $apply
  Assert-InputOnlyProfile (Read-ProfileForAudit $ini)
  $remaining=@(Get-AuditPluginCandidates $context.pluginRoots)
  $retained=@($planData.plugins | Where-Object {$_.decision -ceq 'retain'})
  Assert-PluginAudit $remaining $retained
  $applied=Join-Path $directory 'applied.json'
  Write-Record $applied @{schema=1;owner=$script:CommissioningOwner;status='input-only-prepared';recordSha256=$ExpectedRecordSha256;profileSha256=(Get-Digest $ini);remainingPluginCount=$remaining.Count;applicationLaunched=$false;launchAuditStillRequired=$true}
  [pscustomobject]@{status='input-only-prepared';verification=$applied;verificationSha256=(Get-Digest $applied);profileSha256=(Get-Digest $ini);applicationLaunched=$false;launchAuditStillRequired=$true} | ConvertTo-Json
  return
}
$activeRecord=Read-Record $active
if ($activeRecord.schema -ne 1 -or $activeRecord.owner -cne $script:CommissioningOwner -or $activeRecord.record -ine $recordPath -or $activeRecord.recordSha256 -cne $ExpectedRecordSha256) { throw 'Active transaction ownership mismatch.' }
Assert-ClosedCommissioning
Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
if ($Action -ceq 'InspectRestore') {
  $currentHash=Get-Digest $ini
  Assert-PreparationAcl $prepared.originalAcl (Get-Acl -LiteralPath $ini).Sddl -AllowDaclAutoInherited
  # A partial Apply may still have the untouched baseline. Otherwise all
  # reviewed input connections and chart directories must remain unchanged.
  if ($currentHash -cne $script:CommissioningBaseline) { Assert-CommissioningRestoreIni $inputProfile $ini }
  $id=[guid]::NewGuid().ToString('N')
  $saved=Join-Path $directory ('post-session-'+$id+'.ini')
  Copy-PreparationFile $ini $saved $currentHash (Get-Item -LiteralPath $ini).Length
  $snapshot=Get-PreparationTree $context.profile
  $output=Join-Path $directory ('restore-inspection-'+$id+'.json')
  Write-Record $output @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.RestoreInspection.1';recordSha256=$ExpectedRecordSha256;createdUtc=[DateTime]::UtcNow.ToString('o');context=$context;currentIniSha256=$currentHash;savedIni=$saved;profileBeforeRestore=$snapshot;diff=@(Get-CommissioningIniDiff $original $saved);currentAcl=(Get-Acl -LiteralPath $ini).Sddl;applicationLaunched=$false;requiresOperatorDiffReview=$true}
  [pscustomobject]@{status='review-required';inspection=$output;inspectionSha256=(Get-Digest $output);currentIniSha256=$currentHash;applicationLaunched=$false} | ConvertTo-Json
  return
}
$inspectionPath=Assert-LocalPath $Inspection
if ([IO.Path]::GetDirectoryName($inspectionPath) -ine $directory) { throw 'Restore inspection must belong to this transaction.' }
$inspected=Read-PinnedCommissioningRecord $inspectionPath $ExpectedInspectionSha256 'OpenNavX.ReadOnlyCommissioning.RestoreInspection.1'
if ($inspected.recordSha256 -cne $ExpectedRecordSha256 -or $ReviewedCurrentIniSha256 -cnotmatch '^[a-f0-9]{64}$' -or
    $ReviewedCurrentIniSha256 -cne $inspected.currentIniSha256 -or (Get-Digest $ini) -cne $ReviewedCurrentIniSha256 -or (Get-Digest $inspected.savedIni) -cne $ReviewedCurrentIniSha256) { throw 'Restore requires exact operator-reviewed current bytes; unexpected changes are never overwritten.' }
Assert-CommissioningContext $inspected.context $context
Assert-PreparationTree $inspected.profileBeforeRestore
Assert-PreparationAcl $inspected.currentAcl (Get-Acl -LiteralPath $ini).Sddl
Assert-PreparationAcl $prepared.originalAcl $inspected.currentAcl -AllowDaclAutoInherited
$restore=Join-Path $directory ('restore-intent-'+[guid]::NewGuid().ToString('N')+'.json')
Write-Record $restore @{schema=1;owner=$script:CommissioningOwner;recordSha256=$ExpectedRecordSha256;inspectionSha256=$ExpectedInspectionSha256;beforeSha256=$ReviewedCurrentIniSha256;afterSha256=$script:CommissioningBaseline;applicationLaunched=$false}
Assert-ClosedCommissioning
if ($ReviewedCurrentIniSha256 -cne $script:CommissioningBaseline) { Publish-PreparedProfile $ini $original $ReviewedCurrentIniSha256 $script:CommissioningBaseline 21380 $restore }
foreach ($item in @($prepared.quarantine)) {
  Assert-ClosedCommissioning
  Assert-CommissioningTrees $inventory.trees $prepared.quarantine -AllowMoved
  if ([IO.File]::Exists($item.destination)) {
    Assert-PreparationAcl $item.acl (Get-Acl -LiteralPath $item.destination).Sddl
    Write-Record (Join-Path $directory ('restore-plugin-'+[guid]::NewGuid().ToString('N')+'.json')) @{schema=1;owner=$script:CommissioningOwner;recordSha256=$ExpectedRecordSha256;inspectionSha256=$ExpectedInspectionSha256;source=$item.destination;destination=$item.path;sha256=$item.sha256}
    Assert-ClosedCommissioning
    [IO.File]::Move($item.destination,$item.path)
    Assert-PreparationAcl $item.acl (Get-Acl -LiteralPath $item.path).Sddl
  }
}
Assert-CommissioningTrees $inventory.trees $prepared.quarantine
if ((Get-Digest $ini) -cne $script:CommissioningBaseline) { throw 'Working baseline was not restored exactly.' }
$expected=$inspected.profileBeforeRestore
$iniEntry=@($expected.entries | Where-Object {$_.path -ceq 'opencpn.ini'})
if ($iniEntry.Count -ne 1) { throw 'Ambiguous profile inventory.' }
$iniEntry[0].sha256=$script:CommissioningBaseline;$iniEntry[0].bytes=21380
Assert-PreparationTree $expected
Assert-ClosedCommissioning
$complete=Join-Path $directory ('restored-'+[guid]::NewGuid().ToString('N')+'.json')
Write-Record $complete @{schema=1;owner=$script:CommissioningOwner;status='restored';recordSha256=$ExpectedRecordSha256;inspectionSha256=$ExpectedInspectionSha256;profileSha256=$script:CommissioningBaseline;pluginInventoryRestored=$true;otherProfileFilesPreserved=$true;applicationLaunched=$false;originalOutputConfigurationRestored=$true;doNotAutoLaunch=$true}
# Only remove our exact short-lived ownership marker after durable completion.
if ((Read-Record $active).recordSha256 -cne $ExpectedRecordSha256) { throw 'Active ownership changed before completion.' }
Remove-Item -LiteralPath $active
[pscustomobject]@{status='restored';verification=$complete;verificationSha256=(Get-Digest $complete);applicationLaunched=$false;doNotAutoLaunch=$true} | ConvertTo-Json
