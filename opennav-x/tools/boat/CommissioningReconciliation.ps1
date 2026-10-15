# Explicit cold reconciliation only. Never select, launch or relabel a generation.
$script:ReconciliationOwner='OpenNavX.ColdCommissioningReconciliation.1'
function Assert-ReconciliationContexts($Old,$Current) {
  foreach($key in @('sid','localAppData','profile','application','managed','workspace','executable')) {
    if($Old.$key -cne $Current.$key){throw 'Reconciliation requires the same actual account, profile and original roots.'}
  }
  if(-not $Old.installation -or -not $Current.installation -or $Old.installation.root -cne $Current.installation.root -or
     $Old.installation.generation -ceq $Current.installation.generation -or $Current.session -le 0){throw 'Two distinct owned generations and a current interactive session are required.'}
  foreach($context in @($Old,$Current)) {
    $i=$context.installation;$root=Assert-LocalPath $i.root;$generation=Assert-LocalPath $i.generation
    if([IO.Path]::GetDirectoryName($generation) -ine (Join-Path $root 'generations') -or
       [IO.Path]::GetFileName($generation) -cnotmatch '^[a-f0-9]{32}$' -or
       (Assert-LocalPath $i.executable) -ine (Join-Path $generation 'app/opencpn.exe') -or $i.commit -cnotmatch '^[a-f0-9]{40}$') {throw 'Canonical owned generation identity required.'}
    foreach($hash in @($i.stateSha256,$i.ownershipSha256,$i.executableSha256)){if($hash -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact generation identity hashes required.'}}
    $expected=@($context.managed,(Join-Path $context.application 'plugins'),(Join-Path $generation 'app/plugins')|Sort-Object -Unique)
    if(($expected|ConvertTo-Json -Compress) -cne (@($context.pluginRoots)|ConvertTo-Json -Compress)){throw 'Exact original and installed loader roots required.'}
  }
}
function Read-ReconciliationParent([string]$Workspace,[string]$Record,[string]$Hash,[int]$Depth=0) {
  $record=Assert-LocalPath $Record;$directory=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs')){throw 'Original workspace prepared record required.'}
  $parent=Read-SessionPreservationParent $record $Hash
  if($parent.context.workspace -ine $Workspace){throw 'Original transaction belongs to another workspace.'}
  $null=Get-PreparedCommissioningBaseline $parent $directory $Workspace $Depth
  $inventoryPath=Join-Path $directory 'inventory.json'
  if((Get-Digest $inventoryPath) -cne $parent.inventorySha256){throw 'Original inventory changed.'}
  $inventory=Read-Record $inventoryPath;Assert-CommissioningInventory $inventory $parent.context
  $plan=Read-Record (Join-Path $directory 'review-plan.json')
  $moves=@($plan.plugins|Where-Object {$_.decision -ceq 'quarantine'})
  if(@($parent.quarantine).Count -ne $moves.Count -or $moves.Count -lt 1 -or $moves.Count -gt 256){throw 'Complete bounded original quarantine required.'}
  $seen=@{};$destinations=@{};$backups=@{}
  foreach($item in @($parent.quarantine)) {
    $path=Assert-LocalPath $item.path;$destination=Assert-LocalPath $item.destination;$backup=Assert-LocalPath $item.backup
    $match=@($moves|Where-Object {$_.path -ieq $path -and $_.sha256 -ceq $item.sha256})
    $roots=@($inventory.trees|Where-Object {$path.StartsWith($_.root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)})
    $candidate=@($inventory.plugins|Where-Object {$_.path -ieq $path -and $_.sha256 -ceq $item.sha256})
    if($seen.ContainsKey($path) -or $destinations.ContainsKey($destination) -or $backups.ContainsKey($backup) -or
       $candidate.Count -ne 1 -or $match.Count -ne 1 -or $roots.Count -ne 1 -or [IO.Path]::GetFileName($path) -notlike '*_pi.dll' -or
       [IO.Path]::GetDirectoryName($destination) -ine (Join-Path $directory 'quarantine') -or [IO.Path]::GetDirectoryName($backup) -ine $directory){throw 'Each original quarantine path must have independent, exact ownership.'}
    $seen[$path]=$true;$destinations[$destination]=$true;$backups[$backup]=$true
  }
  return [pscustomobject]@{parent=$parent;inventory=$inventory;directory=$directory;record=$record;sha256=$Hash}
}
function Restore-ReconciliationPlugin($Item) {
  # Caller already owns the plan/profile/transaction locks and exact original
  # path inventory. File.Move never overwrites an occupied destination.
  [IO.File]::Move($Item.destination,$Item.path)
  if((Get-Digest $Item.path) -cne $Item.sha256){throw 'Restored original plugin bytes changed.'}
  Assert-PreparationAcl $Item.acl (Get-PreparationAccessAcl $Item.path)
}
function Assert-ReconciliationLiveIdentity($Proof,$Current) {
  Assert-ReconciliationContexts $Proof.parent.context $Current
  $old=$Proof.parent.context.installation
  if(-not [IO.Directory]::Exists($old.generation) -or (Get-Digest $old.executable) -cne $old.executableSha256 -or
     (Get-Digest (Join-Path $old.generation 'ownership.json')) -cne $old.ownershipSha256){throw 'Original generation retired, missing or changed; no path recreation or plugin transplant.'}
  $owner=Read-Record (Join-Path $old.generation 'ownership.json')
  $exe=@($owner.managedFiles|Where-Object {$_.path -ceq 'app/opencpn.exe'})
  if($owner.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $owner.commit -cne $old.commit -or
     $exe.Count -ne 1 -or $exe[0].sha256 -cne $old.executableSha256){throw 'Original generation ownership does not authenticate its executable.'}
  foreach($item in @($Proof.parent.quarantine)) {
    if($item.path.StartsWith($Current.installation.generation+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)){throw 'Old plugins must never enter the current generation.'}
  }
  Assert-PreparationClosed (@($Proof.parent.context.pluginRoots)+@($Current.pluginRoots))
}
function Assert-ReconciliationActive($Proof,[string]$ExpectedMarkerHash) {
  $path=Join-Path $Proof.parent.context.workspace 'commissioning-active.json'
  if($ExpectedMarkerHash -and (Get-Digest $path) -cne $ExpectedMarkerHash){throw 'Active ownership bytes changed.'}
  $active=Read-Record $path
  if($active.schema -ne 1 -or $active.owner -cne $script:CommissioningOwner -or $active.record -ine $Proof.record -or
     $active.recordSha256 -cne $Proof.sha256){throw 'Exact original active transaction required.'}
}
function Assert-ReconciliationMoves($Proof,[string]$Directory,[switch]$Resume) {
  Assert-CommissioningTrees $Proof.inventory.trees $Proof.parent.quarantine -AllowMoved
  $index=0
  foreach($item in @($Proof.parent.quarantine)) {
    $journal=Join-Path $Directory ('plugin-'+$index+'.json');$index++
    $source=[IO.File]::Exists($item.path);$quarantined=[IO.File]::Exists($item.destination)
    if($source -eq $quarantined){throw 'Ambiguous original/quarantined plugin locations.'}
    if(Test-Path -LiteralPath $journal) {
      $intent=Read-Record $journal
      if(-not $Resume -or $intent.owner -cne $script:ReconciliationOwner -or $intent.parentPreparedSha256 -cne $Proof.sha256 -or
         $intent.source -ine $item.destination -or $intent.destination -ine $item.path -or $intent.sha256 -cne $item.sha256){throw 'Plugin restoration journal differs from its original path.'}
    } elseif($source){throw 'Occupied original plugin destination; no overwrite or inferred restoration.'}
    $actual=if($source){$item.path}else{$item.destination}
    Assert-PreparationAcl $item.acl (Get-PreparationAccessAcl $actual)
    if(-not [IO.Directory]::Exists([IO.Path]::GetDirectoryName($item.path))){throw 'Missing original destination directory; no recreation.'}
  }
}
function Get-ReconciliationProfileBytes($Proof,[string]$Saved) {
  $before=Read-ProfileForAudit (Join-Path $Proof.directory 'input-only.ini')
  $original=Read-ProfileForAudit (Join-Path $Proof.directory 'baseline.ini');$current=Read-ProfileForAudit $Saved
  $key='Settings/NMEADataSource/DataConnections';$bytes=[IO.File]::ReadAllBytes($Saved)
  if($current[$key] -ceq $original[$key]) {
    # The user has already restored the original direction. Preserve ALL live
    # bytes; normalization is a separate review copy, never a live mutation.
    return [pscustomobject]@{input=(Get-CommissioningInputBytes $bytes);baseline=$bytes;disposition='preserve-current-original-direction'}
  }
  if($current[$key] -ceq $before[$key]) {
    return [pscustomobject]@{input=$bytes;baseline=(Get-CommissioningOutputBytes $bytes);disposition='restore-original-temporary-direction'}
  }
  throw 'Connection values differ from both exact original and temporary forms; no schema guessing or network/output changes.'
}
function Assert-ReconciliationReview($Proof,[string]$Saved,[string]$Normalized,$Review,[datetime]$At) {
  $before=Join-Path $Proof.directory 'input-only.ini'
  $bytes=Get-ReconciliationProfileBytes $Proof $Saved
  if($Review.schema -ne 1 -or $Review.owner -cne 'OpenNavX.SessionPreservationReview.1' -or
     $Review.beforeSha256 -cne (Get-Digest $before) -or $Review.afterSha256 -cne (Get-Digest $Saved) -or
     $Review.parentPreparedSha256 -cne $Proof.sha256 -or $Review.connectionDisposition -cne $bytes.disposition -or
     $Review.provenance -cne 'current-user-state;origin-unverified' -or $Review.preservationOnly -isnot [bool] -or -not $Review.preservationOnly -or
     $Review.launchPermission -isnot [bool] -or $Review.launchPermission -or
     (Get-Digest $Normalized) -cne (Get-CommissioningHash $bytes.input)){throw 'Exact explicit current-profile and connection-disposition review required.'}
  $reviewed=[datetime]::Parse($Review.reviewedUtc).ToUniversalTime()
  if($reviewed -gt $At -or ($At-$reviewed).TotalHours -gt 24){throw 'Preservation review expired or future-dated.'}
  $changes=@(Get-CommissioningIniDiff $before $Saved)
  if(@($Review.changes).Count -ne $changes.Count -or $changes.Count -gt 65){throw 'Every actual current-profile difference needs one decision.'}
  foreach($change in $changes) {
    $match=@($Review.changes|Where-Object {$_.key -ceq $change.key})
    if($match.Count -ne 1 -or $match[0].before -cne $change.before -or $match[0].after -cne $change.after -or
       $match[0].origin -cne 'unverified' -or $match[0].decision -cne 'preserve-current' -or
       [string]::IsNullOrWhiteSpace($match[0].reason) -or $match[0].reason.Length -gt 1024){throw 'Missing, duplicate or changed explicit preservation decision.'}
  }
  # Only the separately identified original direction byte is normalized for
  # existing policy checks. All other keys retain the bounded preservation policy.
  $policy=$Review|ConvertTo-Json -Depth 12|ConvertFrom-Json
  $policy.afterSha256=Get-Digest $Normalized
  $policy.changes=@($Review.changes|Where-Object {$_.key -cne 'Settings/NMEADataSource/DataConnections'})
  if((Get-Digest $before) -cne (Get-Digest $Normalized)) {$null=Assert-SessionPreservationReview $before $Normalized $policy $At}
  elseif($policy.changes.Count){throw 'Review lists nonexistent normalized profile differences.'}
  return $changes
}
function Read-ReconciliationPlan([string]$Workspace,[string]$Plan,[string]$Hash,[int]$Depth=0) {
  $plan=Assert-LocalPath $Plan;$directory=[IO.Path]::GetDirectoryName($plan)
  if($Hash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $plan) -cne $Hash -or [IO.Path]::GetFileName($plan) -cne 'plan.json' -or
     [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
     [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-reconciliation-[a-f0-9]{8}$'){throw 'Exact immutable cold reconciliation plan required.'}
  $value=Read-Record $plan
  if($value.schema -ne 1 -or $value.owner -cne $script:ReconciliationOwner -or $value.status -cne 'planned' -or
     $value.provenance -cne 'explicit-cross-generation-reconciliation;current-user-state-origin-unverified' -or
     $value.launchPermission -isnot [bool] -or $value.launchPermission){throw 'Plan cannot claim launch permission or another provenance.'}
  $proof=Read-ReconciliationParent $Workspace $value.parentPrepared $value.parentPreparedSha256 ($Depth+1)
  $inspection=Join-Path $directory 'inspection.json'
  if((Get-Digest $inspection) -cne $value.inspectionSha256){throw 'Reconciliation inspection changed.'}
  $observed=Read-Record $inspection
  if($observed.schema -ne 1 -or $observed.owner -cne $script:ReconciliationOwner -or $observed.status -cne 'inspected' -or
     $observed.parentPrepared -ine $proof.record -or $observed.parentPreparedSha256 -cne $proof.sha256 -or
     $observed.activeMarkerSha256 -cnotmatch '^[a-f0-9]{64}$' -or $observed.profileTree.root -ine $observed.currentContext.profile -or
     $observed.profileTree.exists -isnot [bool] -or -not $observed.profileTree.exists){throw 'Inspection belongs to another transaction or profile.'}
  Assert-ReconciliationContexts $proof.parent.context $observed.currentContext
  $newRoots=@($observed.currentContext.pluginRoots|Where-Object {$_ -cnotin $proof.parent.context.pluginRoots}|Sort-Object)
  if(($newRoots|ConvertTo-Json -Compress) -cne (@($observed.currentPluginTrees|ForEach-Object {$_.root}|Sort-Object)|ConvertTo-Json -Compress)){throw 'Inspection must cover every new current-generation loader root exactly once.'}
  Assert-ColdPrivateEvidence $directory $observed.currentContext.sid
  $backup=Get-PreparationTree (Join-Path $directory 'profile-backup');$saved=Join-Path $backup.root 'opencpn.ini';$baseline=Join-Path $directory 'baseline.ini'
  if(($backup.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($observed.profileTree.entries|ConvertTo-Json -Depth 8 -Compress) -or
     (Get-Digest $saved) -cne $observed.currentIniSha256 -or (Get-Digest $baseline) -cne $value.baselineSha256 -or
     (Get-Item -LiteralPath $baseline).Length -ne $value.baselineBytes -or
     (Get-CommissioningHash (Get-ReconciliationProfileBytes $proof $saved).baseline) -cne $value.baselineSha256){throw 'Full current profile backup or exact connection disposition changed.'}
  if($observed.connectionDisposition -cne (Get-ReconciliationProfileBytes $proof $saved).disposition){throw 'Observed connection disposition differs from exact saved bytes.'}
  $review=Join-Path $directory 'preservation-review.json'
  if((Get-Digest $review) -cne $value.reviewSha256){throw 'Independent reconciliation review changed.'}
  $approval=Read-Record $review
  if($approval.inspectionSha256 -cne $value.inspectionSha256 -or
     @(Assert-ReconciliationReview $proof $saved (Join-Path $directory 'normalized-input.ini') $approval ([datetime]::Parse($value.createdUtc).ToUniversalTime())).Count -ne $value.changedKeys){throw 'Exact review/inspection lineage differs.'}
  return [pscustomobject]@{proof=$proof;value=$value;inspection=$observed;directory=$directory;path=$plan;sha256=$Hash;baseline=$baseline}
}
function ConvertTo-ReconciliationMetadata($Metadata) {
  # Windows PowerShell 5.1 JSON dates retain only milliseconds. Persist exact
  # DateTime ticks as text; no date parser or JSON numeric rounding is admitted.
  return [pscustomobject]@{attributes=$Metadata.attributes;
    creationUtcTicks=$Metadata.creationUtc.Ticks.ToString([Globalization.CultureInfo]::InvariantCulture);
    acl=$Metadata.acl;auditAcl=$Metadata.auditAcl}
}
function ConvertFrom-ReconciliationMetadata($Value) {
  $ticks=[long]0
  if($Value.creationUtcTicks -isnot [string] -or $Value.creationUtcTicks -cnotmatch '^(0|[1-9][0-9]{0,18})$' -or
     -not [long]::TryParse($Value.creationUtcTicks,[Globalization.NumberStyles]::None,[Globalization.CultureInfo]::InvariantCulture,[ref]$ticks) -or
     $ticks -gt [datetime]::MaxValue.Ticks){throw 'Exact canonical profile creation ticks required.'}
  return [pscustomobject]@{attributes=$Value.attributes;creationUtc=(New-Object DateTime($ticks,[DateTimeKind]::Utc));acl=$Value.acl;auditAcl=$Value.auditAcl}
}
function Read-ReconciliationProfileStage($Plan) {
  $journal=Join-Path $Plan.directory 'profile-stage.json'
  if(-not(Test-Path -LiteralPath $journal)){return $null}
  $value=Read-Record $journal;$original=Join-Path $Plan.inspection.currentContext.profile 'opencpn.ini'
  # The name is derived from the pinned plan, not accepted from an arbitrary
  # matching recovery file. The journal must exist before this path is created.
  $stage=$original+'.opennav-recovery-'+$Plan.sha256.Substring(0,32)+'.partial'
  $native=[Environment]::OSVersion.Platform -eq 'Win32NT'
  if($value.schema -ne 1 -or $value.owner -cne $script:ReconciliationOwner -or $value.status -cne 'prepared' -or
     $value.planSha256 -cne $Plan.sha256 -or $value.parentPreparedSha256 -cne $Plan.proof.sha256 -or
     $value.original -cne $original -or $value.stage -cne $stage -or $value.candidate -cne $Plan.baseline -or
     $value.originalSha256 -cne $Plan.inspection.currentIniSha256 -or $value.candidateSha256 -cne $Plan.value.baselineSha256 -or
     $value.bytes -ne $Plan.value.baselineBytes -or $value.native -isnot [bool] -or $value.native -ne $native -or
     $value.originalSha256 -ceq $value.candidateSha256){throw 'Profile staging journal differs from its exact reconciliation plan.'}
  Assert-PreparationStagePath $original $stage
  if($native){
    $value.metadata=ConvertFrom-ReconciliationMetadata $value.metadata
    Assert-PreparationNativeMetadata $value.metadata (Get-PreparationNativeMetadata $original) -AllowDaclAutoInherited
  }else{Assert-PreparationAcl $value.acl (Get-PreparationAccessAcl $original)}
  if(Test-Path -LiteralPath $stage){
    if((Get-Digest $original) -cne $value.originalSha256 -or (Get-Digest $stage) -cne $value.candidateSha256 -or
       (Get-Item -LiteralPath $stage -Force).Length -ne $value.bytes){throw 'Journalled profile stage or original bytes changed; retain evidence without publication.'}
    if($native){Assert-PreparationNativeMetadata $value.metadata (Get-PreparationNativeMetadata $stage) -AllowDaclAutoInherited}
    else{Assert-PreparationAcl $value.acl (Get-PreparationAccessAcl $stage)}
  }
  return $value
}
function Publish-ReconciliationStage([string]$Stage,[string]$Original) {
  if([Environment]::OSVersion.Platform -eq 'Win32NT'){
    [OpenNavX.PreparationNative]::Publish($Stage,$Original)
  }else{[IO.File]::Replace($Stage,$Original,[NullString]::Value,$false)}
}
function Publish-ReconciliationProfile($Plan,$Current) {
  $original=Join-Path $Current.profile 'opencpn.ini';$journal=Join-Path $Plan.directory 'profile-stage.json'
  if(-not(Test-Path -LiteralPath $journal)){
    $native=[Environment]::OSVersion.Platform -eq 'Win32NT'
    $metadata=if($native){ConvertTo-ReconciliationMetadata (Get-PreparationNativeMetadata $original)}else{$null}
    Write-Record $journal @{schema=1;owner=$script:ReconciliationOwner;status='prepared';planSha256=$Plan.sha256;parentPreparedSha256=$Plan.proof.sha256;
      original=$original;stage=($original+'.opennav-recovery-'+$Plan.sha256.Substring(0,32)+'.partial');candidate=$Plan.baseline;
      originalSha256=$Plan.inspection.currentIniSha256;candidateSha256=$Plan.value.baselineSha256;bytes=$Plan.value.baselineBytes;
      native=$native;metadata=$metadata;acl=(Get-PreparationAccessAcl $original)}
  }
  $staged=Read-ReconciliationProfileStage $Plan
  if(-not(Test-Path -LiteralPath $staged.stage)){
    Copy-PreparationFile $Plan.baseline $staged.stage $staged.candidateSha256 $staged.bytes
    if($staged.native){
      Set-PreparationStageAcl $original $staged.stage $staged.metadata.acl
      [IO.File]::SetCreationTimeUtc($staged.stage,$staged.metadata.creationUtc)
      [IO.File]::SetAttributes($staged.stage,[IO.FileAttributes]$staged.metadata.attributes)
    }
  }
  # Recheck the complete trees, both contexts, exact marker, original bytes and
  # staged bytes/metadata immediately before the same atomic publication API.
  Assert-ReconciliationPlanLive $Plan (Get-CommissioningContext $Current.workspace) -Resume
  $staged=Read-ReconciliationProfileStage $Plan
  Publish-ReconciliationStage $staged.stage $original
  if((Get-Digest $original) -cne $Plan.value.baselineSha256 -or (Test-Path -LiteralPath $staged.stage)){throw 'Reconciliation profile publication postcondition failed.'}
  $null=Read-ReconciliationProfileStage $Plan
}
function Remove-ReconciliationActive($Plan) {
  Assert-ReconciliationActive $Plan.proof $Plan.inspection.activeMarkerSha256
  Remove-Item -LiteralPath (Join-Path $Plan.inspection.currentContext.workspace 'commissioning-active.json')
}
function Get-ReconciliationReceipt($Plan) {
  $completion=Join-Path $Plan.directory 'completion.json';$baseline=Join-Path $Plan.directory 'reconciled-baseline.json'
  return [pscustomobject]@{status='reconciled';completion=$completion;completionSha256=(Get-Digest $completion);baselineRecord=$baseline;
    baselineRecordSha256=(Get-Digest $baseline);applicationLaunched=$false;doNotAutoLaunch=$true}
}
function Assert-ReconciliationPlanLive($Plan,$Current,[switch]$Resume,[switch]$Completed) {
  $created=[datetime]::Parse($Plan.value.createdUtc).ToUniversalTime();$now=[datetime]::UtcNow
  if(-not $Resume -and ($created -gt $now -or ($now-$created).TotalHours -gt 24)){throw 'Unstarted reconciliation plan expired or future-dated.'}
  Assert-CommissioningContext $Plan.inspection.currentContext $Current
  Assert-ReconciliationLiveIdentity $Plan.proof $Current
  if($Completed){
    if(-not $Resume -or (Test-Path -LiteralPath (Join-Path $Current.workspace 'commissioning-active.json'))){throw 'Completed reconciliation requires an absent active marker.'}
  }else{Assert-ReconciliationActive $Plan.proof $Plan.inspection.activeMarkerSha256}
  $tree=$Plan.inspection.profileTree|ConvertTo-Json -Depth 10|ConvertFrom-Json
  $ini=Join-Path $tree.root 'opencpn.ini';$hash=Get-Digest $ini
  if(($Completed -and $hash -cne $Plan.value.baselineSha256) -or ($hash -cne $Plan.inspection.currentIniSha256 -and (-not $Resume -or $hash -cne $Plan.value.baselineSha256))){throw 'Current profile drifted from the exact inspected bytes.'}
  $entry=@($tree.entries|Where-Object {$_.path -ceq 'opencpn.ini'})
  if($entry.Count -ne 1){throw 'Ambiguous current profile inventory.'}
  $entry[0].sha256=$hash;$entry[0].bytes=(Get-Item -LiteralPath $ini).Length
  $stage=Read-ReconciliationProfileStage $Plan
  if($Resume -and $hash -ceq $Plan.value.baselineSha256 -and $hash -cne $Plan.inspection.currentIniSha256 -and -not $stage){throw 'Published profile lacks its exact staging journal.'}
  $expectedTree=$tree|ConvertTo-Json -Depth 10|ConvertFrom-Json
  if($stage -and (Test-Path -LiteralPath $stage.stage)){
    if(-not $Resume -or $Completed){throw 'An unpublished stage requires the same unfinished commit.'}
    $expectedTree.entries=@(@($expectedTree.entries)+@([pscustomobject]@{path=[IO.Path]::GetFileName($stage.stage);directory=$false;bytes=$stage.bytes;sha256=$stage.candidateSha256})|Sort-Object path)
  }
  Assert-PreparationTree $expectedTree
  foreach($currentTree in @($Plan.inspection.currentPluginTrees)){Assert-PreparationTree $currentTree}
  Assert-ReconciliationMoves $Plan.proof $Plan.directory -Resume:$Resume
  $acls=Get-SessionPreservationAcls (@($tree)+@($Plan.proof.inventory.trees)+@($Plan.inspection.currentPluginTrees)) $Plan.proof.parent.quarantine
  Assert-SessionPreservationAcls $Plan.inspection.acls $acls $ini
}
function Read-ReconciledBaseline([string]$Workspace,[string]$Record,[string]$Hash,[int]$Depth) {
  $record=Assert-LocalPath $Record;$directory=[IO.Path]::GetDirectoryName($record)
  if((Get-Digest $record) -cne $Hash){throw 'Reconciled baseline changed.'}
  $value=Read-Record $record
  if($value.schema -ne 1 -or $value.owner -cne $script:ReconciliationOwner -or $value.status -cne 'reconciled' -or
     $value.provenance -cne 'explicit-cross-generation-reconciliation;current-user-state-origin-unverified' -or
     $value.launchPermission -isnot [bool] -or $value.launchPermission){throw 'Incomplete or authority-claiming reconciled baseline.'}
  $plan=Read-ReconciliationPlan $Workspace (Join-Path $directory 'plan.json') $value.planSha256 $Depth
  $parent=$plan.proof.parent
  $completion=Join-Path $directory 'completion.json'
  if((Get-Digest $completion) -cne $value.completionSha256){throw 'Reconciliation completion changed.'}
  $done=Read-Record $completion
  if($done.schema -ne 1 -or $done.owner -cne $script:ReconciliationOwner -or $done.status -cne 'reconciled' -or $done.planSha256 -cne $plan.sha256 -or
     $done.parentPreparedSha256 -cne $plan.proof.sha256 -or $done.profileSha256 -cne $plan.value.baselineSha256 -or
     $value.baselineSha256 -cne $plan.value.baselineSha256 -or $value.baselineBytes -ne $plan.value.baselineBytes -or
     ($done.currentContext|ConvertTo-Json -Depth 8 -Compress) -cne ($plan.inspection.currentContext|ConvertTo-Json -Depth 8 -Compress)){throw 'Completed reconciliation differs from its two-context plan.'}
  foreach($key in @('originalPathsRestored','otherProfileFilesPreserved','noPluginTransplant','doNotAutoLaunch')){Assert-TrueBoolean $done.$key ('Reconciliation '+$key)}
  if($done.applicationLaunched -isnot [bool] -or $done.applicationLaunched){throw 'Reconciliation cannot claim an application launch.'}
  $index=0
  foreach($item in @($plan.proof.parent.quarantine)) {
    $journal=Join-Path $directory ('plugin-'+$index+'.json');$index++
    $move=Read-Record $journal
    if($move.owner -cne $script:ReconciliationOwner -or $move.parentPreparedSha256 -cne $plan.proof.sha256 -or
       $move.source -ine $item.destination -or $move.destination -ine $item.path -or $move.sha256 -cne $item.sha256){throw 'Original-path restoration provenance is incomplete.'}
  }
  # Historical lineage remains readable after a later legitimate installation.
  return [pscustomobject]@{sha256=$value.baselineSha256;bytes=$value.baselineBytes;record=$record;recordSha256=$Hash;sid=$parent.context.sid;profile=$parent.context.profile}
}
