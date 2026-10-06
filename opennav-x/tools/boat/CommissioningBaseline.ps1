# Explicit baseline lineage. The recovered a2e4 root is never replaced globally.
# Imported after Commissioning primitives; no launch or transport operations.
. (Join-Path $PSScriptRoot 'RestartCommissioningPolicy.ps1')
. (Join-Path $PSScriptRoot 'ColdBaseline.ps1')
$script:CommissioningBaselineOwner='OpenNavX.ReviewedCommissioningBaseline.1'
function Assert-CommissioningTextureMinimum($Before,$After) {
  # The observed second 5.12.4 launch loads the earlier upgrade's 64MB value.
  # Pinned MyConfig::LoadMyConfig, after LoadMyConfigRaw, clamps non-expert GL
  # texture memory to at least 128MB; UpdateSettings persists that result.
  # This admits only this reviewed normalization, not arbitrary GPU settings.
  $key='Settings/GPUTextureMemSize';$version='Version 5.12.4+37fd0cd Build 2026-09-27'
  $expert='Settings/OpenGLExpert'
  if($Before[$key] -cne '64' -or $After[$key] -cne '128' -or
     $Before['Settings/OpenGL'] -cne '1' -or $After['Settings/OpenGL'] -cne '1' -or
     $Before['Settings/ConfigVersionString'] -cne $version -or $After['Settings/ConfigVersionString'] -cne $version -or
     $Before[$expert] -cne $After[$expert] -or
     ($null -ne $Before[$expert] -and $Before[$expert] -cne '0')) {
    throw 'Only the observed pinned non-expert 64-to-128MB startup normalization may be adopted.'
  }
}
function Assert-CommissioningStockUpgradeDelta([string]$Key,$Before,$After) {
  if($Key -ceq 'Settings/GPUTextureMemSize') {
    # Pinned OCPNPlatform::Initialize_3 selects exactly64MB for the GL-capable
    # upgrade path. The same source path runs when the stored version/build
    # string changes from the exact reviewed Beta executable back to stock.
    # These are two observed transitions, not a general version/GL exception.
    if($Before[$Key] -cne '128' -or $After[$Key] -cne '64' -or
       $Before['Settings/OpenGL'] -cne '1' -or $After['Settings/OpenGL'] -cne '1' -or
       $Before['Settings/ConfigVersionString'] -cnotin @('Version 5.12.2-0+b69f44c Build 2025-08-01','Version 5.12.4+37fd0cd Build 2026-09-27') -or
       $After['Settings/ConfigVersionString'] -cne 'Version 5.12.4-0+37fd0cd Build 2025-09-12') {throw 'Only the observed exact official GL-upgrade texture budget reset may be adopted.'}
    return
  }
  if($Key -ceq 'Settings/MSWFonts/sv-00c6075a') {
    # FontMgr::ScrubList discards "Menu" under locale sv because the pinned
    # catalogue translates it to "Meny". navutil then rewrites the font group.
    # Keep both existing Swedish menu records byte-for-byte; no generic font
    # deletion, value normalization or locale change is authorized here.
    $obsolete='Menu:1;10;-20;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(0, 0, 0)'
    $translated='Meny:1;9;-18;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(0, 0, 0)'
    if($Before[$Key] -cne $obsolete -or $null -ne $After[$Key] -or
       $Before['Settings/Locale'] -cne 'sv' -or $After['Settings/Locale'] -cne 'sv' -or
       $Before['Settings/LocaleOverride'] -cne 'sv_SE' -or $After['Settings/LocaleOverride'] -cne 'sv_SE') {throw 'Only the exact obsolete English menu font under preserved Swedish locale may be removed.'}
    foreach($retained in @('Settings/MSWFonts/sv-f4c5f476','Settings/MSWFonts/sv_SE-f4c5f476')) {
      if($Before[$retained] -cne $translated -or $After[$retained] -cne $translated){throw 'Existing translated menu font must remain byte-for-byte unchanged.'}
    }
    return
  }
  throw 'Unknown stock startup migration.'
}
function Get-CommissioningOutputBytes([byte[]]$Bytes) {
  $encoding=New-Object Text.UTF8Encoding($false,$true);$text=$encoding.GetString($Bytes)
  $connectionMatches=[regex]::Matches($text,'(?m)^DataConnections=([^\r\n]*)\r?$')
  if ($connectionMatches.Count -ne 1) { throw 'Exactly one selected-source connection record required.' }
  $value=$connectionMatches[0].Groups[1];$offset=0;$selected=-1;$count=0
  foreach($connection in $value.Value.Split('|')) {
    if($connection) {
      $fields=$connection.Split(';')
      if($fields.Count -lt 18){throw 'Incomplete marine connection.'}
      if($fields[5] -ceq 'COM8') {
        $count++;if($fields[8] -cne '0'){throw 'Reviewed COM8 must still be input-only.'}
        $within=0;for($i=0;$i -lt 8;$i++){$within+=$fields[$i].Length+1}
        $selected=$encoding.GetByteCount($text.Substring(0,$value.Index+$offset+$within))
      }
    }
    $offset+=$connection.Length+1
  }
  if($count -ne 1 -or $selected -lt 0 -or $Bytes[$selected] -ne 48){throw 'One literal reviewed COM8 input-only byte required.'}
  $result=[byte[]]$Bytes.Clone();$result[$selected]=49
  # Reuse the exact forward contract to verify protocol, direction, source
  # section, enabled state and every other byte; no independent bus mapping.
  if((Get-CommissioningHash (Get-CommissioningInputBytes $result)) -cne (Get-CommissioningHash $Bytes)){throw 'Direction reversal did not preserve the entire migrated profile.'}
  return ,$result
}
function Assert-CommissioningMigrationReview([string]$Before,[string]$After,$Review,[datetime]$At=[datetime]::UtcNow,[string]$InstalledBasemapDefault='') {
  if($Review.schema -ne 1 -or $Review.owner -cne 'OpenNavX.ProfileMigrationReview.1' -or
     $Review.beforeSha256 -cne (Get-Digest $Before) -or $Review.afterSha256 -cne (Get-Digest $After)){throw 'Exact independently reviewed pre/post startup profiles required.'}
  $reviewed=[datetime]::Parse($Review.reviewedUtc).ToUniversalTime()
  if($reviewed -gt $At -or ($At-$reviewed).TotalHours -gt 24){throw 'Migration review expired at the exact adoption preparation time.'}
  Assert-CommissioningProtectedValues (Read-ProfileForAudit $Before) (Read-ProfileForAudit $After) $InstalledBasemapDefault
  $beforeValues=Read-ProfileForAudit $Before;$afterValues=Read-ProfileForAudit $After
  $changes=@(Get-CommissioningIniDiff $Before $After);$entries=@($Review.changes)
  if($changes.Count -gt 512 -or $entries.Count -ne $changes.Count){throw 'Every changed key requires one exact migration review.'}
  $seen=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::Ordinal)
  $policy=Get-RestartDisplayKeys
  foreach($change in $changes) {
    $match=@($entries | Where-Object {$_.key -ceq $change.key})
    $removedMenuFont=$change.key -ceq 'Settings/MSWFonts/sv-00c6075a' -and $null -eq $change.after
    if($match.Count -ne 1 -or -not $seen.Add($change.key) -or ($null -eq $change.after -and -not $removedMenuFont) -or
       $match[0].before -cne $change.before -or $match[0].after -cne $change.after -or
       $match[0].sourceRevision -cne '37fd0cddb7334fe489e9f18aa163977a9c5c84f7' -or
       [string]::IsNullOrWhiteSpace($match[0].sourceBoundary) -or $match[0].sourceBoundary.Length -gt 512 -or
       [string]::IsNullOrWhiteSpace($match[0].reason) -or $match[0].reason.Length -gt 1024){throw 'Missing, duplicate, changed or unreferenced migration approval.'}
    if($policy.ContainsKey($change.key)) {
      Assert-RestartScalar $policy[$change.key] $change.after
      if($null -ne $change.before){Assert-RestartScalar $policy[$change.key] $change.before}
    } elseif($change.key -ceq 'Directories/BaseShapefileDir') {
      if (-not $InstalledBasemapDefault -or $change.before -cne '' -or $change.after -cne $InstalledBasemapDefault) {
        throw 'Only a separately proven installed stock default may fill the existing empty basemap preference.'
      }
    } elseif($change.key -ceq 'Settings/ConfigVersionString') {
      # CMake OCPN_CI_BUILD: +<commit>, or -<release>+<commit>. The exact
      # official 7c6547 binary contains 5.12.4-0+37fd0cd / 2025-09-12;
      # validated fixture-free Windows builds use +37fd0cd and their CI date.
      if($change.after -cne 'Version 5.12.4-0+37fd0cd Build 2025-09-12' -and
         $change.after -cnotmatch '^Version 5\.12\.4\+37fd0cd Build [0-9]{4}-[0-9]{2}-[0-9]{2}$'){throw 'Only the exact official or pinned Windows product build marker may be adopted.'}
      $date=[datetime]::ParseExact($change.after.Substring($change.after.Length-10),'yyyy-MM-dd',[Globalization.CultureInfo]::InvariantCulture)
      if($date -gt $At.Date){throw 'Future build marker is not accepted.'}
    } elseif($change.key -ceq 'Settings/GPUTextureMemSize' -and $change.before -ceq '64' -and $change.after -ceq '128') {
      Assert-CommissioningTextureMinimum $beforeValues $afterValues
    } elseif($change.key -cin @('Settings/GPUTextureMemSize','Settings/MSWFonts/sv-00c6075a')) {
      Assert-CommissioningStockUpgradeDelta $change.key $beforeValues $afterValues
    } elseif($change.key -ceq 'Settings/CommPriority/PriorityVariation') {
      # The ordinary audit parser trims lines. This one observed append must
      # also match the exact raw value, including spaces and final delimiter.
      # A second same-named record anywhere is ambiguous and refuses.
      $encoding=New-Object Text.UTF8Encoding($false,$true)
      $rawBefore=@([IO.File]::ReadAllLines($Before,$encoding) | Where-Object {$_ -cmatch '^PriorityVariation='})
      $rawAfter=@([IO.File]::ReadAllLines($After,$encoding) | Where-Object {$_ -cmatch '^PriorityVariation='})
      if($rawBefore.Count -ne 1 -or $rawAfter.Count -ne 1){throw 'Exactly one literal observed variation-priority record is required.'}
      Assert-ReviewedVariationSourceAppend $rawBefore[0].Substring(18) $rawAfter[0].Substring(18)
    } elseif($change.key -ceq 'Settings/NavMessageShown') {
      if($change.after -cne '1' -or ($null -ne $change.before -and $change.before -cnotin @('0','1'))){throw 'Only actual acknowledged startup notice persistence may be adopted.'}
    } elseif($change.key -cin @('Settings/Locale','Settings/LocaleOverride')) {
      if($change.after.Length -gt 16 -or $change.after -cnotmatch '^(?:[a-z]{2}(?:_[A-Z]{2})?)?$'){throw 'Unexpected locale persistence.'}
    } elseif($change.key -ceq 'AUI/AUIPerspective') {
      if($null -eq $change.before){throw 'A new opaque pane layout needs a separate source-specific policy.'}
      Assert-RestartAuiDelta $change.before $change.after
    } elseif($change.key -ceq 'PlugIns/Dashboard/SumLogNM' -or $change.key -cmatch '^PlugIns/Dashboard/Dashboard(?:[1-9]|1[0-9]|20)/PersistSize[XY]$') {
      if($null -eq $change.before){throw 'New Dashboard state needs a reviewed baseline.'}
      Assert-RestartDashboardDelta $change.key $change.before $change.after
    } else {throw ('Startup change needs a separate source-specific migration policy: '+$change.key)}
  }
  return $changes
}
function Read-CommissioningBaseline([string]$Workspace,[string]$Record,[string]$Sha256,[int]$Depth=0) {
  if(-not $Record -and -not $Sha256){return [pscustomobject]@{sha256=$script:CommissioningBaseline;bytes=21380;record=$null;recordSha256=$null}}
  if($Depth -ge 8 -or $Sha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Malformed or excessive baseline lineage.'}
  $record=Assert-LocalPath $Record;$directory=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetFileName($record) -ceq 'completed-cold-baseline.json') {
    if([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
       [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-baseline-[a-f0-9]{8}$' -or
       (Get-Digest $record) -cne $Sha256){throw 'Expected exact completed cold baseline below this workspace.'}
    $value=Read-Record $record
    if($value.schema -ne 1 -or $value.owner -cne $script:ColdBaselineOwner -or $value.status -cne 'completed' -or
       $value.provenance -cne 'pre-existing-current-user-state;origin-unverified' -or
       $value.preservationOnly -isnot [bool] -or -not $value.preservationOnly -or
       $value.launchPermission -isnot [bool] -or $value.launchPermission -or
       $value.profileChanged -isnot [bool] -or $value.profileChanged -or
       $value.applicationLaunched -isnot [bool] -or $value.applicationLaunched -or
       $value.baselineSha256 -cnotmatch '^[a-f0-9]{64}$' -or $value.baselineBytes -le 0 -or $value.baselineBytes -gt 4194304){throw 'Incomplete or authority-claiming cold baseline.'}
    $capturePath=Assert-LocalPath $value.captureRecord
    if([IO.Path]::GetDirectoryName($capturePath) -ine $directory -or [IO.Path]::GetFileName($capturePath) -cne 'capture.json' -or
       $value.captureSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $capturePath) -cne $value.captureSha256){throw 'Cold capture lineage changed.'}
    $capture=Read-Record $capturePath
    if($capture.schema -ne 1 -or $capture.owner -cne $script:ColdCaptureOwner -or $capture.status -cne 'captured' -or
       $capture.provenance -cne $value.provenance -or $capture.launchPermission -isnot [bool] -or $capture.launchPermission -or
       $capture.profileChanged -isnot [bool] -or $capture.profileChanged -or
       $capture.applicationLaunched -isnot [bool] -or $capture.applicationLaunched -or
       $capture.profileSha256 -cne $value.baselineSha256 -or $capture.profileBytes -ne $value.baselineBytes -or
       $capture.sourceIniLastWriteTicks -le 0 -or $capture.sourceIniCreationTicks -le 0 -or
       $capture.predecessorRecord -ine $value.predecessorRecord -or $capture.predecessorSha256 -cne $value.predecessorSha256 -or
       $capture.stockSha256 -cne '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'){
      throw 'Cold capture identity or preservation claim changed.'
    }
    if((Assert-LocalPath $capture.context.workspace) -ine (Assert-LocalPath $Workspace) -or
       (Assert-LocalPath $capture.profileTree.root) -ine (Assert-LocalPath $capture.context.profile) -or
       $capture.profileTree.exists -isnot [bool] -or -not $capture.profileTree.exists -or
       $capture.context.sid -cnotmatch '^S-1-5-[0-9-]+$'){
      throw 'Cold capture workspace or actual profile root changed.'
    }
    Assert-ColdPrivateEvidence $directory $capture.context.sid
    $prior=Read-CommissioningBaseline $Workspace $capture.predecessorRecord $capture.predecessorSha256 ($Depth+1)
    if(-not $prior.record -or $prior.sid -cne $capture.context.sid -or $prior.profile -ine $capture.context.profile){throw 'Cold predecessor belongs to another user/profile.'}
    $old=Join-Path $directory 'predecessor.ini';$backup=Join-Path $directory 'profile-backup';$saved=Join-Path $backup 'opencpn.ini'
    if((Get-Digest $old) -cne $prior.sha256 -or (Get-Item -LiteralPath $old).Length -ne $prior.bytes -or
       (Get-Digest $saved) -cne $value.baselineSha256 -or (Get-Item -LiteralPath $saved).Length -ne $value.baselineBytes -or
       (Get-Digest (Join-Path $directory 'input-only.ini')) -cne $capture.inputSha256 -or
       (Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($saved)))) -cne $capture.inputSha256){throw 'Cold backup or one-byte input transform differs.'}
    $backupTree=Get-PreparationTree $backup
    if(($backupTree.entries | ConvertTo-Json -Depth 8 -Compress) -cne ($capture.profileTree.entries | ConvertTo-Json -Depth 8 -Compress)){
      throw 'Full cold profile backup changed.'
    }
    $reviewPath=Join-Path $directory 'review.json'
    if($value.reviewSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $reviewPath) -cne $value.reviewSha256){throw 'Cold independent review changed.'}
    $review=Read-Record $reviewPath
    if($review.captureSha256 -cne $value.captureSha256 -or $review.predecessorRecordSha256 -cne $capture.predecessorSha256 -or
       @((Assert-ColdBaselineDelta $old $saved $review ([datetime]::Parse($value.createdUtc).ToUniversalTime()))).Count -ne $value.changedKeys){
      throw 'Cold key review no longer matches copied bytes.'
    }
    return [pscustomobject]@{sha256=$value.baselineSha256;bytes=$value.baselineBytes;record=$record;recordSha256=$Sha256;
      sid=$capture.context.sid;profile=$capture.context.profile}
  }
  if([IO.Path]::GetFileName($record) -ceq 'preserved-baseline.json'){return Read-CompletedSessionPreservation $Workspace $record $Sha256 $Depth}
  if([IO.Path]::GetFileName($record) -cne 'adopted-baseline.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
     [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-baseline-adoption-[a-f0-9]{8}$' -or (Get-Digest $record) -cne $Sha256){throw 'Expected immutable completed baseline lineage below this workspace.'}
  $value=Read-Record $record
  if($value.schema -ne 1 -or $value.owner -cne $script:CommissioningBaselineOwner -or $value.status -cne 'adopted' -or
     $value.baselineSha256 -cnotmatch '^[a-f0-9]{64}$' -or $value.baselineBytes -le 0 -or $value.baselineBytes -gt 4194304){throw 'Incomplete baseline adoption cannot authorize commissioning.'}
  $preparedPath=Assert-LocalPath $value.parentPrepared;$parentDir=[IO.Path]::GetDirectoryName($preparedPath)
  if([IO.Path]::GetFileName($preparedPath) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($parentDir) -ine (Join-Path $Workspace 'runs') -or
     $value.parentPreparedSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $preparedPath) -cne $value.parentPreparedSha256){throw 'Parent transaction lineage changed.'}
  $parent=Read-Record $preparedPath
  $baseline=Get-PreparedCommissioningBaseline $parent $parentDir $Workspace ($Depth+1)
  $proof=Read-CommissioningAdoptionProposal $Workspace (Join-Path $directory 'proposal.json') $value.proposalSha256 $value.parentPrepared $value.parentPreparedSha256
  $proposal=$proof.value
  if($proposal.baselineSha256 -cne $value.baselineSha256 -or $proposal.baselineBytes -ne $value.baselineBytes){throw 'Completed adoption differs from its exact reviewed proposal.'}
  $complete=Read-Record $value.restoreCompletion
  if((Get-Digest $value.restoreCompletion) -cne $value.restoreCompletionSha256 -or [IO.Path]::GetDirectoryName((Assert-LocalPath $value.restoreCompletion)) -ine $parentDir -or
     $complete.owner -cne $script:CommissioningOwner -or $complete.status -cne 'restored' -or $complete.recordSha256 -cne $value.parentPreparedSha256 -or
     $complete.profileSha256 -cne $value.baselineSha256 -or $complete.adoptionProposalSha256 -cne $value.proposalSha256 -or
     $complete.pluginInventoryRestored -isnot [bool] -or -not $complete.pluginInventoryRestored -or
     $complete.originalOutputConfigurationRestored -isnot [bool] -or -not $complete.originalOutputConfigurationRestored){throw 'Complete original plugin/profile restoration is required for baseline adoption.'}
  return [pscustomobject]@{sha256=$value.baselineSha256;bytes=$value.baselineBytes;record=$record;recordSha256=$Sha256;sid=$parent.context.sid;profile=$parent.context.profile}
}
function Get-PreparedCommissioningBaseline($Prepared,[string]$Directory,[string]$Workspace,[int]$Depth=0) {
  if($Prepared.owner -cne $script:CommissioningOwner -or $Prepared.status -cne 'prepared'){throw 'Unrecognized parent commissioning transaction.'}
  $reference=if($Prepared.PSObject.Properties['baselineReference']){$Prepared.baselineReference}else{$null}
  $baseline=if($reference){Read-CommissioningBaseline $Workspace $reference.record $reference.recordSha256 $Depth}else{Read-CommissioningBaseline $Workspace '' '' $Depth}
  if($Prepared.baselineSha256 -cne $baseline.sha256 -or (Get-Digest (Join-Path $Directory 'baseline.ini')) -cne $baseline.sha256 -or
     (Get-Item -LiteralPath (Join-Path $Directory 'baseline.ini')).Length -ne $baseline.bytes){throw 'Prepared baseline differs from its pinned root or immutable adoption lineage.'}
  if((Get-Digest (Join-Path $Directory 'input-only.ini')) -cne $Prepared.inputSha256 -or
     (Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes((Join-Path $Directory 'baseline.ini'))))) -cne $Prepared.inputSha256){throw 'Prepared baseline lineage must retain the exact forward one-byte transform.'}
  if($reference -and ($Prepared.context.sid -cne $baseline.sid -or $Prepared.context.profile -ine $baseline.profile)){throw 'Baseline adoption belongs to another user/profile.'}
  return $baseline
}
function Read-CommissioningAdoptionProposal([string]$Workspace,[string]$Path,[string]$Hash,[string]$ParentRecord,[string]$ParentHash) {
  $path=Assert-LocalPath $Path;$directory=[IO.Path]::GetDirectoryName($path)
  if($Hash -cnotmatch '^[a-f0-9]{64}$' -or [IO.Path]::GetFileName($path) -cne 'proposal.json' -or
     [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
     [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-baseline-adoption-[a-f0-9]{8}$' -or (Get-Digest $path) -cne $Hash){throw 'Exact private adoption proposal required.'}
  $proposal=Read-Record $path;$parent=Read-Record $ParentRecord;$parentDir=[IO.Path]::GetDirectoryName($ParentRecord)
  if($proposal.schema -ne 1 -or $proposal.owner -cne $script:CommissioningBaselineOwner -or $proposal.status -cne 'proposed' -or
     $proposal.parentPrepared -ine $ParentRecord -or $proposal.parentPreparedSha256 -cne $ParentHash -or (Get-Digest $ParentRecord) -cne $ParentHash -or
     $proposal.baselineSha256 -cnotmatch '^[a-f0-9]{64}$' -or $proposal.baselineBytes -le 0 -or $proposal.baselineBytes -gt 4194304){throw 'Adoption must belong to this exact unchanged parent transaction.'}
  $inspectionPath=Assert-LocalPath $proposal.inspection;$inspection=Read-Record $inspectionPath
  if([IO.Path]::GetDirectoryName($inspectionPath) -ine $parentDir -or (Get-Digest $inspectionPath) -cne $proposal.inspectionSha256 -or
     $inspection.owner -cne 'OpenNavX.ReadOnlyCommissioning.RestoreInspection.1' -or $inspection.recordSha256 -cne $ParentHash -or
     $inspection.currentIniSha256 -cne $proposal.currentIniSha256 -or (Get-Digest $inspection.savedIni) -cne $proposal.currentIniSha256){throw 'Reviewed post-close inspection changed.'}
  $saved=Join-Path $directory 'post-session.ini';$baseline=Join-Path $directory 'baseline.ini';$review=Join-Path $directory 'migration-review.json'
  if((Get-Digest $saved) -cne $proposal.currentIniSha256 -or (Get-Digest $review) -cne $proposal.reviewSha256 -or
     (Get-Digest $baseline) -cne $proposal.baselineSha256 -or (Get-Item -LiteralPath $baseline).Length -ne $proposal.baselineBytes -or
     (Get-CommissioningHash (Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($saved)))) -cne $proposal.baselineSha256){throw 'Adoption bytes no longer match the reviewed exact one-byte reversal.'}
  $reviewData=Read-Record $review
  if($reviewData.parentPreparedSha256 -cne $ParentHash -or $reviewData.inspectionSha256 -cne $proposal.inspectionSha256){throw 'Migration approval belongs to another exact transaction or inspection.'}
  $resourceProof=if($inspection.PSObject.Properties['resourceProof']){$inspection.resourceProof}else{$null}
  $default=Assert-CommissioningResourceProof $parent $resourceProof
  $null=Assert-CommissioningMigrationReview (Join-Path $parentDir 'input-only.ini') $saved $reviewData ([datetime]::Parse($proposal.createdUtc).ToUniversalTime()) $default
  return [pscustomobject]@{value=$proposal;directory=$directory;baseline=$baseline;sha256=$Hash}
}

# Once restoration has a durable intent, retries must keep that exact target.
# An omitted adoption argument cannot silently reset a migrated profile to a2e4.
function Assert-CommissioningRestoreTarget([string]$Directory,[string]$RecordHash,[string]$TargetHash,[string]$AdoptionHash,[string]$PreservationHash='') {
  foreach($file in @(Get-ChildItem -LiteralPath $Directory -Filter 'restore-intent-*.json' -File -Force)) {
    $intent=Read-Record (Assert-LocalPath $file.FullName)
    $recordedAdoption=if($intent.PSObject.Properties['adoptionProposalSha256']){[string]$intent.adoptionProposalSha256}else{''}
    $recordedPreservation=if($intent.PSObject.Properties['preservationProposalSha256']){[string]$intent.preservationProposalSha256}else{''}
    if($intent.schema -ne 1 -or $intent.owner -cne $script:CommissioningOwner -or
       $intent.recordSha256 -cne $RecordHash -or $intent.afterSha256 -cne $TargetHash -or
       $recordedAdoption -cne $AdoptionHash -or $recordedPreservation -cne $PreservationHash){throw 'Restoration already has a different or malformed durable target; resume its exact reviewed choice.'}
  }
}

# SCRUM-310: current user-state preservation is separate from startup migration.
# These records never establish provenance, launch permission or source safety.
$script:SessionPreservationOwner='OpenNavX.SessionPreservation.1'
function Open-CommissioningRestoreLock([string]$Directory) {
  $path=Assert-LocalPath (Join-Path $Directory 'restoration.lock')
  try {return [IO.File]::Open($path,[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)}
  catch {throw 'Another inspection/restoration owns this transaction; no mutation attempted.'}
}
function Assert-PreservedSetupScalar([string]$Value,[double]$Minimum,[double]$Maximum,[bool]$AllowEmpty=$false) {
  # Settings.cpp ParseSettingNumber/ValidateSettings and SettingsStore's chart
  # contour bound. No culture-sensitive commas, NaN or infinite spellings.
  if($AllowEmpty -and $Value -ceq ''){return}
  if($Value.Length -gt 64 -or $Value -cnotmatch '^-?[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$'){throw 'Invalid preserved setup scalar.'}
  $number=[double]::Parse($Value,[Globalization.CultureInfo]::InvariantCulture)
  if([double]::IsNaN($number) -or [double]::IsInfinity($number) -or $number -lt $Minimum -or $number -gt $Maximum){throw 'Preserved setup scalar outside current source bounds.'}
}
function Assert-PreservedVesselName([string]$Value) {
  # wxFileConfig Save (wxWidgets 3.2): backslashes escaped; surrounding quotes
  # only for an initial quote or edge whitespace. Quoted embedded quotes escape.
  $quoted=$Value.StartsWith('"');$text=$Value
  if($quoted){if($Value.Length -lt 2 -or -not $Value.EndsWith('"')){throw 'Unbalanced vessel-name encoding.'};$text=$Value.Substring(1,$Value.Length-2)}
  $decoded=New-Object Text.StringBuilder
  for($i=0;$i -lt $text.Length;$i++){
    $c=$text[$i]
    if($c -eq '\'){
      $i++;if($i -ge $text.Length -or ($text[$i] -ne '\' -and (-not $quoted -or $text[$i] -ne '"'))){throw 'Unsupported/control vessel-name escape.'}
      $c=$text[$i]
    }
    if([int]$c -lt 32 -or [int]$c -eq 127){throw 'Vessel name contains a control character.'}
    $null=$decoded.Append($c)
  }
  $name=$decoded.ToString();$utf8=New-Object Text.UTF8Encoding($false,$true)
  if($utf8.GetByteCount($name) -gt 128 -or ($name.Length -gt 0 -and [string]::IsNullOrWhiteSpace($name))){throw 'Vessel name exceeds source bounds or is only whitespace.'}
  $canonical=$name.Replace('\','\\')
  if($name.Length -gt 0 -and ($name.StartsWith('"') -or [char]::IsWhiteSpace($name[0]) -or [char]::IsWhiteSpace($name[$name.Length-1]))){$canonical='"'+$canonical.Replace('"','\"')+'"'}
  if($canonical -cne $Value){throw 'Noncanonical vessel-name encoding requires separate review.'}
}
function Assert-PreservedSetupPreference([string]$Key,[string]$Value) {
  switch -CaseSensitive ($Key) {
    'OpenNav/BoatSetupV1' {if($Value -cnotin @('v1|pending','v1|complete','v1|existing')){throw 'Unknown BoatSetup progress record.'}}
    'OpenNav/DisplayPreferencesV1' {if($Value -cnotmatch '^v1\|(100|125|150)\|(balanced|chart|instruments)$'){throw 'Unknown display preferences record.'}}
    'OpenNav/VesselName' {Assert-PreservedVesselName $Value}
    'Settings/GlobalState/S52_MAR_SAFETY_CONTOUR' {Assert-PreservedSetupScalar $Value 0 1000000}
    default {throw 'Unknown setup preference; no generic OpenNav admission.'}
  }
}
function Assert-PreservedSetupSettingsDelta([string]$Before,[string]$After) {
  # Do not decode/re-serialize opaque source mappings, calibration or identity.
  # Replace only unescaped complete known scalar lines, then demand that every
  # remaining byte matches. std::quoted escapes embedded quotes, so quoted
  # multiline values cannot impersonate these field delimiters.
  $normalized=New-Object 'Collections.Generic.List[string]'
  foreach($record in @($Before,$After)){
    if($record.Length -gt 131072 -or -not $record.StartsWith('OpenNavXSettings 1\n') -or $record -match '[\x00-\x1f\x7f]'){throw 'Unsupported vessel-settings delta encoding.'}
    if([regex]::Matches($record,'(?<=\\n)"model_source" "').Count -ne 1 -or
       [regex]::Matches($record,'(?<=\\n)"pilot.permission" "').Count -gt 1){throw 'Duplicate or missing protected/provenance field.'}
    $copy=$record
    foreach($key in @('capacity','reserve','draft')){
      $pattern='(?<=\\n)"'+$key+'" "([^"\\\x00-\x1f]*)"(?=\\n)'
      $matches=[regex]::Matches($copy,$pattern)
      if($matches.Count -ne 1){throw 'Missing or duplicate setup scalar field.'}
      $value=$matches[0].Groups[1].Value
      switch($key){'capacity'{Assert-PreservedSetupScalar $value .001 100000 $true};'reserve'{Assert-PreservedSetupScalar $value 0 100 $true};'draft'{Assert-PreservedSetupScalar $value 0 100 $true}}
      $copy=[regex]::Replace($copy,$pattern,('"'+$key+'" "<reviewed-scalar>"'))
    }
    # Existing Vessel form writes this exact provenance. Setup itself preserves
    # any preexisting provenance, including unknown source text, unchanged.
    $copy=[regex]::Replace($copy,'(?<=\\n)"model_source" "(?:|User-configured usable battery energy and reserve / OpenCPN profile)"(?=\\n)','"model_source" "<reviewed-provenance>"')
    $normalized.Add($copy)
  }
  if($normalized[0] -ceq $normalized[1]){return}
  # SettingsStore::RestoreBackup keeps the local binding but always revokes
  # permission. No reverse transition or identity/source edit is admitted.
  $beforeOff=[regex]::Replace($normalized[0],'(?<=\\n)"pilot.permission" "manual"(?=\\n)','"pilot.permission" "display-only"')
  if($beforeOff -cne $normalized[0] -and $beforeOff -ceq $normalized[1]){return}
  throw 'Setup preservation cannot change source mappings, calibration, pilot identity or grant control.'
}
function Assert-PreservedAlphaSettings([string]$Value) {
  # Settings.cpp EncodeSettings and wxFileConfig's literal newline encoding.
  # Deliberately exclude pilot/bridge, sensor mappings, calibration and unknown
  # fields. This validates this small base-model shape, not arbitrary settings.
  if($Value.Length -gt 4096){throw 'Vessel settings exceed preservation bound.'}
  $lines=$Value.Split([string[]]@('\n'),[StringSplitOptions]::None)
  if($lines.Count -ne 16 -or $lines[0] -cne 'OpenNavXSettings 1' -or $lines[-1] -cne ''){throw 'Unknown vessel settings encoding.'}
  $fields=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
  foreach($line in $lines[1..14]){
    if($line -cnotmatch '^"([a-z_.]+)" "([^"\\\x00-\x1f]*)"$' -or $fields.ContainsKey($Matches[1])){throw 'Ambiguous vessel settings field.'}
    $fields.Add($Matches[1],$Matches[2])
  }
  $required=@('battery','capacity','consumption','corridor','current','display.instruments','display.rail','draft','efficiency','hotel','margin','minimum_speed','model_source','reserve')
  foreach($key in $required){if(-not $fields.ContainsKey($key)){throw 'Unknown protected vessel settings field set.'}}
  if($fields['battery'] -cne '' -or $fields['model_source'] -cnotin @('','User-configured usable battery energy and reserve / OpenCPN profile') -or $fields['consumption'] -cne 'measured' -or
      $fields['current'] -cnotin @('unconfigured','charge','discharge')){throw 'Source binding or calibrated model needs separate preservation policy.'}
  $bounds=@{capacity=@(.001,100000);reserve=@(0,100);minimum_speed=@(.1,20);hotel=@(0,10000);efficiency=@(.001,1);draft=@(0,100);margin=@(0,100);corridor=@(1,10000)}
  foreach($key in $bounds.Keys){
    $value=$fields[$key]
    if($value -ceq '' -and $key -cne 'minimum_speed'){continue}
    if($value.Length -gt 64 -or $value -cnotmatch '^[0-9]+(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$'){throw 'Invalid preserved model scalar.'}
    $number=[double]::Parse($value,[Globalization.CultureInfo]::InvariantCulture)
    if([double]::IsInfinity($number) -or [double]::IsNaN($number) -or $number -lt $bounds[$key][0] -or $number -gt $bounds[$key][1]){throw 'Preserved model scalar outside its source bounds.'}
  }
  $known=@('sog','cog','heading','stw','aws','awa','tws','twa','depth','water_temp','pressure','rudder','heel','soc','voltage','current','pack_power','motor_power','rpm','motor_temp','fresh_water','fuel','waste')
  foreach($key in @('display.rail','display.instruments')){
    $items=$fields[$key].Split(',');$maximum=if($key -ceq 'display.rail'){6}else{23}
    if($items.Count -lt 1 -or $items.Count -gt $maximum -or @($items|Sort-Object -Unique).Count -ne $items.Count){throw 'Invalid preserved display selection.'}
    foreach($item in $items){if($item -cnotin $known){throw 'Unknown preserved display item.'}}
  }
}
function Assert-SessionPreservationReview([string]$Before,[string]$After,$Review,[datetime]$At=[datetime]::UtcNow,[string]$InstalledBasemapDefault='',$WmmResourceProof=$null) {
  if($Review.schema -ne 1 -or $Review.owner -cne 'OpenNavX.SessionPreservationReview.1' -or
      $Review.beforeSha256 -cne (Get-Digest $Before) -or $Review.afterSha256 -cne (Get-Digest $After) -or
      $Review.provenance -cne 'current-user-state;origin-unverified' -or
      $Review.preservationOnly -isnot [bool] -or -not $Review.preservationOnly -or
      $Review.launchPermission -isnot [bool] -or $Review.launchPermission){throw 'Exact preservation-only independent review required.'}
  $reviewed=[datetime]::Parse($Review.reviewedUtc).ToUniversalTime()
  if($reviewed -gt $At -or ($At-$reviewed).TotalHours -gt 24){throw 'Preservation review expired or future-dated.'}
  $old=Read-ProfileForAudit $Before;$new=Read-ProfileForAudit $After
  Assert-CommissioningProtectedValues $old $new $InstalledBasemapDefault $WmmResourceProof
  # Navigation source priorities, route persistence and all unknown plugin
  # settings remain fixed. The one explicit WMM switch below is preservation,
  # not approval to load a DLL. Every future launch needs a new plugin audit.
  foreach($key in @(@($old.Keys)+@($new.Keys)|Sort-Object -Unique)){
    if($old[$key] -cne $new[$key] -and $key -cmatch '^(Settings/CommPriority/|Settings/PersistActiveRoute$|OpenNav/(?:Autopilot|Sources|BoatBridge)|PlugIns/)' -and
        $key -cne 'PlugIns/wmm_pi.dll/bEnabled'){throw 'Protected source, route-persistence or plugin settings changed.'}
  }
  $changes=@(Get-CommissioningIniDiff $Before $After);$entries=@($Review.changes)
  if($changes.Count -lt 1 -or $changes.Count -gt 64 -or $entries.Count -ne $changes.Count){throw 'Every bounded current-profile difference requires review.'}
  $display=Get-RestartDisplayKeys
  foreach($change in $changes){
    $key=$change.key;$match=@($entries|Where-Object{$_.key -ceq $key})
    if($match.Count -ne 1 -or $match[0].before -cne $change.before -or $match[0].after -cne $change.after -or
        $match[0].origin -cne 'unverified' -or $match[0].decision -cne 'preserve-current' -or
        [string]::IsNullOrWhiteSpace($match[0].reason) -or $match[0].reason.Length -gt 1024){throw 'Missing, duplicate or changed preservation decision.'}
    if($display.ContainsKey($key)){
      Assert-RestartScalar $display[$key] $change.after
      if($null -ne $change.before){Assert-RestartScalar $display[$key] $change.before}
    }elseif($key -ceq 'OpenNav/AlphaSettings'){
      if($null -eq $change.after){throw 'Settings deletion requires separate review.'}
      if($null -ne $change.before){
        try {Assert-PreservedAlphaSettings $change.after;Assert-PreservedAlphaSettings $change.before}
        catch {Assert-PreservedSetupSettingsDelta $change.before $change.after}
      }else{Assert-PreservedAlphaSettings $change.after}
    }elseif($key -cin @('OpenNav/BoatSetupV1','OpenNav/DisplayPreferencesV1','OpenNav/VesselName','Settings/GlobalState/S52_MAR_SAFETY_CONTOUR')){
      if($null -eq $change.after){throw 'Setup preference deletion requires separate review.'}
      Assert-PreservedSetupPreference $key $change.after
      if($null -ne $change.before){Assert-PreservedSetupPreference $key $change.before}
    }elseif($key -ceq 'OpenNav/OnlineAIS/v1/Enabled' -or $key -ceq 'PlugIns/wmm_pi.dll/bEnabled'){
      if($change.after -cnotin @('0','1') -or ($null -ne $change.before -and $change.before -cnotin @('0','1'))){throw 'Preserved enable flag must be an exact boolean.'}
    }elseif($key -ceq 'Settings/ActiveRoute'){
      if($old['Settings/PersistActiveRoute'] -cne '0' -or $new['Settings/PersistActiveRoute'] -cne '0'){throw 'Route persistence requires separate policy; no navigation changes made.'}
      foreach($value in @($change.before,$change.after)){
        if($null -ne $value -and $value -cne '' -and $value -cnotmatch '^[a-fA-F0-9]{8}-(?:[a-fA-F0-9]{4}-){3}[a-fA-F0-9]{12}$'){throw 'Unrecognized stored route identifier.'}
      }
    }elseif($key -ceq 'Settings/ConfigVersionString'){
      foreach($value in @($change.before,$change.after)){
        if($value -cnotmatch '^Version 5\.12\.4(?:-0)?\+37fd0cd Build 20[0-9]{2}-[0-9]{2}-[0-9]{2}$' -or
            [datetime]::ParseExact($value.Substring($value.Length-10),'yyyy-MM-dd',[Globalization.CultureInfo]::InvariantCulture) -gt $At.Date){throw 'Unknown preserved build marker.'}
      }
    }elseif($key -ceq 'Settings/GPUTextureMemSize'){
      if($change.before -cnotin @('64','128') -or $change.after -cnotin @('64','128')){throw 'Unreviewed texture setting.'}
    }elseif($key -ceq 'Settings/MSWFonts/sv-00c6075a' -and $null -eq $change.after){
      Assert-CommissioningStockUpgradeDelta $key $old $new
    }elseif($key -ceq 'Settings/MSWFonts/sv-6f52a406'){
      if($old['Settings/Locale'] -cne 'sv' -or $new['Settings/Locale'] -cne 'sv' -or $old['Settings/LocaleOverride'] -cne 'sv_SE' -or $new['Settings/LocaleOverride'] -cne 'sv_SE'){throw 'Font locale changed.'}
      foreach($value in @($change.before,$change.after)){
        if($null -eq $value){continue}
        if($value.Length -gt 256 -or $value -cnotmatch '^[^:;\r\n]{1,64}:(?:-?[0-9]+;){15}[^:;\r\n]{1,64}:rgb\(([0-9]{1,3}), ?([0-9]{1,3}), ?([0-9]{1,3})\)$'){throw 'Unknown preserved menu font encoding.'}
        foreach($component in @($Matches[1],$Matches[2],$Matches[3])){if([int]$component -gt 255){throw 'Invalid preserved font color.'}}
      }
    }elseif($key -ceq 'Directories/BaseShapefileDir' -and $InstalledBasemapDefault -and $change.before -ceq '' -and $change.after -ceq $InstalledBasemapDefault){
      # Existing hash-bound resource proof; never an arbitrary chart path.
    }elseif($key -ceq 'Directories/WMMDataLocation' -and $WmmResourceProof -and $change.before -ceq $WmmResourceProof.stockLocation -and $change.after -ceq $WmmResourceProof.installedLocation){
      # Frozen owned/pinned WMM resources prove only this exact save-time delta.
    }else{throw ('Current-state preservation needs a separate key policy: '+$key)}
  }
  $null=Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($After))
  return $changes
}
function Read-SessionPreservationProposal([string]$Workspace,[string]$Path,[string]$Hash,[string]$ParentRecord,[string]$ParentHash) {
  $path=Assert-LocalPath $Path;$directory=[IO.Path]::GetDirectoryName($path)
  if($Hash -cnotmatch '^[a-f0-9]{64}$' -or [IO.Path]::GetFileName($path) -cne 'proposal.json' -or
      [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-session-preservation-[a-f0-9]{8}$' -or (Get-Digest $path) -cne $Hash){throw 'Exact private session-preservation proposal required.'}
  $value=Read-Record $path;$parent=Read-SessionPreservationParent $ParentRecord $ParentHash;$parentDir=[IO.Path]::GetDirectoryName($ParentRecord)
  if($value.schema -ne 1 -or $value.owner -cne $script:SessionPreservationOwner -or $value.status -cne 'proposed' -or
      $value.parentPrepared -ine $ParentRecord -or $value.parentPreparedSha256 -cne $ParentHash -or (Get-Digest $ParentRecord) -cne $ParentHash -or
      $value.provenance -cne 'current-user-state;origin-unverified' -or $value.launchPermission -isnot [bool] -or $value.launchPermission -or
      $value.baselineSha256 -cnotmatch '^[a-f0-9]{64}$' -or $value.baselineBytes -le 0 -or $value.baselineBytes -gt 4194304){throw 'Preservation proposal belongs to another parent or claims authority.'}
  Assert-ColdPrivateEvidence $directory $parent.context.sid
  $inspectionPath=Assert-LocalPath $value.inspection;$inspection=Read-Record $inspectionPath
  if([IO.Path]::GetDirectoryName($inspectionPath) -ine $parentDir -or (Get-Digest $inspectionPath) -cne $value.inspectionSha256 -or
      $inspection.owner -cne 'OpenNavX.ReadOnlyCommissioning.RestoreInspection.1' -or $inspection.recordSha256 -cne $ParentHash -or
      $inspection.currentIniSha256 -cne $value.currentIniSha256 -or (Get-Digest $inspection.savedIni) -cne $value.currentIniSha256){throw 'Exact original closed-session inspection changed.'}
  Assert-CommissioningContext $parent.context $inspection.context
  $saved=Join-Path $directory 'post-session.ini';$baseline=Join-Path $directory 'baseline.ini';$review=Join-Path $directory 'preservation-review.json'
  $backup=Get-PreparationTree (Join-Path $directory 'profile-backup')
  if(($backup.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($inspection.profileBeforeRestore.entries|ConvertTo-Json -Depth 8 -Compress) -or
      (Get-Digest $saved) -cne $value.currentIniSha256 -or (Get-Digest $review) -cne $value.reviewSha256 -or
      (Get-Digest $baseline) -cne $value.baselineSha256 -or (Get-Item -LiteralPath $baseline).Length -ne $value.baselineBytes -or
      (Get-CommissioningHash (Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($saved)))) -cne $value.baselineSha256){throw 'Preserved full profile or exact one-byte recovery target changed.'}
  $approval=Read-Record $review
  if($approval.parentPreparedSha256 -cne $ParentHash -or $approval.inspectionSha256 -cne $value.inspectionSha256){throw 'Preservation review belongs to another inspection.'}
  $resourceProof=if($inspection.PSObject.Properties['resourceProof']){$inspection.resourceProof}else{$null}
  $default=Assert-CommissioningResourceProof $parent $resourceProof
  $wmmProof=if($inspection.PSObject.Properties['wmmResourceProof']){$inspection.wmmResourceProof}else{$null}
  $wmmProof=Assert-CommissioningWmmResourceProof $parent $wmmProof
  $changes=@(Assert-SessionPreservationReview (Join-Path $parentDir 'input-only.ini') $saved $approval ([datetime]::Parse($value.createdUtc).ToUniversalTime()) $default $wmmProof)
  if($changes.Count -ne $value.changedKeys){throw 'Preservation review count differs.'}
  return [pscustomobject]@{value=$value;directory=$directory;baseline=$baseline;sha256=$Hash}
}
function Read-CompletedSessionPreservation([string]$Workspace,[string]$Record,[string]$Hash,[int]$Depth) {
  $directory=[IO.Path]::GetDirectoryName($Record)
  if([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-session-preservation-[a-f0-9]{8}$' -or (Get-Digest $Record) -cne $Hash){throw 'Exact completed preservation lineage required.'}
  $value=Read-Record $Record
  if($value.schema -ne 1 -or $value.owner -cne $script:SessionPreservationOwner -or $value.status -cne 'preserved' -or
      $value.provenance -cne 'current-user-state;origin-unverified' -or $value.launchPermission -isnot [bool] -or $value.launchPermission){throw 'Preservation is incomplete or claims launch permission.'}
  $parentPath=Assert-LocalPath $value.parentPrepared;$parentDir=[IO.Path]::GetDirectoryName($parentPath)
  if([IO.Path]::GetDirectoryName($parentDir) -ine (Join-Path $Workspace 'runs') -or [IO.Path]::GetFileName($parentPath) -cne 'prepared.json' -or
      $value.parentPreparedSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $parentPath) -cne $value.parentPreparedSha256){throw 'Preserved parent lineage changed.'}
  $parent=Read-Record $parentPath;$null=Get-PreparedCommissioningBaseline $parent $parentDir $Workspace ($Depth+1)
  $proof=Read-SessionPreservationProposal $Workspace (Join-Path $directory 'proposal.json') $value.proposalSha256 $parentPath $value.parentPreparedSha256
  $completePath=Assert-LocalPath $value.restoreCompletion;$complete=Read-Record $completePath
  if([IO.Path]::GetDirectoryName($completePath) -ine $parentDir -or (Get-Digest $completePath) -cne $value.restoreCompletionSha256 -or
      $complete.owner -cne $script:CommissioningOwner -or $complete.status -cne 'restored' -or $complete.recordSha256 -cne $value.parentPreparedSha256 -or
      $complete.preservationProposalSha256 -cne $value.proposalSha256 -or $complete.profileSha256 -cne $proof.value.baselineSha256 -or
      $value.baselineSha256 -cne $proof.value.baselineSha256 -or $value.baselineBytes -ne $proof.value.baselineBytes){throw 'Preservation lacks exact durable original restoration.'}
  foreach($flag in @('pluginInventoryRestored','otherProfileFilesPreserved','originalOutputConfigurationRestored','doNotAutoLaunch')){Assert-TrueBoolean $complete.$flag ('Preservation completion '+$flag)}
  if($complete.applicationLaunched -isnot [bool] -or $complete.applicationLaunched){throw 'Unexpected launch claim.'}
  # Historical proof deliberately does not require its old generation current.
  return [pscustomobject]@{sha256=$value.baselineSha256;bytes=$value.baselineBytes;record=$Record;recordSha256=$Hash;sid=$parent.context.sid;profile=$parent.context.profile}
}
function Get-SessionPreservationAcls($Trees,$Quarantine) {
  $result=New-Object 'Collections.Generic.List[object]'
  foreach($tree in @($Trees)){
    if(-not $tree.exists){continue}
    foreach($path in @($tree.root)+@($tree.entries|ForEach-Object {Join-Path $tree.root $_.path})){
      $actual=Assert-LocalPath $path
      $moved=@($Quarantine|Where-Object {$_.path -ieq $path})
      if($moved.Count -gt 1){throw 'Ambiguous plugin ACL mapping.'}
      if($moved.Count -eq 1 -and -not(Test-Path -LiteralPath $actual)){$actual=Assert-LocalPath $moved[0].destination}
      $result.Add([pscustomobject]@{path=$path;sddl=(Get-PreparationAccessAcl $actual)})
    }
  }
  return @($result.ToArray()|Sort-Object path)
}
function Assert-SessionPreservationAcls($Expected,$Actual,[string]$Ini) {
  if(@($Expected).Count -ne @($Actual).Count){throw 'Preserved ACL inventory changed.'}
  for($i=0;$i -lt @($Expected).Count;$i++){
    if($Expected[$i].path -cne $Actual[$i].path){throw 'Preserved ACL path changed.'}
    Assert-PreparationAcl $Expected[$i].sddl $Actual[$i].sddl -AllowDaclAutoInherited:($Expected[$i].path -ieq $Ini)
  }
}
function Assert-SessionPreservationLiveState($Proof,$Context) {
  $parent=Read-SessionPreservationParent $Proof.value.parentPrepared $Proof.value.parentPreparedSha256;$parentDir=[IO.Path]::GetDirectoryName($Proof.value.parentPrepared)
  Assert-CommissioningContext $parent.context $Context
  $inspection=Read-Record $Proof.value.inspection
  $wmmProof=if($inspection.PSObject.Properties['wmmResourceProof']){$inspection.wmmResourceProof}else{$null}
  Assert-CommissioningWmmLiveProof $parent $wmmProof
  $snapshot=$inspection.profileBeforeRestore|ConvertTo-Json -Depth 8|ConvertFrom-Json
  $ini=Join-Path $Context.profile 'opencpn.ini';$current=Get-Digest $ini
  if($current -cnotin @($Proof.value.currentIniSha256,$Proof.value.baselineSha256)){throw 'Current bytes are neither preserved input nor exact recovery target.'}
  $entry=@($snapshot.entries|Where-Object {$_.path -ceq 'opencpn.ini'})
  if($entry.Count -ne 1){throw 'Ambiguous preserved profile inventory.'}
  $entry[0].sha256=$current;$entry[0].bytes=(Get-Item -LiteralPath $ini).Length
  Assert-PreparationTree $snapshot
  $inventory=Read-Record (Join-Path $parentDir 'inventory.json')
  if((Get-Digest (Join-Path $parentDir 'inventory.json')) -cne $parent.inventorySha256){throw 'Prepared plugin inventory changed.'}
  Assert-CommissioningInventory $inventory $Context
  Assert-CommissioningTrees $inventory.trees $parent.quarantine -AllowMoved
  foreach($item in @($parent.quarantine)){
    $path=if([IO.File]::Exists($item.path)){$item.path}else{$item.destination}
    Assert-PreparationAcl $item.acl (Get-PreparationAccessAcl $path)
  }
  $actual=Get-SessionPreservationAcls (@($snapshot)+@($inventory.trees)) $parent.quarantine
  Assert-SessionPreservationAcls $Proof.value.acls $actual $ini
}
function Read-SessionPreservationParent([string]$Path,[string]$Hash) {
  if($Hash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Path) -cne $Hash){throw 'Original prepared transaction changed.'}
  $parent=Read-Record $Path;$directory=[IO.Path]::GetDirectoryName($Path)
  if((Get-Digest (Join-Path $directory 'review-plan.json')) -cne $parent.planSha256){throw 'Original source review plan changed.'}
  foreach($evidence in @($parent.evidence)){
    if((Get-Digest $evidence.path) -cne $evidence.sha256){throw 'Saved original source evidence changed.'}
  }
  foreach($item in @($parent.quarantine)){
    if((Get-Digest $item.backup) -cne $item.sha256){throw 'Saved original plugin backup changed.'}
  }
  return $parent
}
