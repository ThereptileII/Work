# Explicit baseline lineage. The recovered a2e4 root is never replaced globally.
# Imported after Commissioning primitives; no launch or transport operations.
. (Join-Path $PSScriptRoot 'RestartCommissioningPolicy.ps1')
$script:CommissioningBaselineOwner='OpenNavX.ReviewedCommissioningBaseline.1'
function Assert-CommissioningStockUpgradeDelta([string]$Key,$Before,$After) {
  if($Key -ceq 'Settings/GPUTextureMemSize') {
    # Pinned OCPNPlatform::Initialize_3 selects exactly64MB for the GL-capable
    # upgrade path. This is the observed official5.12.2->5.12.4 migration only.
    if($Before[$Key] -cne '128' -or $After[$Key] -cne '64' -or
       $Before['Settings/OpenGL'] -cne '1' -or $After['Settings/OpenGL'] -cne '1' -or
       $Before['Settings/ConfigVersionString'] -cne 'Version 5.12.2-0+b69f44c Build 2025-08-01' -or
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
function Assert-CommissioningRestoreTarget([string]$Directory,[string]$RecordHash,[string]$TargetHash,[string]$AdoptionHash) {
  foreach($file in @(Get-ChildItem -LiteralPath $Directory -Filter 'restore-intent-*.json' -File -Force)) {
    $intent=Read-Record (Assert-LocalPath $file.FullName)
    $recordedAdoption=if($intent.PSObject.Properties['adoptionProposalSha256']){[string]$intent.adoptionProposalSha256}else{''}
    if($intent.schema -ne 1 -or $intent.owner -cne $script:CommissioningOwner -or
       $intent.recordSha256 -cne $RecordHash -or $intent.afterSha256 -cne $TargetHash -or
       $recordedAdoption -cne $AdoptionHash){throw 'Restoration already has a different or malformed durable target; resume its exact reviewed choice.'}
  }
}
