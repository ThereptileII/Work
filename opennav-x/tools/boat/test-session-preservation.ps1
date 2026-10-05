# SCRUM-310 pure contracts on disposable files. Native entrypoint/ACL coverage:
# test-commissioning.ps1 -PreservationFixture (CI, or -IsolatedLocal).
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable contracts required.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable CI or explicit isolated local tests required.'}
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('session-preservation-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory $testRoot
# These portable stand-ins do not qualify native identity or ACL semantics.
function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($testRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Escaped fixture root'};return $p}
function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid){$null=Assert-LocalPath $Directory;if($Sid -cne 'S-1-5-21-123'){throw 'Wrong fixture SID'}}
$script:fixtureAcl='fixture ACL'
function Get-PreparationAccessAcl([string]$Path){$null=Assert-LocalPath $Path;return $script:fixtureAcl}
function Assert-PreparationAcl([string]$Expected,[string]$Actual,[switch]$AllowDaclAutoInherited){if($Expected -cne $Actual){throw 'Changed fixture ACL'}}
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$refused=$false;try{$null=& $Body}catch{$refused=$true};if(-not $refused){throw ('Accepted invalid preservation: '+$Name)};$checks.Add($Name)}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 20|ConvertFrom-Json)}
function Review([string]$Before,[string]$After){return [pscustomobject]@{schema=1;owner='OpenNavX.SessionPreservationReview.1';beforeSha256=(Get-Digest $Before);afterSha256=(Get-Digest $After);reviewedUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;changes=@(Get-CommissioningIniDiff $Before $After|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;decision='preserve-current';origin='unverified';reason='Explicit synthetic preservation fixture; no boat or launch authority'}})}}
$originalBaseline=$script:CommissioningBaseline
try{
  $workspace=Join-Path $testRoot 'workspace';$parent=Join-Path $workspace 'runs/20261001-000000-read-only-commissioning-aaaaaaaa'
  $directory=Join-Path $workspace 'runs/20261002-000000-session-preservation-bbbbbbbb';$profile=Join-Path $testRoot 'profile';$plugins=Join-Path $testRoot 'plugins'
  $null=New-Item -ItemType Directory -Path $parent,$directory,$profile,$plugins -Force
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  # Exact production pins are checked before the inert resource byte adapter.
  $pinnedWmm=Get-CommissioningWmmResourcePins
  Pass 'WMM proof pins the three exact reviewed upstream resources' {
    if($pinnedWmm.Count -ne 3 -or $pinnedWmm['WMM.COF'] -cne 'dfa8597825af4e0b87ff4198a5b4fb661b3c49f4cd090cd0164e0259b075582f' -or
       $pinnedWmm['wmm_live.svg'] -cne '044064c5a0af3fc3d41fb884155f8dc3a7638b6de375af722f7862546481267f' -or
       $pinnedWmm['wmm_pi.svg'] -cne 'e055e85ce274aa37b78a9169e7d0dd3c182e0d268864a519d537cad870a9655d'){throw 'Production WMM pins changed'}
  }
  $originalWmmPins=${function:Get-CommissioningWmmResourcePins};$originalInstalled=${function:Get-Installed}
  try {
    $wmmRoot=Join-Path $testRoot 'wmm';$wmmStock=Join-Path $wmmRoot 'stock';$wmmInstall=Join-Path $wmmRoot 'installed';$wmmGeneration=Join-Path $wmmInstall 'generations/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa'
    foreach($appRoot in @($wmmStock,(Join-Path $wmmGeneration 'app'))){$null=New-Item -ItemType Directory (Join-Path $appRoot 'plugins/wmm_pi/data') -Force}
    $script:wmmFixturePins=[ordered]@{};$owned=@()
    foreach($name in $pinnedWmm.Keys){
      foreach($appRoot in @($wmmStock,(Join-Path $wmmGeneration 'app'))){[IO.File]::WriteAllText((Join-Path $appRoot ('plugins/wmm_pi/data/'+$name)),('inert WMM resource '+$name),$encoding)}
      $script:wmmFixturePins[$name]=Get-Digest (Join-Path $wmmStock ('plugins/wmm_pi/data/'+$name))
      $owned+=@{path=('app/plugins/wmm_pi/data/'+$name);sha256=$script:wmmFixturePins[$name]}
    }
    function Get-CommissioningWmmResourcePins {return $script:wmmFixturePins}
    $wmmExe=Join-Path $wmmGeneration 'app/opencpn.exe';[IO.File]::WriteAllText($wmmExe,'INERT; never executed',$encoding)
    $wmmOwn=Join-Path $wmmGeneration 'ownership.json';Write-Record $wmmOwn @{owner='OpenNavX.Alpha1.SideBySide.1';commit=('a'*40);managedFiles=$owned}
    $wmmState=Join-Path $wmmInstall 'state.json';Write-Record $wmmState @{fixture='old state'}
    $script:wmmInstalled=[pscustomobject]@{root=$wmmInstall;generation=$wmmGeneration;executable=$wmmExe;ownership=(Read-Record $wmmOwn)}
    function Get-Installed {return $script:wmmInstalled}
    $wmmParent=[pscustomobject]@{context=[pscustomobject]@{application=$wmmStock;installation=[pscustomobject]@{generation=$wmmGeneration;executable=$wmmExe;commit=('a'*40);executableSha256=(Get-Digest $wmmExe);ownershipSha256=(Get-Digest $wmmOwn);stateSha256=(Get-Digest $wmmState)}}}
    $wmmProof=Get-CommissioningWmmResourceProof $wmmParent
    Pass 'WMM stock and owned installed resources freeze exact parent identity and manifest bytes' {$null=Assert-CommissioningWmmResourceProof $wmmParent (Clone $wmmProof);Assert-CommissioningWmmLiveProof $wmmParent $wmmProof}
    foreach($field in @('owner','generation','commit','stateSha256','ownershipSha256','stockLocation','installedLocation','ownershipBase64')){Refuse ('Changed WMM proof '+$field) {$bad=Clone $wmmProof;$bad.$field='changed';Assert-CommissioningWmmResourceProof $wmmParent $bad}}
    Refuse 'Missing WMM resource proof entry' {$bad=Clone $wmmProof;$bad.resources=@($bad.resources|Select-Object -Skip 1);Assert-CommissioningWmmResourceProof $wmmParent $bad}
    Refuse 'Changed frozen WMM resource hash' {$bad=Clone $wmmProof;$bad.resources[0].sha256='0'*64;Assert-CommissioningWmmResourceProof $wmmParent $bad}
    foreach($kind in @('missing','hash','duplicate')) {
      Refuse ('Frozen WMM manifest must actually own each exact resource: '+$kind) {
        $badOwn=Clone $script:wmmInstalled.ownership
        if($kind -ceq 'missing'){$badOwn.managedFiles=@($badOwn.managedFiles|Select-Object -Skip 1)}
        elseif($kind -ceq 'hash'){$badOwn.managedFiles[0].sha256='0'*64}
        else{$badOwn.managedFiles+=@($badOwn.managedFiles[0])}
        $raw=$encoding.GetBytes(($badOwn|ConvertTo-Json -Depth 8));$bad=Clone $wmmProof;$parentCopy=Clone $wmmParent
        $bad.ownershipBase64=[Convert]::ToBase64String($raw);$bad.ownershipSha256=Get-CommissioningHash $raw;$parentCopy.context.installation.ownershipSha256=$bad.ownershipSha256
        Assert-CommissioningWmmResourceProof $parentCopy $bad
      }
    }
    foreach($path in @($wmmExe,$wmmOwn,$wmmState,(Join-Path $wmmStock 'plugins/wmm_pi/data/WMM.COF'),(Join-Path $wmmGeneration 'app/plugins/wmm_pi/data/wmm_pi.svg'))){
      $savedBytes=[IO.File]::ReadAllBytes($path);[IO.File]::WriteAllText($path,'changed');Refuse 'Live WMM executable/manifest/state/stock/installed resource drift' {Assert-CommissioningWmmLiveProof $wmmParent $wmmProof};[IO.File]::WriteAllBytes($path,$savedBytes)
    }
    $extra=Join-Path $wmmGeneration 'app/plugins/wmm_pi/data/extra';[IO.File]::WriteAllText($extra,'extra');Refuse 'Unowned additional WMM resource refused' {Get-CommissioningWmmResourceProof $wmmParent};Remove-Item $extra
    $wmmBefore=Join-Path $wmmRoot 'before.ini';$wmmAfter=Join-Path $wmmRoot 'after.ini'
    $wmmText="[Settings]`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;0;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n[Directories]`r`nWMMDataLocation="+$wmmProof.stockLocation+"`r`n"
    [IO.File]::WriteAllText($wmmBefore,$wmmText,$encoding);[IO.File]::WriteAllText($wmmAfter,$wmmText.Replace($wmmProof.stockLocation,$wmmProof.installedLocation),$encoding)
    $wmmReview=Review $wmmBefore $wmmAfter
    Pass 'Explicit WMM preservation keeps exactly the derived location and trailing separator' {$null=Assert-SessionPreservationReview $wmmBefore $wmmAfter $wmmReview ([datetime]::UtcNow) '' $wmmProof}
    Refuse 'WMM change without resource proof is still protected' {Assert-SessionPreservationReview $wmmBefore $wmmAfter $wmmReview}
    $migration=Clone $wmmReview;$migration.owner='OpenNavX.ProfileMigrationReview.1'
    Refuse 'Owned WMM change does not broaden automatic migration' {Assert-CommissioningMigrationReview $wmmBefore $wmmAfter $migration}
    foreach($badPath in @(($wmmProof.installedLocation+'other'),($wmmProof.installedLocation.TrimEnd([IO.Path]::DirectorySeparatorChar)),($wmmProof.stockLocation+'other'))){
      [IO.File]::WriteAllText($wmmAfter,$wmmText.Replace($wmmProof.stockLocation,$badPath),$encoding);$badReview=Review $wmmBefore $wmmAfter
      Refuse 'Reviewed arbitrary WMM location or missing trailing separator refused' {Assert-SessionPreservationReview $wmmBefore $wmmAfter $badReview ([datetime]::UtcNow) '' $wmmProof}
    }
    [IO.File]::WriteAllText($wmmBefore,$wmmText.Replace($wmmProof.stockLocation,($wmmProof.installedLocation+'old/')),$encoding)
    [IO.File]::WriteAllText($wmmAfter,$wmmText.Replace($wmmProof.stockLocation,$wmmProof.installedLocation),$encoding)
    $badReview=Review $wmmBefore $wmmAfter
    Refuse 'Historical-generation-to-new-generation location is a separate unsupported preservation case' {Assert-SessionPreservationReview $wmmBefore $wmmAfter $badReview ([datetime]::UtcNow) '' $wmmProof}
    $script:wmmInstalled.generation='retired';Remove-Item $wmmGeneration -Recurse -Force
    Pass 'Historical WMM ownership proof remains readable after the old generation is retired' {$null=Assert-CommissioningWmmResourceProof $wmmParent (Clone $wmmProof)}
    Refuse 'Historical proof cannot satisfy current WMM verification after generation change' {Assert-CommissioningWmmLiveProof $wmmParent $wmmProof}
  } finally {Set-Item Function:Get-CommissioningWmmResourcePins $originalWmmPins;Set-Item Function:Get-Installed $originalInstalled}
  $text="[Settings]`r`nPersistActiveRoute=0`r`nActiveRoute=`r`nLocale=sv`r`nLocaleOverride=sv_SE`r`nConfigVersionString=Version 5.12.4+37fd0cd Build 2026-09-27`r`nGPUTextureMemSize=64`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n[Directories]`r`nChartDir=original`r`n[PlugIns/wmm_pi.dll]`r`nbEnabled=0`r`n[Settings/GlobalState]`r`nFrameWinX=1024`r`n"
  $text+='#'+(' '*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  $baseline=Join-Path $parent 'baseline.ini';$inputFile=Join-Path $parent 'input-only.ini';$current=Join-Path $profile 'opencpn.ini'
  [IO.File]::WriteAllText($baseline,$text,$encoding);$script:CommissioningBaseline=Get-Digest $baseline
  [IO.File]::WriteAllBytes($inputFile,(Get-CommissioningInputBytes ($encoding.GetBytes($text))))
  $alpha='OpenNavXSettings 1\n"battery" ""\n"capacity" "20"\n"consumption" "measured"\n"corridor" "50"\n"current" "unconfigured"\n"display.instruments" "sog,depth"\n"display.rail" "sog,heading"\n"draft" "1"\n"efficiency" ""\n"hotel" ""\n"margin" "1"\n"minimum_speed" "1"\n"model_source" ""\n"reserve" "20"\n'
  $modified=$encoding.GetString([IO.File]::ReadAllBytes($inputFile)).Replace('ActiveRoute=', 'ActiveRoute=11111111-2222-3333-4444-555555555555').Replace('bEnabled=0','bEnabled=1').Replace('FrameWinX=1024','FrameWinX=1280')
  # Replace only the ActiveRoute line, not PersistActiveRoute (different suffix).
  $modified=$modified.Replace('PersistActiveRoute=11111111-2222-3333-4444-5555555555550','PersistActiveRoute=0')
  $modified+="[OpenNav]`r`nAlphaSettings=$alpha`r`n[OpenNav/OnlineAIS/v1]`r`nEnabled=1`r`n"
  [IO.File]::WriteAllText($current,$modified,$encoding)
  [IO.File]::WriteAllText((Join-Path $profile 'navobj.xml'),'<gpx>inert original route fixture</gpx>',$encoding)
  [IO.File]::WriteAllText((Join-Path $plugins 'inert_pi.dll'),'not executable',$encoding)
  $review=Review $inputFile $current
  Pass 'Explicit substantive settings, WMM, Online AIS and stored route preserved as unverified state' {$null=Assert-SessionPreservationReview $inputFile $current $review}
  Pass 'Only recovery direction byte changes; all user settings and route identifier remain exact' {
    $bytes=[IO.File]::ReadAllBytes($current);$output=Get-CommissioningOutputBytes $bytes;$differences=0
    for($i=0;$i -lt $bytes.Length;$i++){if($bytes[$i] -ne $output[$i]){$differences++;if($bytes[$i] -ne 48 -or $output[$i] -ne 49){throw 'Wrong changed byte'}}}
    if($differences -ne 1 -or (Get-CommissioningHash (Get-CommissioningInputBytes $output)) -cne (Get-Digest $current)){throw 'Non-exact preservation inverse'}
  }
  $migration=Clone $review;$migration.owner='OpenNavX.ProfileMigrationReview.1'
  Refuse 'Preservation does not broaden automatic migration policy' {Assert-CommissioningMigrationReview $inputFile $current $migration}
  foreach($field in @('owner','beforeSha256','afterSha256','provenance')){Refuse "Changed review $field" {$bad=Clone $review;$bad.$field='changed';Assert-SessionPreservationReview $inputFile $current $bad}}
  foreach($flag in @('preservationOnly','launchPermission')){Refuse "Changed authority flag $flag" {$bad=Clone $review;$bad.$flag=-not $bad.$flag;Assert-SessionPreservationReview $inputFile $current $bad}}
  foreach($field in @('key','before','after','decision','origin','reason')){Refuse "Changed per-key $field" {$bad=Clone $review;$bad.changes[0].$field='';Assert-SessionPreservationReview $inputFile $current $bad}}
  Refuse 'Missing review entry' {$bad=Clone $review;$bad.changes=@($bad.changes|Select-Object -Skip 1);Assert-SessionPreservationReview $inputFile $current $bad}
  Refuse 'Duplicate review entry' {$bad=Clone $review;$bad.changes[0]=$bad.changes[1];Assert-SessionPreservationReview $inputFile $current $bad}
  foreach($time in @([datetime]::UtcNow.AddHours(-25),[datetime]::UtcNow.AddMinutes(1))){Refuse 'Expired or future independent review' {$bad=Clone $review;$bad.reviewedUtc=$time.ToString('o');Assert-SessionPreservationReview $inputFile $current $bad}}
  $badIni=Join-Path $testRoot 'changed.ini'
  $badValues=@($modified.Replace('COM8','COM9'),$modified.Replace('ChartDir=original','ChartDir=other'),$modified.Replace('PersistActiveRoute=0','PersistActiveRoute=1'),$modified.Replace('11111111-2222-3333-4444-555555555555','unknown route'),($modified+"[Settings/CommPriority]`r`nPriorityHeading=unknown`r`n"),($modified+"[PlugIns/unknown_pi.dll]`r`nbEnabled=1`r`n"),($modified+"[OpenNav/Unknown]`r`nEnabled=1`r`n"),$modified.Replace('"battery" ""','"battery" "new-source"'),$modified.Replace('"reserve" "20"','"pilot.permission" "manual"'),$modified.Replace('"minimum_speed" "1"','"minimum_speed" "999"'),$modified.Replace('"display.rail" "sog,heading"','"display.rail" "sog,sog"'))
  $index=0;foreach($badText in $badValues){$index++;[IO.File]::WriteAllText($badIni,$badText,$encoding);$badReview=Review $inputFile $badIni;Refuse "Matching review cannot approve protected/unknown configuration $index" {Assert-SessionPreservationReview $inputFile $badIni $badReview}}
  $context=[pscustomobject]@{workspace=$workspace;profile=$profile;sid='S-1-5-21-123';installation=[pscustomobject]@{generation='old-fixture-generation'};pluginRoots=@($plugins)}
  $trees=@(Get-PreparationTree $plugins);$inventory=Join-Path $parent 'inventory.json'
  Write-Record $inventory @{context=$context;trees=$trees;plugins=@(Get-CommissioningCandidates $trees)}
  $plan=Join-Path $parent 'review-plan.json';Write-Record $plan @{schema=1;owner='fixture-independent-plugin-review'}
  $sourceEvidence=Join-Path $parent 'source-evidence.txt';[IO.File]::WriteAllText($sourceEvidence,'Synthetic source review')
  $prepared=Join-Path $parent 'prepared.json'
  Write-Record $prepared @{schema=1;owner=$script:CommissioningOwner;status='prepared';context=$context;planSha256=(Get-Digest $plan);evidence=@(@{path=$sourceEvidence;sha256=(Get-Digest $sourceEvidence)});baselineSha256=$script:CommissioningBaseline;inputSha256=(Get-Digest $inputFile);inventorySha256=(Get-Digest $inventory);quarantine=@()}
  $parentHash=Get-Digest $prepared;$snapshot=Get-PreparationTree $profile
  $saved=Join-Path $parent 'post-session-fixture.ini';Copy-PreparationFile $current $saved (Get-Digest $current) (Get-Item $current).Length
  $inspection=Join-Path $parent 'restore-inspection-fixture.json'
  Write-Record $inspection @{owner='OpenNavX.ReadOnlyCommissioning.RestoreInspection.1';recordSha256=$parentHash;context=$context;currentIniSha256=(Get-Digest $current);savedIni=$saved;profileBeforeRestore=$snapshot;resourceProof=$null}
  $review|Add-Member parentPreparedSha256 $parentHash;$review|Add-Member inspectionSha256 (Get-Digest $inspection)
  $reviewPath=Join-Path $directory 'preservation-review.json';Write-Record $reviewPath $review
  Copy-PreparationFile $current (Join-Path $directory 'post-session.ini') (Get-Digest $current) (Get-Item $current).Length
  Copy-PreparationTree $snapshot (Join-Path $directory 'profile-backup')
  $target=Join-Path $directory 'baseline.ini';[IO.File]::WriteAllBytes($target,(Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($current))))
  $proposal=Join-Path $directory 'proposal.json'
  Write-Record $proposal @{schema=1;owner=$script:SessionPreservationOwner;status='proposed';parentPrepared=$prepared;parentPreparedSha256=$parentHash;inspection=$inspection;inspectionSha256=(Get-Digest $inspection);currentIniSha256=(Get-Digest $current);reviewSha256=(Get-Digest $reviewPath);baselineSha256=(Get-Digest $target);baselineBytes=(Get-Item $target).Length;createdUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';launchPermission=$false;changedKeys=@($review.changes).Count;acls=@(Get-SessionPreservationAcls (@($snapshot)+$trees) @())}
  $proposalHash=Get-Digest $proposal
  Pass 'Frozen preservation proposal binds full profile backup and independent exact review' {$script:proof=Read-SessionPreservationProposal $workspace $proposal $proposalHash $prepared $parentHash;Assert-SessionPreservationLiveState $proof $context}
  Refuse 'Incomplete proposal is not usable as baseline' {Read-CommissioningBaseline $workspace $proposal $proposalHash}
  Refuse 'Changed proposal hash' {Read-SessionPreservationProposal $workspace $proposal ('0'*64) $prepared $parentHash}
  foreach($file in @($saved,$reviewPath,$plan,$sourceEvidence,(Join-Path $directory 'post-session.ini'),(Join-Path $directory 'profile-backup/navobj.xml'),$target)){
    $original=[IO.File]::ReadAllBytes($file);[IO.File]::WriteAllText($file,'changed')
    Refuse 'Changed immutable preservation proof or backup' {Read-SessionPreservationProposal $workspace $proposal $proposalHash $prepared $parentHash}
    [IO.File]::WriteAllBytes($file,$original)
  }
  foreach($file in @($current,(Join-Path $profile 'navobj.xml'),(Join-Path $plugins 'inert_pi.dll'))){
    $original=[IO.File]::ReadAllBytes($file);[IO.File]::WriteAllText($file,'changed')
    Refuse 'Late current profile/navigation/plugin drift' {Assert-SessionPreservationLiveState $proof $context}
    [IO.File]::WriteAllBytes($file,$original)
  }
  Refuse 'Changed active generation' {$changed=Clone $context;$changed.installation.generation='other';Assert-SessionPreservationLiveState $proof $changed}
  Refuse 'Changed account' {$changed=Clone $context;$changed.sid='S-1-5-21-456';Assert-SessionPreservationLiveState $proof $changed}
  $script:fixtureAcl='changed ACL';Refuse 'Changed full profile/plugin ACL snapshot' {Assert-SessionPreservationLiveState $proof $context};$script:fixtureAcl='fixture ACL'
  $lock=Open-CommissioningRestoreLock $parent
  try{Refuse 'Exclusive transaction lock refuses competing restoration before mutation' {$other=Open-CommissioningRestoreLock $parent;try{}finally{$other.Dispose()}}}finally{$lock.Dispose()}
  Pass 'Released restoration lock can be reacquired' {$other=Open-CommissioningRestoreLock $parent;$other.Dispose()}
  $intent=Join-Path $parent 'restore-intent-fixture.json'
  Write-Record $intent @{schema=1;owner=$script:CommissioningOwner;recordSha256=$parentHash;afterSha256=(Get-Digest $target);adoptionProposalSha256=$null;preservationProposalSha256=$proposalHash}
  Pass 'Durable target resumes only the identical preservation choice' {Assert-CommissioningRestoreTarget $parent $parentHash (Get-Digest $target) '' $proposalHash}
  Refuse 'Omitted preservation cannot fall back to old baseline' {Assert-CommissioningRestoreTarget $parent $parentHash $script:CommissioningBaseline ''}
  Refuse 'Preservation cannot be relabelled automatic adoption' {Assert-CommissioningRestoreTarget $parent $parentHash (Get-Digest $target) $proposalHash ''}
  [IO.File]::WriteAllBytes($current,[IO.File]::ReadAllBytes($target))
  Pass 'Interrupted publication accepts only the one-byte target, still checks other files' {Assert-SessionPreservationLiveState $proof $context}
  $complete=Join-Path $parent 'restored-fixture.json'
  Write-Record $complete @{owner=$script:CommissioningOwner;status='restored';recordSha256=$parentHash;preservationProposalSha256=$proposalHash;profileSha256=(Get-Digest $target);pluginInventoryRestored=$true;otherProfileFilesPreserved=$true;originalOutputConfigurationRestored=$true;doNotAutoLaunch=$true;applicationLaunched=$false}
  $lineage=Join-Path $directory 'preserved-baseline.json'
  Write-Record $lineage @{schema=1;owner=$script:SessionPreservationOwner;status='preserved';parentPrepared=$prepared;parentPreparedSha256=$parentHash;proposalSha256=$proposalHash;baselineSha256=(Get-Digest $target);baselineBytes=(Get-Item $target).Length;restoreCompletion=$complete;restoreCompletionSha256=(Get-Digest $complete);provenance='current-user-state;origin-unverified';launchPermission=$false}
  Pass 'Completed preservation lineage resolves exact current settings and original direction' {$resolved=Read-CommissioningBaseline $workspace $lineage (Get-Digest $lineage);if($resolved.sha256 -cne (Get-Digest $current)){throw 'Wrong preserved baseline'}}
  Pass 'Historical lineage remains readable after installed generation changes' {$context.installation.generation='new-fixture-generation';$null=Read-CommissioningBaseline $workspace $lineage (Get-Digest $lineage)}
  [IO.File]::WriteAllText($complete,'changed')
  Refuse 'Changed completion cannot authorize fresh commissioning' {Read-CommissioningBaseline $workspace $lineage (Get-Digest $lineage)}
  [pscustomobject]@{status='passed';environment='portable-disposable-contracts';count=$checks.Count;checks=$checks.ToArray();nativeAclOrProcessAcceptance=$false;boatAccess=$false;applicationLaunched=$false}|ConvertTo-Json -Depth 5
}finally{$script:CommissioningBaseline=$originalBaseline;Remove-Item -LiteralPath $testRoot -Recurse -Force}
