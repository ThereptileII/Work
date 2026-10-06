# Separate physical-manual commissioning policy. Never restores a plugin or sends
# pilot bytes. Commands remain explicit product UI actions with actual feedback.
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
$script:ManualPilotOwner='OpenNavX.ManualPilotCommissioning.1'

function Assert-ManualPilotCandidate($Installed,$Expected,$Quarantine=@()) {
  foreach($field in @('commit','executableSha256','ownershipSha256','packageSha256')) {
    $pattern=if($field -ceq 'commit'){'^[a-f0-9]{40}$'}else{'^[a-f0-9]{64}$'}
    if($Expected.$field -cnotmatch $pattern){throw 'Explicit exact qualified candidate hashes required.'}
  }
  if($Expected.generation -cnotmatch '^[a-f0-9]{32}$' -or
     $Installed.state.current -cne $Expected.generation -or $Installed.ownership.commit -cne $Expected.commit -or
     $Installed.ownership.packageSha256 -cne $Expected.packageSha256 -or
     $Installed.ownership.xnavHardwareOutputPolicy -cne 'manual-commissioning' -or
     $Installed.ownership.xnavManualControlContract -isnot [int] -or $Installed.ownership.xnavManualControlContract -ne 1 -or
     (Get-Digest $Installed.executable) -cne $Expected.executableSha256 -or
     (Get-Digest (Join-Path $Installed.generation 'ownership.json')) -cne $Expected.ownershipSha256) {
    throw 'Installed candidate is not the exact qualified manual-control contract.'
  }
  # Authenticate pre-existing runtime bytes against the pinned ownership record,
  # before taking a snapshot. A snapshot alone cannot qualify an altered DLL.
  $files=New-Object 'Collections.Generic.Dictionary[string,string]' ([StringComparer]::OrdinalIgnoreCase)
  foreach($entry in @($Installed.ownership.files)+@($Installed.ownership.managedFiles)) {
    if($entry.path -cnotmatch '^[A-Za-z0-9_. -]+(?:/[A-Za-z0-9_. -]+)*$' -or
       @($entry.path.Split('/')|Where-Object {$_ -cin @('.','..')}).Count -or $entry.sha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Unknown owned runtime path/hash.'}
    if($files.ContainsKey($entry.path) -and $files[$entry.path] -cne $entry.sha256){throw 'Conflicting owned runtime records.'}
    $files[$entry.path]=$entry.sha256
  }
  if(-not $files.ContainsKey('app/opencpn.exe')){throw 'Complete product runtime inventory required.'}
  foreach($relative in $files.Keys) {
    $path=Assert-LocalPath (Join-Path $Installed.generation $relative)
    if([IO.File]::Exists($path)) {
      if((Get-Digest $path) -cne $files[$relative]){throw ('Owned runtime changed: '+$relative)}
    }else {
      $moves=@($Quarantine|Where-Object {$_.path -ieq $path -and $_.sha256 -ceq $files[$relative]})
      if($moves.Count -ne 1 -or (Get-Digest $moves[0].destination) -cne $files[$relative] -or
         (Get-Digest $moves[0].backup) -cne $files[$relative]){throw 'Missing runtime is not the exact parent-owned quarantined plugin.'}
      if([IO.Path]::GetFileName($path) -notlike '*_pi.dll'){throw 'Only reviewed plugin DLLs may be quarantined.'}
    }
  }
  $tree=Get-PreparationTree $Installed.generation
  foreach($entry in @($tree.entries|Where-Object {-not $_.directory})) {
    $relative=$entry.path.Replace('\','/')
    if($relative -cne 'ownership.json' -and -not $files.ContainsKey($relative)){throw 'Unowned file in qualified product generation.'}
  }
}
function Assert-ManualPilotProfile($Values,[bool]$Output) {
  Assert-ManualInactiveRouteSettings $Values
  if($Values['Directories/pluginInstallDir']){throw 'No custom plugin loader may accompany manual commissioning.'}
  $connections=$Values['Settings/NMEADataSource/DataConnections'];$count=0
  if(-not $connections){throw 'Existing COM8 connection required.'}
  foreach($connection in $connections.Split('|')) {
    if(-not $connection){continue};$f=$connection.Split(';')
    if($f.Count -lt 18 -or $f[8] -cnotmatch '^[012]$' -or $f[17] -cnotmatch '^[01]$'){throw 'Ambiguous connection record.'}
    if($f[5] -ceq 'COM8') {
      $count++;$direction=if($Output){'1'}else{'0'}
      if($f[0] -cne '0' -or $f[4] -cne '1' -or $f[8] -cne $direction -or $f[17] -cne '1' -or $f[14] -cne '0') {throw 'Only the existing enabled Actisense COM8 serial direction is admitted.'}
    }elseif($f[17] -ceq '1' -and $f[8] -cne '0'){throw 'Another enabled output is forbidden.'}
  }
  if($count -ne 1){throw 'Exactly one COM8 connection required.'}
}
function Get-ManualPilotBinding([string]$Alpha) {
  # Match entire unescaped scalar lines in the existing wxFileConfig encoding.
  # Preserve all unrelated bytes, including opaque sources and calibration.
  if(-not $Alpha.StartsWith('OpenNavXSettings 1\n') -or $Alpha.Length -gt 65536){throw 'Existing bounded version-1 settings required.'}
  $values=@{};$remainder=$Alpha
  foreach($field in @('interface','name','permission')) {
    $pattern='(?<=\\n)"pilot\.'+$field+'" "([^"\\\x00-\x1f]*)"\\n'
    $matches=[regex]::Matches($Alpha,$pattern)
    $all=[regex]::Matches($Alpha,'(?<=\\n)"pilot\.'+$field+'" ')
    if($matches.Count -gt 1 -or $matches.Count -ne $all.Count){throw 'Ambiguous/escaped pilot setting.'}
    $values[$field]=if($matches.Count){$matches[0].Groups[1].Value}else{''}
    $remainder=[regex]::Replace($remainder,$pattern,'')
  }
  if([regex]::IsMatch($remainder,'(?<=\\n)"pilot\.')){throw 'Unknown pilot setting.'}
  if($values.interface -cnotin @('','COM8') -or $values.name -cnotmatch '^(?:|[a-f0-9]{16})$' -or
     $values.permission -cnotin @('','display-only','manual')){throw 'Only exact COM8 pilot binding is supported.'}
  if($values.name) {
    $name=[Convert]::ToUInt64($values.name,16)
    if((($name -shr 21) -band 0x7ff) -ne 1851 -or (($name -shr 40) -band 0xff) -ne 135 -or
       (($name -shr 49) -band 0x7f) -ne 40 -or (($name -shr 60) -band 7) -ne 4){throw 'NAME must have the supported translator class/function/manufacturer/industry.'}
  }
  if($values.permission -ceq 'manual' -and (-not $values.interface -or -not $values.name)){throw 'Manual permission without exact binding is forbidden.'}
  return [pscustomobject]@{interface=$values.interface;name=$values.name;permission=$values.permission;remainder=$remainder}
}
function Assert-ManualPilotDelta([string]$Before,[string]$After) {
  $priorBinding=Get-ManualPilotBinding $Before;$nextBinding=Get-ManualPilotBinding $After
  if($priorBinding.remainder -cne $nextBinding.remainder){throw 'Pilot preservation cannot change opaque sources, calibration or other vessel settings.'}
  if($nextBinding.permission -ceq 'manual'){throw 'Return to display-only in the product before rollback; permission is never granted by this helper.'}
  return $nextBinding
}
function Assert-ManualPilotProfileDelta([string]$Before,[string]$After) {
  $old=Read-ProfileForAudit $Before;$new=Read-ProfileForAudit $After
  Assert-ManualPilotProfile $old $true;Assert-ManualPilotProfile $new $true
  if($old['Settings/ActiveRoute'] -cne $new['Settings/ActiveRoute']){throw 'Manual session must preserve the inert stored route GUID verbatim.'}
  $diff=@(Get-CommissioningIniDiff $Before $After)
  if($diff.Count -gt 64){throw 'Too many profile changes for bounded manual review.'}
  $binding=Assert-ManualPilotDelta $old['OpenNav/AlphaSettings'] $new['OpenNav/AlphaSettings']
  # Validate the remaining delta with the unchanged session-preservation policy.
  # These private derived copies are validation inputs only: never publish them.
  # The caller approves the real complete inspection hash before invoking rollback.
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  $beforeInput=Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($Before))
  $afterText=$encoding.GetString((Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($After))))
  $pattern='(?m)^AlphaSettings='+[regex]::Escape($new['OpenNav/AlphaSettings'])+'(?=\r?$)'
  if([regex]::Matches($afterText,$pattern).Count -ne 1){throw 'Exact unique settings record required for separate pilot review.'}
  $afterText=[regex]::Replace($afterText,$pattern,[Text.RegularExpressions.MatchEvaluator]{param($m) return 'AlphaSettings='+$old['OpenNav/AlphaSettings']})
  $stem=Join-Path ([IO.Path]::GetDirectoryName($Before)) ('delta-check-'+[guid]::NewGuid().ToString('N'))
  $beforeFile=$stem+'-before.ini';$afterFile=$stem+'-after.ini'
  try {
    [IO.File]::WriteAllBytes($beforeFile,$beforeInput);[IO.File]::WriteAllBytes($afterFile,$encoding.GetBytes($afterText))
    $other=@(Get-CommissioningIniDiff $beforeFile $afterFile)
    if($other.Count) {
      $review=[pscustomobject]@{schema=1;owner='OpenNavX.SessionPreservationReview.1';beforeSha256=(Get-Digest $beforeFile);afterSha256=(Get-Digest $afterFile);
        reviewedUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;
        changes=@($other|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';decision='preserve-current';reason='Caller approved the exact closed manual-child inspection; pilot delta separately validated.'}})}
      $null=Assert-SessionPreservationReview $beforeFile $afterFile $review
    }
  }finally {
    foreach($file in @($beforeFile,$afterFile)){if(Test-Path $file){Remove-Item -LiteralPath $file}}
  }
  return $diff
}
function Get-ManualPilotEngine {
  $windows=Assert-LocalPath ([Environment]::GetFolderPath('Windows'))
  $system=if([Environment]::Is64BitOperatingSystem -and -not [Environment]::Is64BitProcess){'SysWOW64'}else{'System32'}
  $image=Assert-LocalPath (Join-Path $windows ($system+'/WindowsPowerShell/v1.0/powershell.exe'))
  $signature=Get-AuthenticodeSignature -LiteralPath $image
  if($signature.Status -ne 'Valid' -or -not $signature.SignerCertificate -or
     $signature.SignerCertificate.Subject -notmatch '(^|, )O=Microsoft Corporation(,|$)'){throw 'Verified signed system Windows PowerShell required.'}
  return [pscustomobject]@{path=$image;sha256=(Get-Digest $image)}
}
function Assert-ManualPilotFresh([string]$Time) {
  $at=[datetime]::Parse($Time).ToUniversalTime();$now=[datetime]::UtcNow
  if($at -gt $now -or ($now-$at).TotalHours -gt 4){throw 'Manual launch qualification is older than four hours or future-dated.'}
}
function Get-ManualPilotPaths([string]$Workspace,[string]$Record) {
  $workspace=Assert-LocalPath $Workspace;$record=Assert-LocalPath $Record
  $dir=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($dir) -ine (Join-Path $workspace 'runs') -or
     [IO.Path]::GetFileName($dir) -cnotmatch '^\d{8}-\d{6}-manual-pilot-[a-f0-9]{8}$'){throw 'Private manual commissioning record required.'}
  return [pscustomobject]@{workspace=$workspace;record=$record;directory=$dir;active=(Join-Path $workspace 'manual-pilot-active.json')}
}
function Read-ManualPilot([string]$Workspace,[string]$Record,[string]$Hash,[switch]$Active,[switch]$Launching) {
  $paths=Get-ManualPilotPaths $Workspace $Record
  if($Hash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $paths.record) -cne $Hash){throw 'Manual record hash changed.'}
  $r=Read-Record $paths.record
  if($r.schema -ne 1 -or $r.owner -cne $script:ManualPilotOwner -or $r.status -cne 'prepared'){throw 'Unknown manual transaction.'}
  $context=Get-CommissioningIdentityContext $Workspace
  Assert-CommissioningContext $r.context $context
  $boundParent=Read-Record $r.parentRecord
  Assert-ManualPilotCandidate (Get-Installed) $r.candidate @($boundParent.quarantine)
  if((Get-Digest (Join-Path $Workspace 'boat-target.json')) -cne $r.targetSha256 -or
     (Get-Digest (Join-Path $Workspace 'commissioning-active.json')) -cne $r.parentActiveSha256){throw 'Parent ownership or target changed.'}
  foreach($f in @($r.proofFiles)) {if((Get-Digest $f.path) -cne $f.sha256){throw 'Bound source/qualification/parent evidence changed.'}}
  if(@(Get-ChildItem -LiteralPath $r.parentDirectory -Filter 'restore*-*.json').Count -or
     (Test-Path -LiteralPath (Join-Path $r.parentDirectory 'restored.json'))){throw 'Parent restoration has begun.'}
  $parent=Read-Record $r.parentRecord;$inventory=Read-Record (Join-Path $r.parentDirectory 'inventory.json')
  Assert-CommissioningTrees $inventory.trees $parent.quarantine -AllowMoved
  foreach($move in @($parent.quarantine)) {
    if((Test-Path -LiteralPath $move.path) -or (Get-Digest $move.destination) -cne $move.sha256 -or
       (Get-Digest $move.backup) -cne $move.sha256){throw 'Parent plugin quarantine changed.'}
    Assert-PreparationAcl $move.acl (Get-Acl -LiteralPath $move.destination).Sddl
  }
  Assert-PreparationTree $r.generationTree
  Assert-PreparationTree $r.toolsTree
  Assert-PreparationAcl $r.profileAcl (Get-Acl -LiteralPath (Join-Path $context.profile 'opencpn.ini')).Sddl -AllowDaclAutoInherited
  if($Active) {
    $activeRecord=Read-Record $paths.active
    if($activeRecord.owner -cne $script:ManualPilotOwner -or $activeRecord.record -ine $paths.record -or $activeRecord.recordSha256 -cne $Hash){throw 'Another child owns manual commissioning.'}
  }
  if($Launching -and (Test-Path -LiteralPath (Join-Path $paths.directory 'rolled-back.json'))){throw 'Manual transaction has ended.'}
  if($Launching) {
    if(Test-Path (Join-Path $context.installation.root 'update-pending.json')){throw 'Pending update requires separate recovery; manual launch refused.'}
    Assert-ManualPilotFresh $r.createdUtc
    if((Test-Path (Join-Path $paths.directory 'launch-intent.json')) -or (Test-Path (Join-Path $paths.directory 'rollback-intent.json'))){throw 'One launch attempt only; inspect prior intent, never replay.'}
    $applied=Read-Record (Join-Path $paths.directory 'applied.json')
    if($applied.owner -cne $script:ManualPilotOwner -or $applied.recordSha256 -cne $Hash -or $applied.profileSha256 -cne $r.outputSha256){throw 'Complete child Apply required.'}
    $ini=Join-Path $context.profile 'opencpn.ini'
    if((Get-Digest $ini) -cne $r.outputSha256){throw 'Launch profile differs from exact prepared output bytes.'}
    Assert-ManualPilotProfile (Read-ProfileForAudit $ini) $true
    $binding=Get-ManualPilotBinding ((Read-ProfileForAudit $ini)['OpenNav/AlphaSettings'])
    if($binding.permission -ceq 'manual'){throw 'Manual launch must begin with saved permission off.'}
    Assert-PreparationClosed (@($context.application,$context.managed,$context.installation.root)+$context.pluginRoots)
  }
  return [pscustomobject]@{record=$r;paths=$paths;context=$context}
}
function New-ManualPilot([string]$Workspace,$Candidate,[string]$Qualification,[string]$QualificationSha256) {
  $context=Get-CommissioningContext $Workspace;$installed=Get-Installed;$config=Get-Target $Workspace
  if(Test-Path (Join-Path $installed.root 'update-pending.json')){throw 'Pending update requires separate recovery; manual preparation refused.'}
  $boundParent=Read-Record $config.readOnlyAudit.commissioning.record
  Assert-ManualPilotCandidate $installed $Candidate @($boundParent.quarantine)
  if(Test-Path (Join-Path $Workspace 'manual-pilot-active.json')){throw 'Existing manual child requires inspection.'}
  $null=Assert-ReadOnlyAudit $config $installed $Workspace
  if($QualificationSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Qualification) -cne $QualificationSha256){throw 'Exact independent qualification evidence required.'}
  $q=Read-Record $Qualification
  if($q.owner -cne 'OpenNavX.ManualPilotQualification.1' -or $q.commit -cne $Candidate.commit -or
     $q.packageSha256 -cne $Candidate.packageSha256 -or $q.executableSha256 -cne $Candidate.executableSha256){throw 'Qualification evidence belongs to another candidate.'}
  Assert-TrueBoolean $q.nativeSerialGatePassed 'Native serial gate';Assert-TrueBoolean $q.defaultOffDiagnosticsPassed 'Default-off candidate diagnostics'
  Assert-TrueBoolean $q.retainedPluginsReviewedForBidirectional 'Retained plugin startup/idle/shutdown with bidirectional COM8'
  Assert-ManualPilotFresh $q.reviewedUtc
  $ini=Join-Path $context.profile 'opencpn.ini';$values=Read-ProfileForAudit $ini
  Assert-ManualPilotProfile $values $false
  $binding=Get-ManualPilotBinding $values['OpenNav/AlphaSettings']
  if($binding.permission -ceq 'manual'){throw 'Return to display-only before preparing manual commissioning.'}
  $engine=Get-ManualPilotEngine
  $parent=Assert-LocalPath $config.readOnlyAudit.commissioning.record;$parentDir=[IO.Path]::GetDirectoryName($parent)
  $plan=Read-Record (Join-Path $parentDir 'review-plan.json')
  Assert-RestartShutdownReview $q.shutdown @($plan.plugins | Where-Object {$_.decision -ceq 'retain'})
  $parentLock=Open-CommissioningRestoreLock $parentDir
  try {
  if(Test-Path (Join-Path $Workspace 'manual-pilot-active.json')){throw 'Another manual child was prepared.'}
  $null=Assert-ReadOnlyAudit $config $installed $Workspace
  $directory=New-PreparationDirectory $context 'manual-pilot'
  $input=Join-Path $directory 'input.ini';$output=Join-Path $directory 'output.ini'
  $inputHash=Get-Digest $ini;Copy-PreparationFile $ini $input $inputHash (Get-Item $ini).Length
  [IO.File]::WriteAllBytes($output,(Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($input))))
  $outputHash=Get-Digest $output
  $proof=New-Object 'Collections.Generic.List[object]'
  foreach($file in @(Get-ChildItem -LiteralPath $parentDir -File | Where-Object {$_.Name -cne 'restoration.lock'})) {
    $proof.Add([pscustomobject]@{path=$file.FullName;sha256=(Get-Digest $file.FullName)})
  }
  foreach($file in @($input,$output,(Assert-LocalPath $Qualification))) {$proof.Add([pscustomobject]@{path=$file;sha256=(Get-Digest $file)})}
  $record=Join-Path $directory 'prepared.json'
  Write-Record $record @{schema=1;owner=$script:ManualPilotOwner;status='prepared';createdUtc=[datetime]::UtcNow.ToString('o');context=$context;
    candidate=$Candidate;engine=$engine;inputSha256=$inputHash;outputSha256=$outputHash;profileAcl=(Get-Acl -LiteralPath $ini).Sddl;
    parentRecord=$parent;parentDirectory=$parentDir;parentActiveSha256=(Get-Digest (Join-Path $Workspace 'commissioning-active.json'));
    targetSha256=(Get-Digest (Join-Path $Workspace 'boat-target.json'));proofFiles=$proof.ToArray();
    generationTree=(Get-PreparationTree $installed.generation);toolsTree=(Get-PreparationTree $PSScriptRoot);
    outputScope='Existing COM8 only; six explicit product manual commands; discovery60928; no plugin restoration; all session control starts off'}
  Write-Record (Join-Path $Workspace 'manual-pilot-active.json') @{schema=1;owner=$script:ManualPilotOwner;record=$record;recordSha256=(Get-Digest $record)}
  return [pscustomobject]@{status='prepared';record=$record;recordSha256=(Get-Digest $record);profileModified=$false}
  }finally{$parentLock.Dispose()}
}
function Apply-ManualPilot([string]$Workspace,[string]$Record,[string]$Hash) {
  $v=Read-ManualPilot $Workspace $Record $Hash -Active;$p=$v.paths;$r=$v.record
  $parentLock=Open-CommissioningRestoreLock $v.record.parentDirectory
  $lock=$null
  try {
    $lock=Open-CommissioningRestoreLock $p.directory
    if(Test-Path (Join-Path $p.directory 'apply-intent.json')){throw 'Existing Apply intent: inspect, never replay Apply.'}
    Assert-ManualPilotFresh $r.createdUtc
    Assert-PreparationClosed (@($v.context.application,$v.context.managed,$v.context.installation.root)+$v.context.pluginRoots)
    $ini=Join-Path $v.context.profile 'opencpn.ini'
    if((Get-Digest $ini) -cne $r.inputSha256){throw 'Current input profile changed.'}
    $intent=Join-Path $p.directory 'apply-intent.json'
    Write-Record $intent @{owner=$script:ManualPilotOwner;recordSha256=$Hash;beforeSha256=$r.inputSha256;afterSha256=$r.outputSha256}
    Publish-PreparedProfile $ini (Join-Path $p.directory 'output.ini') $r.inputSha256 $r.outputSha256 (Get-Item $ini).Length $intent
    Write-Record (Join-Path $p.directory 'applied.json') @{owner=$script:ManualPilotOwner;recordSha256=$Hash;profileSha256=$r.outputSha256}
    return [pscustomobject]@{status='applied';profileSha256=$r.outputSha256;pluginsRestored=$false;applicationLaunched=$false}
  }finally{if($lock){$lock.Dispose()};$parentLock.Dispose()}
}
function Inspect-ManualPilot([string]$Workspace,[string]$Record,[string]$Hash) {
  $v=Read-ManualPilot $Workspace $Record $Hash -Active;$p=$v.paths
  Assert-PreparationClosed (@($v.context.application,$v.context.managed,$v.context.installation.root)+$v.context.pluginRoots)
  $parentLock=Open-CommissioningRestoreLock $v.record.parentDirectory
  $lock=$null
  try {
    $lock=Open-CommissioningRestoreLock $p.directory
    $ini=Join-Path $v.context.profile 'opencpn.ini';$currentHash=Get-Digest $ini
    $stem='inspection-'+[guid]::NewGuid().ToString('N');$copy=Join-Path $p.directory ($stem+'.ini')
    Copy-PreparationFile $ini $copy $currentHash (Get-Item $ini).Length
    $inspection=Join-Path $p.directory ($stem+'.json')
    Write-Record $inspection @{owner=$script:ManualPilotOwner;recordSha256=$Hash;createdUtc=[datetime]::UtcNow.ToString('o');currentIniSha256=$currentHash;savedIni=$copy;
      profileTree=(Get-PreparationTree $v.context.profile);diff=@(Get-CommissioningIniDiff (Join-Path $p.directory 'output.ini') $copy);
      meaning='Exact closed current state for independent review; not approval, inferred feedback or permission to relaunch'}
    return [pscustomobject]@{status='inspected';inspection=$inspection;inspectionSha256=(Get-Digest $inspection);currentIniSha256=$currentHash}
  }finally{if($lock){$lock.Dispose()};$parentLock.Dispose()}
}
function Rollback-ManualPilot([string]$Workspace,[string]$Record,[string]$Hash,[string]$Inspection,[string]$InspectionHash,[string]$ReviewedCurrentHash) {
  $v=Read-ManualPilot $Workspace $Record $Hash -Active;$p=$v.paths;$r=$v.record
  $parentLock=Open-CommissioningRestoreLock $v.record.parentDirectory
  $lock=$null
  try {
    $lock=Open-CommissioningRestoreLock $p.directory
    Assert-PreparationClosed (@($v.context.application,$v.context.managed,$v.context.installation.root)+$v.context.pluginRoots)
    if([IO.Path]::GetDirectoryName((Assert-LocalPath $Inspection)) -ine $p.directory -or (Get-Digest $Inspection) -cne $InspectionHash){throw 'Exact child inspection required.'}
    $i=Read-Record $Inspection
    if($i.owner -cne $script:ManualPilotOwner -or $i.recordSha256 -cne $Hash -or $ReviewedCurrentHash -cnotmatch '^[a-f0-9]{64}$' -or
       $i.currentIniSha256 -cne $ReviewedCurrentHash -or (Get-Digest $i.savedIni) -cne $ReviewedCurrentHash){throw 'Review must bind the complete current profile bytes.'}
    $ini=Join-Path $v.context.profile 'opencpn.ini'
    $saved=Join-Path $p.directory 'rollback.ini';$intent=Join-Path $p.directory 'rollback-intent.json'
    if(Test-Path $intent) {
      $prior=Read-Record $intent
      if($prior.owner -cne $script:ManualPilotOwner -or $prior.recordSha256 -cne $Hash -or
         $prior.inspectionSha256 -cne $InspectionHash -or $prior.beforeSha256 -cne $ReviewedCurrentHash -or
         (Get-Digest $saved) -cne $prior.afterSha256){throw 'Interrupted rollback proof changed.'}
      $target=[IO.File]::ReadAllBytes($saved);$targetHash=$prior.afterSha256
    }else {
      if($ReviewedCurrentHash -ceq $r.inputSha256) {$target=[IO.File]::ReadAllBytes($i.savedIni)}
      else {
        $null=Assert-ManualPilotProfileDelta (Join-Path $p.directory 'output.ini') $i.savedIni
        $target=Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($i.savedIni))
      }
      $targetHash=Get-CommissioningHash $target
    }
    # Recompute the exact one-byte target even when resuming an interrupted publication.
    $expected=if($ReviewedCurrentHash -ceq $r.inputSha256){[IO.File]::ReadAllBytes($i.savedIni)}else{
      $null=Assert-ManualPilotProfileDelta (Join-Path $p.directory 'output.ini') $i.savedIni
      Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($i.savedIni))
    }
    if((Get-CommissioningHash $expected) -cne $targetHash){throw 'Rollback target differs from reviewed byte-only inverse.'}
    $current=Get-Digest $ini
    if($current -cne $ReviewedCurrentHash -and $current -cne $targetHash){throw 'Current profile changed after inspection/publication.'}
    $tree=$i.profileTree
    if($current -ceq $targetHash) {
      $entry=@($tree.entries|Where-Object {$_.path -ceq 'opencpn.ini'})
      if($entry.Count -ne 1){throw 'Inspected profile tree is ambiguous.'}
      $entry[0].sha256=$targetHash;$entry[0].bytes=$target.Length
    }
    Assert-PreparationTree $tree
    if(-not (Test-Path $intent)) {
      if(Test-Path $saved){throw 'Unjournaled rollback bytes require inspection.'}
      [IO.File]::WriteAllBytes($saved,$target)
      Write-Record $intent @{owner=$script:ManualPilotOwner;recordSha256=$Hash;inspectionSha256=$InspectionHash;beforeSha256=$ReviewedCurrentHash;afterSha256=$targetHash;pluginsRestored=$false}
    }
    if($current -cne $targetHash){Publish-PreparedProfile $ini $saved $ReviewedCurrentHash $targetHash $target.Length $intent}
    Assert-ManualPilotProfile (Read-ProfileForAudit $ini) $false
    $complete=Join-Path $p.directory 'rolled-back.json'
    if(-not (Test-Path $complete)) {
      Write-Record $complete @{owner=$script:ManualPilotOwner;recordSha256=$Hash;profileSha256=$targetHash;pluginsRestored=$false;parentRemainsActive=$true;launchPermission=$false}
    }else {
      $done=Read-Record $complete
      if($done.owner -cne $script:ManualPilotOwner -or $done.recordSha256 -cne $Hash -or $done.profileSha256 -cne $targetHash){throw 'Rollback completion changed.'}
    }
    Remove-Item -LiteralPath $p.active
    return [pscustomobject]@{status='rolled-back';profileSha256=$targetHash;pluginsRestored=$false;parentRemainsActive=$true}
  }finally{if($lock){$lock.Dispose()};$parentLock.Dispose()}
}
function Initialize-ManualPilotDiagnosticsNative {
  if(-not ('OpenNavX.ManualPilotDiagnosticsNative' -as [type])) {
    Add-Type -Path (Join-Path $PSScriptRoot 'ManualPilotDiagnosticsNative.cs')
  }
}
function Get-ManualPilotDiagnosticsMetadata([IO.FileStream]$Stream) {
  return [OpenNavX.ManualPilotDiagnosticsNative]::Inspect($Stream.SafeFileHandle)
}
function Read-ManualPilotDiagnosticsSnapshot([string]$Path) {
  Initialize-ManualPilotDiagnosticsNative
  $path=Assert-LocalPath $Path
  # Win32 FileShare.Read excludes existing/new writers and delete/rename. The
  # producer may skip a publication while held; no replacement or read retry is
  # performed here. Metadata remains tied to the handle, even if a path changes.
  $stream=[IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try {
    $length=$stream.Length
    if($length -le 0 -or $length -gt 4194304){throw 'Diagnostics snapshot size outside bound.'}
    $bytes=New-Object byte[] ([int]$length);$offset=0
    while($offset -lt $bytes.Length) {
      $read=$stream.Read($bytes,$offset,$bytes.Length-$offset)
      if($read -le 0){throw 'Diagnostics snapshot was truncated.'};$offset+=$read
    }
    if($stream.ReadByte() -ne -1){throw 'Diagnostics snapshot grew during read.'}
    $metadata=Get-ManualPilotDiagnosticsMetadata $stream
    Assert-PreparationAttributes $metadata.Attributes
    if($metadata.Length -ne $bytes.Length){throw 'Held diagnostics metadata differs from its exact bytes.'}
    $sha256=Get-CommissioningHash $bytes
    $encoding=New-Object Text.UTF8Encoding($false,$true)
    $data=$encoding.GetString($bytes)|ConvertFrom-Json
    return [pscustomobject]@{data=$data;writtenUtc=$metadata.WrittenUtc;sha256=$sha256;bytes=$bytes.Length}
  }finally{$stream.Dispose()}
}
function Assert-ManualPilotNoActiveRoute($Data,[datetime]$Written,[datetime]$Started,[datetime]$Now) {
  # NoActiveRoute is a real model state, not absent/invalid distance. Its quality
  # is Unavailable and AssessRoute returns before ageing non-Valid states, so
  # assess the original completed-pass timestamp ourselves. UI reads cannot
  # renew it (RouteProgressInput::CheckCurrent); a new file mtime is insufficient.
  $route=$Data.route
  if($Written -lt $Started -or $Written -gt $Now -or ($Now-$Written).TotalMilliseconds -gt 5000 -or
     $Data.publication_clock -cne 'live monotonic clock' -or
     $route.state -cne 'NoActiveRoute' -or $route.id -isnot [string] -or $route.id -cne '' -or
     $route.waypoint -isnot [string] -or $route.waypoint -cne '' -or
     ($route.waypoint_count -isnot [int] -and $route.waypoint_count -isnot [long]) -or $route.waypoint_count -ne 0 -or
     $route.source -cne 'OpenCPN 5.12.4 normal route progress: active range + subsequent stored legs; cross-track error (NM)' -or
     $route.revision_scope -cnotmatch '\ASKAGER session [0-9]+\z') {
    throw 'Fresh explicit normal-progress NoActiveRoute proof required; unavailable route data is not inactivity.'
  }
  $published=[uint64]0;$observed=[uint64]0;$revision=[uint64]0
  foreach($value in @($Data.publication_monotonic_ms,$route.observed_monotonic_ms,$route.revision)) {
    if($value -isnot [string] -or $value -cnotmatch '\A[1-9][0-9]{0,19}\z'){throw 'Valid live route observation timestamps/revision required.'}
  }
  if(-not [uint64]::TryParse($Data.publication_monotonic_ms,[ref]$published) -or
     -not [uint64]::TryParse($route.observed_monotonic_ms,[ref]$observed) -or
     -not [uint64]::TryParse($route.revision,[ref]$revision) -or $observed -gt $published -or
     ([decimal]$published-[decimal]$observed)+[decimal]($Now-$Written).TotalMilliseconds -gt 5000) {
    throw 'Route observation is stale, future-dated or invalid; fresh UI publication cannot renew it.'
  }
}
function Assert-ManualPilotStartupDiagnostics($Data,[string]$Commit,[datetime]$Written,[datetime]$Started,[datetime]$Now) {
  if($Written -lt $Started -or $Written -gt $Now -or ($Now-$Written).TotalSeconds -gt 5 -or
     $Data.build_commit -cne $Commit -or $Data.build_purpose -cne 'INSTALLED PRODUCT' -or
     $Data.data_mode -cne 'OPENCPN selected navigation' -or $Data.xnav_hardware_output_policy -cne 'manual-commissioning' -or
     $Data.xnav_manual_control_contract -isnot [int] -or $Data.xnav_manual_control_contract -ne 1){throw 'Fresh exact manual product diagnostics required.'}
  foreach($value in @($Data.runtime.pilot.enabled,$Data.runtime.pilot.serial_session_enabled,$Data.runtime.pilot.configured_permission,
      $Data.runtime.pilot.simulated,$Data.runtime.pilot.track_capability,$Data.runtime.pilot.wind_capability,$Data.runtime.display.route_creation_active)) {
    if($value -isnot [bool] -or $value){throw 'Default-off manual commissioning diagnostics failed.'}
  }
  if($Data.runtime.replay.active -isnot [bool] -or $Data.runtime.replay.active){throw 'Replay cannot accompany physical manual commissioning.'}
  Assert-ManualPilotNoActiveRoute $Data $Written $Started $Now
}
function Assert-ManualPilotInteractive($V) {
  $actual=Get-ManualPilotEngine;$expected=$V.record.engine
  $hostProcess=Get-Process -Id $PID
  try {
    if($actual.path -ine $expected.path -or $actual.sha256 -cne $expected.sha256 -or
       (Assert-LocalPath $hostProcess.Path) -ine $expected.path -or $hostProcess.SessionId -ne $V.context.session -or
       [Security.Principal.WindowsIdentity]::GetCurrent().User.Value -cne $V.context.sid){throw 'Interactive host is not the bound signed system PowerShell/current desktop.'}
  }finally{$hostProcess.Dispose()}
}
function Get-ManualPilotProcess($V,$Receipt) {
  if($Receipt.owner -cne $script:ManualPilotOwner -or $Receipt.recordSha256 -cne (Get-Digest $V.paths.record) -or
     $Receipt.pid -le 0 -or $Receipt.startedTicks -le 0){throw 'Exact manual launch receipt required.'}
  $process=Get-Process -Id $Receipt.pid
  try {
    $null=$process.get_Handle()
    $native=@(Get-CimInstance Win32_Process -Filter ('ProcessId='+$Receipt.pid))
    if($native.Count -ne 1){throw 'Manual process disappeared.'}
    $owner=Invoke-CimMethod -InputObject $native[0] -MethodName GetOwnerSid
    if($process.HasExited -or $process.StartTime.ToUniversalTime().Ticks -ne $Receipt.startedTicks -or
       $process.SessionId -ne $V.context.session -or $owner.ReturnValue -ne 0 -or $owner.Sid -cne $V.context.sid -or
       (Assert-LocalPath $process.Path) -ine $V.context.installation.executable -or
       (Get-Digest $process.Path) -cne $V.record.candidate.executableSha256){throw 'Manual process identity changed.'}
    return $process
  }catch{$process.Dispose();throw}
}
function Invoke-ManualPilotInteractive($Job) {
  $launch=$Job.action -ceq 'LaunchManualPilot'
  if(-not $launch -and $Job.action -cne 'CloseManualPilot'){throw 'Unknown manual interactive action.'}
  $v=Read-ManualPilot $Job.workspace $Job.manualRecord $Job.manualRecordSha256 -Active -Launching:$launch
  Assert-ManualPilotInteractive $v
  if($Job.executable -ine $v.context.installation.executable -or $Job.executableSha256 -cne $v.record.candidate.executableSha256){throw 'Manual job executable changed.'}
  $parentLock=Open-CommissioningRestoreLock $v.record.parentDirectory
  $lock=$null
  try {
    $lock=Open-CommissioningRestoreLock $v.paths.directory
    if($launch) {
      # Repeat after acquiring the exclusive child lock, immediately before start.
      $v=Read-ManualPilot $Job.workspace $Job.manualRecord $Job.manualRecordSha256 -Active -Launching
      $intent=Join-Path $v.paths.directory 'launch-intent.json'
      Write-Record $intent @{owner=$script:ManualPilotOwner;recordSha256=$Job.manualRecordSha256;createdUtc=[datetime]::UtcNow.ToString('o');action='--xnav';retryAllowed=$false}
      $start=New-Object Diagnostics.ProcessStartInfo
      $start.FileName=$v.context.installation.executable;$start.Arguments='--xnav';$start.UseShellExecute=$false
      $start.WorkingDirectory=$v.context.launchEnvironment.workingDirectory;$start.EnvironmentVariables['PATH']=$v.context.launchEnvironment.path
      # No restart/test/replay state may be inherited from another helper.
      foreach($name in @($start.EnvironmentVariables.Keys)) {
        if($name -match '^(OPENNAV|XNAV|SKAGER)_'){throw 'Unexpected product-control environment at manual launch.'}
      }
      $process=[Diagnostics.Process]::Start($start)
      try {
        $null=$process.get_Handle()
        $receipt=@{owner=$script:ManualPilotOwner;recordSha256=$Job.manualRecordSha256;pid=$process.Id;startedTicks=$process.StartTime.ToUniversalTime().Ticks;
          sid=$v.context.sid;session=$v.context.session;executableSha256=$v.record.candidate.executableSha256;commit=$v.record.candidate.commit}
        # Durable creation identity before window polling; failure never retries.
        Write-Record (Join-Path $v.paths.directory 'launched.json') $receipt
        $verified=Get-ManualPilotProcess $v ([pscustomobject]$receipt);$verified.Dispose()
        $deadline=[datetime]::UtcNow.AddSeconds(45)
        do {Start-Sleep -Milliseconds 250;$process.Refresh()}while(-not $process.HasExited -and -not $process.MainWindowHandle -and [datetime]::UtcNow -lt $deadline)
        if($process.HasExited -or -not $process.MainWindowHandle){throw 'Manual process did not expose a window; inspect, never automatically retry.'}
        $diagnostics=Join-Path $v.context.profile 'opennav-logs/opennav-diagnostics.json'
        $deadline=[datetime]::UtcNow.AddSeconds(15);$verifiedOff=$false
        do {
          try {
            $diagnosticSnapshot=Read-ManualPilotDiagnosticsSnapshot $diagnostics
            Assert-ManualPilotStartupDiagnostics $diagnosticSnapshot.data $receipt.commit $diagnosticSnapshot.writtenUtc $process.StartTime.ToUniversalTime() ([datetime]::UtcNow)
            $verifiedOff=$true
          }catch {if([datetime]::UtcNow -ge $deadline){throw};Start-Sleep -Milliseconds 250}
        }while(-not $verifiedOff)
        Write-Record (Join-Path $v.paths.directory 'startup-off.json') @{owner=$script:ManualPilotOwner;recordSha256=$Job.manualRecordSha256;pid=$process.Id;diagnosticsSha256=$diagnosticSnapshot.sha256;diagnosticsWrittenUtc=$diagnosticSnapshot.writtenUtc.ToString('o');verifiedUtc=[datetime]::UtcNow.ToString('o')}
        return [pscustomobject]@{status='passed';action=$Job.action;pid=$process.Id;startedTicks=$receipt.startedTicks;commit=$receipt.commit;commandsSent=$false;sessionControlVerifiedOff=$true}
      }finally{$process.Dispose()}
    }
    $binding=Get-ManualPilotBinding ((Read-ProfileForAudit (Join-Path $v.context.profile 'opencpn.ini'))['OpenNav/AlphaSettings'])
    if($binding.permission -ceq 'manual'){throw 'Return to display-only in the product before normal child Close.'}
    $receipt=Read-Record (Join-Path $v.paths.directory 'launched.json')
    $process=Get-ManualPilotProcess $v $receipt
    try {
      $intent=Join-Path $v.paths.directory 'close-intent.json'
      Write-Record $intent @{owner=$script:ManualPilotOwner;recordSha256=$Job.manualRecordSha256;pid=$receipt.pid;startedTicks=$receipt.startedTicks;forceAllowed=$false}
      $result=Invoke-ReviewedNormalClose $process $receipt.pid $receipt.startedTicks
      Write-Record (Join-Path $v.paths.directory 'closed.json') @{owner=$script:ManualPilotOwner;recordSha256=$Job.manualRecordSha256;process=$receipt;close=$result}
      return [pscustomobject]@{status='passed';action=$Job.action;pid=$receipt.pid;close=$result;pluginsRestored=$false}
    }finally{$process.Dispose()}
  }finally{if($lock){$lock.Dispose()};$parentLock.Dispose()}
}
function Invoke-ManualPilotJob([string]$Workspace,[string]$Record,[string]$Hash,[bool]$Launch) {
  $v=Read-ManualPilot $Workspace $Record $Hash -Active -Launching:$Launch
  if(-not $Launch){$process=Get-ManualPilotProcess $v (Read-Record (Join-Path $v.paths.directory 'launched.json'));$process.Dispose()}
  $job=[pscustomobject]@{action=$(if($Launch){'LaunchManualPilot'}else{'CloseManualPilot'});executable=$v.context.installation.executable;
    executableSha256=$v.record.candidate.executableSha256;manualRecord=$Record;manualRecordSha256=$Hash}
  return Invoke-InteractiveJob $Workspace $job -TimeoutSeconds 90
}
