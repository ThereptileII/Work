# Disposable files only. Native mode exercises actual profile locks/ACLs/moves.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable contract mode required.'}
if($native -and $env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal){throw 'Disposable native CI or explicit isolated local tests required.'}
if($native -and @(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Never run reconciliation fixtures beside a navigation application.'}
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('commissioning-reconciliation-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory $testRoot
$checks=New-Object 'Collections.Generic.List[string]';$originalBaseline=$script:CommissioningBaseline
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$bad=$false;try{$null=& $Body}catch{$bad=$true};if(-not $bad){throw ('Unsafe reconciliation accepted: '+$Name)};$checks.Add($Name)}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 16|ConvertFrom-Json)}
$encoding=New-Object Text.UTF8Encoding($false,$true)
if(-not $native) {
  # Explicit POSIX adapters do not qualify Win32 identity, metadata or ACLs.
  function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($testRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Escaped disposable root.'};return $p}
  function New-PreparationDirectory($Context,[string]$Purpose){return New-RunDirectory $Context.workspace $Purpose}
  function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid){$null=Assert-LocalPath $Directory;if($Sid -cne 'S-1-5-21-123'){throw 'Wrong fixture account.'}}
  function Get-PreparationAccessAcl([string]$Path){$null=Assert-LocalPath $Path;return 'portable-fixture-acl'}
  function Assert-PreparationAcl([string]$Expected,[string]$Actual,[switch]$AllowDaclAutoInherited){if($Expected -cne $Actual){throw 'Changed ACL.'}}
  function Open-CommissioningProfileLock([string]$Directory){return [IO.File]::Open((Join-Path $testRoot 'profile.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)}
}
# Only actual host identity/process discovery is substituted. The unchanged
# production entrypoint body, profile lock, transaction lock, byte policy,
# backups, publication, original-path moves and baseline reader execute.
function Get-CommissioningContext([string]$Workspace) {
  if($Workspace -ine $script:fixture.current.workspace){throw 'Wrong fixture workspace.'}
  $value=Clone $script:fixture.current;$i=$value.installation
  $i.stateSha256=Get-Digest (Join-Path $i.root 'state.json')
  $i.ownershipSha256=Get-Digest (Join-Path $i.generation 'ownership.json');$i.executableSha256=Get-Digest $i.executable
  return $value
}
function Assert-PreparationClosed([string[]]$Roots){foreach($root in $Roots){if(-not $root.StartsWith($testRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)){throw 'Process guard escaped fixture.'}}}
function NewFixture([bool]$InputOnly=$false) {
  $base=Join-Path $testRoot ([guid]::NewGuid().ToString('N'));$workspace=Join-Path $base 'workspace'
  $parent=Join-Path $workspace 'runs/20261001-000000-read-only-commissioning-aaaaaaaa'
  $profile=Join-Path $base 'profile';$stock=Join-Path $base 'stock';$managed=Join-Path $base 'managed';$installed=Join-Path $base 'installed'
  $oldGeneration=Join-Path $installed ('generations/'+('a'*32));$newGeneration=Join-Path $installed ('generations/'+('b'*32))
  $roots=@($managed,(Join-Path $stock 'plugins'),(Join-Path $oldGeneration 'app/plugins')|Sort-Object -Unique)
  $newRoots=@($managed,(Join-Path $stock 'plugins'),(Join-Path $newGeneration 'app/plugins')|Sort-Object -Unique)
  $null=New-Item -ItemType Directory -Force -Path (@($parent,$profile)+$roots+@((Join-Path $newGeneration 'app/plugins'),(Join-Path $parent 'quarantine')))
  foreach($name in @('stock/opencpn.exe',('installed/generations/'+('a'*32)+'/app/opencpn.exe'),('installed/generations/'+('b'*32)+'/app/opencpn.exe'))) {
    [IO.File]::WriteAllText((Join-Path $base $name),('Inert, never executed: '+$name),$encoding)
  }
  $oldExe=Join-Path $oldGeneration 'app/opencpn.exe';$newExe=Join-Path $newGeneration 'app/opencpn.exe'
  foreach($spec in @(@($oldGeneration,$oldExe,'c'),@($newGeneration,$newExe,'d'))) {
    Write-Record (Join-Path $spec[0] 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=($spec[2]*40);version='0.4.0-beta2.4';managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $spec[1])})}
  }
  $state=Join-Path $installed 'state.json';[IO.File]::WriteAllText($state,'original inert selected state',$encoding)
  $sid=if($native){[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}else{'S-1-5-21-123'}
  $old=[pscustomobject]@{sid=$sid;session=1;localAppData=$base;profile=$profile;application=$stock;managed=$managed;workspace=$workspace;executable=(Join-Path $stock 'opencpn.exe');
    installation=[pscustomobject]@{root=$installed;generation=$oldGeneration;executable=$oldExe;commit=('c'*40);stateSha256=(Get-Digest $state);ownershipSha256=(Get-Digest (Join-Path $oldGeneration 'ownership.json'));executableSha256=(Get-Digest $oldExe)};
    pluginRoots=$roots;launchEnvironment=[pscustomobject]@{workingDirectory=(Join-Path $oldGeneration 'app');path=(Join-Path $oldGeneration 'app')}}
  $text="[Settings]`r`nPersistActiveRoute=0`r`nActiveRoute=`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n[Directories]`r`nChartDir=original`r`n[Settings/GlobalState]`r`nFrameWinX=1024`r`n"
  $text+='#'+(' '*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  $baseline=Join-Path $parent 'baseline.ini';$input=Join-Path $parent 'input-only.ini'
  [IO.File]::WriteAllText($baseline,$text,$encoding);$script:CommissioningBaseline=Get-Digest $baseline
  [IO.File]::WriteAllBytes($input,(Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($baseline))))
  $current=Join-Path $profile 'opencpn.ini';$live=if($InputOnly){[IO.File]::ReadAllText($input)}else{$text}
  [IO.File]::WriteAllText($current,$live.Replace('FrameWinX=1024','FrameWinX=1280'),$encoding)
  [IO.File]::WriteAllText((Join-Path $profile 'navobj.xml'),'<gpx>private current route</gpx>',$encoding)
  $paths=@((Join-Path $managed 'ais-vd_pi.dll'),(Join-Path $managed 'autopilot_route_pi.dll'),(Join-Path $stock 'plugins/rtlsdr_pi.dll'),(Join-Path $oldGeneration 'app/plugins/old_pi.dll'))
  foreach($path in $paths){[IO.File]::WriteAllText($path,('Inert quarantine '+[IO.Path]::GetFileName($path)),$encoding)}
  [IO.File]::WriteAllText((Join-Path $newGeneration 'app/plugins/current_pi.dll'),'inert current generation plugin',$encoding)
  $trees=Get-CommissioningTrees $roots;$inventory=Join-Path $parent 'inventory.json'
  Write-Record $inventory @{context=$old;trees=@($trees);plugins=@(Get-CommissioningCandidates $trees)}
  $reviewPlan=Join-Path $parent 'review-plan.json'
  Write-Record $reviewPlan @{plugins=@($paths|ForEach-Object {@{path=$_;sha256=(Get-Digest $_);decision='quarantine'}})}
  $moves=@();$index=0
  foreach($path in $paths) {
    $backup=Join-Path $parent ('plugin-backup-'+$index+'.dll');$destination=Join-Path $parent ('quarantine/plugin-'+$index+'.dll');$index++
    $hash=Get-Digest $path;Copy-PreparationFile $path $backup $hash (Get-Item $path).Length
    $moves+=@{path=$path;destination=$destination;backup=$backup;sha256=$hash;acl=(Get-PreparationAccessAcl $path)}
    [IO.File]::Move($path,$destination)
  }
  $record=Join-Path $parent 'prepared.json'
  Write-Record $record @{schema=1;owner=$script:CommissioningOwner;status='prepared';context=$old;planSha256=(Get-Digest $reviewPlan);inventorySha256=(Get-Digest $inventory);
    baselineSha256=$script:CommissioningBaseline;inputSha256=(Get-Digest $input);evidence=@();quarantine=$moves}
  $recordHash=Get-Digest $record
  Write-Record (Join-Path $workspace 'commissioning-active.json') @{schema=1;owner=$script:CommissioningOwner;record=$record;recordSha256=$recordHash}
  [IO.File]::WriteAllText($state,'later explicitly user-installed inert selected state',$encoding)
  $new=Clone $old;$new.session=2;$new.pluginRoots=$newRoots;$new.installation.generation=$newGeneration;$new.installation.executable=$newExe;$new.installation.commit='d'*40
  $new.installation.stateSha256=Get-Digest $state;$new.installation.ownershipSha256=Get-Digest (Join-Path $newGeneration 'ownership.json');$new.installation.executableSha256=Get-Digest $newExe
  $new.launchEnvironment.workingDirectory=Join-Path $newGeneration 'app';$new.launchEnvironment.path=$new.launchEnvironment.workingDirectory
  $script:fixture=[pscustomobject]@{current=$new;record=$record;recordHash=$recordHash;ini=$current;parent=$parent;workspace=$workspace;old=$old;moves=$moves;originalCurrentHash=(Get-Digest $current)}
  return $script:fixture
}
$entry=Join-Path $PSScriptRoot 'reconcile-commissioning.ps1'
$body=[IO.File]::ReadAllText($entry).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",'')
$invoke=[scriptblock]::Create($body)
function InspectPlan {
  $f=$script:fixture
  $inspection=(& $invoke -Action Inspect -Workspace $f.workspace -Record $f.record -ExpectedRecordSha256 $f.recordHash)|ConvertFrom-Json
  $observed=Read-Record $inspection.inspection
  $review=Join-Path ([IO.Path]::GetDirectoryName($inspection.inspection)) 'independent-review.json'
  Write-Record $review @{schema=1;owner='OpenNavX.SessionPreservationReview.1';parentPreparedSha256=$f.recordHash;inspectionSha256=$inspection.inspectionSha256;
    beforeSha256=(Get-Digest (Join-Path $f.parent 'input-only.ini'));afterSha256=$observed.currentIniSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');connectionDisposition=$observed.connectionDisposition;
    provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;
    changes=@($observed.diff|ForEach-Object {@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';decision='preserve-current';reason='Explicit inert fixture preservation decision'}})}
  return ((& $invoke -Action Plan -Workspace $f.workspace -Inspection $inspection.inspection -ExpectedInspectionSha256 $inspection.inspectionSha256 -PreservationReview $review -ExpectedReviewSha256 (Get-Digest $review))|ConvertFrom-Json)
}
function Commit($Plan){return ((& $invoke -Action Commit -Workspace $script:fixture.workspace -Plan $Plan.plan -ExpectedPlanSha256 $Plan.planSha256)|ConvertFrom-Json)}
try {
  Pass 'Entrypoint and helper parse before any mutation' {
    foreach($file in @($entry,(Join-Path $PSScriptRoot 'CommissioningReconciliation.ps1'))) {$errors=$null;$tokens=$null;$null=[Management.Automation.Language.Parser]::ParseFile($file,[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}
  }
  Pass 'Already original IO1 user choice is preserved byte-for-byte; four DLLs return only to original paths' {
    $f=NewFixture;$plan=InspectPlan
    if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or @($f.moves|Where-Object {[IO.File]::Exists($_.path)}).Count){throw 'Inspect/Plan changed live state.'}
    $done=Commit $plan
    if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or (Get-Digest $f.record) -cne $f.recordHash -or (Test-Path (Join-Path $f.workspace 'commissioning-active.json'))){throw 'Profile/evidence/marker postcondition failed.'}
    foreach($item in $f.moves){if((Get-Digest $item.path) -cne $item.sha256 -or (Get-Digest $item.backup) -cne $item.sha256){throw 'Original bytes or backup lost.'}}
    $baseline=Read-CommissioningBaseline $f.workspace $done.baselineRecord $done.baselineRecordSha256
    if($baseline.sha256 -cne $f.originalCurrentHash){throw 'Current IO1 choice relabelled or lost.'}
    $f.current.installation.generation='a later unrelated generation'
    $null=Read-CommissioningBaseline $f.workspace $done.baselineRecord $done.baselineRecordSha256
  }
  Pass 'Original temporary IO0 changes only its one known direction byte on explicit restore disposition' {
    $f=NewFixture $true;$plan=InspectPlan;$before=[IO.File]::ReadAllBytes($f.ini);$done=Commit $plan;$after=[IO.File]::ReadAllBytes($f.ini)
    $differences=@(0..($before.Length-1)|Where-Object {$before[$_] -ne $after[$_]})
    if($differences.Count -ne 1 -or $before[$differences[0]] -ne 48 -or $after[$differences[0]] -ne 49){throw 'Wrong recovery mutation.'}
  }
  foreach($index in 0..3) {
    Refuse ('Each occupied original quarantine destination refuses independently: '+$index) {
      $f=NewFixture;[IO.File]::WriteAllText($f.moves[$index].path,'Occupied current user bytes');InspectPlan
    }
  }
  Refuse 'Retired/missing old generation refuses without path recreation' {$f=NewFixture;Remove-Item -LiteralPath $f.old.installation.executable;InspectPlan}
  Refuse 'Unknown current profile delta refuses before a commit intent' {$f=NewFixture;[IO.File]::AppendAllText($f.ini,"[OpenNav]`r`nUnknown=unreviewed`r`n");InspectPlan}
  Refuse 'Connection fields beyond the exact known direction refuse' {$f=NewFixture;$text=[IO.File]::ReadAllText($f.ini).Replace('COM8','COM9');[IO.File]::WriteAllText($f.ini,$text,$encoding);InspectPlan}
  foreach($kind in @('profile','navigation','state','generation-plugin','marker','backup')) {
    Refuse ('Stale plan refuses late '+$kind+' drift') {
      $f=NewFixture;$plan=InspectPlan
      $path=switch($kind){'profile'{$f.ini};'navigation'{Join-Path $f.current.profile 'navobj.xml'};'state'{Join-Path $f.current.installation.root 'state.json'};'generation-plugin'{Join-Path $f.current.installation.generation 'app/plugins/current_pi.dll'};'marker'{Join-Path $f.workspace 'commissioning-active.json'};'backup'{$f.moves[0].backup}}
      [IO.File]::AppendAllText($path,'changed');Commit $plan
    }
  }
  Pass 'Interrupted commit resumes only the same intent; no changed plan or occupied plugin is accepted' {
    $f=NewFixture $true;$plan=InspectPlan
    $originalMove=${function:Publish-PreparedProfile}
    function Publish-PreparedProfile([string]$Original,[string]$SavedCandidate,[string]$OriginalHash,[string]$CandidateHash,[int]$Length,[string]$Journal) {
      & $originalMove $Original $SavedCandidate $OriginalHash $CandidateHash $Length $Journal
      throw 'Inert forced interruption after actual profile publication.'
    }
    try{Refuse 'Controlled interruption retains the active ownership marker' {Commit $plan}}
    finally{Set-Item Function:Publish-PreparedProfile $originalMove}
    if(-not(Test-Path (Join-Path $f.workspace 'commissioning-active.json'))){throw 'Marker cleared by incomplete commit.'}
    $other=Clone $plan;$other.planSha256='0'*64
    Refuse 'Interrupted commit refuses plan substitution' {Commit $other}
    $done=Commit $plan
    if($done.status -cne 'reconciled'){throw 'Exact interrupted target did not resume.'}
  }
  Pass 'Profile exclusion lock prevents concurrent reconciliation' {
    $f=NewFixture;$held=Open-CommissioningProfileLock $f.current.profile
    try {Refuse 'Second profile owner refuses' {InspectPlan}}finally{$held.Dispose()}
  }
  Pass 'Partial original-path DLL restoration resumes from the same per-plugin journals' {
    $f=NewFixture;$plan=InspectPlan;$originalRestore=${function:Restore-ReconciliationPlugin}
    function Restore-ReconciliationPlugin($Item){& $originalRestore $Item;throw 'Inert interruption after the first actual original-path move.'}
    try{Refuse 'DLL move interruption retains ownership and immutable backups' {Commit $plan}}
    finally{Set-Item Function:Restore-ReconciliationPlugin $originalRestore}
    if(@($f.moves|Where-Object {[IO.File]::Exists($_.path)}).Count -ne 1 -or -not(Test-Path (Join-Path $f.workspace 'commissioning-active.json'))){throw 'Partial move did not retain expected custody.'}
    $done=Commit $plan
    if($done.status -cne 'reconciled' -or @($f.moves|Where-Object {[IO.File]::Exists($_.destination)}).Count){throw 'Partial DLL move did not resume exactly.'}
  }
  Refuse 'Expired unstarted plan refuses despite unchanged bytes' {
    $f=NewFixture;$plan=InspectPlan;$selected=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256
    $selected.value.createdUtc=[datetime]::UtcNow.AddHours(-25).ToString('o')
    Assert-ReconciliationPlanLive $selected (Get-CommissioningContext $f.workspace)
  }
  [pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeProfileLockAndAcl=$native;
    identityAndProcessDiscovery='inert fixture adapters';applicationLaunched=$false;boatAccess=$false;physicalOutput=$false}|ConvertTo-Json -Depth 5
} finally {$script:CommissioningBaseline=$originalBaseline;Remove-Item -LiteralPath $testRoot -Recurse -Force}
