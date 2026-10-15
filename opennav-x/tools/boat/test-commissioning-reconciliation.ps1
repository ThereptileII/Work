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
  function Get-Acl {param([string]$LiteralPath);$null=Assert-LocalPath $LiteralPath;return [pscustomobject]@{Sddl='portable-fixture-acl'}}
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
function NewFixture([bool]$InputOnly=$false,[switch]$Retention) {
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
    $managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $spec[1])})
    if($Retention){
      $wmmDirectory=Join-Path $spec[0] 'app/plugins/wmm_pi/data';$null=New-Item -ItemType Directory -Force $wmmDirectory
      foreach($name in (Get-CommissioningWmmResourcePins).Keys){
        $file=Join-Path $wmmDirectory $name;[IO.File]::WriteAllText($file,('Inert source-pinned WMM fixture '+$name),$encoding)
        $managedFiles+=@{path=('app/plugins/wmm_pi/data/'+$name);sha256=(Get-Digest $file)}
      }
    }
    Write-Record (Join-Path $spec[0] 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=($spec[2]*40);version='0.4.0-beta2.4';managedFiles=$managedFiles}
  }
  $state=Join-Path $installed 'state.json';[IO.File]::WriteAllText($state,'original inert selected state',$encoding)
  $sid=if($native){[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}else{'S-1-5-21-123'}
  $old=[pscustomobject]@{sid=$sid;session=1;localAppData=$base;profile=$profile;application=$stock;managed=$managed;workspace=$workspace;executable=(Join-Path $stock 'opencpn.exe');
    installation=[pscustomobject]@{root=$installed;generation=$oldGeneration;executable=$oldExe;commit=('c'*40);stateSha256=(Get-Digest $state);ownershipSha256=(Get-Digest (Join-Path $oldGeneration 'ownership.json'));executableSha256=(Get-Digest $oldExe)};
    pluginRoots=$roots;launchEnvironment=[pscustomobject]@{workingDirectory=(Join-Path $oldGeneration 'app');path=(Join-Path $oldGeneration 'app')}}
  $text="[Settings]`r`nPersistActiveRoute=0`r`nActiveRoute=`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n[Directories]`r`nChartDir=original`r`n[Settings/GlobalState]`r`nFrameWinX=1024`r`n"
  if($Retention){
    # Entirely invented fixture data; never copy an actual vessel profile.
    $alpha='OpenNavXSettings 1\n"battery" ""\n"capacity" "10"\n"consumption" "measured"\n"corridor" "100"\n"current" "unconfigured"\n"display.instruments" "sog,depth"\n"display.rail" "sog,depth"\n"draft" "1"\n"efficiency" ""\n"hotel" ""\n"margin" ""\n"minimum_speed" "1"\n"model_source" ""\n"reserve" "10"\n'
    $text=$text.Replace("ChartDir=original`r`n",("ChartDir=original`r`nWMMDataLocation="+(Get-CommissioningWmmLocation (Join-Path $oldGeneration 'app'))+"`r`n"))
    $text+="[OpenNav]`r`nAlphaSettings=$alpha`r`n[PlugIns/Dashboard]`r`nSumLogNM=10`r`n"
  }
  $text+='#'+(' '*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  $baseline=Join-Path $parent 'baseline.ini';$input=Join-Path $parent 'input-only.ini'
  [IO.File]::WriteAllText($baseline,$text,$encoding);$script:CommissioningBaseline=Get-Digest $baseline
  [IO.File]::WriteAllBytes($input,(Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($baseline))))
  $current=Join-Path $profile 'opencpn.ini';$live=if($InputOnly){[IO.File]::ReadAllText($input)}else{$text}
  $live=$live.Replace('FrameWinX=1024','FrameWinX=1280')
  if($Retention){
    $pilot='"pilot.address" "42"\n"pilot.interface" "COM8"\n"pilot.name" ""\n"pilot.permission" "manual"\n'
    $live=$live.Replace((Get-CommissioningWmmLocation (Join-Path $oldGeneration 'app')),(Get-CommissioningWmmLocation (Join-Path $newGeneration 'app')))
    $live=$live.Replace('"capacity" "10"','"capacity" "20"').Replace('"reserve" "10"','"reserve" "20"').Replace('"draft" "1"','"draft" "2"').Replace('"display.rail" "sog,depth"','"display.rail" "depth,sog"').Replace('"reserve" "20"\n',('"reserve" "20"\n'+$pilot)).Replace('SumLogNM=10','SumLogNM=12')
    $live+="[OpenNav/OnlineAIS/v1]`r`nRadiusNm=50`r`n[OpenNav/Weather/v1]`r`nEnabled=1`r`n"
  }
  [IO.File]::WriteAllText($current,$live,$encoding)
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
function InspectPlan([switch]$Retention,[scriptblock]$ChangeReview=$null) {
  $f=$script:fixture
  $inspection=(& $invoke -Action Inspect -Workspace $f.workspace -Record $f.record -ExpectedRecordSha256 $f.recordHash)|ConvertFrom-Json
  $observed=Read-Record $inspection.inspection
  $review=Join-Path ([IO.Path]::GetDirectoryName($inspection.inspection)) 'independent-review.json'
  Write-Record $review @{schema=1;owner='OpenNavX.SessionPreservationReview.1';parentPreparedSha256=$f.recordHash;inspectionSha256=$inspection.inspectionSha256;
    beforeSha256=(Get-Digest (Join-Path $f.parent 'input-only.ini'));afterSha256=$observed.currentIniSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');connectionDisposition=$observed.connectionDisposition;
    provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;
    changes=@($observed.diff|ForEach-Object {@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';decision='preserve-current';reason='Explicit inert fixture preservation decision'}})}
  if($Retention){
    $value=Read-Record $review;$value.schema=2;$value.owner='OpenNavX.ColdReconciliationRetentionReview.2'
    $value|Add-Member -NotePropertyName mode -NotePropertyValue 'retain-existing-original-direction-no-grants'
    $ack=[ordered]@{};foreach($name in @('retainExistingBytes','originUnverified','pilotIdentityUnverified','savedPermissionIsNotAuthorization','noProfilePublication','noApplicationLaunch','noCommands','freshCommissioningRequired')){$ack[$name]=$true}
    $value|Add-Member -NotePropertyName retentionAcknowledgments -NotePropertyValue ([pscustomobject]$ack)
    $proof=Read-ReconciliationParent $f.workspace $f.record $f.recordHash
    $value|Add-Member -NotePropertyName wmmRetention -NotePropertyValue (Get-ReconciliationWmmRetention $proof $observed.currentContext)
    if($ChangeReview){& $ChangeReview $value}
    [IO.File]::WriteAllText($review,($value|ConvertTo-Json -Depth 16),$encoding)
  }
  return ((& $invoke -Action Plan -Workspace $f.workspace -Inspection $inspection.inspection -ExpectedInspectionSha256 $inspection.inspectionSha256 -PreservationReview $review -ExpectedReviewSha256 (Get-Digest $review))|ConvertFrom-Json)
}
function Commit($Plan){return ((& $invoke -Action Commit -Workspace $script:fixture.workspace -Plan $Plan.plan -ExpectedPlanSha256 $Plan.planSha256)|ConvertFrom-Json)}
function Test-FreshPassiveCommissioning($F,$Done) {
  $body=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'commission-read-only.ps1')).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",'')
  $commission=[scriptblock]::Create($body)
  $choice=@{Workspace=$F.workspace;BaselineRecord=$Done.baselineRecord;ExpectedBaselineSha256=$Done.baselineRecordSha256}
  $inventoryResult=(& $commission -Action Inventory @choice)|ConvertFrom-Json
  $inventory=Read-Record $inventoryResult.record;$directory=[IO.Path]::GetDirectoryName($inventoryResult.record)
  $source=Join-Path $directory 'source.txt';[IO.File]::WriteAllText($source,'Explicit invented source review. No product or vessel acceptance.',$encoding)
  $decisions=@($inventory.plugins|ForEach-Object {@{path=$_.path;sha256=$_.sha256;decision='quarantine';reason='Unknown inert fixture plugin';sourceBoundary='Inert fixture constructor/helper boundary';sourceRevision=$null;startupAndIdleReadOnly=$false;evidencePath=$source;evidenceSha256=(Get-Digest $source)}})
  $passivePath=Join-Path $directory 'passive-source.json'
  $inputHash=Get-CommissioningHash (Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($F.ini)))
  $passive=@{schema=1;owner='OpenNavX.ReadOnlyCommissioning.PassiveReview.1';scope='passive-startup-idle-shutdown-no-commands';reviewedUtc=[datetime]::UtcNow.ToString('o');
    sourceRevision=$inventory.context.installation.commit;executableSha256=$inventory.context.installation.executableSha256;baselineSha256=$F.originalCurrentHash;inputSha256=$inputHash;inventorySha256=$inventoryResult.recordSha256;pluginReviewSha256=(Get-CommissioningPluginReviewHash $decisions);
    transport='COM8-serial-NMEA2000-input-only;gateway-management-traffic-allowed';
    boundaries=@(@('core-generic-send-call-sites','retained-plugins-helpers-adjacent-libraries','skager-pilot-sink','gateway-initialization','startup-idle-shutdown-route-state')|ForEach-Object {@{name=$_;conclusion='no-actuator-command-reachable-in-passive-scope';sourceReferences='Invented fixture source only; no actual product acceptance';reason='All executable files in this fixture are inert text, never executed'}})}
  Write-Record $passivePath $passive
  $planPath=Join-Path $directory 'plan.json'
  $plan=@{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';reviewedUtc=[datetime]::UtcNow.ToString('o');inventoryPath=$inventoryResult.record;inventorySha256=$inventoryResult.recordSha256;plugins=$decisions;
    passiveLaunchReview=@{evidencePath=$passivePath;evidenceSha256=(Get-Digest $passivePath)}}
  Write-Record $planPath $plan
  $prepare=@{Action='Prepare';Plan=$planPath;ExpectedPlanSha256=(Get-Digest $planPath)}
  $backup=[IO.File]::ReadAllBytes($passivePath)
  try {
    $bad=Clone $passive;$bad.sourceRevision='f'*40;$bad.inputSha256='e'*64
    [IO.File]::WriteAllText($passivePath,($bad|ConvertTo-Json -Depth 16),$encoding)
    $message=$null
    try{Assert-CommissioningPassiveReview $passivePath (Get-Digest $passivePath) $inventory.context $F.originalCurrentHash $inputHash $inventoryResult.recordSha256 $decisions -Fresh}
    catch{$message=$_.Exception.Message}
    if($message -cne 'Passive source evidence must bind this exact build, profile, inventory and transport scope. Mismatched fields: sourceRevision, inputSha256'){
      throw 'Passive mismatch diagnostics must expose only the fixed mismatched field names.'
    }
    $checks.Add('Passive mismatch diagnostics retain all comparisons without exposing hashes, values or paths')
  }finally{[IO.File]::WriteAllBytes($passivePath,$backup)}
  try {
    foreach($kind in @('source','executable','baseline','input','inventory','plugin-decisions','transport','scope','expired','future','boundary','claim-only','extra-authority')){
      $changed=Clone $passive
      switch($kind){
        'source'{$changed.sourceRevision='0'*40};'executable'{$changed.executableSha256='0'*64};'baseline'{$changed.baselineSha256='0'*64};'input'{$changed.inputSha256='0'*64};'inventory'{$changed.inventorySha256='0'*64};'plugin-decisions'{$changed.pluginReviewSha256='0'*64}
        'transport'{$changed.transport='IO0-guarantees-all-writers-blocked'};'scope'{$changed.scope='manual-control'};'expired'{$changed.reviewedUtc=[datetime]::UtcNow.AddHours(-25).ToString('o')}
        'future'{$changed.reviewedUtc=[datetime]::UtcNow.AddHours(1).ToString('o')};'boundary'{$changed.boundaries[0].sourceReferences=''};'claim-only'{$changed=[pscustomobject]@{startupAndIdleReadOnly=$true}}
        'extra-authority'{$changed|Add-Member -NotePropertyName commandsAuthorized -NotePropertyValue $true}
      }
      [IO.File]::WriteAllText($passivePath,($changed|ConvertTo-Json -Depth 16),$encoding)
      $plan.passiveLaunchReview.evidenceSha256=Get-Digest $passivePath
      [IO.File]::WriteAllText($planPath,($plan|ConvertTo-Json -Depth 16),$encoding);$prepare.ExpectedPlanSha256=Get-Digest $planPath
      Refuse ('Fresh Prepare refuses unsupported passive evidence: '+$kind) {& $commission @choice @prepare}
    }
  }finally{[IO.File]::WriteAllBytes($passivePath,$backup)}
  $plan.passiveLaunchReview.evidenceSha256=Get-Digest $passivePath
  $savedDecision=Clone $plan.plugins[0]
  try {
    $plan.plugins[0].decision='retain';$plan.plugins[0].sourceRevision='1'*40;$plan.plugins[0].startupAndIdleReadOnly=$true
    [IO.File]::WriteAllText($planPath,($plan|ConvertTo-Json -Depth 16),$encoding);$prepare.ExpectedPlanSha256=Get-Digest $planPath
    Refuse 'Changing quarantine to a separately valid retained plugin decision requires a new passive closure' {& $commission @choice @prepare}
  }finally{$plan.plugins[0]=$savedDecision}
  [IO.File]::WriteAllText($planPath,($plan|ConvertTo-Json -Depth 16),$encoding);$prepare.ExpectedPlanSha256=Get-Digest $planPath
  $preparedResult=(& $commission @choice @prepare)|ConvertFrom-Json
  $prepared=Read-Record $preparedResult.record;Assert-CommissioningUnqualifiedClaims $prepared
  $preparedDir=[IO.Path]::GetDirectoryName($preparedResult.record)
  $null=Get-PreparedCommissioningBaseline $prepared $preparedDir $F.workspace
  foreach($kind in @('missing','true','string','commands','proof')){
    $changed=Clone $prepared
    switch($kind){'missing'{$changed.PSObject.Properties.Remove('pilotBindingQualified')};'true'{$changed.pilotBindingQualified=$true};'string'{$changed.pilotBindingQualified='false'};'commands'{$changed.commandsAuthorized=$true};'proof'{$changed.passiveReviewSha256='0'*64}}
    Refuse ('Prepared lineage refuses qualification/evidence substitution: '+$kind) {Get-PreparedCommissioningBaseline $changed $preparedDir $F.workspace}
  }
  $applied=(& $commission -Action Apply -Workspace $F.workspace -Record $preparedResult.record -ExpectedRecordSha256 $preparedResult.recordSha256)|ConvertFrom-Json
  $verify=[scriptblock]::Create([IO.File]::ReadAllText((Join-Path $PSScriptRoot 'verify-commissioning-launch.ps1')).Replace(". (Join-Path `$PSScriptRoot 'Commissioning.ps1')",''))
  $audit=[pscustomobject]@{profileIniSha256=$applied.profileSha256;buildCommit=$F.current.installation.commit;reviewedUtc=[datetime]::UtcNow.ToString('o');passiveReviewSha256=$prepared.passiveReviewSha256;
    commissioning=[pscustomobject]@{record=$preparedResult.record;recordSha256=$preparedResult.recordSha256;appliedSha256=$applied.verificationSha256}}
  $installed=[pscustomobject]@{executable=$F.current.installation.executable;ownership=[pscustomobject]@{commit=$F.current.installation.commit}}
  $null=& $verify -Workspace $F.workspace -Audit $audit -Installed $installed
  $checks.Add('Fresh Inventory/Prepare/Apply/launch verification accepts passive scope without physical pilot qualification; no executable is launched')
  $appliedBytes=[IO.File]::ReadAllBytes($applied.verification)
  try {
    foreach($kind in @('missing','true','commands')){
      $changed=Read-Record $applied.verification
      switch($kind){'missing'{$changed.PSObject.Properties.Remove('pilotBindingQualified')};'true'{$changed.pilotBindingQualified=$true};'commands'{$changed.commandsAuthorized=$true}}
      [IO.File]::WriteAllText($applied.verification,($changed|ConvertTo-Json -Depth 16),$encoding)
      $audit.commissioning.appliedSha256=Get-Digest $applied.verification
      Refuse ('Launch refuses altered applied authority even with its new exact hash: '+$kind) {& $verify -Workspace $F.workspace -Audit $audit -Installed $installed}
      [IO.File]::WriteAllBytes($applied.verification,$appliedBytes)
    }
  }finally{[IO.File]::WriteAllBytes($applied.verification,$appliedBytes);$audit.commissioning.appliedSha256=Get-Digest $applied.verification}
  $audit.passiveReviewSha256='0'*64
  Refuse 'Independent launch audit must bind the exact passive source evidence' {& $verify -Workspace $F.workspace -Audit $audit -Installed $installed}
  $audit.passiveReviewSha256=$prepared.passiveReviewSha256
  $saved=[IO.File]::ReadAllBytes((Join-Path $preparedDir 'passive-review.json'))
  try{[IO.File]::AppendAllText((Join-Path $preparedDir 'passive-review.json'),'tampered');Refuse 'Immutable copied source evidence cannot drift after preparation' {& $verify -Workspace $F.workspace -Audit $audit -Installed $installed}}
  finally{[IO.File]::WriteAllBytes((Join-Path $preparedDir 'passive-review.json'),$saved)}
  # A later ordinary reconciliation may preserve unrelated display changes;
  # it must carry forward the unqualified ancestor even without a v2 review.
  $F.record=$preparedResult.record;$F.recordHash=$preparedResult.recordSha256;$F.parent=$preparedDir;$F.old=$prepared.context;$F.moves=$prepared.quarantine
  $generation=Join-Path $F.current.installation.root ('generations/'+('e'*32))
  $null=New-Item -ItemType Directory -Force (Join-Path $generation 'app/plugins')
  $exe=Join-Path $generation 'app/opencpn.exe';[IO.File]::WriteAllText($exe,'Inert later generation, never launched',$encoding)
  Write-Record (Join-Path $generation 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=('f'*40);managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $exe)})}
  [IO.File]::AppendAllText((Join-Path $F.current.installation.root 'state.json'),'later inert selection')
  $F.current.installation.generation=$generation;$F.current.installation.executable=$exe;$F.current.installation.commit='f'*40
  $F.current.pluginRoots=@($F.current.managed,(Join-Path $F.current.application 'plugins'),(Join-Path $generation 'app/plugins')|Sort-Object -Unique)
  $F.current.launchEnvironment.workingDirectory=Join-Path $generation 'app';$F.current.launchEnvironment.path=$F.current.launchEnvironment.workingDirectory
  $text=[IO.File]::ReadAllText($F.ini).Replace('FrameWinX=1280','FrameWinX=1440');[IO.File]::WriteAllText($F.ini,$text,$encoding)
  $ordinary=InspectPlan;$completed=Commit $ordinary
  $inherited=Read-CommissioningBaseline $F.workspace $completed.baselineRecord $completed.baselineRecordSha256
  if(-not(Test-CommissioningUnqualifiedPilot $inherited)){throw 'Ordinary reconciliation erased inherited unqualified pilot state.'}
  $checks.Add('Later ordinary v1 reconciliation retains ancestor pilot restriction without relabelling its review as v2')
}
try {
  Pass 'Plugin review digest is stable across raw/JSON records and permutations, with every reviewed field protected' {
    $raw=@(@{path='C:\Synthetic\one_pi.dll';sha256=('a'*64);decision='quarantine';sourceRevision=$null;sourceBoundary='Synthetic source boundary';startupAndIdleReadOnly=$false;evidenceSha256=('b'*64)},
      @{path='C:\Synthetic\two_pi.dll';sha256=('c'*64);decision='quarantine';sourceRevision=$null;sourceBoundary='Synthetic source boundary';startupAndIdleReadOnly=$false;evidenceSha256=('d'*64)})
    $reference=Get-CommissioningPluginReviewHash $raw
    $parsed=Clone ([pscustomobject]@{plugins=$raw})
    foreach($value in @($raw,$parsed.plugins,@($raw[1],$raw[0]),@($parsed.plugins[1],$parsed.plugins[0]))){
      if((Get-CommissioningPluginReviewHash $value) -cne $reference){throw 'Plugin review digest depends on source representation or array order.'}
    }
    foreach($field in @('path','sha256','decision','sourceRevision','sourceBoundary','startupAndIdleReadOnly','evidenceSha256')){
      $changed=Clone ([pscustomobject]@{plugins=$raw})
      if($field -ceq 'startupAndIdleReadOnly'){$changed.plugins[0].$field=$true}
      elseif($field -ceq 'sourceRevision'){$changed.plugins[0].$field='1'*40}
      else{$changed.plugins[0].$field=[string]$changed.plugins[0].$field+'changed'}
      if((Get-CommissioningPluginReviewHash $changed.plugins) -ceq $reference){throw ('Reviewed field omitted from canonical digest: '+$field)}
    }
  }
  Pass 'Entrypoint and helper parse before any mutation' {
    foreach($file in @($entry,(Join-Path $PSScriptRoot 'CommissioningReconciliation.ps1'))) {$errors=$null;$tokens=$null;$null=[Management.Automation.Language.Parser]::ParseFile($file,[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}
  }
  Pass 'Journal JSON retains every native creation-time tick across the supported PowerShell engines' {
    $date=[datetime]::SpecifyKind([datetime]'2020-02-03T04:05:06',[DateTimeKind]::Utc).AddTicks(1234567)
    $acl='O:SYG:SYD:P(A;;FA;;;SY)'
    $before=[pscustomobject]@{attributes=32;creationUtc=$date;acl=$acl;auditAcl=$acl}
    $journal=Join-Path $testRoot 'metadata-roundtrip.json';Write-Record $journal (ConvertTo-ReconciliationMetadata $before)
    $raw=Read-Record $journal;$after=ConvertFrom-ReconciliationMetadata $raw
    if($raw.creationUtcTicks -isnot [string] -or $raw.PSObject.Properties['creationUtc'] -or
       $after.creationUtc.Ticks -ne $date.Ticks -or $after.creationUtc.Kind -ne [DateTimeKind]::Utc -or
       $after.auditAcl -cne $before.auditAcl){throw 'Journal rounded or relabelled exact creation metadata.'}
    Assert-PreparationNativeMetadata $before $after
    foreach($invalid in @([long]$date.Ticks,('0'+$raw.creationUtcTicks),'1.5','3155378976000000000')){
      $changed=Clone $raw;$changed.creationUtcTicks=$invalid
      Refuse 'Noncanonical, numeric, fractional or out-of-range creation ticks refuse' {ConvertFrom-ReconciliationMetadata $changed}
    }
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
    $originalMove=${function:Publish-ReconciliationProfile}
    function Publish-ReconciliationProfile($Plan,$Current) {
      & $originalMove $Plan $Current
      throw 'Inert forced interruption after actual profile publication.'
    }
    try{Refuse 'Controlled interruption retains the active ownership marker' {Commit $plan}}
    finally{Set-Item Function:Publish-ReconciliationProfile $originalMove}
    if(-not(Test-Path (Join-Path $f.workspace 'commissioning-active.json'))){throw 'Marker cleared by incomplete commit.'}
    $other=Clone $plan;$other.planSha256='0'*64
    Refuse 'Interrupted commit refuses plan substitution' {Commit $other}
    $done=Commit $plan
    if($done.status -cne 'reconciled'){throw 'Exact interrupted target did not resume.'}
  }
  Pass 'Interruption before atomic publication retains exactly one journalled stage and resumes it' {
    $f=NewFixture $true;$plan=InspectPlan;$originalPublish=${function:Publish-ReconciliationStage}
    function Publish-ReconciliationStage([string]$Stage,[string]$Original){throw 'Inert interruption immediately before atomic publication.'}
    try{Refuse 'Staging interruption preserves the original live profile and active marker' {Commit $plan}}
    finally{Set-Item Function:Publish-ReconciliationStage $originalPublish}
    $selected=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256;$stage=Read-ReconciliationProfileStage $selected
    if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or -not(Test-Path (Join-Path $f.workspace 'commissioning-active.json')) -or
       (Get-Digest $stage.stage) -cne $selected.value.baselineSha256 -or
       @(Get-ChildItem -LiteralPath $f.current.profile -Filter '*.partial' -Force).Count -ne 1){throw 'Exact staged recovery boundary not retained.'}
    $done=Commit $plan
    if($done.status -cne 'reconciled' -or (Test-Path -LiteralPath $stage.stage) -or (Get-Digest $f.ini) -cne $selected.value.baselineSha256){throw 'Same journalled stage did not resume and publish exactly.'}
  }
  $stageKinds=@('stage-bytes','stage-path','unowned-extra');if($native){$stageKinds+='stage-metadata'}
  foreach($kind in $stageKinds){
    Pass ('Interrupted staging refuses '+$kind+' without changing live bytes or ownership') {
      $f=NewFixture $true;$plan=InspectPlan;$originalPublish=${function:Publish-ReconciliationStage}
      function Publish-ReconciliationStage([string]$Stage,[string]$Original){throw 'Inert interruption before atomic publication.'}
      try{Refuse ('Retain staged fixture for '+$kind) {Commit $plan}}
      finally{Set-Item Function:Publish-ReconciliationStage $originalPublish}
      $selected=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256;$stage=Read-ReconciliationProfileStage $selected
      switch($kind){
        'stage-bytes'{[IO.File]::AppendAllText($stage.stage,'tampered')}
        'stage-metadata'{[IO.File]::SetCreationTimeUtc($stage.stage,[IO.File]::GetCreationTimeUtc($stage.stage).AddMinutes(1))}
        'stage-path'{
          $journal=Join-Path $selected.directory 'profile-stage.json';$value=Read-Record $journal
          $value.stage=$f.ini+'.opennav-recovery-'+('f'*32)+'.partial'
          [IO.File]::WriteAllText($journal,($value|ConvertTo-Json -Depth 16),$encoding)
        }
        'unowned-extra'{[IO.File]::WriteAllText(($f.ini+'.opennav-recovery-'+('e'*32)+'.partial'),'Unowned recovery-looking bytes',$encoding)}
      }
      $before=Get-PreparationTree $f.current.profile
      Refuse ('Same plan cannot accept '+$kind) {Commit $plan}
      Assert-PreparationTree $before
      if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or -not(Test-Path (Join-Path $f.workspace 'commissioning-active.json')) -or
         @($f.moves|Where-Object {[IO.File]::Exists($_.path)}).Count){throw 'Refused staging recovery changed live state.'}
    }
  }
  foreach($inputOnly in @($false,$true)){
    Pass ('Interruption immediately after marker removal returns the same completed receipt without mutation; input-only='+$inputOnly) {
      $f=NewFixture $inputOnly;$plan=InspectPlan;$originalRemove=${function:Remove-ReconciliationActive}
      function Remove-ReconciliationActive($Plan){& $originalRemove $Plan;throw 'Inert interruption immediately after active marker removal.'}
      try{Refuse 'Marker-removal interruption occurs only after durable completion' {Commit $plan}}
      finally{Set-Item Function:Remove-ReconciliationActive $originalRemove}
      if(Test-Path (Join-Path $f.workspace 'commissioning-active.json')){throw 'Fault did not reach the marker-removed boundary.'}
      $selected=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256
      $expected=Get-ReconciliationReceipt $selected|ConvertTo-Json -Compress
      $before=Get-PreparationTree ([IO.Path]::GetDirectoryName($f.workspace))
      foreach($attempt in 1..2){
        $done=Commit $plan
        if(($done|ConvertTo-Json -Compress) -cne $expected){throw 'Repeated commit did not return the existing exact receipt.'}
        Assert-PreparationTree $before
      }
      if(-not $inputOnly -and (Get-Digest $f.ini) -cne $f.originalCurrentHash){throw 'Original IO1 current profile bytes changed.'}
    }
  }
  foreach($kind in @('wrong-plan','profile','navigation','context','unknown-marker','missing-completion')){
    Pass ('Completed retry refuses '+$kind+' without changing any retained bytes') {
      $f=NewFixture;$plan=InspectPlan;$done=Commit $plan;$selected=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256
      switch($kind){
        'wrong-plan'{
          $other=Clone (Read-Record $plan.plan);$other.createdUtc=[datetime]::UtcNow.AddSeconds(1).ToString('o')
          [IO.File]::WriteAllText($plan.plan,($other|ConvertTo-Json -Depth 16),$encoding)
          $plan.planSha256=Get-Digest $plan.plan
          $null=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256
        }
        'profile'{[IO.File]::AppendAllText($f.ini,'changed')}
        'navigation'{[IO.File]::AppendAllText((Join-Path $f.current.profile 'navobj.xml'),'changed')}
        'context'{$f.current.session++}
        'unknown-marker'{Write-Record (Join-Path $f.workspace 'commissioning-active.json') @{schema=1;owner='Unknown';record=$f.record;recordSha256=$f.recordHash}}
        'missing-completion'{Remove-Item -LiteralPath $done.completion}
      }
      $before=Get-PreparationTree ([IO.Path]::GetDirectoryName($f.workspace))
      Refuse ('Completed plan cannot accept '+$kind) {Commit $plan}
      Assert-PreparationTree $before
    }
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
  # Resource content is inert. Substitute only the source-pin oracle; the real
  # ownership, exact-tree, path, review, transaction and digest readers execute.
  $savedWmmPins=${function:Get-CommissioningWmmResourcePins}
  function Get-CommissioningWmmResourcePins {
    $pins=[ordered]@{}
    foreach($name in @('WMM.COF','wmm_live.svg','wmm_pi.svg')){$pins[$name]=Get-CommissioningHash ($encoding.GetBytes('Inert source-pinned WMM fixture '+$name))}
    return $pins
  }
  try {
    Pass 'Version 1 review still refuses pilot identity, cross-generation resources and new product preferences' {
      $f=NewFixture -Retention
      Refuse 'An ordinary preservation review is not a retention acknowledgment' {InspectPlan}
    }
    Pass 'Explicit v2 retention closes the old transaction with whole current bytes and no pilot qualification' {
      $f=NewFixture -Retention;$plan=InspectPlan -Retention;$selected=Read-ReconciliationPlan $f.workspace $plan.plan $plan.planSha256
      $oldBytes=[IO.File]::ReadAllBytes($f.ini);$done=Commit $plan
      if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or (Test-Path (Join-Path $selected.directory 'profile-stage.json')) -or
         (Test-Path (Join-Path $f.workspace 'commissioning-active.json'))){throw 'Retention changed profile bytes, staged publication or retained completed ownership.'}
      $value=Read-Record $done.baselineRecord;Assert-ReconciliationRetentionClaims $value
      $baseline=Read-CommissioningBaseline $f.workspace $done.baselineRecord $done.baselineRecordSha256
      if($baseline.pilotBindingQualified -isnot [bool] -or $baseline.pilotBindingQualified -or $baseline.sha256 -cne $f.originalCurrentHash){throw 'Retained baseline claims pilot qualification or different bytes.'}
      foreach($item in $f.moves){if((Get-Digest $item.path) -cne $item.sha256 -or (Get-Digest $item.backup) -cne $item.sha256){throw 'Retention changed original plugin/backups.'}}
      $before=Get-PreparationTree ([IO.Path]::GetDirectoryName($f.workspace));$again=Commit $plan
      if(($again|ConvertTo-Json -Compress) -cne ($done|ConvertTo-Json -Compress)){throw 'Retention retry changed its existing receipt.'}
      Assert-PreparationTree $before
      # Historical lineage reads frozen ownership/source pins, not today's
      # selected generation or a re-created historical resource directory.
      [IO.Directory]::Move($f.old.installation.generation,($f.old.installation.generation+'-retired'))
      $historical=Read-CommissioningBaseline $f.workspace $done.baselineRecord $done.baselineRecordSha256
      if($historical.pilotBindingQualified){throw 'Generation retirement erased unqualified provenance.'}
    }
    Pass 'Retained baseline is consumable by fresh passive commissioning with durable no-command lineage' {
      $f=NewFixture -Retention;$plan=InspectPlan -Retention;$done=Commit $plan
      Test-FreshPassiveCommissioning $f $done
    }
    foreach($name in @('retainExistingBytes','originUnverified','pilotIdentityUnverified','savedPermissionIsNotAuthorization','noProfilePublication','noApplicationLaunch','noCommands','freshCommissioningRequired')){
      Pass ('Every explicit retention acknowledgment is required: '+$name) {
        $f=NewFixture -Retention
        Refuse 'A false risk acknowledgment cannot authorize retention' {InspectPlan -Retention -ChangeReview {param($v);$v.retentionAcknowledgments.$name=$false}}
        if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or (Test-Path (Join-Path $f.parent 'restore-reconciliation-intent.json'))){throw 'Refused acknowledgment changed current state.'}
      }
    }
    Pass 'Permission-bearing retention refuses temporary input-only disposition without publication' {
      $f=NewFixture $true -Retention
      Refuse 'V2 cannot grant a connection direction or publish settings' {InspectPlan -Retention}
      if((Get-Digest $f.ini) -cne $f.originalCurrentHash){throw 'Rejected input-only retention changed live bytes.'}
    }
    Pass 'Known pilot field parser refuses malformed and unsupported identity or opaque settings' {
      $f=NewFixture -Retention;$alpha=(Read-ProfileForAudit $f.ini)['OpenNav/AlphaSettings']
      $null=Read-ReconciliationPilotSettings $alpha
      foreach($invalid in @(
        $alpha.Replace('"pilot.address" "42"','"pilot.address" "254"'),
        $alpha.Replace('"pilot.interface" "COM8"','"pilot.interface" "COM9"'),
        $alpha.Replace('"pilot.name" ""','"pilot.name" "0000000000000000"'),
        $alpha.Replace('"pilot.permission" "manual"','"pilot.permission" "unknown"'),
        ($alpha+'"pilot.address" "42"\n'),($alpha+'"pilot.extra" "x"\n'),
        ($alpha+'"source.heading" "x"\n'),($alpha+'"curve" "x"\n'),
        $alpha.Replace('"pilot.interface" "COM8"','"pilot.interface" "CO\\M8"'))){
        Refuse 'An exact per-key review cannot override pilot encoding and known-field bounds' {Read-ReconciliationPilotSettings $invalid}
      }
      $old=(Read-ProfileForAudit (Join-Path $f.parent 'input-only.ini'))['OpenNav/AlphaSettings']
      Refuse 'A changed calibrated/current-source scalar is not a supported setup edit' {Assert-ReconciliationAlphaRetention $old ($alpha.Replace('"current" "unconfigured"','"current" "charge"'))}
    }
    foreach($kind in @('radius','weather','counter','unknown-key')){
      Pass ('Exact v2 review cannot override source-backed scalar/key policy: '+$kind) {
        $f=NewFixture -Retention;$text=[IO.File]::ReadAllText($f.ini)
        switch($kind){
          'radius'{$text=$text.Replace('RadiusNm=50','RadiusNm=201')}
          'weather'{$text=$text.Replace('Enabled=1','Enabled=true')}
          'counter'{$text=$text.Replace('SumLogNM=12','SumLogNM=9')}
          'unknown-key'{$text+="[OpenNav/Unknown]`r`nOutput=1`r`n"}
        }
        [IO.File]::WriteAllText($f.ini,$text,$encoding);$before=Get-Digest $f.ini
        Refuse 'Unsupported existing state remains preserved and unaccepted' {InspectPlan -Retention}
        if((Get-Digest $f.ini) -cne $before){throw 'Refused scalar policy changed existing settings.'}
      }
    }
    foreach($kind in @('path','pin','identity','missing-file')){
      Pass ('WMM retention requires both exact owned generations and live source bytes: '+$kind) {
        $f=NewFixture -Retention
        if($kind -ceq 'missing-file'){
          $plan=InspectPlan -Retention
          Remove-Item -LiteralPath (Join-Path $f.current.installation.generation 'app/plugins/wmm_pi/data/WMM.COF')
          Refuse 'No commit after a pinned current resource disappears' {Commit $plan}
        }else{
          Refuse 'No invented cross-generation resource proof' {InspectPlan -Retention -ChangeReview {
            param($v)
            switch($kind){'path'{$v.wmmRetention.current.location+='other'};'pin'{$v.wmmRetention.old.resources[0].sha256='0'*64};'identity'{$v.wmmRetention.current.commit='0'*40}}
          }}
        }
        if((Get-Digest $f.ini) -cne $f.originalCurrentHash -or -not(Test-Path (Join-Path $f.workspace 'commissioning-active.json'))){throw 'Refused WMM evidence changed profile/ownership.'}
      }
    }
  } finally {Set-Item Function:Get-CommissioningWmmResourcePins $savedWmmPins}
  [pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();nativeProfileLockAndAcl=$native;
    identityAndProcessDiscovery='inert fixture adapters';applicationLaunched=$false;boatAccess=$false;physicalOutput=$false}|ConvertTo-Json -Depth 5
} finally {$script:CommissioningBaseline=$originalBaseline;Remove-Item -LiteralPath $testRoot -Recurse -Force}
