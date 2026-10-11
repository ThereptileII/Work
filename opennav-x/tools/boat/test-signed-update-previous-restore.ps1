# Actual commissioning engine + durable live store on disposable files. The
# import adapters substitute only Windows identity/ACL/mutex/handle boundaries;
# neither Restore, preservation lineage nor journal reducers are mocked.
[CmdletBinding()]
param([switch]$PortableContracts,[string]$StoreSource=(Join-Path $PSScriptRoot 'SignedUpdateSessionStore.ps1'))
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if(-not $PortableContracts -or [Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT){throw 'This fixture requires explicit non-Windows portable contracts; native qualification is separate.'}
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('signed-previous-restore-'+[guid]::NewGuid().ToString('N'))
$null=[IO.Directory]::CreateDirectory($fixture)
$global:SignedPreviousFixture=@{root=$fixture;baseline='';context=$null}
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Value,[string]$Name){if(-not $Value){throw $Name};$checks.Add($Name)}
function Reject([scriptblock]$Call,[string]$Name){$failed=$false;try{$null=& $Call}catch{$failed=$true};Check $failed $Name}
function Hash([int]$N){return $N.ToString('x64')}
function Quoted([string]$Value){return "'"+$Value.Replace("'","''")+"'"}
$tools=Join-Path $fixture 'tools';$null=[IO.Directory]::CreateDirectory($tools)
foreach($name in @('SignedUpdatePreviousRestore.ps1','commission-read-only.ps1','prepare-session-preservation.ps1')){
  [IO.File]::Copy((Join-Path $PSScriptRoot $name),(Join-Path $tools $name))
  Check ((Get-FileHash (Join-Path $PSScriptRoot $name)).Hash -ceq (Get-FileHash (Join-Path $tools $name)).Hash) ('Exact transaction source copy: '+$name)
}
$commonAdapter=@'
function Assert-LocalPath([string]$Path) {
  $p=[IO.Path]::GetFullPath($Path)
  if(-not $p.StartsWith($global:SignedPreviousFixture.root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw ('Escaped disposable fixture: '+$p)}
  $walk=$p
  while($walk.Length -gt $global:SignedPreviousFixture.root.Length){if([IO.Path]::Exists($walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)){throw 'Fixture links refused'};$walk=[IO.Path]::GetDirectoryName($walk)}
  return $p
}
'@
$commissioningAdapter=@'
function Get-CommissioningContext([string]$Workspace) {
  if($Workspace -cne $global:SignedPreviousFixture.context.workspace){throw 'Wrong disposable workspace'}
  $c=$global:SignedPreviousFixture.context|ConvertTo-Json -Depth 12|ConvertFrom-Json
  $i=Read-InstalledIdentity $c.installation.root
  $c.installation.generation=$i.generation;$c.installation.commit=$i.ownership.commit
  $c.installation.executableSha256=Get-Digest $i.executable
  $c.installation.stateSha256=Get-Digest (Join-Path $i.root 'state.json')
  $c.installation.ownershipSha256=Get-Digest (Join-Path $i.generation 'ownership.json')
  return $c
}
function Assert-PreparationClosed([string[]]$Roots){foreach($root in $Roots){$null=Assert-LocalPath $root}}
function Get-PreparationAccessAcl([string]$Path){$path=Assert-LocalPath $Path;return 'unix-mode:'+([int][IO.File]::GetUnixFileMode($path))}
function Get-Acl([string]$LiteralPath){return [pscustomobject]@{Sddl=(Get-PreparationAccessAcl $LiteralPath)}}
function Assert-PreparationAcl([string]$Expected,[string]$Actual,[switch]$AllowDaclAutoInherited){if($Expected -cne $Actual){throw 'Actual disposable file permissions changed'}}
function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid){$null=Assert-LocalPath $Directory;if(-not [IO.Directory]::Exists($Directory) -or $Sid -cne 'S-1-5-21-123'){throw 'Wrong private fixture identity'}}
function New-PreparationDirectory($Context,[string]$Purpose){return New-RunDirectory $Context.workspace $Purpose}
function Open-CommissioningProfileLock([string]$ProfileDirectory){
  $p=Assert-LocalPath $ProfileDirectory
  $key=Get-CommissioningHash ([Text.Encoding]::UTF8.GetBytes($p))
  return [IO.File]::Open((Join-Path $global:SignedPreviousFixture.root ($key+'.profile-lock')),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
}
$script:CommissioningBaseline=$global:SignedPreviousFixture.baseline
'@
$storeAdapter=@'
function Get-SignedStoreOwner {return [pscustomobject]@{sid='S-1-5-21-123';pid=$PID;createdFiletime='fixture';sessionId=1;image='inert';imageSha256=(Hash 99)}}
function Enter-SignedInstallationRoot([string]$Root){$null=Assert-LocalPath $Root;if(-not [IO.Directory]::Exists($Root)){throw 'Missing fixture root'};return ,@()}
function Get-SignedStorePinnedDirectory([string]$Requested,$Pins){return Assert-LocalPath $Requested}
function Assert-SignedStoreChild($Stream,$Pins){Assert-SignedInstallationHandle $Stream}
function Open-SignedInstallationRecordFile([string]$Path){$null=Assert-LocalPath $Path;return [IO.File]::Open($Path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)}
function Open-SignedInstallationLock([string]$Root){$null=Assert-LocalPath $Root;return [IO.File]::Open((Join-Path $Root 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)}
function Assert-SignedInstallationHandle($Stream,[string]$Sid=''){if($Stream.SafeFileHandle.IsClosed){throw 'Closed fixture handle'}}
function Enter-SignedStoreDirectory([string]$Directory,$Owner){$null=Assert-LocalPath $Directory;if(-not [IO.Directory]::Exists($Directory)){throw 'Missing private fixture directory'};return ,@()}
function New-SignedStoreFile([string]$Path,[string]$Sid){$null=Assert-LocalPath $Path;return [IO.FileStream]::new($Path,[IO.FileMode]::CreateNew,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None,4096,[IO.FileOptions]::WriteThrough)}
function Assert-SignedStoreNative($Store){if($Store.stream.SafeFileHandle.IsClosed){throw 'Closed fixture journal'};if($Store.installationBarrier){Assert-SignedInstallationBarrier $Store.installationBarrier}}
'@
[IO.File]::WriteAllText((Join-Path $tools 'Commissioning.ps1'),('. '+(Quoted (Join-Path $PSScriptRoot 'Commissioning.ps1'))+"`n"+$commonAdapter+"`n"+$commissioningAdapter))
[IO.File]::WriteAllText((Join-Path $tools 'SignedUpdateSessionStore.ps1'),('. '+(Quoted ([IO.Path]::GetFullPath($StoreSource)))+"`n"+$commonAdapter+"`n"+$storeAdapter))
. (Join-Path $tools 'SignedUpdatePreviousRestore.ps1')
# Native physical pin validation is unavailable on Linux. This portable adapter
# checks only bounded disposable canonical paths; it is NOT alias qualification.
function Assert-SignedPreviousRootBinding($Store){if(-not $Store.installationBarrier -or $Store.installationBarrier.physicalRoot -cne (Assert-LocalPath $Store.installationBarrier.root)){throw 'Fixture root binding changed'}}
function WriteJson([string]$Path,$Value){[IO.File]::WriteAllBytes($Path,(Get-SignedStoreBytes $Value))}
function Fresh([switch]$Changed) {
  $base=Join-Path $fixture ([guid]::NewGuid().ToString('N'))
  $root=Join-Path $base 'installed';$generation=Join-Path $root ('generations/'+('a'*32));$app=Join-Path $generation 'app'
  $profile=Join-Path $base 'profile';$workspace=Join-Path $base 'workspace';$plugins=Join-Path $base 'plugins';$journal=Join-Path $base 'private-journal'
  foreach($p in @($app,$profile,$workspace,$plugins,$journal)){$null=[IO.Directory]::CreateDirectory($p)}
  $exe=Join-Path $app 'opencpn.exe';[IO.File]::WriteAllText($exe,'INERT non-executable fixture; never launched')
  WriteJson (Join-Path $root 'owner.json') @{owner='OpenNavX.Alpha1.SideBySide.1'}
  WriteJson (Join-Path $root 'state.json') @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=('a'*32)}
  WriteJson (Join-Path $generation 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=('a'*40);packageSha256=(Hash 5);managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $exe)})}
  $ini=Join-Path $profile 'opencpn.ini';$encoding=[Text.UTF8Encoding]::new($false,$true)
  $text="[Settings]`r`nPersistActiveRoute=0`r`nLocale=sv`r`nGPUTextureMemSize=64`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n[Directories]`r`nChartDir=original`r`n"
  $text+='#'+('x'*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  [IO.File]::WriteAllText($ini,$text,$encoding);$global:SignedPreviousFixture.baseline=Get-Digest $ini;$script:CommissioningBaseline=$global:SignedPreviousFixture.baseline
  [IO.File]::WriteAllText((Join-Path $profile 'navobj.xml'),'<gpx>inert retained user route</gpx>')
  $unsafe=Join-Path $plugins 'unsafe_pi.dll';$safe=Join-Path $plugins 'safe_pi.dll'
  [IO.File]::WriteAllText($unsafe,'inert quarantined bytes');[IO.File]::WriteAllText($safe,'inert retained bytes')
  $global:SignedPreviousFixture.context=[pscustomobject][ordered]@{sid='S-1-5-21-123';session=1;workspace=$workspace;profile=$profile;application=$app;managed=$plugins;pluginRoots=@($plugins);installation=[pscustomobject]@{root=$root;generation=$generation;executable=$exe;commit=('a'*40);stateSha256=(Get-Digest (Join-Path $root 'state.json'));ownershipSha256=(Get-Digest (Join-Path $generation 'ownership.json'));executableSha256=(Get-Digest $exe)};launchEnvironment=[pscustomobject]@{path=$app;workingDirectory=$app}}
  $inventoryResult=(& (Join-Path $tools 'commission-read-only.ps1') -Action Inventory -Workspace $workspace)|ConvertFrom-Json
  $inventory=Read-Record $inventoryResult.record
  $review=Join-Path $workspace 'source-review.txt';[IO.File]::WriteAllText($review,'Explicit inert fixture source review; no real plugin qualification')
  $decisions=@($inventory.plugins|ForEach-Object{@{path=$_.path;sha256=$_.sha256;decision=$(if($_.path -ceq $safe){'retain'}else{'quarantine'});reason='Disposable fixture';sourceBoundary='No product plugin claim';sourceRevision=$(if($_.path -ceq $safe){'1'*40}else{$null});startupAndIdleReadOnly=($_.path -ceq $safe);evidencePath=$review;evidenceSha256=(Get-Digest $review)}})
  $plan=Join-Path $workspace 'plan.json'
  Write-Record $plan @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';inventoryPath=$inventoryResult.record;inventorySha256=$inventoryResult.recordSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');plugins=$decisions}
  $prepared=(& (Join-Path $tools 'commission-read-only.ps1') -Action Prepare -Workspace $workspace -Plan $plan -ExpectedPlanSha256 (Get-Digest $plan))|ConvertFrom-Json
  $arguments=@{Workspace=$workspace;Record=$prepared.record;ExpectedRecordSha256=$prepared.recordSha256}
  $null=& (Join-Path $tools 'commission-read-only.ps1') -Action Apply @arguments
  Check (-not [IO.File]::Exists($unsafe) -and (Get-Digest $ini) -cne $global:SignedPreviousFixture.baseline) 'Actual Apply quarantines plugin and publishes input-only profile'
  if($Changed){[IO.File]::WriteAllText($ini,([IO.File]::ReadAllText($ini).Replace('GPUTextureMemSize=64','GPUTextureMemSize=128')),$encoding)}
  $now=[DateTimeOffset]::UtcNow.ToUnixTimeSeconds()
  $binding=[pscustomobject]@{session=(Get-CommissioningHash ([Text.Encoding]::UTF8.GetBytes($base)));previous=[pscustomobject]@{generation=('a'*32);commit=('a'*40);packageSha256=(Hash 5);executableSha256=(Get-Digest $exe)};release=[pscustomobject]@{commit=('b'*40);packageSha256=(Hash 6);executableSha256=(Hash 7);policySha256=(Hash 8);sourceReviewSha256=(Hash 9)};previousCommissioningSha256=$prepared.recordSha256;fallbackSourceReviewSha256=(Hash 10);createdUnix=$now;expiresUnix=($now+3600)}
  $handle=New-SignedUpdateStore $journal (Get-SignedStoreBytes $binding) -InstallationRoot $root -OperationEvents
  $store=Get-SignedStoreLive $handle;$store.installationBarrier.physicalRoot=$root
  $arguments.StoreHandle=$handle
  $seen=Inspect-SignedUpdatePreviousRestore @arguments
  $complete=@{};foreach($key in $arguments.Keys){$complete[$key]=$arguments[$key]}
  $complete.Inspection=$seen.inspection;$complete.ExpectedInspectionSha256=$seen.inspectionSha256;$complete.ReviewedCurrentIniSha256=$seen.currentIniSha256;$complete.RequestNonce=Hash 11
  return @{arguments=$arguments;complete=$complete;seen=$seen;store=$store;handle=$handle;root=$root;workspace=$workspace;ini=$ini;unsafe=$unsafe;inventory=$inventory;active=(Join-Path $workspace 'commissioning-active.json');journal=(Join-Path $journal 'signed-update-armed.jsonl')}
}
function Preservation($Case) {
  $parent=[IO.Path]::GetDirectoryName($Case.arguments.Record);$before=Join-Path $parent 'input-only.ini';$after=(Read-Record $Case.seen.inspection).savedIni
  $review=[pscustomobject]@{schema=1;owner='OpenNavX.SessionPreservationReview.1';beforeSha256=(Get-Digest $before);afterSha256=(Get-Digest $after);parentPreparedSha256=$Case.arguments.ExpectedRecordSha256;inspectionSha256=$Case.seen.inspectionSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;changes=@(Get-CommissioningIniDiff $before $after|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';decision='preserve-current';reason='Explicit independent exact fixture settings review'}})}
  $reviewPath=Join-Path $Case.workspace 'preservation-review.json';Write-Record $reviewPath $review
  $choice=(& (Join-Path $tools 'prepare-session-preservation.ps1') -Workspace $Case.workspace -Record $Case.arguments.Record -ExpectedRecordSha256 $Case.arguments.ExpectedRecordSha256 -Inspection $Case.seen.inspection -ExpectedInspectionSha256 $Case.seen.inspectionSha256 -PreservationReview $reviewPath -ExpectedReviewSha256 (Get-Digest $reviewPath))|ConvertFrom-Json
  $Case.complete.PreservationProposal=$choice.proposal;$Case.complete.ExpectedPreservationSha256=$choice.proposalSha256
  return $choice
}
function Complete($Case){$arguments=$Case.complete;return Invoke-SignedUpdatePreviousRestore @arguments}
function Inspect($Case){$arguments=$Case.arguments;return Inspect-SignedUpdatePreviousRestore @arguments}
function NoEvent($Case,[string]$Name){Check (@($Case.store.operations.events).Count -eq 0 -and [IO.File]::Exists((Join-Path $Case.root 'signed-update-armed.jsonl'))) $Name}
$write=${function:Write-SignedStoreFrame}
try {
  $case=Fresh -Changed;$iniBefore=Get-Digest $case.ini
  Reject {Complete $case} 'Changed settings cannot be erased without exact preservation proposal'
  Check ((Get-Digest $case.ini) -ceq $iniBefore -and [IO.File]::Exists($case.active)) 'Preservation refusal leaves actual profile and active transaction unchanged'
  NoEvent $case 'Preservation refusal retains denial and publishes no event'
  $choice=Preservation $case
  $bad=@{};foreach($key in $case.complete.Keys){$bad[$key]=$case.complete[$key]};$bad.ExpectedInspectionSha256=Hash 20
  Reject {Invoke-SignedUpdatePreviousRestore @bad} 'Wrong retained inspection hash refuses'
  $bad=@{};foreach($key in $case.complete.Keys){$bad[$key]=$case.complete[$key]};$bad.ExpectedPreservationSha256=Hash 21
  Reject {Invoke-SignedUpdatePreviousRestore @bad} 'Wrong retained preservation hash refuses'
  $result=Complete $case
  Check ($result.status -ceq 'previous-restored' -and -not $result.launchAuthority -and -not $result.applicationLaunched) 'Concrete restore completion has no launch authority'
  Check ((Get-Digest $case.ini) -ceq $choice.baselineSha256 -and [IO.File]::ReadAllText($case.ini).Contains('GPUTextureMemSize=128')) 'Actual user setting survives and only original connection direction is restored'
  Check ([IO.File]::Exists($case.unsafe) -and -not [IO.File]::Exists($case.active)) 'Actual quarantined plugin restored and active marker absent'
  $completion=Read-Record $result.completion
  $event=@($case.store.operations.events)[0];$evidence=$event.evidence
  Check ($case.store.operations.events.Count -eq 1 -and $event.action -ceq 'PreviousRestored') 'Only verified actual restore appends PreviousRestored'
  Check ($evidence.restoreCompletionSha256 -ceq (Get-Digest $result.completion) -and $evidence.profileSha256 -ceq (Get-Digest $case.ini) -and $evidence.inspectionSha256 -ceq (Get-Digest $case.seen.inspection)) 'Event hashes derive from exact retained records and actual profile bytes'
  $context=Get-CommissioningContext $case.workspace
  $trees=[pscustomobject][ordered]@{profile=(Get-PreparationTree $context.profile);plugins=@(Get-CommissioningTrees $context.pluginRoots);acls=(Get-SessionPreservationAcls (@((Get-PreparationTree $context.profile))+@($case.inventory.trees)) @())}
  Check ($evidence.treesSha256 -ceq (Get-SignedUpdateReceiptHash $trees)) 'Event whole-tree and permission digest derives from current real files'
  $journalBytes=$case.store.stream.Length
  Reject {Complete $case} 'Completed operation cannot replay as a fresh restoration'
  Check ($case.store.stream.Length -eq $journalBytes) 'Replay does not append journal bytes'
  Close-SignedUpdateStore $case.handle

  $case=Fresh
  $null=Complete $case
  Check ((Get-Digest $case.ini) -ceq $global:SignedPreviousFixture.baseline) 'Unchanged input-only state restores exact original baseline without preservation'
  Close-SignedUpdateStore $case.handle

  $case=Fresh -Changed;$null=Preservation $case
  [IO.File]::AppendAllText((Join-Path ([IO.Path]::GetDirectoryName($case.ini)) 'navobj.xml'),'unexpected')
  Reject {Complete $case} 'Late actual route/profile-file drift refuses before restoration'
  NoEvent $case 'Late profile drift leaves denial without event'
  Close-SignedUpdateStore $case.handle

  $case=Fresh;$prepared=Read-Record $case.arguments.Record
  [IO.File]::AppendAllText($prepared.quarantine[0].destination,'corrupt')
  Reject {Complete $case} 'Actual quarantined plugin corruption refuses'
  NoEvent $case 'Corrupt plugin never produces PreviousRestored'
  Close-SignedUpdateStore $case.handle

  $case=Fresh
  $statePath=Join-Path $case.root 'state.json';$original=[IO.File]::ReadAllBytes($statePath)
  WriteJson $statePath @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=('b'*32)}
  Reject {Inspect $case} 'Changed selected generation refuses actual operation'
  NoEvent $case 'Generation change cannot become restoration evidence'
  [IO.File]::WriteAllBytes($statePath,$original);Close-SignedUpdateStore $case.handle

  $case=Fresh
  $lock=Open-SignedInstallationLock $case.root
  try{Reject {Complete $case} 'Existing installation transaction lock blocks restoration'}finally{$lock.Dispose()}
  Check ([IO.File]::Exists($case.active) -and -not [IO.File]::Exists($case.unsafe)) 'Lock refusal leaves actual commissioning unchanged'
  Close-SignedUpdateStore $case.handle

  $case=Fresh -Changed;$null=Preservation $case
  $route=Join-Path ([IO.Path]::GetDirectoryName($case.ini)) 'navobj.xml'
  $permissions=[IO.File]::GetUnixFileMode($route)
  [IO.File]::SetUnixFileMode($route,($permissions -bxor [IO.UnixFileMode]::UserExecute))
  Reject {Complete $case} 'Actual preserved route permission drift refuses'
  NoEvent $case 'Permission drift never produces restoration evidence'
  [IO.File]::SetUnixFileMode($route,$permissions);Close-SignedUpdateStore $case.handle

  $case=Fresh;Close-SignedUpdateStore $case.handle
  Reject {Complete $case} 'Closed live store handle cannot invoke restoration'
  Check ([IO.File]::Exists($case.active) -and -not [IO.File]::Exists($case.unsafe)) 'Lost owner custody leaves actual commissioning unchanged'

  $case=Fresh
  $case.store.installationBarrier.physicalRoot=Join-Path $case.root 'wrong-root'
  Reject {Complete $case} 'Missing matching held physical-root binding refuses'
  NoEvent $case 'Root-binding refusal retains installed denial and no event'
  Close-SignedUpdateStore $case.handle

  $case=Fresh
  $profileAcquire=${function:Open-CommissioningProfileLock};$acquisitions=0
  function Open-CommissioningProfileLock([string]$ProfileDirectory) {
    $held=& $profileAcquire $ProfileDirectory
    $script:acquisitions++
    if($script:acquisitions -eq 2){WriteJson (Join-Path $case.root 'state.json') @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=('b'*32)}}
    return $held
  }
  # The engine imports its adapter in its own scope; the wrapper's second
  # acquisition here is its post-engine check. Inject the concurrent pointer
  # mutation on that wrapper acquisition, without changing transaction code.
  $acquisitions=1
  Reject {Complete $case} 'Post-engine selected-generation drift cannot publish completion'
  Check ($case.store.poisoned -and (Get-Digest $case.ini) -ceq $global:SignedPreviousFixture.baseline -and -not [IO.File]::Exists($case.active)) 'Postcheck failure retains completed real restore but poisons publication custody'
  NoEvent $case 'Postcheck drift publishes no event and preserves denial'
  ${function:Open-CommissioningProfileLock}=$profileAcquire;Close-SignedUpdateStore $case.handle

  $case=Fresh;$journalLength=$case.store.stream.Length;$journalDigest=$case.store.digest
  function Write-SignedStoreFrame($Store,[byte[]]$Bytes){if($Store.path -ceq $case.journal){throw 'Injected private journal write failure'};& $write $Store $Bytes}
  Reject {Complete $case} 'Journal persistence failure cannot publish restoration authority'
  Check ((Get-Digest $case.ini) -ceq $global:SignedPreviousFixture.baseline -and [IO.File]::Exists($case.unsafe) -and -not [IO.File]::Exists($case.active)) 'Write failure follows real completed restoration, not a mock success'
  Check ($case.store.poisoned -and $case.store.stream.Length -eq $journalLength -and $case.store.digest -ceq $journalDigest) 'Failed publication poisons live custody and leaves prior durable journal unchanged'
  NoEvent $case 'Write failure retains installed denial and no event'
  ${function:Write-SignedStoreFrame}=$write;Close-SignedUpdateStore $case.handle
  [pscustomobject]@{status='passed';count=$checks.Count;checks=@($checks);scope='exact real offline engine and live durable store on disposable files with Windows boundary adapters';storeSourceSha256=((Get-FileHash -LiteralPath $StoreSource).Hash.ToLowerInvariant());nativeCustodyAccepted=$false;productWired=$false;applicationLaunched=$false;boatAccess=$false}|ConvertTo-Json -Depth 5
} finally {
  ${function:Write-SignedStoreFrame}=$write
  foreach($handle in @($script:SignedUpdateStores.Keys)){try{Close-SignedUpdateStore $handle}catch{}}
  Remove-Variable SignedPreviousFixture -Scope Global -ErrorAction SilentlyContinue
  Remove-Item -LiteralPath $fixture -Recurse -Force
}
