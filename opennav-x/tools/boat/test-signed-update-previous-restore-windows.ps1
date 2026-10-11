# Exact restoration engine/native custody on disposable NTFS files only.
# Synthetic context, baseline and closed-process observation are explicit fixture
# substitutions. No installed product, actual profile, process or boat acceptance.
[CmdletBinding()]
param([switch]$IsolatedLocal)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Native Windows PowerShell 5.1 is required; no portable native acceptance.'}
if($PSVersionTable.PSVersion.Major -ne 5 -or $PSVersionTable.PSVersion.Minor -ne 1){throw 'Run this fixture in a fresh Windows PowerShell 5.1 x86 or x64 process.'}
if($env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal){throw 'Native execution requires CI or explicit -IsolatedLocal temporary-file testing.'}
if(('SignedUpdateStoreNative' -as [type]) -or ('OpenNavX.CommissioningProfileLock' -as [type])){throw 'Use a fresh process so exact native helper source is loaded once.'}
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('signed-previous-restore-'+[guid]::NewGuid().ToString('N'))
$null=[IO.Directory]::CreateDirectory($fixture)
$global:SignedPreviousFixture=@{root=$fixture;baseline='';context=$null}
$write=$null;$profileAcquire=$null;$script:publicBaselineChecked=$false
try {
$fixtureSid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
$self=[Diagnostics.Process]::GetCurrentProcess()
try {$fixtureSession=$self.SessionId} finally {$self.Dispose()}
function Set-FixturePrivateDirectory([string]$Path) {
  if($Path -ine $fixture -and -not $Path.StartsWith($fixture+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Private ACL setup escaped disposable fixture.'}
  $sid=[Security.Principal.SecurityIdentifier]::new($fixtureSid)
  $acl=[Security.AccessControl.DirectorySecurity]::new();$acl.SetOwner($sid);$acl.SetAccessRuleProtection($true,$false)
  foreach($principal in @($sid,[Security.Principal.SecurityIdentifier]::new('S-1-5-18'))){
    $acl.AddAccessRule([Security.AccessControl.FileSystemAccessRule]::new($principal,[Security.AccessControl.FileSystemRights]::FullControl,([Security.AccessControl.InheritanceFlags]::ContainerInherit -bor [Security.AccessControl.InheritanceFlags]::ObjectInherit),[Security.AccessControl.PropagationFlags]::None,[Security.AccessControl.AccessControlType]::Allow))
  }
  Set-Acl -LiteralPath $Path -AclObject $acl
}
$drive=[IO.DriveInfo]::new([IO.Path]::GetPathRoot($fixture))
if($drive.DriveFormat -cne 'NTFS'){throw 'Native fixture requires local NTFS temporary storage; no boundary substitution.'}
Set-FixturePrivateDirectory $fixture
$sourceHashes=[ordered]@{}
foreach($name in @('test-signed-update-previous-restore-windows.ps1','SignedUpdatePreviousRestore.ps1','SignedUpdateSessionStore.ps1','SignedUpdateOperations.ps1','SignedUpdateSession.ps1','commission-read-only.ps1','prepare-session-preservation.ps1','Commissioning.ps1','CommissioningBaseline.ps1','CommissioningProfileLock.cs','Preparation.ps1','Common.ps1','ColdBaseline.ps1','InstalledResourceReview.ps1','RestartCommissioningPolicy.ps1','RestartAuiPersistence.ps1','RestartDashboardPersistence.ps1')){
  $sourceHashes[$name]=(Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $name) -Algorithm SHA256).Hash.ToLowerInvariant()
}
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
# Bound all engine data paths before applying the unchanged native path policy.
$script:SignedPreviousNativePath=${function:Assert-LocalPath}
function Assert-LocalPath([string]$Path) {
  $p=[IO.Path]::GetFullPath($Path).TrimEnd('\')
  if(-not $p.StartsWith($global:SignedPreviousFixture.root+'\',[StringComparison]::OrdinalIgnoreCase)){throw ('Escaped disposable native fixture: '+$p)}
  return & $script:SignedPreviousNativePath $p
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
# No real process discovery. This is synthetic closure for inert fixture bytes.
function Assert-PreparationClosed([string[]]$Roots){foreach($root in $Roots){$null=Assert-LocalPath $root}}
if($global:SignedPreviousFixture.baseline){$script:CommissioningBaseline=$global:SignedPreviousFixture.baseline}
'@
[IO.File]::WriteAllText((Join-Path $tools 'Commissioning.ps1'),('. '+(Quoted (Join-Path $PSScriptRoot 'Commissioning.ps1'))+"`n"+$commonAdapter+"`n"+$commissioningAdapter))
[IO.File]::WriteAllText((Join-Path $tools 'SignedUpdateSessionStore.ps1'),('. '+(Quoted (Join-Path $PSScriptRoot 'SignedUpdateSessionStore.ps1'))))
. (Join-Path $tools 'SignedUpdatePreviousRestore.ps1')
# Native owner, Store pins/ACLs, metadata and publication stay real. Marked
# failure cases below wrap actual mutex/frame calls to inject an error.
$probe=Join-Path $fixture 'native-prerequisite-profile';$null=[IO.Directory]::CreateDirectory($probe)
$probeIni=Join-Path $probe 'opencpn.ini';[IO.File]::WriteAllText($probeIni,'inert metadata prerequisite')
try {
  $null=Get-PreparationNativeMetadata $probeIni
  $held=Open-CommissioningProfileLock $probe
  try {Check ($held.Identity.Length -gt 0) 'Exact native profile mutex and physical ancestry acquire'} finally {$held.Dispose()}
} catch {throw ('Native NTFS/ACL/SACL/profile-mutex prerequisite failed; no adapter allowed: '+$_.Exception.Message)}
Check $true 'Real native profile metadata and audit ACL retrieval succeed'
function WriteJson([string]$Path,$Value){[IO.File]::WriteAllBytes($Path,(Get-SignedStoreBytes $Value))}
function Fresh([switch]$Changed) {
  $base=Join-Path $fixture ([guid]::NewGuid().ToString('N'))
  $root=Join-Path $base 'installed';$generation=Join-Path $root ('generations/'+('a'*32));$app=Join-Path $generation 'app'
  $profile=Join-Path $base 'profile';$workspace=Join-Path $base 'workspace';$plugins=Join-Path $base 'plugins';$journal=Join-Path $base 'private-journal'
  foreach($p in @($app,$profile,$workspace,$plugins,$journal)){$null=[IO.Directory]::CreateDirectory($p)}
  Set-FixturePrivateDirectory $journal
  $exe=Join-Path $app 'opencpn.exe';[IO.File]::WriteAllText($exe,'INERT non-executable fixture; never launched')
  WriteJson (Join-Path $root 'owner.json') @{owner='OpenNavX.Alpha1.SideBySide.1'}
  WriteJson (Join-Path $root 'state.json') @{schema=1;owner='OpenNavX.Alpha1.SideBySide.1';current=('a'*32)}
  WriteJson (Join-Path $generation 'ownership.json') @{owner='OpenNavX.Alpha1.SideBySide.1';commit=('a'*40);packageSha256=(Hash 5);managedFiles=@(@{path='app/opencpn.exe';sha256=(Get-Digest $exe)})}
  $ini=Join-Path $profile 'opencpn.ini';$encoding=[Text.UTF8Encoding]::new($false,$true)
  $text="[Settings]`r`nPersistActiveRoute=0`r`nLocale=sv`r`nGPUTextureMemSize=64`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Fixture;0;;0`r`n[Directories]`r`nChartDir=original`r`n"
  $text+='#'+('x'*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  [IO.File]::WriteAllText($ini,$text,$encoding);$fixtureBaseline=Get-Digest $ini
  [IO.File]::WriteAllText((Join-Path $profile 'navobj.xml'),'<gpx>inert retained user route</gpx>')
  $unsafe=Join-Path $plugins 'unsafe_pi.dll';$safe=Join-Path $plugins 'safe_pi.dll'
  [IO.File]::WriteAllText($unsafe,'inert quarantined bytes');[IO.File]::WriteAllText($safe,'inert retained bytes')
  $global:SignedPreviousFixture.context=[pscustomobject][ordered]@{sid=$fixtureSid;session=$fixtureSession;workspace=$workspace;profile=$profile;application=$app;managed=$plugins;pluginRoots=@($plugins);installation=[pscustomobject]@{root=$root;generation=$generation;executable=$exe;commit=('a'*40);stateSha256=(Get-Digest (Join-Path $root 'state.json'));ownershipSha256=(Get-Digest (Join-Path $generation 'ownership.json'));executableSha256=(Get-Digest $exe)};launchEnvironment=[pscustomobject]@{path=$app;workingDirectory=$app}}
  if(-not $script:publicBaselineChecked){
    $publicError=''
    try {$null=& (Join-Path $tools 'commission-read-only.ps1') -Action Inventory -Workspace $workspace} catch {$publicError=$_.Exception.Message}
    Check ($publicError -like '*exact recovered or explicitly adopted baseline*') 'Unchanged public engine rejects the synthetic baseline for the expected reason'
    Check ((Get-Digest $ini) -ceq $fixtureBaseline -and -not [IO.File]::Exists((Join-Path $workspace 'commissioning-active.json'))) 'Public baseline refusal leaves disposable profile unchanged'
    $script:publicBaselineChecked=$true
  }
  # Only after public refusal: test-only binding, never a product parameter.
  $global:SignedPreviousFixture.baseline=$fixtureBaseline;$script:CommissioningBaseline=$fixtureBaseline
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
  $store=Get-SignedStoreLive $handle
  Check ($store.owner.sid -ceq $fixtureSid -and $store.owner.sessionId -eq $fixtureSession) 'Actual native owner SID and session bind the operation'
  Check ($store.installationBarrier.physicalRoot.StartsWith('\\?\GLOBALROOT\Device\HarddiskVolume',[StringComparison]::OrdinalIgnoreCase)) 'Actual native installation barrier retains physical NTFS root'
  $null=Assert-SignedStoreCurrent $store
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
# Actual engine/store operation cases.
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

  $case=Fresh;$before=Get-Digest $case.ini
  $heldProfile=Open-CommissioningProfileLock ([IO.Path]::GetDirectoryName($case.ini))
  try {Reject {Complete $case} 'Held exact native profile mutex blocks real Restore'} finally {$heldProfile.Dispose()}
  Check ($case.store.poisoned -and (Get-Digest $case.ini) -ceq $before -and [IO.File]::Exists($case.active) -and -not [IO.File]::Exists($case.unsafe)) 'Native mutex refusal poisons attempted operation without changing profile or plugins'
  NoEvent $case 'Native profile exclusion cannot publish restoration evidence'
  Close-SignedUpdateStore $case.handle

  $case=Fresh -Changed;$null=Preservation $case
  $route=Join-Path ([IO.Path]::GetDirectoryName($case.ini)) 'navobj.xml'
  $originalAcl=(Get-Acl -LiteralPath $route).Sddl
  $changedAcl=[Security.AccessControl.FileSecurity]::new()
  $changedAcl.SetSecurityDescriptorSddlForm($originalAcl)
  $changedAcl.AddAccessRule([Security.AccessControl.FileSystemAccessRule]::new([Security.Principal.SecurityIdentifier]::new($fixtureSid),[Security.AccessControl.FileSystemRights]::WriteData,[Security.AccessControl.AccessControlType]::Deny))
  Set-Acl -LiteralPath $route -AclObject $changedAcl
  Reject {Complete $case} 'Actual preserved route DACL drift refuses'
  NoEvent $case 'DACL drift never produces restoration evidence'
  $restoreAcl=[Security.AccessControl.FileSecurity]::new();$restoreAcl.SetSecurityDescriptorSddlForm($originalAcl)
  Set-Acl -LiteralPath $route -AclObject $restoreAcl
  Close-SignedUpdateStore $case.handle

  $case=Fresh;Close-SignedUpdateStore $case.handle
  Reject {Complete $case} 'Closed live store handle cannot invoke restoration'
  Check ([IO.File]::Exists($case.active) -and -not [IO.File]::Exists($case.unsafe)) 'Lost owner custody leaves actual commissioning unchanged'

  $case=Fresh
  $case.store.installationBarrier.physicalRoot='\\?\GLOBALROOT\Device\HarddiskVolume0\wrong-root'
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
  $case=Fresh;$profileAcquire=${function:Open-CommissioningProfileLock}
  function Open-CommissioningProfileLock([string]$ProfileDirectory){
    $held=& $profileAcquire $ProfileDirectory
    $probe=[pscustomobject]@{held=$held}
    $probe|Add-Member -MemberType ScriptMethod -Name Dispose -Value {$this.held.Dispose();throw 'Injected post-engine lock disposal failure'}
    return $probe
  }
  $published=@();$cleanupError=''
  try{$published=@(Complete $case)}catch{$cleanupError=$_.Exception.Message}
  Check ($cleanupError -like '*Injected post-engine lock disposal failure*' -and $published.Count -eq 0) 'Failed lock cleanup cannot emit operation success'
  Check ($case.store.poisoned -and @($case.store.operations.events).Count -eq 1) 'Cleanup failure poisons custody after consumed PreviousRestored event'
  Check ((Get-Digest $case.ini) -ceq $global:SignedPreviousFixture.baseline -and -not [IO.File]::Exists($case.active) -and [IO.File]::Exists((Join-Path $case.root 'signed-update-armed.jsonl'))) 'Cleanup failure retains real restoration, consumed journal and installed denial'
  $released=Open-SignedInstallationLock $case.root;$released.Dispose()
  Check $true 'Profile disposal failure still releases independent installation lock'
  Reject {Assert-SignedStoreCurrent $case.store} 'Consumed event cannot authorize further operations after cleanup failure'
  ${function:Open-CommissioningProfileLock}=$profileAcquire;Close-SignedUpdateStore $case.handle
  foreach($name in $sourceHashes.Keys){Check ((Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $name) -Algorithm SHA256).Hash.ToLowerInvariant() -ceq $sourceHashes[$name]) ('Source stayed exact: '+$name)}
  $report=[pscustomobject]@{status='passed';count=$checks.Count;checks=@($checks);sourceSha256=$sourceHashes;
    powerShellVersion=$PSVersionTable.PSVersion.ToString();processBits=([IntPtr]::Size*8);
    scope='Exact real offline engine, native owner/store/physical handles/ACL/SACL/publication and profile mutex on disposable NTFS files';
    fixtureSubstitutions=@('synthetic bounded commissioning identity/context','synthetic closed-process observation; no discovery','test-only baseline after public rejection','marked pointer mutation and frame/disposal fault seams');
    continuousNamespaceCustodyAccepted=$false;installedProductAccepted=$false;productWired=$false;applicationLaunched=$false;boatAccess=$false}
} finally {
  if($write){${function:Write-SignedStoreFrame}=$write}
  if(Get-Variable SignedUpdateStores -Scope Script -ErrorAction SilentlyContinue){foreach($handle in @($script:SignedUpdateStores.Keys)){try{Close-SignedUpdateStore $handle}catch{}}}
  Remove-Variable SignedPreviousFixture -Scope Global -ErrorAction SilentlyContinue
  Remove-Item -LiteralPath $fixture -Recurse -Force
}
# A fixture cleanup error must also prevent a successful result.
$report|ConvertTo-Json -Depth 6
