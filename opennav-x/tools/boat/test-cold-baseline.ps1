# Pure policy and disposable filesystem contracts. No real boat profile, registry,
# application, service, plugin or hardware access.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native -and -not $PortableContracts){throw 'Explicit portable contracts required.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable CI or explicit isolated temporary tests required.'}
$testRoot=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav cold baseline '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $testRoot
$script:actualDigest=(Get-Command Get-Digest).ScriptBlock
$script:actualAssertLocalPath=(Get-Command Assert-LocalPath).ScriptBlock
$script:actualBaselineReader=(Get-Command Read-CommissioningBaseline).ScriptBlock
$script:actualNewPreparationDirectory=(Get-Command New-PreparationDirectory).ScriptBlock
$script:actualCopyPreparationTree=(Get-Command Copy-PreparationTree).ScriptBlock
$script:actualWriteRecord=(Get-Command Write-Record).ScriptBlock
$script:actualPrivateEvidence=(Get-Command Assert-ColdPrivateEvidence).ScriptBlock
$script:actualPreparationAcl=(Get-Command Assert-PreparationAcl).ScriptBlock
$script:actualGetAcl=Get-Command Get-Acl -ErrorAction SilentlyContinue
$script:stockFixtureExe=$null;$script:seedRecord=$null;$script:seedRecordHash=$null;$script:seedInfo=$null
$script:fixtureContext=$null;$script:fixtureProcesses=@();$script:contextCalls=0;$script:contextMismatchAt=0
$script:copyRacePath=$null;$script:copyRaceBytes=$null;$script:copyRaceDone=$false;$script:failCompleteWrite=$false
function Assert-LocalPath([string]$Path){
  if(-not $Path){throw 'Empty fixture path.'}
  $p=[IO.Path]::GetFullPath($Path)
  $prefix=$testRoot.TrimEnd([IO.Path]::DirectorySeparatorChar,[IO.Path]::AltDirectorySeparatorChar)+[IO.Path]::DirectorySeparatorChar
  if(-not $p.Equals($testRoot,[StringComparison]::OrdinalIgnoreCase) -and -not $p.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase)){throw 'Cold-baseline fixture path escaped its disposable root.'}
  $walk=$p
  while($walk -and $walk.Length -ge $testRoot.Length){
    if((Test-Path -LiteralPath $walk) -and ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint)){throw 'Reparse fixture path refused.'}
    if($walk.Equals($testRoot,[StringComparison]::OrdinalIgnoreCase)){break}
    $walk=[IO.Path]::GetDirectoryName($walk)
  }
  if($native){return & $script:actualAssertLocalPath $p}
  return $p
}
function Get-Digest([string]$Path){
  if($script:stockFixtureExe -and [IO.Path]::GetFullPath($Path).Equals([IO.Path]::GetFullPath($script:stockFixtureExe),[StringComparison]::OrdinalIgnoreCase)){
    # Test-only machine identity boundary: this inert file stands in for the
    # already validated official executable. Every other path uses real SHA-256.
    return '7c6547562cca7954671eaab72833ca9d788710fd9808b6a699b6dc823852ae0c'
  }
  return & $script:actualDigest $Path
}
function Get-CommissioningContext([string]$Workspace){
  $script:contextCalls++
  if(-not $script:fixtureContext -or (Assert-LocalPath $Workspace) -ine $script:fixtureContext.workspace){throw 'Unexpected fixture workspace.'}
  Assert-PreparationProcesses $script:fixtureProcesses (@($script:fixtureContext.application,$script:fixtureContext.managed)+@($script:fixtureContext.pluginRoots))
  if($script:contextMismatchAt -and $script:contextCalls -ge $script:contextMismatchAt){$changed=Clone $script:fixtureContext;$changed.sid='S-1-5-21-999-1001';return $changed}
  return $script:fixtureContext
}
function Assert-PreparationClosed([string[]]$Roots){
  foreach($root in $Roots){
    $checked=Assert-LocalPath $root
    if(-not $checked.StartsWith($testRoot+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)){
      throw 'Disposable process guard escaped its fixture root.'
    }
  }
  Assert-PreparationProcesses $script:fixtureProcesses $Roots
}
function New-PreparationDirectory($Context,[string]$Purpose){
  if($native){return & $script:actualNewPreparationDirectory $Context $Purpose}
  $path=New-RunDirectory $Context.workspace $Purpose
  try{[IO.File]::SetUnixFileMode($path,[IO.UnixFileMode]::UserRead -bor [IO.UnixFileMode]::UserWrite -bor [IO.UnixFileMode]::UserExecute)}catch{}
  return $path
}
function Copy-PreparationTree($Snapshot,[string]$Destination){
  & $script:actualCopyPreparationTree $Snapshot $Destination
  if(-not $script:copyRaceDone -and $script:copyRacePath -and $Snapshot.root -ieq $script:fixtureContext.profile){
    [IO.File]::WriteAllBytes($script:copyRacePath,$script:copyRaceBytes);$script:copyRaceDone=$true
  }
}
function Write-Record([string]$Path,$Record){
  if($script:failCompleteWrite -and [IO.Path]::GetFileName($Path) -ceq 'completed-cold-baseline.json'){throw 'Injected final record publication failure.'}
  & $script:actualWriteRecord $Path $Record
}
function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid){
  if($native){return & $script:actualPrivateEvidence $Directory $Sid}
  # Portable tests still reject redirects; POSIX ownership/mode is not a
  # substitute for the native Windows protected-ACL acceptance gate.
  $root=Assert-LocalPath $Directory
  foreach($item in @(Get-ChildItem -LiteralPath $root -Force -Recurse)){
    if($item.Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Portable fixture evidence contains redirected path.'}
  }
}
function Get-Acl([string]$LiteralPath){
  if($native -and $script:actualGetAcl){return Microsoft.PowerShell.Security\Get-Acl -LiteralPath $LiteralPath}
  # Linux PowerShell may not ship the Windows ACL module. This stable marker
  # permits transaction plumbing only; native ACL acceptance is CI-only.
  return [pscustomobject]@{Sddl='portable-acl-not-native'}
}
function Assert-PreparationAcl([string]$Expected,[string]$Actual,[switch]$AllowDaclAutoInherited){
  if($native){return & $script:actualPreparationAcl $Expected $Actual -AllowDaclAutoInherited:$AllowDaclAutoInherited}
  if($Expected -cne $Actual){throw 'Portable fixture ACL marker changed.'}
}
$checks=New-Object 'Collections.Generic.List[string]'
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$refusalObserved=$false;try{$null=& $Body}catch{$refusalObserved=$true};if(-not $refusalObserved){throw ('Accepted unsafe cold baseline: '+$Name)};$checks.Add($Name)}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 20|ConvertFrom-Json)}
function Test-BytesEqual([byte[]]$Left,[byte[]]$Right){
  if($Left.Length -ne $Right.Length){return $false}
  for($i=0;$i -lt $Left.Length;$i++){if($Left[$i] -ne $Right[$i]){return $false}}
  return $true
}
function MakeReview([string]$Before,[string]$After){
  return [pscustomobject]@{schema=1;owner='OpenNavX.ColdProfileReview.1';captureSha256=('a'*64);predecessorRecordSha256=('b'*64);
    beforeSha256=(Get-Digest $Before);afterSha256=(Get-Digest $After);reviewedUtc=[datetime]::UtcNow.ToString('o');
    provenance='pre-existing-current-user-state;origin-unverified';preservationOnly=$true;
    changes=@(Get-CommissioningIniDiff $Before $After|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;
      origin='unverified';reason='Preserve this exact observed user setting after independent review.'}})}
}
function New-ColdFixture([string]$Name){
  $root=Join-Path $testRoot ('e2e-'+$Name);$workspace=Join-Path $root 'workspace';$profile=Join-Path $root 'profile'
  $application=Join-Path $root 'stock';$managed=Join-Path $root 'managed-plugins';$stockPlugins=Join-Path $application 'plugins'
  foreach($path in @($workspace,$profile,$application,$managed,$stockPlugins)){[void](New-Item -ItemType Directory -Path $path -Force)}
  $exe=Join-Path $application 'opencpn.exe';[IO.File]::WriteAllBytes($exe,[byte[]]@(77,90,0,0,1,2,3,4));$script:stockFixtureExe=$exe
  $local=Join-Path $root 'local';[void](New-Item -ItemType Directory -Path $local -Force)
  $sid=if($native){[Security.Principal.WindowsIdentity]::GetCurrent().User.Value}else{'S-1-5-21-100-1001'}
  $script:fixtureContext=[pscustomobject]@{sid=$sid;session=1;localAppData=$local;profile=$profile;application=$application;managed=$managed;workspace=$workspace;executable=$exe;installation=$null;pluginRoots=@($managed,$stockPlugins);launchEnvironment=[pscustomobject]@{workingDirectory=$application;path=$application}}
  $script:fixtureProcesses=@([pscustomobject]@{Name='explorer.exe';ExecutablePath=(Join-Path $root 'explorer.exe')})
  $script:contextCalls=0;$script:contextMismatchAt=0;$script:copyRacePath=$null;$script:copyRaceBytes=$null;$script:copyRaceDone=$false;$script:failCompleteWrite=$false
  return [pscustomobject]@{root=$root;workspace=$workspace;profile=$profile;application=$application;managed=$managed;stockPlugins=$stockPlugins;exe=$exe;context=$script:fixtureContext}
}
function New-AdoptedPredecessor($Fixture){
  $runs=Join-Path $Fixture.workspace 'runs';[void](New-Item -ItemType Directory -Path $runs -Force)
  $stamp=[datetime]::UtcNow.ToString('yyyyMMdd-HHmmss')
  $parent=Join-Path $runs ($stamp+'-read-only-commissioning-aaaaaaaa')
  $adoption=Join-Path $runs ($stamp+'-baseline-adoption-bbbbbbbb')
  [void](New-Item -ItemType Directory -Path $parent,$adoption -Force)
  $connection='0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0'
  $prefix="[Settings]`r`nLocale=sv`r`nLocaleOverride=sv_SE`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=$connection`r`n[Directories]`r`nChartDir=fixture`r`n[Canvas/CanvasConfig1]`r`ncanvasSizeY=800`r`n"
  $encoding=New-Object Text.UTF8Encoding($false,$true);$tail=21380-$encoding.GetByteCount($prefix)-3
  if($tail -lt 0){throw 'Predecessor fixture exceeds fixed profile bound.'}
  $text=$prefix+'#'+(' '*$tail)+"`r`n";$baseline=Join-Path $parent 'baseline.ini';$inputPath=Join-Path $parent 'input-only.ini'
  [IO.File]::WriteAllText($baseline,$text,$encoding);[IO.File]::WriteAllBytes($inputPath,(Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($baseline))))
  $baselineHash=Get-Digest $baseline;$inputHash=Get-Digest $inputPath;$script:CommissioningBaseline=$baselineHash
  $preparedPath=Join-Path $parent 'prepared.json'
  Write-Record $preparedPath @{owner=$script:CommissioningOwner;status='prepared';context=$Fixture.context;baselineSha256=$baselineHash;inputSha256=$inputHash}
  $preparedHash=Get-Digest $preparedPath
  $saved=Join-Path $adoption 'post-session.ini';$adoptedBaseline=Join-Path $adoption 'baseline.ini'
  Copy-PreparationFile $inputPath $saved $inputHash (Get-Item -LiteralPath $inputPath).Length
  [IO.File]::WriteAllBytes($adoptedBaseline,(Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($saved))))
  $inspectionPath=Join-Path $parent 'restore-inspection-fixture.json'
  Write-Record $inspectionPath @{owner='OpenNavX.ReadOnlyCommissioning.RestoreInspection.1';recordSha256=$preparedHash;currentIniSha256=$inputHash;savedIni=$saved}
  $reviewPath=Join-Path $adoption 'migration-review.json';$reviewed=[datetime]::UtcNow.AddSeconds(-1)
  Write-Record $reviewPath @{schema=1;owner='OpenNavX.ProfileMigrationReview.1';beforeSha256=$inputHash;afterSha256=$inputHash;reviewedUtc=$reviewed.ToString('o');parentPreparedSha256=$preparedHash;inspectionSha256=(Get-Digest $inspectionPath);changes=@()}
  $proposalPath=Join-Path $adoption 'proposal.json';$created=[datetime]::UtcNow
  Write-Record $proposalPath @{schema=1;owner=$script:CommissioningBaselineOwner;status='proposed';createdUtc=$created.ToString('o');parentPrepared=$preparedPath;parentPreparedSha256=$preparedHash;inspection=$inspectionPath;inspectionSha256=(Get-Digest $inspectionPath);currentIniSha256=$inputHash;reviewSha256=(Get-Digest $reviewPath);baselineSha256=(Get-Digest $adoptedBaseline);baselineBytes=(Get-Item -LiteralPath $adoptedBaseline).Length}
  $proposalHash=Get-Digest $proposalPath;$completePath=Join-Path $parent 'restore-completion-fixture.json'
  Write-Record $completePath @{owner=$script:CommissioningOwner;status='restored';recordSha256=$preparedHash;profileSha256=(Get-Digest $adoptedBaseline);adoptionProposalSha256=$proposalHash;pluginInventoryRestored=$true;originalOutputConfigurationRestored=$true}
  $record=Join-Path $adoption 'adopted-baseline.json'
  Write-Record $record @{schema=1;owner=$script:CommissioningBaselineOwner;status='adopted';parentPrepared=$preparedPath;parentPreparedSha256=$preparedHash;proposalSha256=$proposalHash;baselineSha256=(Get-Digest $adoptedBaseline);baselineBytes=(Get-Item -LiteralPath $adoptedBaseline).Length;restoreCompletion=$completePath;restoreCompletionSha256=(Get-Digest $completePath)}
  $recordHash=Get-Digest $record
  $resolved=& $script:actualBaselineReader $Fixture.workspace $record $recordHash
  if($resolved.sha256 -cne (Get-Digest $adoptedBaseline) -or $resolved.sid -cne $Fixture.context.sid -or $resolved.profile -ine $Fixture.context.profile){throw 'Temporary adopted predecessor did not pass the actual baseline reader.'}
  $script:seedRecord=$record;$script:seedRecordHash=$recordHash;$script:seedInfo=$resolved
  return $resolved
}
function Invoke-ColdCapture($Fixture,[string]$Action='Capture',$CaptureRecord='',[string]$CaptureHash='',[string]$Review='',[string]$ReviewHash=''){
  $args=@{Action=$Action;Workspace=$Fixture.workspace}
  if($Action -ceq 'Capture'){$args.PredecessorRecord=$script:seedRecord;$args.ExpectedPredecessorSha256=$script:seedRecordHash}
  else{$args.CaptureRecord=$CaptureRecord;$args.ExpectedCaptureSha256=$CaptureHash;$args.Review=$Review;$args.ExpectedReviewSha256=$ReviewHash}
  return (& $script:coldEntry @args | ConvertFrom-Json)
}
function New-ColdReviewForCapture([string]$Before,[string]$After,[string]$CaptureHash,[string]$PredecessorHash){
  return [pscustomobject]@{schema=1;owner=$script:ColdReviewOwner;captureSha256=$CaptureHash;predecessorRecordSha256=$PredecessorHash;
    beforeSha256=(Get-Digest $Before);afterSha256=(Get-Digest $After);reviewedUtc=[datetime]::UtcNow.ToString('o');
    provenance='pre-existing-current-user-state;origin-unverified';preservationOnly=$true;
    changes=@(Get-CommissioningIniDiff $Before $After|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';reason='Preserve this exact current user setting with its history left unverified.'}})}
}
function Initialize-ColdEntry {
  $entryPath=Join-Path $PSScriptRoot 'capture-cold-baseline.ps1'
  $entryText=[IO.File]::ReadAllText($entryPath)
  $import=". (Join-Path `$PSScriptRoot 'Commissioning.ps1')"
  if(-not $entryText.Contains($import)){throw 'Capture entrypoint import boundary changed; refusing to execute test body.'}
  $entryText=$entryText.Replace($import,'')
  $script:coldEntry=[scriptblock]::Create($entryText)
}
function Initialize-ColdLiveProfile($Fixture,$Predecessor){
  $source=Join-Path ([IO.Path]::GetDirectoryName($Predecessor.record)) 'baseline.ini'
  $text=[IO.File]::ReadAllText($source,$encoding)
  $text=$text.Replace('canvasSizeY=800','canvasSizeY=900')
  if($text -ceq [IO.File]::ReadAllText($source,$encoding)){throw 'Fixture profile did not contain the expected reviewed display key.'}
  $ini=Join-Path $Fixture.profile 'opencpn.ini';[IO.File]::WriteAllText($ini,$text,$encoding)
  $data=Join-Path $Fixture.profile 'user-data';[void](New-Item -ItemType Directory -Path $data)
  [IO.File]::WriteAllBytes((Join-Path $data 'opaque.dat'),[byte[]]@(0,1,2,3,250,255))
  return $ini
}
try {
  $encoding=New-Object Text.UTF8Encoding($false,$true)
  Initialize-ColdEntry
  $source=Join-Path $testRoot 'source';$copy=Join-Path $testRoot 'copy';$null=New-Item -ItemType Directory -Path $source
  $before=Join-Path $source 'before.ini';$after=Join-Path $source 'opencpn.ini'
  $base="[Settings]`r`nLocale=sv`r`nLocaleOverride=sv_SE`r`nConfigVersionString=Version 5.12.4+37fd0cd Build 2026-09-27`r`nPersistActiveRoute=0`r`n[Settings/NMEADataSource]`r`nDataConnections=0;0;;0;1;COM8;115200;0;1;0;;0;;0;0;1;0;1;Gateway;0;;0`r`n[Canvas/CanvasConfig1]`r`ncanvasSizeY=800`r`n[PlugIns/Dashboard]`r`nSumLogNM=120.50`r`n[Settings/AutoTrackRaymarine]`r`nPosX=100`r`n[Settings/GlobalState]`r`nOwnShipLatLon=`"   57.1000,   16.2000`"`r`n"
  $changed=$base.Replace('canvasSizeY=800','canvasSizeY=900').Replace('SumLogNM=120.50','SumLogNM=99.0').Replace('PosX=100','PosX=-100')
  $changed=$changed.Replace('[Canvas/CanvasConfig1]',"[OpenNav]`r`nInterfaceMode=legacy`r`n[Canvas/CanvasConfig1]")
  $font='Menu:1;10;-20;0;0;0;400;0;0;0;1;0;0;2;32;Segoe UI:rgb(0, 0, 0)'
  $changed+="[Settings/MSWFonts]`r`nsv-00c6075a=$font`r`n"
  [IO.File]::WriteAllText($before,$base,$encoding);[IO.File]::WriteAllText($after,$changed,$encoding)
  $review=MakeReview $before $after
  Pass 'Exact preservation review accepts bounded user display state, reset log and documented Menu descriptor' {
    $diff=@(Assert-ColdBaselineDelta $before $after $review)
    if($diff.Count -ne 5){throw 'Unexpected reviewed changed-key count.'}
  }
  Refuse 'Observed key does not approve itself' {$r=Clone $review;$r.changes=@($r.changes|Select-Object -Skip 1);Assert-ColdBaselineDelta $before $after $r}
  foreach($field in @('owner','beforeSha256','afterSha256','provenance','preservationOnly')) {
    Refuse "Changed review binding $field" {$r=Clone $review;$r.$field='wrong';Assert-ColdBaselineDelta $before $after $r}
  }
  foreach($field in @('origin','reason','after')) {
    Refuse "Changed per-key decision $field" {$r=Clone $review;if($field -ceq 'reason'){$r.changes[0].reason=''}else{$r.changes[0].$field='wrong'};Assert-ColdBaselineDelta $before $after $r}
  }
  foreach($date in @([datetime]::UtcNow.AddHours(-25),[datetime]::UtcNow.AddMinutes(2))) {
    Refuse 'Expired/future independent review' {$r=Clone $review;$r.reviewedUtc=$date.ToString('o');Assert-ColdBaselineDelta $before $after $r}
  }
  $bad=Join-Path $testRoot 'bad.ini'
  $cases=@(
    $changed.Replace('COM8;115200;0;1','COM9;115200;0;1'),
    ($changed+"[Directories]`r`nChartDir=other`r`n"),
    ($changed+"[OpenNav/Autopilot]`r`nControlEnabled=1`r`n"),
    $changed.Replace('canvasSizeY=900','canvasSizeY=999999'),
    $changed.Replace('InterfaceMode=legacy','InterfaceMode=pilot'),
    $changed.Replace('rgb(0, 0, 0)','rgb(999, 0, 0)'),
    $changed.Replace('sv-00c6075a='+$font,'sv-00c6075a=arbitrary'),
    ($changed+"[Settings]`r`nLocale=en`r`n")
  )
  foreach($case in $cases){Refuse 'Matching text review cannot allow unsafe or malformed current profile' {[IO.File]::WriteAllText($bad,$case,$encoding);$r=MakeReview $before $bad;Assert-ColdBaselineDelta $before $bad $r}}
  $tree=Get-PreparationTree $source
  Pass 'Cold copy has byte-identical complete inventory' {Copy-PreparationTree $tree $copy;$copied=Get-PreparationTree $copy;
    if(($tree.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($copied.entries|ConvertTo-Json -Depth 8 -Compress)){throw 'Copied tree differs'}}
  [IO.File]::AppendAllText($after,'#changed', $encoding)
  Refuse 'Changed source after first inventory cannot complete a cold copy' {Assert-PreparationTree $tree}

  # End-to-end transaction: the only substituted boundaries are machine
  # identity/process enumeration, inert stock executable digest and portable
  # private-evidence ACL inspection. Capture/copy/review/Complete/reader and
  # every profile/evidence SHA-256 operate on real disposable bytes.
  $fixture=New-ColdFixture 'happy';$prior=New-AdoptedPredecessor $fixture;$liveIni=Initialize-ColdLiveProfile $fixture $prior
  $originalBytes=[IO.File]::ReadAllBytes($liveIni);$originalHash=Get-Digest $liveIni
  $captured=Invoke-ColdCapture $fixture
  Pass 'Actual Capture preserves the full live profile and publishes review-required evidence' {
    if($captured.status -cne 'captured-review-required' -or $captured.profileChanged -ne $false -or $captured.applicationLaunched -ne $false -or
       (Get-Digest $liveIni) -cne $originalHash -or -not (Test-BytesEqual ([byte[]]$originalBytes) ([IO.File]::ReadAllBytes($liveIni)))){throw 'Capture changed live profile bytes or claimed authority.'}
    if((Get-Digest $captured.captureRecord) -cne $captured.captureSha256){throw 'Capture record hash did not bind actual published bytes.'}
    $record=Read-Record $captured.captureRecord;$backup=Join-Path ([IO.Path]::GetDirectoryName($captured.captureRecord)) 'profile-backup\opencpn.ini'
    if((Get-Digest $backup) -cne $record.profileSha256 -or (Get-PreparationTree (Join-Path ([IO.Path]::GetDirectoryName($captured.captureRecord)) 'profile-backup')).entries.Count -ne 3){throw 'Capture did not preserve the complete profile tree.'}
  }
  $captureDir=[IO.Path]::GetDirectoryName($captured.captureRecord);$predIni=Join-Path ([IO.Path]::GetDirectoryName($prior.record)) 'baseline.ini'
  $reviewPath=Join-Path $fixture.workspace 'independent-review.json'
  $reviewObj=New-ColdReviewForCapture $predIni $liveIni $captured.captureSha256 $prior.recordSha256
  Write-Record $reviewPath $reviewObj;$reviewHash=Get-Digest $reviewPath
  $completed=Invoke-ColdCapture $fixture 'Complete' $captured.captureRecord $captured.captureSha256 $reviewPath $reviewHash
  Pass 'Actual Complete and actual cold reader validate preservation-only lineage' {
    if($completed.status -cne 'completed-preservation-only' -or $completed.launchPermission -ne $false -or $completed.profileChanged -ne $false -or $completed.applicationLaunched -ne $false){throw 'Complete claimed more than preservation.'}
    $resolved=& $script:actualBaselineReader $fixture.workspace $completed.baselineRecord $completed.baselineRecordSha256
    if($resolved.sha256 -cne $originalHash -or $resolved.bytes -ne $originalBytes.Length -or $resolved.sid -cne $fixture.context.sid -or $resolved.profile -ine $fixture.profile){throw 'Actual reader did not validate the completed cold lineage.'}
    if((Get-Digest $liveIni) -cne $originalHash){throw 'Complete changed the live profile.'}
  }
  if($native) {
    # Resume through the unmodified production commissioning entrypoint. Only
    # its dependency import is removed so the disposable machine/context guard
    # above supplies paths; all inventory, plan, one-byte Apply and Restore code
    # runs as shipped. Empty plugin roots still require an explicit empty plan.
    $commissionPath=Join-Path $PSScriptRoot 'commission-read-only.ps1'
    $opaquePath=Join-Path $fixture.profile 'user-data\opaque.dat';$opaqueHash=Get-Digest $opaquePath
    $commissionText=[IO.File]::ReadAllText($commissionPath)
    $import=". (Join-Path `$PSScriptRoot 'Commissioning.ps1')"
    if(-not $commissionText.Contains($import)){throw 'Commissioning entrypoint import boundary changed.'}
    $commission=[scriptblock]::Create($commissionText.Replace($import,''))
    $baselineArgs=@{Workspace=$fixture.workspace;BaselineRecord=$completed.baselineRecord;
      ExpectedBaselineSha256=$completed.baselineRecordSha256}
    Refuse 'Cold lineage is never selected implicitly' {$null=& $commission -Action Inventory -Workspace $fixture.workspace}
    $inventoryResult=(& $commission -Action Inventory @baselineArgs)|ConvertFrom-Json
    $inventoryData=Read-Record $inventoryResult.record
    if(@($inventoryData.plugins).Count -ne 0){throw 'Fixture expected a complete empty plugin inventory.'}
    $planPath=Join-Path $fixture.workspace 'fresh-plugin-review.json'
    Write-Record $planPath @{schema=1;owner='OpenNavX.ReadOnlyCommissioning.Plan.1';inventoryPath=$inventoryResult.record;
      inventorySha256=$inventoryResult.recordSha256;reviewedUtc=[datetime]::UtcNow.ToString('o');plugins=@()}
    $preparedResult=(& $commission -Action Prepare @baselineArgs -Plan $planPath -ExpectedPlanSha256 (Get-Digest $planPath))|ConvertFrom-Json
    $preparedData=Read-Record $preparedResult.record
    if((Get-Digest $liveIni) -cne $originalHash -or $preparedData.baselineSha256 -cne $originalHash){throw 'Prepare changed or rebound the reviewed current baseline.'}
    $transactionArgs=@{Workspace=$fixture.workspace;Record=$preparedResult.record;ExpectedRecordSha256=$preparedResult.recordSha256}
    $applied=(& $commission -Action Apply @transactionArgs)|ConvertFrom-Json
    $expectedInput=Get-CommissioningHash (Get-CommissioningInputBytes ([byte[]]$originalBytes))
    if((Get-Digest $liveIni) -cne $expectedInput -or -not (Test-Path -LiteralPath (Join-Path $fixture.workspace 'commissioning-active.json'))){
      throw 'Apply did not publish exactly the reviewed one-byte input-only profile.'
    }
    $inspected=(& $commission -Action InspectRestore @transactionArgs)|ConvertFrom-Json
    $restored=(& $commission -Action Restore @transactionArgs -Inspection $inspected.inspection -ExpectedInspectionSha256 $inspected.inspectionSha256 -ReviewedCurrentIniSha256 $inspected.currentIniSha256)|ConvertFrom-Json
    if((Get-Digest $liveIni) -cne $originalHash -or (Get-Digest $opaquePath) -cne $opaqueHash -or
       -not (Test-BytesEqual ([byte[]]$originalBytes) ([IO.File]::ReadAllBytes($liveIni))) -or
       (Test-Path -LiteralPath (Join-Path $fixture.workspace 'commissioning-active.json')) -or
       $restored.applicationLaunched -ne $false -or $restored.doNotAutoLaunch -ne $true){
      throw 'Restoration failed to return every captured user byte without launch authority.'
    }
    $null=& $script:actualBaselineReader $fixture.workspace $completed.baselineRecord $completed.baselineRecordSha256
    $checks.Add('Native fresh Inventory/Prepare/Apply/InspectRestore/Restore preserves every captured user byte and requires explicit cold lineage')
    $evidenceFile=Join-Path $captureDir 'review.json'
    $evidenceAcl=Microsoft.PowerShell.Security\Get-Acl -LiteralPath $evidenceFile
    try {
      $tamperedAcl=Microsoft.PowerShell.Security\Get-Acl -LiteralPath $evidenceFile
      $everyone=New-Object Security.Principal.SecurityIdentifier('S-1-1-0')
      $readRule=New-Object Security.AccessControl.FileSystemAccessRule($everyone,'Read','Allow')
      $tamperedAcl.AddAccessRule($readRule)
      Set-Acl -LiteralPath $evidenceFile -AclObject $tamperedAcl
      Refuse 'Native cold reader refuses a foreign read ACE on private review evidence' {
        $null=& $script:actualBaselineReader $fixture.workspace $completed.baselineRecord $completed.baselineRecordSha256
      }
    } finally {Set-Acl -LiteralPath $evidenceFile -AclObject $evidenceAcl}
    $null=& $script:actualBaselineReader $fixture.workspace $completed.baselineRecord $completed.baselineRecordSha256
    $checks.Add('Native private evidence ACL restoration returns cold reader to valid lineage')
  }
  $completedBytes=[IO.File]::ReadAllBytes($completed.baselineRecord);$tampered=Read-Record $completed.baselineRecord;$tampered.status='proposed'
  [IO.File]::WriteAllText($completed.baselineRecord,($tampered|ConvertTo-Json -Depth 16),[Text.UTF8Encoding]::new($false))
  Refuse 'Actual reader rejects altered completion status despite a fresh supplied hash' {$null=& $script:actualBaselineReader $fixture.workspace $completed.baselineRecord (Get-Digest $completed.baselineRecord)}
  [IO.File]::WriteAllBytes($completed.baselineRecord,$completedBytes)

  $active=New-ColdFixture 'active';$activePrior=New-AdoptedPredecessor $active;$activeIni=Initialize-ColdLiveProfile $active $activePrior;$activeBytes=[IO.File]::ReadAllBytes($activeIni)
  [IO.File]::WriteAllText((Join-Path $active.workspace 'commissioning-active.json'),'active')
  Refuse 'Capture refuses an active commissioning transaction without changing profile bytes' {$null=Invoke-ColdCapture $active}
  if(-not (Test-BytesEqual ([byte[]]$activeBytes) ([IO.File]::ReadAllBytes($activeIni)))){throw 'Active-session refusal changed fixture profile bytes.'}

  $process=New-ColdFixture 'process';$processPrior=New-AdoptedPredecessor $process;$processIni=Initialize-ColdLiveProfile $process $processPrior;$processBytes=[IO.File]::ReadAllBytes($processIni)
  $script:fixtureProcesses=@([pscustomobject]@{Name='opencpn.exe';ExecutablePath=(Join-Path $process.application 'opencpn.exe')})
  Refuse 'Capture refuses a live application process without changing profile bytes' {$null=Invoke-ColdCapture $process}
  if(-not (Test-BytesEqual ([byte[]]$processBytes) ([IO.File]::ReadAllBytes($processIni)))){throw 'Live-process refusal changed fixture profile bytes.'}

  $contextRace=New-ColdFixture 'context';$contextPrior=New-AdoptedPredecessor $contextRace;$contextIni=Initialize-ColdLiveProfile $contextRace $contextPrior;$contextBytes=[IO.File]::ReadAllBytes($contextIni)
  $script:contextMismatchAt=2
  Refuse 'Capture refuses identity/profile context drift before publication' {$null=Invoke-ColdCapture $contextRace}
  if(-not (Test-BytesEqual ([byte[]]$contextBytes) ([IO.File]::ReadAllBytes($contextIni)))){throw 'Context-race refusal changed fixture profile bytes.'}

  $race=New-ColdFixture 'copy-race';$racePrior=New-AdoptedPredecessor $race;$raceIni=Initialize-ColdLiveProfile $race $racePrior;$racedBytes=[IO.File]::ReadAllBytes($raceIni);$racedBytes+= [byte[]]@(35,114,97,99,101,100)
  $script:copyRacePath=$raceIni;$script:copyRaceBytes=$racedBytes
  Refuse 'Capture refuses a profile mutation between inventory and copy publication' {$null=Invoke-ColdCapture $race}
  if(-not $script:copyRaceDone -or -not (Test-BytesEqual ([byte[]]$racedBytes) ([IO.File]::ReadAllBytes($raceIni)))){throw 'Race fixture was not preserved exactly.'}
  $raceRuns=Join-Path $race.workspace 'runs';$partial=@(Get-ChildItem -LiteralPath $raceRuns -Directory -Filter '*-cold-baseline-*' -ErrorAction SilentlyContinue)
  if(@($partial|Where-Object{Test-Path -LiteralPath (Join-Path $_.FullName 'capture.json')}).Count -ne 0){throw 'Raced capture incorrectly published capture.json.'}

  $changed=New-ColdFixture 'changed-before-complete';$changedPrior=New-AdoptedPredecessor $changed;$changedIni=Initialize-ColdLiveProfile $changed $changedPrior
  $changedCapture=Invoke-ColdCapture $changed;$changedDir=[IO.Path]::GetDirectoryName($changedCapture.captureRecord);$changedPred=Join-Path ([IO.Path]::GetDirectoryName($changedPrior.record)) 'baseline.ini'
  $changedReview=Join-Path $changed.workspace 'review.json';Write-Record $changedReview (New-ColdReviewForCapture $changedPred $changedIni $changedCapture.captureSha256 $changedPrior.recordSha256)
  $changedBytes=[IO.File]::ReadAllBytes($changedIni);[IO.File]::AppendAllText($changedIni,'# external change')
  $postMutation=[IO.File]::ReadAllBytes($changedIni);$changedReviewHash=Get-Digest $changedReview
  Refuse 'Complete refuses changed live profile and leaves those bytes untouched' {$null=Invoke-ColdCapture $changed 'Complete' $changedCapture.captureRecord $changedCapture.captureSha256 $changedReview $changedReviewHash}
  if(-not (Test-BytesEqual ([byte[]]$postMutation) ([IO.File]::ReadAllBytes($changedIni))) -or (Test-Path -LiteralPath (Join-Path $changedDir 'completed-cold-baseline.json'))){throw 'Complete refusal overwrote the changed profile or published completion.'}

  $partialComplete=New-ColdFixture 'partial-complete';$partialPrior=New-AdoptedPredecessor $partialComplete;$partialIni=Initialize-ColdLiveProfile $partialComplete $partialPrior
  $partialCapture=Invoke-ColdCapture $partialComplete;$partialDir=[IO.Path]::GetDirectoryName($partialCapture.captureRecord);$partialPred=Join-Path ([IO.Path]::GetDirectoryName($partialPrior.record)) 'baseline.ini'
  $partialReview=Join-Path $partialComplete.workspace 'review.json';Write-Record $partialReview (New-ColdReviewForCapture $partialPred $partialIni $partialCapture.captureSha256 $partialPrior.recordSha256)
  $partialHash=Get-Digest $partialReview;$partialLiveHash=Get-Digest $partialIni;$script:failCompleteWrite=$true
  Refuse 'Failed final publication leaves no completed record and preserves the live profile' {$null=Invoke-ColdCapture $partialComplete 'Complete' $partialCapture.captureRecord $partialCapture.captureSha256 $partialReview $partialHash}
  if((Test-Path -LiteralPath (Join-Path $partialDir 'completed-cold-baseline.json')) -or (Get-Digest $partialIni) -cne $partialLiveHash -or -not (Test-Path -LiteralPath (Join-Path $partialDir 'review.json'))){throw 'Partial publication state was not retained safely.'}
  $script:failCompleteWrite=$false
  Refuse 'Retry after partial publication refuses existing review evidence and preserves live bytes' {$null=Invoke-ColdCapture $partialComplete 'Complete' $partialCapture.captureRecord $partialCapture.captureSha256 $partialReview $partialHash}
  if((Test-Path -LiteralPath (Join-Path $partialDir 'completed-cold-baseline.json')) -or (Get-Digest $partialIni) -cne $partialLiveHash){throw 'Retry overwrote partial evidence or live profile.'}
  foreach($file in @('ColdBaseline.ps1','CommissioningBaseline.ps1','capture-cold-baseline.ps1','commission-read-only.ps1')) {
    Pass "Parses $file" {$tokens=$null;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $file),[ref]$tokens,[ref]$errors);if($errors.Count){throw ($errors|Out-String)}}
  }
  $substitutions=@('machine/profile context identity','process enumeration','official-stock identity for one exact inert fixture executable')
  if(-not $native){$substitutions+=@('Windows ACL ownership/protection (portable marker and reparse checks only)')}
  [pscustomobject]@{status='passed';platform=if($native){'native-disposable'}else{'portable'};groups=$checks.Count;checks=$checks.ToArray();realProfileAccess=$false;applicationLaunched=$false;substitutedBoundaries=$substitutions;nativeAclExecuted=$native} | ConvertTo-Json -Depth 5
} finally {Remove-Item -LiteralPath $testRoot -Recurse -Force -ErrorAction SilentlyContinue}
