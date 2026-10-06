# Inert local proof/lineage tests. No application, task, port, boat or profile access.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
if([Environment]::OSVersion.Platform -ne 'Win32NT' -and -not $PortableContracts){throw 'Explicit portable contracts required.'}
if([Environment]::OSVersion.Platform -eq 'Win32NT' -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable CI or explicit isolated fixture mode required.'}
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('manual-preservation-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory $fixture
function Assert-LocalPath([string]$Path){$p=[IO.Path]::GetFullPath($Path);if(-not $p.StartsWith($fixture+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal)){throw 'Escaped fixture root'};return $p}
function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid){$null=Assert-LocalPath $Directory;if($Sid -cne 'S-1-5-21-123'){throw 'Wrong fixture SID'}}
$checks=0
$originalBaseline=$script:CommissioningBaseline
function Pass([scriptblock]$Call){$null=& $Call;$script:checks++}
function Refuse([scriptblock]$Call){$refused=$false;try{$null=& $Call}catch{$refused=$true};if(-not $refused){throw 'Invalid proof accepted'};$script:checks++}
function Same($Actual,$Expected){if($Actual -cne $Expected){throw 'Fixture equality failed'}}
$encoding=New-Object Text.UTF8Encoding($false,$true)
function SetJson([string]$Path,$Value){[IO.File]::WriteAllText($Path,($Value|ConvertTo-Json -Depth 20),$encoding)}
function Review([string]$Before,[string]$After){return [pscustomobject]@{schema=1;owner='OpenNavX.SessionPreservationReview.1';beforeSha256=(Get-Digest $Before);afterSha256=(Get-Digest $After);reviewedUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';preservationOnly=$true;launchPermission=$false;changes=@(Get-CommissioningIniDiff $Before $After|ForEach-Object{[pscustomobject]@{key=$_.key;before=$_.before;after=$_.after;origin='unverified';decision='preserve-current';reason='Explicit inert fixture review'}})}}
function MakeChild([string]$AfterAlpha=$bound,[scriptblock]$ChangeRecord={},[scriptblock]$ChangeComplete={}) {
  $input=Join-Path $child 'input.ini';$output=Join-Path $child 'output.ini';$rollback=Join-Path $child 'rollback.ini'
  [IO.File]::WriteAllText($input,$text,$encoding)
  [IO.File]::WriteAllBytes($output,(Get-CommissioningOutputBytes $encoding.GetBytes($text)))
  $afterText=$text.Replace($alpha,$AfterAlpha)
  [IO.File]::WriteAllText($rollback,$afterText,$encoding);[IO.File]::WriteAllText($current,$afterText,$encoding)
  $saved=Join-Path $child ('inspection-'+('d'*32)+'.ini')
  [IO.File]::WriteAllBytes($saved,(Get-CommissioningOutputBytes $encoding.GetBytes($afterText)))
  $r=[pscustomobject]@{schema=1;owner='OpenNavX.ManualPilotCommissioning.1';status='prepared';context=$context;
    candidate=[pscustomobject]@{generation=('a'*32);commit=('b'*40);ownershipSha256=('c'*64);executableSha256=('e'*64);packageSha256=('f'*64)};
    parentRecord=$parentRecord;parentDirectory=$parent;parentActiveSha256=(Get-Digest $active);
    inputSha256=(Get-Digest $input);outputSha256=(Get-Digest $output);
    proofFiles=@(@{path=$parentRecord;sha256=$parentHash},@{path=$input;sha256=(Get-Digest $input)},@{path=$output;sha256=(Get-Digest $output)})}
  & $ChangeRecord $r
  SetJson (Join-Path $child 'prepared.json') $r;$rh=Get-Digest (Join-Path $child 'prepared.json')
  SetJson $childInspection @{owner=$r.owner;recordSha256=$rh;currentIniSha256=(Get-Digest $saved);savedIni=$saved}
  SetJson (Join-Path $child 'rollback-intent.json') @{owner=$r.owner;recordSha256=$rh;inspectionSha256=(Get-Digest $childInspection);beforeSha256=(Get-Digest $saved);afterSha256=(Get-Digest $rollback);pluginsRestored=$false}
  $c=[pscustomobject]@{owner=$r.owner;recordSha256=$rh;profileSha256=(Get-Digest $rollback);pluginsRestored=$false;parentRemainsActive=$true;launchPermission=$false}
  & $ChangeComplete $c
  SetJson $completion $c
  return [pscustomobject]@{completion=$completion;completionSha256=(Get-Digest $completion);inspection=$childInspection;inspectionSha256=(Get-Digest $childInspection)}
}
function Proof($Selection,[switch]$Live){return Read-ManualChildPreservationProof $workspace $parentRecord $parentHash $current $Selection -Live:$Live}
try {
  foreach($name in @('CommissioningBaseline.ps1','prepare-session-preservation.ps1')) {
    $errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $PSScriptRoot $name),[ref]$null,[ref]$errors)
    Pass {if($errors){throw ($errors|Out-String)}}
  }
  $workspace=Join-Path $fixture 'workspace';$parent=Join-Path $workspace 'runs/20261006-120000-read-only-commissioning-aaaaaaaa'
  $child=Join-Path $workspace 'runs/20261006-130000-manual-pilot-bbbbbbbb';$profile=Join-Path $fixture 'profile'
  $proposalDir=Join-Path $workspace 'runs/20261006-140000-session-preservation-cccccccc'
  $null=New-Item -ItemType Directory $parent,$child,$profile,$proposalDir -Force
  $current=Join-Path $profile 'opencpn.ini';$parentRecord=Join-Path $parent 'prepared.json';$active=Join-Path $workspace 'commissioning-active.json'
  $completion=Join-Path $child 'rolled-back.json';$childInspection=Join-Path $child ('inspection-'+('d'*32)+'.json')
  $context=[pscustomobject]@{workspace=$workspace;profile=$profile;sid='S-1-5-21-123';session=1;
    installation=[pscustomobject]@{generation=(Join-Path $fixture ('generation/'+('a'*32)));commit=('b'*40);ownershipSha256=('c'*64);executableSha256=('e'*64)}}
  # Opaque calibration/source bytes must survive even though the base parser
  # does not accept them. No settings are decoded and rewritten for publication.
  $alpha='OpenNavXSettings 1\n"capacity" "20"\n"reserve" "20"\n"draft" "1"\n"model_source" "opaque provenance"\n"curve" "opaque calibration"\n'
  $bound=$alpha+'"pilot.interface" "COM8"\n"pilot.name" "c0508700e76004d2"\n"pilot.permission" "display-only"\n'
  $connection='0;0;;0;1;COM8;115200;0;0;0;;0;;0;0;1;0;1;Gateway;0;;0'
  $text="[Settings]`r`nPersistActiveRoute=0`r`nActiveRoute=12345678-90AB-cdef-1234-567890abcdef`r`n[Settings/NMEADataSource]`r`nDataConnections=$connection`r`n[OpenNav]`r`nAlphaSettings=$alpha`r`n"
  $text+=';'+('x'*(21380-$encoding.GetByteCount($text)-3))+"`r`n"
  $baseline=Join-Path $parent 'baseline.ini';[IO.File]::WriteAllBytes($baseline,(Get-CommissioningOutputBytes $encoding.GetBytes($text)));$script:CommissioningBaseline=Get-Digest $baseline
  $parentInput=Join-Path $parent 'input-only.ini';[IO.File]::WriteAllText($parentInput,$text,$encoding)
  SetJson (Join-Path $parent 'review-plan.json') @{fixture='independent parent review'}
  SetJson $parentRecord @{schema=1;owner=$script:CommissioningOwner;status='prepared';context=$context;baselineSha256=$script:CommissioningBaseline;inputSha256=(Get-Digest $parentInput);planSha256=(Get-Digest (Join-Path $parent 'review-plan.json'));evidence=@();quarantine=@()}
  $parentHash=Get-Digest $parentRecord
  SetJson $active @{owner=$script:CommissioningOwner;record=$parentRecord;recordSha256=$parentHash}
  $selection=MakeChild
  Pass {$script:manual=Proof $selection -Live;Same $manual.beforeAlpha $alpha;Same $manual.afterAlpha $bound}
  $review=Review $parentInput $current
  Refuse {Assert-SessionPreservationReview $parentInput $current $review}
  Pass {Same @(Assert-SessionPreservationReview $parentInput $current $review ([datetime]::UtcNow) '' $null $manual).Count 1}
  foreach($bad in @($bound.Replace('display-only','manual'),$bound.Replace('COM8','COM9'),$bound.Replace('c0508700e76004d2','0000000000000000'),$bound.Replace('opaque calibration','modified'),($bound+'"pilot.name" "c0508700e76004d2"\n'),($bound+'"pilot.extra" "x"\n'))) {
    $selection=MakeChild $bad;Refuse {Proof $selection}
  }
  foreach($change in @({param($r)$r.parentRecord=Join-Path $parent 'other.json'}, {param($r)$r.proofFiles[0].sha256='0'*64}, {param($r)$r.candidate.generation='f'*32}, {param($r)$r.candidate.ownershipSha256='0'*64}, {param($r)$r.parentActiveSha256='0'*64})) {
    $selection=MakeChild $bound $change;Refuse {Proof $selection -Live}
  }
  foreach($flag in @('pluginsRestored','parentRemainsActive','launchPermission')){
    $selection=MakeChild $bound {} {param($c)$c.$flag=-not $c.$flag};Refuse {Proof $selection}
  }
  # Keep a real inert mixed-case GUID through the complete historical lineage.
  # Rebinding every child hash must not admit a changed or cleared stored GUID.
  foreach($nextGuid in @('aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee','')) {
    $selection=MakeChild
    $rollback=Join-Path $child 'rollback.ini';$saved=Join-Path $child ('inspection-'+('d'*32)+'.ini')
    foreach($path in @($rollback,$saved,$current)){[IO.File]::WriteAllText($path,([IO.File]::ReadAllText($path).Replace('12345678-90AB-cdef-1234-567890abcdef',$nextGuid)),$encoding)}
    $i=Read-Record $childInspection;$i.currentIniSha256=Get-Digest $saved;SetJson $childInspection $i
    $intentPath=Join-Path $child 'rollback-intent.json';$intent=Read-Record $intentPath
    $intent.inspectionSha256=Get-Digest $childInspection;$intent.beforeSha256=Get-Digest $saved;$intent.afterSha256=Get-Digest $rollback;SetJson $intentPath $intent
    $c=Read-Record $completion;$c.profileSha256=Get-Digest $rollback;SetJson $completion $c
    $selection.inspectionSha256=Get-Digest $childInspection;$selection.completionSha256=Get-Digest $completion
    Refuse {Proof $selection}
  }
  foreach($value in @('some-route','{12345678-90ab-cdef-1234-567890abcdef}')){Refuse {Assert-ManualInactiveRouteSettings @{'Settings/PersistActiveRoute'='0';'Settings/ActiveRoute'=$value}}}
  Refuse {Assert-ManualInactiveRouteSettings @{'Settings/PersistActiveRoute'='1';'Settings/ActiveRoute'='12345678-90ab-cdef-1234-567890abcdef'}}
  $selection=MakeChild
  $marker=Join-Path $workspace 'manual-pilot-active.json';SetJson $marker @{fixture='still active'};Refuse {Proof $selection -Live};Remove-Item $marker
  $badSelection=$selection|ConvertTo-Json|ConvertFrom-Json;$badSelection.completionSha256='0'*64;Refuse {Proof $badSelection}
  $badSelection=$selection|ConvertTo-Json|ConvertFrom-Json;$badSelection.inspectionSha256='0'*64;Refuse {Proof $badSelection}
  foreach($name in @('prepared.json','input.ini','output.ini','rollback.ini','rollback-intent.json',('inspection-'+('d'*32)+'.ini'),'rolled-back.json')) {
    $path=Join-Path $child $name;$bytes=[IO.File]::ReadAllBytes($path);[IO.File]::WriteAllText($path,'tampered');Refuse {Proof $selection};[IO.File]::WriteAllBytes($path,$bytes)
  }
  $oversized=Join-Path $child 'input.ini';$bytes=[IO.File]::ReadAllBytes($oversized)
  $stream=[IO.File]::OpenWrite($oversized);try{$stream.SetLength(4194305)}finally{$stream.Dispose()}
  Refuse {Proof $selection};[IO.File]::WriteAllBytes($oversized,$bytes)
  # Construct the real immutable parent proposal and reread through production
  # validation, including backup and exact independent per-key review.
  $review=Review $parentInput $current;$snapshot=Get-PreparationTree $profile
  $parentSaved=Join-Path $parent 'post-session-fixture.ini';Copy-PreparationFile $current $parentSaved (Get-Digest $current) (Get-Item $current).Length
  $parentInspection=Join-Path $parent 'restore-inspection-fixture.json'
  SetJson $parentInspection @{owner='OpenNavX.ReadOnlyCommissioning.RestoreInspection.1';recordSha256=$parentHash;context=$context;currentIniSha256=(Get-Digest $current);savedIni=$parentSaved;profileBeforeRestore=$snapshot;resourceProof=$null}
  $review|Add-Member parentPreparedSha256 $parentHash;$review|Add-Member inspectionSha256 (Get-Digest $parentInspection)
  $reviewPath=Join-Path $proposalDir 'preservation-review.json';SetJson $reviewPath $review
  Copy-PreparationFile $current (Join-Path $proposalDir 'post-session.ini') (Get-Digest $current) (Get-Item $current).Length
  Copy-PreparationTree $snapshot (Join-Path $proposalDir 'profile-backup')
  $target=Join-Path $proposalDir 'baseline.ini';[IO.File]::WriteAllBytes($target,(Get-CommissioningOutputBytes ([IO.File]::ReadAllBytes($current))))
  $proposal=Join-Path $proposalDir 'proposal.json'
  $value=@{schema=1;owner=$script:SessionPreservationOwner;status='proposed';parentPrepared=$parentRecord;parentPreparedSha256=$parentHash;inspection=$parentInspection;inspectionSha256=(Get-Digest $parentInspection);currentIniSha256=(Get-Digest $current);reviewSha256=(Get-Digest $reviewPath);baselineSha256=(Get-Digest $target);baselineBytes=(Get-Item $target).Length;createdUtc=[datetime]::UtcNow.ToString('o');provenance='current-user-state;origin-unverified';launchPermission=$false;changedKeys=@($review.changes).Count;manualChild=$selection}
  SetJson $proposal $value;$proposalHash=Get-Digest $proposal
  Pass {Read-SessionPreservationProposal $workspace $proposal $proposalHash $parentRecord $parentHash}
  # Historical validation requires preserved evidence, never a current old
  # generation, live profile, active marker or executable/toolchain probe.
  Remove-Item $active
  Pass {Read-SessionPreservationProposal $workspace $proposal $proposalHash $parentRecord $parentHash}
  Refuse {Proof $selection -Live}
  $intent=Join-Path $child 'rollback-intent.json';$bytes=[IO.File]::ReadAllBytes($intent)
  $bad=Read-Record $intent;$bad.afterSha256='0'*64;SetJson $intent $bad
  Refuse {Read-SessionPreservationProposal $workspace $proposal $proposalHash $parentRecord $parentHash};[IO.File]::WriteAllBytes($intent,$bytes)
  $restored=Join-Path $parent 'restored-fixture.json'
  SetJson $restored @{owner=$script:CommissioningOwner;status='restored';recordSha256=$parentHash;preservationProposalSha256=$proposalHash;profileSha256=(Get-Digest $target);pluginInventoryRestored=$true;otherProfileFilesPreserved=$true;originalOutputConfigurationRestored=$true;doNotAutoLaunch=$true;applicationLaunched=$false}
  $lineage=Join-Path $proposalDir 'preserved-baseline.json'
  SetJson $lineage @{schema=1;owner=$script:SessionPreservationOwner;status='preserved';parentPrepared=$parentRecord;parentPreparedSha256=$parentHash;proposalSha256=$proposalHash;baselineSha256=(Get-Digest $target);baselineBytes=(Get-Item $target).Length;restoreCompletion=$restored;restoreCompletionSha256=(Get-Digest $restored);provenance='current-user-state;origin-unverified';launchPermission=$false}
  $context.installation.generation=Join-Path $fixture 'retired-original-generation'
  function Get-Installed {throw 'Historical proof must not inspect a live installation'}
  Pass {Same (Read-CommissioningBaseline $workspace $lineage (Get-Digest $lineage)).sha256 (Get-Digest $target)}
  # A historical proposal still cannot substitute another same-profile child.
  $bad=Read-Record (Join-Path $child 'prepared.json');$bad.parentDirectory=Join-Path $workspace 'other-parent'
  $path=Join-Path $child 'prepared.json';$bytes=[IO.File]::ReadAllBytes($path);SetJson $path $bad
  Refuse {Read-CommissioningBaseline $workspace $lineage (Get-Digest $lineage)};[IO.File]::WriteAllBytes($path,$bytes)
  $value.Remove('manualChild');SetJson $proposal $value
  Refuse {Read-SessionPreservationProposal $workspace $proposal (Get-Digest $proposal) $parentRecord $parentHash}
  [pscustomobject]@{status='passed';checks=$checks;scope='Inert explicit manual-child proof, parent preservation and historical proposal reread; no native identity/ACL, launch, port or boat qualification'}|ConvertTo-Json
}finally{$script:CommissioningBaseline=$originalBaseline;Remove-Item -LiteralPath $fixture -Recurse -Force}
