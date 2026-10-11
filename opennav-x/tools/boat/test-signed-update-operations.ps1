# Pure inert operation-state checks; no artifact/process/launch authority.
. (Join-Path $PSScriptRoot 'SignedUpdateOperations.ps1')
$checks=0
function Check([bool]$Ok,[string]$Why){if(-not $Ok){throw $Why};$script:checks++}
function Clone($x){Copy-SignedUpdateValue $x}
function Hash([int]$n){$n.ToString('x64')}
function Reject($s,[scriptblock]$Call,[string]$Why){$before=Get-SignedUpdateReceiptHash $s;$failed=$false;try{$null=& $Call}catch{$failed=$true};Check $failed $Why;Check ((Get-SignedUpdateReceiptHash $s) -ceq $before) 'Refusal cannot mutate state'}
$old=[pscustomobject]@{generation=('a'*32);commit=('a'*40);packageSha256=(Hash 2);executableSha256=(Hash 3)}
$new=[pscustomobject]@{generation=('b'*32);commit=('b'*40);packageSha256=(Hash 4);executableSha256=(Hash 5)}
$binding=[pscustomobject]@{session=(Hash 1);previous=$old;release=[pscustomobject]@{commit=$new.commit;packageSha256=$new.packageSha256;executableSha256=$new.executableSha256;policySha256=(Hash 6);sourceReviewSha256=(Hash 7)};previousCommissioningSha256=(Hash 8);fallbackSourceReviewSha256=(Hash 9);createdUnix=1000;expiresUnix=4000}
function Restore($identity,$prepared,[int]$n){[pscustomobject]@{identity=$identity;stateSha256=(Hash ($n+1));ownershipSha256=(Hash ($n+2));preparedSha256=$prepared;restoreCompletionSha256=(Hash ($n+3));profileSha256=(Hash ($n+4));treesSha256=(Hash ($n+5));inspectionSha256=(Hash ($n+6))}}
function Commission($identity,$baseline,$source,[int]$n){[pscustomobject]@{identity=$identity;stateSha256=(Hash ($n+1));ownershipSha256=(Hash ($n+2));preparedSha256=(Hash ($n+3));appliedSha256=(Hash ($n+4));baselineSha256=$baseline;profileSha256=(Hash ($n+5));treesSha256=(Hash ($n+6));sourceReviewSha256=$source;independentAuditSha256=(Hash ($n+7));approvalSha256=(Hash ($n+8));environmentSha256=(Hash ($n+9));reviewedUnix=1000}}
$first=Restore $old $binding.previousCommissioningSha256 10
$candidate=Commission $new $first.profileSha256 $binding.release.sourceReviewSha256 20
$restored=Restore $new $candidate.preparedSha256 30
$fallback=Commission $old $restored.profileSha256 $binding.fallbackSourceReviewSha256 40
$prep=[pscustomobject]@{identity=$new;stateSha256=$candidate.stateSha256;ownershipSha256=$candidate.ownershipSha256;preparedSha256=$candidate.preparedSha256;transactionDirectory='C:\private\runs\candidate';contextSha256=(Hash 51);planSha256=(Hash 52);inventorySha256=(Hash 53);quarantineSha256=(Hash 54);baselineSha256=$first.profileSha256;sourceReviewSha256=$binding.release.sourceReviewSha256}
function Request($s,$action,$e){[pscustomobject]@{session=$s.session.binding.session;sequence=@($s.events).Count+1;action=$action;requestNonce=(Hash (500+@($s.events).Count));previousEventSha256=$s.headSha256;evidence=(Clone $e)}}
function Advance($s,$action,$e){Add-SignedUpdateOperationEvent $s (Request $s $action $e) (1001+@($s.events).Count)}
function Prepared {
 $s=New-SignedUpdateOperationSession $binding 1000
 $null=Advance $s PreviousRestored $first;$null=Advance $s CandidatePreparationArmed $prep
 return $s
}
function Denial($s){[pscustomobject]@{preparationEventSha256=$s.events[1].eventSha256;denialSha256=(Hash 60)}}
function Intent($s){[pscustomobject]@{identity=$new;stateSha256=$prep.stateSha256;ownershipSha256=$prep.ownershipSha256;preparedSha256=$prep.preparedSha256;transactionDirectory=$prep.transactionDirectory;preparationEventSha256=$s.events[1].eventSha256;denialEventSha256=$s.events[2].eventSha256;inspectionSha256=(Hash 61);currentProfileSha256=(Hash 62);targetProfileSha256=$prep.baselineSha256;targetTreesSha256=(Hash 63);preservationSha256=$null}}
function Completion($s){$i=$s.events[-1];[pscustomobject]@{identity=$new;stateSha256=$prep.stateSha256;ownershipSha256=$prep.ownershipSha256;preparedSha256=$prep.preparedSha256;transactionDirectory=$prep.transactionDirectory;restoreIntentEventSha256=$i.eventSha256;inspectionSha256=$i.evidence.inspectionSha256;restoreCompletionSha256=(Hash 64);profileSha256=$i.evidence.targetProfileSha256;treesSha256=$i.evidence.targetTreesSha256;activeMarkerAbsent=$true}}
function Spawn($s){$c=$s.session.receipts[-1];[pscustomobject]@{commissionReceiptSha256=$c.receiptSha256;creationRequestSha256=(Hash (700+@($s.events).Count));environmentSha256=$c.evidence.environmentSha256}}
$s=New-SignedUpdateOperationSession $binding 1000
Reject $s {Advance $s CandidatePreparationArmed $prep} 'Cannot prepare before receipt1'
$null=Advance $s PreviousRestored $first
Reject $s {Advance $s CandidateRestored $restored} 'Missing receipt2 alone cannot authorize restoration'
Reject $s {Advance $s CandidateCommissioned $candidate} 'Operation mode requires durable preparation before Apply/commissioning'
foreach($field in @('commit','packageSha256','executableSha256','generation')){
 $bad=Clone $prep;$bad.identity.$field=$old.$field
 Reject $s {Advance $s CandidatePreparationArmed $bad} ('Substituted candidate '+$field)
}
foreach($path in @('relative','\\server\share','C:\private\..\runs','C:\private\\runs','C:\private\runs.')){
 $bad=Clone $prep;$bad.transactionDirectory=$path
 Reject $s {Advance $s CandidatePreparationArmed $bad} 'Noncanonical transaction directory refused'
}
# Device aliases remain reserved in parent components and with extensions;
# construct Unicode explicitly so native Windows PowerShell 5.1 reads ASCII.
$devices=@('CON','nul.txt','Aux','prn.json','COM1','com9.bin','LPT1','lpt9.log','CON .txt')
foreach($digit in @(0x00b9,0x00b2,0x00b3)){$devices+=@(('COM'+[char]$digit),('lpt'+[char]$digit+'.json'))}
foreach($device in $devices){
 $bad=Clone $prep;$bad.transactionDirectory='C:\private\'+$device+'\candidate'
 Reject $s {Advance $s CandidatePreparationArmed $bad} ('Reserved Windows device component '+$device)
}
foreach($component in @('CONSOLE','COM10','LPT10','candidate.json')){
 $copy=Clone $s;$valid=Clone $prep;$valid.transactionDirectory='C:\private\'+$component+'\candidate'
 $null=Advance $copy CandidatePreparationArmed $valid
 Check ($copy.events[-1].evidence.transactionDirectory -ceq $valid.transactionDirectory) 'Ordinary device-prefix directory stays valid'
}
$s=Prepared
foreach($field in @('session','previousEventSha256','requestNonce')){
 $r=Request $s DeniedBeforeLaunch (Denial $s)
 $r.$field=if($field -ceq 'requestNonce'){$s.events[0].requestNonce}else{Hash 999}
 Reject $s {Add-SignedUpdateOperationEvent $s $r 1003} ('Stale/replayed '+$field)
}
$r=Request $s DeniedBeforeLaunch (Denial $s)
Reject $s {Add-SignedUpdateOperationEvent $s $r 1000} 'Time cannot reverse'
Reject $s {Add-SignedUpdateOperationEvent $s $r 4000} 'Expired owner cannot deny/restore'
$r|Add-Member -NotePropertyName neverLaunched -NotePropertyValue $true
Reject $s {Add-SignedUpdateOperationEvent $s $r 1003} 'Caller no-launch boolean is not authority'
# No applied/commissioned receipt is invented for interrupted/partial Apply.
$denied=Advance $s DeniedBeforeLaunch (Denial $s)
Reject $s {Advance $s CandidateCommissioned $candidate} 'Denied preparation cannot resume candidate approval'
Reject $s {Advance $s CandidateSpawnConsumed (Spawn $s)} 'Denied preparation cannot consume a spawn'
$intent=Intent $s
foreach($field in @('preparedSha256','ownershipSha256','stateSha256','denialEventSha256','preparationEventSha256','targetProfileSha256')){
 $bad=Clone $intent;$bad.$field=Hash 999
 Reject $s {Advance $s DeniedRestoreIntent $bad} ('Wrong restoration binding '+$field)
}
$bad=Clone $intent;$bad.transactionDirectory='C:\private\runs\other'
Reject $s {Advance $s DeniedRestoreIntent $bad} 'Transaction path cannot transfer'
$null=Advance $s DeniedRestoreIntent $intent
Reject $s {Advance $s DeniedRestoreIntent $intent} 'Lost intent response cannot retry or change target'
$complete=Completion $s
foreach($field in @('profileSha256','treesSha256','inspectionSha256','restoreIntentEventSha256')){
 $bad=Clone $complete;$bad.$field=Hash 999
 Reject $s {Advance $s CandidateDeniedRestored $bad} ('Completion differs from exact target '+$field)
}
$bad=Clone $complete;$bad.activeMarkerAbsent=$false
Reject $s {Advance $s CandidateDeniedRestored $bad} 'Completion alone cannot waive active marker'
$done=Advance $s CandidateDeniedRestored $complete
Check (@($s.session.receipts).Count -eq 1 -and @($s.events).Count -eq 5) 'Denied restoration does not forge receipts2/3/4'
foreach($a in @('CandidateDeniedRestored','CandidateCommissioned','FallbackCommissioned','CandidatePreparationArmed')){
 Reject $s {Advance $s $a $complete} 'Restored denied branch is terminal; no automatic rollback or launch'
}
$done.evidence.profileSha256=Hash 999
Check ($s.events[-1].evidence.profileSha256 -ceq $prep.baselineSha256) 'Returned event cannot mutate retained evidence'
# Explicit reviewed current-state preservation is pinned into intent and completion.
$s=Prepared;$null=Advance $s DeniedBeforeLaunch (Denial $s);$i=Intent $s
$i.targetProfileSha256=Hash 71;$i.preservationSha256=Hash 72
$null=Advance $s DeniedRestoreIntent $i;$null=Advance $s CandidateDeniedRestored (Completion $s)
Check ($s.events[-1].evidence.profileSha256 -ceq (Hash 71)) 'Exact reviewed preservation target retained'
# The normal four receipt bodies still pass the original reducer, with operation
# events ordered around them rather than changing their proof/authority shape.
$s=Prepared
$bad=Clone $candidate;$bad.preparedSha256=Hash 999
Reject $s {Advance $s CandidateCommissioned $bad} 'Receipt2 cannot switch armed transaction'
$null=Advance $s CandidateCommissioned $candidate
Reject $s {Advance $s DeniedBeforeLaunch (Denial $s)} 'Even approved-but-not-consumed case is outside narrow denial branch'
Reject $s {Advance $s CandidateRestored $restored} 'Restoration cannot skip consumed creation request'
$spawn=Spawn $s;$bad=Clone $spawn;$bad.environmentSha256=Hash 999
Reject $s {Advance $s CandidateSpawnConsumed $bad} 'Spawn environment must match receipt2'
$r=Request $s CandidateSpawnConsumed $spawn;$null=Add-SignedUpdateOperationEvent $s $r 1004
Reject $s {Add-SignedUpdateOperationEvent $s $r 1005} 'Lost spawn response remains consumed'
Reject $s {Advance $s DeniedBeforeLaunch (Denial $s)} 'Unrecorded/uncertain child cannot become denied-before-launch'
$null=Advance $s CandidateRestored $restored;$null=Advance $s FallbackCommissioned $fallback
$null=Advance $s FallbackSpawnConsumed (Spawn $s)
Check (@($s.session.receipts).Count -eq 4 -and @($s.events).Count -eq 7) 'Normal candidate/fallback retains four ordered receipts'
Reject $s {Advance $s FallbackSpawnConsumed (Spawn $s)} 'Fallback spawn consumed once'
$normal=New-SignedUpdateSession $binding 1000
foreach($receipt in $s.session.receipts){$r=[pscustomobject]@{session=$receipt.session;sequence=$receipt.sequence;action=$receipt.action;requestNonce=$receipt.requestNonce;previousReceiptSha256=$receipt.previousReceiptSha256;evidence=$receipt.evidence};$null=Add-SignedUpdateReceipt $normal $r $receipt.recordedUnix}
Check ((Get-SignedUpdateReceiptHash $normal) -ceq (Get-SignedUpdateReceiptHash $s.session)) 'All four original receipt contracts preserved exactly'
$s=Prepared;$s.events[1].evidence.baselineSha256=Hash 999
Reject $s {Advance $s DeniedBeforeLaunch (Denial $s)} 'Mutated operation history cannot authorize recovery'
Write-Output ('Signed update operations: '+$checks+' pure checks passed; no broker, mutation or launch authority.')
