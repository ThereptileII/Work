# Pure in-memory adversarial contracts. No application/profile/network access.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'SignedUpdateSession.ps1')
$checks=0
function Check([bool]$Ok,[string]$Reason){if(-not $Ok){throw $Reason};$script:checks++}
function Clone($Value){Copy-SignedUpdateValue $Value}
function Reject([scriptblock]$Call,[string]$Reason){$rejected=$false;try{$null=& $Call}catch{$rejected=$true};Check $rejected $Reason}
function Hash([int]$Number){return $Number.ToString('x64')}
# Avoid conditional expressions requiring PowerShell 7.
$old=[pscustomobject]@{generation=('a'*32);commit=('a'*40);packageSha256=('a'*64);executableSha256=('c'*64)}
$new=[pscustomobject]@{generation=('b'*32);commit=('b'*40);packageSha256=('b'*64);executableSha256=('d'*64)}
$binding=[pscustomobject]@{session=(Hash 1);previous=$old;release=[pscustomobject]@{commit=$new.commit;packageSha256=$new.packageSha256;executableSha256=$new.executableSha256;policySha256=(Hash 2);sourceReviewSha256=(Hash 3)};previousCommissioningSha256=(Hash 4);fallbackSourceReviewSha256=(Hash 5);createdUnix=1000;expiresUnix=4000}
function Session {return New-SignedUpdateSession (Clone $binding) 1000}
function Restore($identity,[string]$prepared,[int]$offset){return [pscustomobject]@{identity=(Clone $identity);stateSha256=(Hash ($offset+1));ownershipSha256=(Hash ($offset+2));preparedSha256=$prepared;restoreCompletionSha256=(Hash ($offset+3));profileSha256=(Hash ($offset+4));treesSha256=(Hash ($offset+5));inspectionSha256=(Hash ($offset+6))}}
function Commission($identity,[string]$source,[int]$offset){return [pscustomobject]@{identity=(Clone $identity);stateSha256=(Hash ($offset+1));ownershipSha256=(Hash ($offset+2));preparedSha256=(Hash ($offset+3));appliedSha256=(Hash ($offset+4));baselineSha256=(Hash 999);profileSha256=(Hash ($offset+5));treesSha256=(Hash ($offset+6));sourceReviewSha256=$source;independentAuditSha256=(Hash ($offset+7));approvalSha256=(Hash ($offset+8));environmentSha256=(Hash ($offset+9));reviewedUnix=1001}}
function Request($session,[string]$action,$evidence,[int]$nonce){return [pscustomobject]@{session=$session.binding.session;sequence=(@($session.receipts).Count+1);action=$action;requestNonce=(Hash $nonce);previousReceiptSha256=$session.headSha256;evidence=(Clone $evidence)}}
$restored=Restore $old $binding.previousCommissioningSha256 10
$commissioned=Commission $new $binding.release.sourceReviewSha256 20
$commissioned.baselineSha256=$restored.profileSha256
$candidateRestored=Restore $new $commissioned.preparedSha256 30
$fallback=Commission $old $binding.fallbackSourceReviewSha256 40
$fallback.baselineSha256=$candidateRestored.profileSha256
foreach($action in @('CandidateCommissioned','CandidateRestored','FallbackCommissioned')){
 $s=Session;$r=Request $s $action $commissioned 100;Reject {Add-SignedUpdateReceipt $s $r 1001} 'No step skip at start'
 Check (@($s.receipts).Count -eq 0) 'Refusal leaves journal unchanged'
}
foreach($field in @('profileSha256','treesSha256','inspectionSha256','restoreCompletionSha256','ownershipSha256','stateSha256')){
 $s=Session;$r=Request $s 'PreviousRestored' $restored 100;$r.evidence.$field='';Reject {Add-SignedUpdateReceipt $s $r 1001} ('Missing actual restore evidence: '+$field)
}
$s=Session;$r=Request $s 'PreviousRestored' $restored 100
$first=Add-SignedUpdateReceipt $s $r 1001
Check ($s.receipts.Count -eq 1 -and $s.headSha256 -ceq $first.receiptSha256) 'Old commissioning receipt consumed first'
Reject {Add-SignedUpdateReceipt $s $r 1002} 'Old transition request cannot replay'
$before=Get-SignedUpdateReceiptHash $s
foreach($field in @('commit','packageSha256','executableSha256')){
 $bad=Clone $commissioned;$bad.identity.$field=('e'*$bad.identity.$field.Length)
 $r=Request $s 'CandidateCommissioned' $bad 101
 Reject {Add-SignedUpdateReceipt $s $r 1002} ('Wrong authenticated successor '+$field)
}
foreach($field in @('profileSha256','treesSha256','sourceReviewSha256','independentAuditSha256','approvalSha256','environmentSha256','appliedSha256')){
 $bad=Clone $commissioned;$bad.$field='';$r=Request $s 'CandidateCommissioned' $bad 101
 Reject {Add-SignedUpdateReceipt $s $r 1002} ('Missing target commissioning evidence '+$field)
}
$bad=Clone $commissioned;$bad.preparedSha256=$binding.previousCommissioningSha256
$r=Request $s 'CandidateCommissioned' $bad 101;Reject {Add-SignedUpdateReceipt $s $r 1002} 'Cannot transfer old active commissioning'
foreach($time in @(999,1003)){
 $bad=Clone $commissioned;$bad.reviewedUnix=$time;$r=Request $s 'CandidateCommissioned' $bad 101
 Reject {Add-SignedUpdateReceipt $s $r 1002} 'Review must be fresh and not future'
}
$bad=Clone $commissioned;$bad.baselineSha256=Hash 900;$r=Request $s 'CandidateCommissioned' $bad 101
Reject {Add-SignedUpdateReceipt $s $r 1002} 'Candidate cannot silently switch the restored profile baseline'
$r=Request $s 'CandidateCommissioned' $commissioned 100;Reject {Add-SignedUpdateReceipt $s $r 1002} 'Nonce cannot be reused for a later action'
Check ((Get-SignedUpdateReceiptHash $s) -ceq $before) 'All refused candidate requests leave exact journal unchanged'
$r=Request $s 'CandidateCommissioned' $commissioned 101
$second=Add-SignedUpdateReceipt $s $r 1002
Check ($s.receipts.Count -eq 2 -and $second.action -ceq 'CandidateCommissioned') 'Actual successor commissioning receipt accepted'
$r=Request $s 'FallbackCommissioned' $fallback 102;Reject {Add-SignedUpdateReceipt $s $r 1003} 'Rollback cannot skip candidate restoration'
$bad=Clone $candidateRestored;$bad.preparedSha256=$binding.previousCommissioningSha256
$r=Request $s 'CandidateRestored' $bad 102;Reject {Add-SignedUpdateReceipt $s $r 1003} 'Wrong active candidate restoration rejected'
$r=Request $s 'CandidateRestored' $candidateRestored 102;$null=Add-SignedUpdateReceipt $s $r 1003
foreach($field in @('preparedSha256','appliedSha256','approvalSha256','independentAuditSha256')){
 $bad=Clone $fallback;$bad.$field=$commissioned.$field;$r=Request $s 'FallbackCommissioned' $bad 103
 Reject {Add-SignedUpdateReceipt $s $r 1004} ('Fallback cannot reuse candidate '+$field)
}
$bad=Clone $fallback;$bad.identity=$new;$r=Request $s 'FallbackCommissioned' $bad 103
Reject {Add-SignedUpdateReceipt $s $r 1004} 'Fallback must be exact previous generation'
$r=Request $s 'FallbackCommissioned' $fallback 103;$last=Add-SignedUpdateReceipt $s $r 1004
Check ($s.receipts.Count -eq 4 -and $last.action -ceq 'FallbackCommissioned') 'Fresh fallback review completes bounded sequence'
Reject {Add-SignedUpdateReceipt $s $r 1005} 'Terminal receipt cannot replay'
$s=Session;$r=Request $s 'PreviousRestored' $restored 100
foreach($field in @('session','previousReceiptSha256')){
 $bad=Clone $r;$bad.$field=Hash 999;Reject {Add-SignedUpdateReceipt $s $bad 1001} ('Wrong request '+$field)
}
$bad=Clone $r;$bad|Add-Member -NotePropertyName allow -NotePropertyValue $true
Reject {Add-SignedUpdateReceipt $s $bad 1001} 'Generic allow flag is not a supported proof'
$bad=Clone $r;$bad.sequence='1';Reject {Add-SignedUpdateReceipt $s $bad 1001} 'String sequence refused'
Reject {Add-SignedUpdateReceipt $s $r 4000} 'Expired session cannot advance'
Reject {Add-SignedUpdateReceipt $s $r 999} 'Clock reversal cannot advance'
$null=Add-SignedUpdateReceipt $s $r 1001
$s.receipts[0].evidence.profileSha256=Hash 900
$r=Request $s 'CandidateCommissioned' $commissioned 101
Reject {Add-SignedUpdateReceipt $s $r 1002} 'Modified historical profile evidence invalidates chain'
$bad=Clone $binding;$bad.expiresUnix=20000;Reject {New-SignedUpdateSession $bad 1000} 'Session lifetime bound enforced'
$bad=Clone $binding;$bad.release.packageSha256=$old.packageSha256;Reject {New-SignedUpdateSession $bad 1000} 'Same package is not an update successor'
Write-Output ('Signed update session: '+$checks+' pure contract checks passed; no launch permission or native qualification claimed.')
