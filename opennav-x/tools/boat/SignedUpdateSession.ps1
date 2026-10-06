# Pure, bounded receipt reducer for SCRUM-311. No filesystem, processes, network,
# profile changes, signatures or launch permissions. The caller must validate
# real evidence and atomically persist this journal under an exclusive lock
# BEFORE allowing an operation. Replaying an older disk journal is outside this
# in-memory model and must be refused by the armed native boundary.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
function Assert-SignedUpdateShape($Value,[string[]]$Fields) {
  if($null -eq $Value -or $Value -isnot [pscustomobject]){throw 'Expected a closed evidence object.'}
  $names=@($Value.PSObject.Properties.Name)
  if($names.Count -ne $Fields.Count){throw 'Signed-update evidence schema differs.'}
  foreach($field in $Fields){if($names -cnotcontains $field){throw 'Signed-update evidence field missing or mis-cased.'}}
}
function Assert-SignedUpdateHex($Value,[int]$Length) {
  if($Value -isnot [string] -or $Value -cnotmatch ('^[a-f0-9]{'+$Length+'}$') -or $Value -cmatch '^0+$'){throw 'Invalid exact evidence identity.'}
}
function Assert-SignedUpdateInteger($Value) {
  if(($Value -isnot [int] -and $Value -isnot [long]) -or $Value -lt 0){throw 'Expected nonnegative integer time/sequence.'}
}
function Assert-SignedUpdateIdentity($Value) {
  Assert-SignedUpdateShape $Value @('generation','commit','packageSha256','executableSha256')
  Assert-SignedUpdateHex $Value.generation 32;Assert-SignedUpdateHex $Value.commit 40
  Assert-SignedUpdateHex $Value.packageSha256 64;Assert-SignedUpdateHex $Value.executableSha256 64
}
function Test-SignedUpdateIdentity($A,$B) {
  foreach($key in @('generation','commit','packageSha256','executableSha256')){if($A.$key -cne $B.$key){return $false}}
  return $true
}
function Get-SignedUpdateReceiptHash($Value) {
  $bytes=[Text.Encoding]::UTF8.GetBytes(($Value|ConvertTo-Json -Depth 12 -Compress))
  $hash=[Security.Cryptography.SHA256]::Create()
  try{return ([BitConverter]::ToString($hash.ComputeHash($bytes))).Replace('-','').ToLowerInvariant()}
  finally{$hash.Dispose()}
}
function Copy-SignedUpdateValue($Value) { return ($Value|ConvertTo-Json -Depth 12 -Compress|ConvertFrom-Json) }
function New-SignedUpdateSession($Binding,[long]$Now) {
  Assert-SignedUpdateShape $Binding @('session','previous','release','previousCommissioningSha256','fallbackSourceReviewSha256','createdUnix','expiresUnix')
  Assert-SignedUpdateHex $Binding.session 64;Assert-SignedUpdateIdentity $Binding.previous
  Assert-SignedUpdateHex $Binding.previousCommissioningSha256 64;Assert-SignedUpdateHex $Binding.fallbackSourceReviewSha256 64
  Assert-SignedUpdateShape $Binding.release @('commit','packageSha256','executableSha256','policySha256','sourceReviewSha256')
  Assert-SignedUpdateHex $Binding.release.commit 40
  foreach($key in @('packageSha256','executableSha256','policySha256','sourceReviewSha256')){Assert-SignedUpdateHex $Binding.release.$key 64}
  foreach($time in @($Now,$Binding.createdUnix,$Binding.expiresUnix)){Assert-SignedUpdateInteger $time}
  if($Binding.createdUnix -gt $Now -or $Binding.expiresUnix -le $Now -or
     $Binding.expiresUnix -le $Binding.createdUnix -or $Binding.expiresUnix-$Binding.createdUnix -gt 14400){throw 'Session lifetime is invalid or expired.'}
  if($Binding.release.packageSha256 -ceq $Binding.previous.packageSha256){throw 'Successor must be a distinct reviewed release.'}
  $copy=Copy-SignedUpdateValue $Binding
  return [pscustomobject]@{binding=$copy;bindingSha256=(Get-SignedUpdateReceiptHash $copy);receipts=@();headSha256=(Get-SignedUpdateReceiptHash $copy);lastUnix=$Now}
}
function Add-SignedUpdateReceipt($Session,$Request,[long]$Now) {
  Assert-SignedUpdateShape $Session @('binding','bindingSha256','receipts','headSha256','lastUnix')
  Assert-SignedUpdateInteger $Now;Assert-SignedUpdateInteger $Session.lastUnix
  $null=New-SignedUpdateSession $Session.binding $Now
  if((Get-SignedUpdateReceiptHash $Session.binding) -cne $Session.bindingSha256 -or $Now -lt $Session.lastUnix){throw 'Session identity/time changed.'}
  $steps=@('PreviousRestored','CandidateCommissioned','CandidateRestored','FallbackCommissioned')
  $history=@($Session.receipts)
  if($history.Count -ge $steps.Count){throw 'Session is already terminal.'}
  # Check the retained chain instead of trusting a caller-writable sequence/head.
  $head=$Session.bindingSha256;$seen=@{};$candidate=$null;$candidatePrepared=''
  for($i=0;$i -lt $history.Count;$i++) {
    $receipt=$history[$i]
    Assert-SignedUpdateShape $receipt @('session','sequence','action','requestNonce','previousReceiptSha256','recordedUnix','evidence','receiptSha256')
    if($receipt.session -cne $Session.binding.session -or $receipt.sequence -ne $i+1 -or
       $receipt.action -cne $steps[$i] -or $receipt.previousReceiptSha256 -cne $head -or $seen.ContainsKey($receipt.requestNonce)){throw 'Receipt chain changed or replayed.'}
    $body=[ordered]@{session=$receipt.session;sequence=$receipt.sequence;action=$receipt.action;requestNonce=$receipt.requestNonce;previousReceiptSha256=$receipt.previousReceiptSha256;recordedUnix=$receipt.recordedUnix;evidence=$receipt.evidence}
    if((Get-SignedUpdateReceiptHash $body) -cne $receipt.receiptSha256){throw 'Receipt content changed.'}
    $head=$receipt.receiptSha256;$seen[$receipt.requestNonce]=$true
    if($i -eq 1){$candidate=$receipt.evidence.identity;$candidatePrepared=$receipt.evidence.preparedSha256}
  }
  if($Session.headSha256 -cne $head){throw 'Journal head changed.'}
  Assert-SignedUpdateShape $Request @('session','sequence','action','requestNonce','previousReceiptSha256','evidence')
  Assert-SignedUpdateInteger $Request.sequence;Assert-SignedUpdateHex $Request.requestNonce 64
  if($Request.session -cne $Session.binding.session -or $Request.sequence -ne $history.Count+1 -or
     $Request.action -cne $steps[$history.Count] -or $Request.previousReceiptSha256 -cne $head -or $seen.ContainsKey($Request.requestNonce)) {throw 'Request is stale, replayed, out of order or from another session.'}
  $e=$Request.evidence
  $restore=$history.Count -eq 0 -or $history.Count -eq 2
  if($restore) {
    Assert-SignedUpdateShape $e @('identity','stateSha256','ownershipSha256','preparedSha256','restoreCompletionSha256','profileSha256','treesSha256','inspectionSha256')
  } else {
    Assert-SignedUpdateShape $e @('identity','stateSha256','ownershipSha256','preparedSha256','appliedSha256','baselineSha256','profileSha256','treesSha256','sourceReviewSha256','independentAuditSha256','approvalSha256','environmentSha256','reviewedUnix')
  }
  Assert-SignedUpdateIdentity $e.identity
  foreach($property in $e.PSObject.Properties){if($property.Name.EndsWith('Sha256',[StringComparison]::Ordinal)){Assert-SignedUpdateHex $property.Value 64}}
  if($history.Count -eq 0) {
    if(-not (Test-SignedUpdateIdentity $e.identity $Session.binding.previous) -or $e.preparedSha256 -cne $Session.binding.previousCommissioningSha256){throw 'Old commissioning must be restored under its original generation.'}
  } elseif($history.Count -eq 1) {
    $release=$Session.binding.release
    if($e.identity.generation -ceq $Session.binding.previous.generation -or $e.identity.commit -cne $release.commit -or
       $e.identity.packageSha256 -cne $release.packageSha256 -or $e.identity.executableSha256 -cne $release.executableSha256 -or
       $e.sourceReviewSha256 -cne $release.sourceReviewSha256 -or $e.preparedSha256 -ceq $Session.binding.previousCommissioningSha256){throw 'Candidate must match the authenticated release and fresh commissioning.'}
  } elseif($history.Count -eq 2) {
    if(-not (Test-SignedUpdateIdentity $e.identity $candidate) -or $e.preparedSha256 -cne $candidatePrepared){throw 'Restore the candidate transaction before changing the fallback pointer.'}
  } else {
    if(-not (Test-SignedUpdateIdentity $e.identity $Session.binding.previous) -or $e.sourceReviewSha256 -cne $Session.binding.fallbackSourceReviewSha256 -or
       $e.preparedSha256 -ceq $candidatePrepared -or $e.preparedSha256 -ceq $Session.binding.previousCommissioningSha256){throw 'Fallback requires its exact old generation and a new commissioning transaction.'}
  }
  if(-not $restore) {
    if($e.baselineSha256 -cne $history[$history.Count-1].evidence.profileSha256){throw 'Fresh commissioning baseline must equal the just-restored profile.'}
    Assert-SignedUpdateInteger $e.reviewedUnix
    if($e.reviewedUnix -lt $Session.binding.createdUnix -or $e.reviewedUnix -gt $Now){throw 'Fresh review must belong to this update session.'}
    foreach($prior in $history) {
      if($prior.action -cin @('CandidateCommissioned','FallbackCommissioned') -and
         ($e.appliedSha256 -ceq $prior.evidence.appliedSha256 -or $e.approvalSha256 -ceq $prior.evidence.approvalSha256 -or $e.independentAuditSha256 -ceq $prior.evidence.independentAuditSha256)) {throw 'Commissioning, audit or approval evidence cannot transfer between launches.'}
    }
  }
  $body=[ordered]@{session=$Session.binding.session;sequence=$Request.sequence;action=$Request.action;requestNonce=$Request.requestNonce;previousReceiptSha256=$head;recordedUnix=$Now;evidence=(Copy-SignedUpdateValue $e)}
  $digest=Get-SignedUpdateReceiptHash $body
  $body.Add('receiptSha256',$digest)
  $receipt=[pscustomobject]$body
  # No mutation occurs before all checks pass. These assignments are only the
  # in-memory reducer: caller must journal atomically before using its result.
  $Session.receipts=@($history)+@($receipt);$Session.headSha256=$digest;$Session.lastUnix=$Now
  return Copy-SignedUpdateValue $receipt
}
