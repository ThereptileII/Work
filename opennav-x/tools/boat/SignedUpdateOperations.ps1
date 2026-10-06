# Pure operation-order foundation. These records do not authenticate artifacts,
# prove absence of a process, or authorize mutation/spawn. Only a future broker
# holding continuous live custody may supply independently verified evidence.
. (Join-Path $PSScriptRoot 'SignedUpdateSession.ps1')
function New-SignedUpdateOperationSession($Binding,[long]$Now) {
  $session=New-SignedUpdateSession $Binding $Now
  return [pscustomobject]@{session=$session;events=@();headSha256=$session.bindingSha256;lastUnix=$Now}
}
function Assert-SignedUpdateTransactionDirectory($Value) {
  if($Value -isnot [string] -or $Value.Length -gt 240 -or $Value -cnotmatch '^[A-Za-z]:\\[^/<>:"|?*\x00-\x1f]+$'){throw 'Canonical local transaction directory required.'}
  foreach($part in $Value.Substring(3).Split('\')) {
    if(-not $part -or $part -cin @('.','..') -or $part.EndsWith('.') -or $part.EndsWith(' ')){throw 'Noncanonical transaction directory.'}
    # Same component policy as PortableReview, including Windows' superscript
    # device digits. This validates spelling only, never filesystem identity.
    if($part -match '^(?i:CON|PRN|AUX|NUL|COM[1-9\u00b9\u00b2\u00b3]|LPT[1-9\u00b9\u00b2\u00b3])(?: *\.|$)'){throw 'Windows device transaction component refused.'}
  }
}
function Assert-SignedUpdateOperationHashes($Evidence) {
  foreach($p in $Evidence.PSObject.Properties){if($p.Name.EndsWith('Sha256',[StringComparison]::Ordinal)){Assert-SignedUpdateHex $p.Value 64}}
}
function Assert-SignedUpdateCandidateOperation($Evidence,$Preparation,[switch]$Receipt) {
  Assert-SignedUpdateIdentity $Evidence.identity
  if(-not (Test-SignedUpdateIdentity $Evidence.identity $Preparation.identity)){throw 'Operation candidate identity changed.'}
  $keys=@('stateSha256','ownershipSha256','preparedSha256')
  if(-not $Receipt){$keys+=@('transactionDirectory')}
  foreach($key in $keys) {
    if($Evidence.$key -cne $Preparation.$key){throw 'Candidate selection or transaction changed.'}
  }
}
# Internal deterministic step; public Add replays and validates the full bounded
# event history before calling this. It never accepts a supplied phase/allow bit.
function Step-SignedUpdateOperation($State,$Request,[long]$Now) {
  $session=$State.session;$history=@($State.events)
  if($history.Count -ge 8){throw 'Operation session is terminal.'}
  Assert-SignedUpdateShape $Request @('session','sequence','action','requestNonce','previousEventSha256','evidence')
  Assert-SignedUpdateInteger $Now;Assert-SignedUpdateInteger $Request.sequence
  Assert-SignedUpdateHex $Request.requestNonce 64
  Assert-SignedUpdateHex $Request.session 64;Assert-SignedUpdateHex $Request.previousEventSha256 64
  if($Request.action -isnot [string]){throw 'One exact operation action required.'}
  $null=New-SignedUpdateSession $session.binding $Now
  if($Now -lt $State.lastUnix -or $Request.session -cne $session.binding.session -or
     $Request.sequence -ne $history.Count+1 -or $Request.previousEventSha256 -cne $State.headSha256){throw 'Stale operation identity, time or head.'}
  foreach($event in $history){if($event.requestNonce -ceq $Request.requestNonce){throw 'Operation nonce already consumed.'}}
  $previous=if($history.Count){$history[-1].action}else{''}
  $allowed=switch -CaseSensitive ($previous) {
    '' {@('PreviousRestored')}
    'PreviousRestored' {@('CandidatePreparationArmed')}
    'CandidatePreparationArmed' {@('CandidateCommissioned','DeniedBeforeLaunch')}
    'CandidateCommissioned' {@('CandidateSpawnConsumed')}
    'CandidateSpawnConsumed' {@('CandidateRestored')}
    'CandidateRestored' {@('FallbackCommissioned')}
    'FallbackCommissioned' {@('FallbackSpawnConsumed')}
    'DeniedBeforeLaunch' {@('DeniedRestoreIntent')}
    'DeniedRestoreIntent' {@('CandidateDeniedRestored')}
    default {@()}
  }
  if($Request.action -cnotin @($allowed)){throw 'Operation is terminal, uncertain, or out of order.'}
  $e=$Request.evidence
  $preparation=if($history.Count -ge 2){$history[1].evidence}else{$null}
  if($Request.action -cin @('PreviousRestored','CandidateCommissioned','CandidateRestored','FallbackCommissioned')) {
    if($Request.action -ceq 'CandidateCommissioned') {
      Assert-SignedUpdateCandidateOperation $e $preparation -Receipt
      if($e.baselineSha256 -cne $preparation.baselineSha256 -or $e.sourceReviewSha256 -cne $preparation.sourceReviewSha256){throw 'Commissioning escaped armed preparation.'}
    }
    $receiptRequest=[pscustomobject]@{session=$Request.session;sequence=@($session.receipts).Count+1;action=$Request.action;requestNonce=$Request.requestNonce;previousReceiptSha256=$session.headSha256;evidence=$e}
    $null=Add-SignedUpdateReceipt $session $receiptRequest $Now
  } elseif($Request.action -ceq 'CandidatePreparationArmed') {
    Assert-SignedUpdateShape $e @('identity','stateSha256','ownershipSha256','preparedSha256','transactionDirectory','contextSha256','planSha256','inventorySha256','quarantineSha256','baselineSha256','sourceReviewSha256')
    Assert-SignedUpdateIdentity $e.identity;Assert-SignedUpdateOperationHashes $e
    Assert-SignedUpdateTransactionDirectory $e.transactionDirectory
    $release=$session.binding.release
    if($e.identity.generation -ceq $session.binding.previous.generation -or
       $e.identity.commit -cne $release.commit -or $e.identity.packageSha256 -cne $release.packageSha256 -or
       $e.identity.executableSha256 -cne $release.executableSha256 -or $e.sourceReviewSha256 -cne $release.sourceReviewSha256 -or
       $e.preparedSha256 -ceq $session.binding.previousCommissioningSha256 -or
       $e.baselineSha256 -cne $session.receipts[0].evidence.profileSha256){throw 'Preparation must bind the exact new candidate and restored baseline.'}
  } elseif($Request.action -ceq 'DeniedBeforeLaunch') {
    Assert-SignedUpdateShape $e @('preparationEventSha256','denialSha256');Assert-SignedUpdateOperationHashes $e
    if($e.preparationEventSha256 -cne $history[1].eventSha256){throw 'Denial belongs to another preparation.'}
  } elseif($Request.action -ceq 'DeniedRestoreIntent') {
    Assert-SignedUpdateShape $e @('identity','stateSha256','ownershipSha256','preparedSha256','transactionDirectory','preparationEventSha256','denialEventSha256','inspectionSha256','currentProfileSha256','targetProfileSha256','targetTreesSha256','preservationSha256')
    Assert-SignedUpdateCandidateOperation $e $preparation
    foreach($p in $e.PSObject.Properties){if($p.Name -cne 'preservationSha256' -and $p.Name.EndsWith('Sha256',[StringComparison]::Ordinal)){Assert-SignedUpdateHex $p.Value 64}}
    if($null -eq $e.preservationSha256) {
      if($e.targetProfileSha256 -cne $preparation.baselineSha256){throw 'A changed restore target requires reviewed preservation.'}
    }else{Assert-SignedUpdateHex $e.preservationSha256 64}
    if($e.preparationEventSha256 -cne $history[1].eventSha256 -or $e.denialEventSha256 -cne $history[2].eventSha256){throw 'Restore intent belongs to another denied preparation.'}
  } elseif($Request.action -ceq 'CandidateDeniedRestored') {
    Assert-SignedUpdateShape $e @('identity','stateSha256','ownershipSha256','preparedSha256','transactionDirectory','restoreIntentEventSha256','inspectionSha256','restoreCompletionSha256','profileSha256','treesSha256','activeMarkerAbsent')
    Assert-SignedUpdateCandidateOperation $e $preparation;Assert-SignedUpdateOperationHashes $e
    $intent=$history[-1]
    if($e.restoreIntentEventSha256 -cne $intent.eventSha256 -or $e.inspectionSha256 -cne $intent.evidence.inspectionSha256 -or
       $e.profileSha256 -cne $intent.evidence.targetProfileSha256 -or $e.treesSha256 -cne $intent.evidence.targetTreesSha256 -or
       $e.activeMarkerAbsent -isnot [bool] -or -not $e.activeMarkerAbsent){throw 'Exact denied restoration completion required.'}
  } else {
    Assert-SignedUpdateShape $e @('commissionReceiptSha256','creationRequestSha256','environmentSha256');Assert-SignedUpdateOperationHashes $e
    $commission=$session.receipts[-1]
    if($e.commissionReceiptSha256 -cne $commission.receiptSha256 -or $e.environmentSha256 -cne $commission.evidence.environmentSha256){throw 'Spawn consumption must bind its exact commissioning receipt/environment.'}
  }
  $body=[ordered]@{session=$Request.session;sequence=$Request.sequence;action=$Request.action;requestNonce=$Request.requestNonce;previousEventSha256=$State.headSha256;recordedUnix=$Now;evidence=(Copy-SignedUpdateValue $e)}
  $hash=Get-SignedUpdateReceiptHash $body;$body.Add('eventSha256',$hash)
  $event=[pscustomobject]$body
  $State.events=@($history)+@($event);$State.headSha256=$hash;$State.lastUnix=$Now
  return Copy-SignedUpdateValue $event
}
function Add-SignedUpdateOperationEvent($State,$Request,[long]$Now) {
  Assert-SignedUpdateShape $State @('session','events','headSha256','lastUnix')
  Assert-SignedUpdateInteger $State.lastUnix
  $history=@($State.events)
  if($history.Count -gt 8){throw 'Operation history exceeds bound.'}
  $initial=if($history.Count){$history[0].recordedUnix}else{$State.lastUnix}
  $next=New-SignedUpdateOperationSession $State.session.binding $initial
  foreach($event in $history) {
    Assert-SignedUpdateShape $event @('session','sequence','action','requestNonce','previousEventSha256','recordedUnix','evidence','eventSha256')
    $prior=[pscustomobject]@{session=$event.session;sequence=$event.sequence;action=$event.action;requestNonce=$event.requestNonce;previousEventSha256=$event.previousEventSha256;evidence=$event.evidence}
    $replayed=Step-SignedUpdateOperation $next $prior $event.recordedUnix
    if($replayed.eventSha256 -cne $event.eventSha256){throw 'Operation history was changed.'}
  }
  if((Get-SignedUpdateReceiptHash $next) -cne (Get-SignedUpdateReceiptHash $State)){throw 'Retained operation state/head differs from history.'}
  $result=Step-SignedUpdateOperation $next $Request $Now
  # All mutation is on a private replay until every check has passed.
  $State.session=$next.session;$State.events=$next.events;$State.headSha256=$next.headSha256;$State.lastUnix=$next.lastUnix
  return $result
}
