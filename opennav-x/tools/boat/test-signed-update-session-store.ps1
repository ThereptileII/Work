# Inert files only. Native Windows tests exercise real ACL/share/reparse custody;
# portable runs replace ONLY the unsupported native boundary, never persistence,
# strict JSON parsing or the receipt reducer. Neither result qualifies a launch.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'SignedUpdateSessionStore.ps1')
$checks=0;$native=[Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT
$fixtureRoot=Join-Path ([IO.Path]::GetTempPath()) ('signed-update-store-tests-'+[guid]::NewGuid().ToString('N'))
$null=[IO.Directory]::CreateDirectory($fixtureRoot)
function Check([bool]$OK,[string]$Reason){if(-not $OK){throw $Reason};$script:checks++}
function Reject([scriptblock]$Call,[string]$Reason){$failed=$false;try{$null=& $Call}catch{$failed=$true};Check $failed $Reason}
function Hash([int]$N){return $N.ToString('x64')}
function Json($Value){return ,(Get-SignedStoreBytes $Value)}
function Raw([string]$Text){return ,([Text.Encoding]::UTF8.GetBytes($Text))}
$clock=1000
function Get-SignedStoreNow {return [long]$script:clock}
$old=[pscustomobject]@{generation=('a'*32);commit=('a'*40);packageSha256=('a'*64);executableSha256=('c'*64)}
$new=[pscustomobject]@{generation=('b'*32);commit=('b'*40);packageSha256=('b'*64);executableSha256=('d'*64)}
$binding=[pscustomobject]@{session=(Hash 1);previous=$old;release=[pscustomobject]@{commit=$new.commit;packageSha256=$new.packageSha256;executableSha256=$new.executableSha256;policySha256=(Hash 2);sourceReviewSha256=(Hash 3)};previousCommissioningSha256=(Hash 4);fallbackSourceReviewSha256=(Hash 5);createdUnix=1000;expiresUnix=4000}
function Restore($Identity,[string]$Prepared,[int]$Offset){return [pscustomobject]@{identity=$Identity;stateSha256=(Hash ($Offset+1));ownershipSha256=(Hash ($Offset+2));preparedSha256=$Prepared;restoreCompletionSha256=(Hash ($Offset+3));profileSha256=(Hash ($Offset+4));treesSha256=(Hash ($Offset+5));inspectionSha256=(Hash ($Offset+6))}}
function Commission($Identity,[string]$Source,[int]$Offset,[string]$Baseline){return [pscustomobject]@{identity=$Identity;stateSha256=(Hash ($Offset+1));ownershipSha256=(Hash ($Offset+2));preparedSha256=(Hash ($Offset+3));appliedSha256=(Hash ($Offset+4));baselineSha256=$Baseline;profileSha256=(Hash ($Offset+5));treesSha256=(Hash ($Offset+6));sourceReviewSha256=$Source;independentAuditSha256=(Hash ($Offset+7));approvalSha256=(Hash ($Offset+8));environmentSha256=(Hash ($Offset+9));reviewedUnix=1000}}
$restored=Restore $old $binding.previousCommissioningSha256 10
$commissioned=Commission $new $binding.release.sourceReviewSha256 20 $restored.profileSha256
$candidateRestored=Restore $new $commissioned.preparedSha256 30
$fallback=Commission $old $binding.fallbackSourceReviewSha256 40 $candidateRestored.profileSha256
function Request([string]$Handle,[string]$Action,$Evidence,[int]$Nonce) {
 $s=(Get-SignedStoreLive $Handle).session
 return [pscustomobject]@{session=$s.binding.session;sequence=@($s.receipts).Count+1;action=$Action;requestNonce=(Hash $Nonce);previousReceiptSha256=$s.headSha256;evidence=$Evidence}
}
function PrivateDirectory {
 $path=Join-Path $script:fixtureRoot ([guid]::NewGuid().ToString('N'))
 $null=[IO.Directory]::CreateDirectory($path)
 if($script:native){
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User
  $acl=[Security.AccessControl.DirectorySecurity]::new();$acl.SetOwner($sid);$acl.SetAccessRuleProtection($true,$false)
  foreach($principal in @($sid,[Security.Principal.SecurityIdentifier]::new('S-1-5-18'))){
   $rule=[Security.AccessControl.FileSystemAccessRule]::new($principal,[Security.AccessControl.FileSystemRights]::FullControl,([Security.AccessControl.InheritanceFlags]::ContainerInherit -bor [Security.AccessControl.InheritanceFlags]::ObjectInherit),[Security.AccessControl.PropagationFlags]::None,[Security.AccessControl.AccessControlType]::Allow)
   $acl.AddAccessRule($rule)
  }
  Set-Acl -LiteralPath $path -AclObject $acl
 }
 return $path
}
function Fresh {return New-SignedUpdateStore (PrivateDirectory) (Json $script:binding)}
$writeImpl=${function:Write-SignedStoreFrame}
$ownerImpl=${function:Get-SignedStoreOwner}
$nativeImpl=${function:Assert-SignedStoreNative}
$fileImpl=${function:New-SignedStoreFile}
try {
 if(-not $native){
  $p=PrivateDirectory
  Reject {New-SignedUpdateStore $p (Json $binding)} 'Portable production entry must refuse before arming'
  Check (-not (Test-Path -LiteralPath (Join-Path $p 'signed-update-armed.jsonl'))) 'Native refusal created no marker'
  function Get-SignedStoreOwner {return [pscustomobject]@{sid='fixture';pid=$PID;createdFiletime='fixture';sessionId=1;image='inert';imageSha256=(Hash 500)}}
  function Enter-SignedStoreDirectory([string]$Directory,$Owner){if(-not [IO.Directory]::Exists($Directory)){throw 'Missing fixture directory'};return ,@()}
  function New-SignedStoreFile([string]$Path,[string]$Sid){return [IO.FileStream]::new($Path,[IO.FileMode]::CreateNew,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None,4096,[IO.FileOptions]::WriteThrough)}
  function Assert-SignedStoreNative($Store){if(-not [IO.File]::Exists($Store.path)){throw 'Missing armed file'};if($Store.stream.SafeFileHandle.IsClosed){throw 'Closed fixture handle'}}
  $ownerImpl=${function:Get-SignedStoreOwner};$nativeImpl=${function:Assert-SignedStoreNative}
 }
 # Strict parser rejects ambiguity before PowerShell can collapse keys or numbers.
 foreach($bad in @('{"x":1,"x":2}','{"x":1,"X":2}','{"x":1,"\u0078":2}','{"e":{"a":1,"A":2}}','{"a":[{"z":1,"Z":2}]}','{"x":1}{}','{"x":01}','{"x":1.0}','{"x":1e0}','{"x":9223372036854775808}','{"x":"\ud800"}','{"x":"\udc00"}','{"x":true,}','{"x":"unterminated}','[]','null')){
  Reject {ConvertFrom-SignedStoreJson (Raw $bad)} ('Reject ambiguous/malformed JSON '+$bad)
 }
 Reject {ConvertFrom-SignedStoreJson ([byte[]]@(123,34,120,34,58,34,255,34,125))} 'Invalid UTF8 refused'
 Reject {ConvertFrom-SignedStoreJson (Raw ('{"x":"'+('a'*8193)+'"}'))} 'String bound'
 Reject {ConvertFrom-SignedStoreJson (Raw ('{"x":'+('['*14)+'0'+(']'*14)+'}'))} 'Depth bound'
 Reject {ConvertFrom-SignedStoreJson (Raw ('{"x":['+((@('0')*4097)-join ',')+']}'))} 'Node bound'
 Reject {ConvertFrom-SignedStoreJson (Raw (' '*65537))} 'Byte bound'
 Check ((ConvertFrom-SignedStoreJson (Raw '{"x":"\ud83d\ude80"}')).x.Length -eq 2) 'Valid surrogate pair retained'
 $p=PrivateDirectory
 foreach($bad in @((Raw ('['+([Text.Encoding]::UTF8.GetString((Json $binding)))+']')),(Raw (([Text.Encoding]::UTF8.GetString((Json $binding))).Replace('"session"','"Session"'))))){
  Reject {New-SignedUpdateStore $p $bad} 'Binding must be closed exact root object'
 }
 Check (-not (Test-Path -LiteralPath (Join-Path $p 'signed-update-armed.jsonl'))) 'Rejected binding creates no marker'
 $h=Fresh;$s=Get-SignedStoreLive $h;$initial=Read-SignedStoreHeldBytes $s
 Check ($initial.Length -gt 0 -and $s.stream.CanWrite) 'Armed state flushed with held exclusive file'
 Reject {New-SignedUpdateStore $s.directory (Json $binding)} 'Second owner cannot arm same directory'
 Reject {$other=[IO.File]::Open($s.path,[IO.FileMode]::Open,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None);$other.Dispose()} 'Competing exclusive handle refused'
 $r=Request $h 'PreviousRestored' $restored 100
 $bad=Copy-SignedUpdateValue $r;$bad|Add-Member -NotePropertyName allow -NotePropertyValue $true
 Reject {Add-SignedUpdateStoreReceipt $h (Json $bad)} 'Generic allow request refused'
 $bad=Copy-SignedUpdateValue $r;$bad.action='CandidateCommissioned'
 Reject {Add-SignedUpdateStoreReceipt $h (Json $bad)} 'Out-of-order request refused'
 $duplicate=([Text.Encoding]::UTF8.GetString((Json $r))).Replace('"sequence":1','"sequence":1,"Sequence":1')
 Reject {Add-SignedUpdateStoreReceipt $h (Raw $duplicate)} 'Duplicate request field refused before reducer'
 Check ((Get-SignedStoreDigest (Read-SignedStoreHeldBytes $s)) -ceq (Get-SignedStoreDigest $initial)) 'Refused requests leave exact durable state unchanged'
 $first=Add-SignedUpdateStoreReceipt $h (Json $r)
 Check ($s.session.receipts.Count -eq 1 -and $s.stream.Length -gt $initial.Length) 'Transition consumed before returning receipt'
 Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} 'Lost result cannot retry consumed request'
 $first.evidence.profileSha256=Hash 999
 Check ($s.session.receipts[0].evidence.profileSha256 -ceq $restored.profileSha256) 'Returned receipt cannot mutate retained custody'
 $r=Request $h 'CandidateCommissioned' $commissioned 101;$null=Add-SignedUpdateStoreReceipt $h (Json $r)
 $r=Request $h 'CandidateRestored' $candidateRestored 102;$null=Add-SignedUpdateStoreReceipt $h (Json $r)
 $r=Request $h 'FallbackCommissioned' $fallback 103;$null=Add-SignedUpdateStoreReceipt $h (Json $r)
 Check ($s.session.receipts.Count -eq 4) 'Four real ordered transitions persisted'
 Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} 'Terminal session cannot replay'
 $file=$s.path;$directory=$s.directory;$expected=$s.digest
 Close-SignedUpdateStore $h
 Check ((Get-SignedStoreDigest ([IO.File]::ReadAllBytes($file))) -ceq $expected) 'Closed durable bytes equal pinned last head'
 $lines=[IO.File]::ReadAllLines($file);Check ($lines.Length -eq 5) 'One complete state/head frame per consumed transition'
 $prefix=[byte[]]@();$previous=('0'*64)
 for($i=0;$i -lt $lines.Length;$i++){
  $frame=ConvertFrom-SignedStoreJson (Raw $lines[$i]);Assert-SignedUpdateShape $frame @('schema','owner','sequence','previousFrameSha256','state')
  Check ($frame.schema -eq 1 -and $frame.sequence -eq $i -and $frame.previousFrameSha256 -ceq $previous) 'Durable frame chain and sequence coherent'
  $prefix=[byte[]]($prefix+(Raw ($lines[$i]+"`n")));$previous=Get-SignedStoreDigest $prefix
  $replayed=New-SignedUpdateSession $frame.state.binding 1000
  foreach($receipt in $frame.state.receipts){
   $request=[pscustomobject]@{session=$receipt.session;sequence=$receipt.sequence;action=$receipt.action;requestNonce=$receipt.requestNonce;previousReceiptSha256=$receipt.previousReceiptSha256;evidence=$receipt.evidence}
   $null=Add-SignedUpdateReceipt $replayed $request $receipt.recordedUnix
  }
  Check ((Get-SignedUpdateReceiptHash $replayed) -ceq (Get-SignedUpdateReceiptHash $frame.state)) 'Saved full reducer state independently replays'
 }
 Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} 'Lost live owner never resumes from disk'
 Reject {New-SignedUpdateStore $directory (Json $binding)} 'Completed marker cannot be overwritten or silently rearmed'
 foreach($content in @('','{','{"schema":1}')){
  $p=PrivateDirectory;[IO.File]::WriteAllText((Join-Path $p 'signed-update-armed.jsonl'),$content)
  Reject {New-SignedUpdateStore $p (Json $binding)} 'Empty/corrupt/foreign abandoned marker refuses new owner'
 }
 # Reverting all disk bytes to a valid older head still conflicts with live memory.
 $h=Fresh;$s=Get-SignedStoreLive $h;$before=Read-SignedStoreHeldBytes $s
 $r=Request $h 'PreviousRestored' $restored 110;$null=Add-SignedUpdateStoreReceipt $h (Json $r)
 $s.stream.SetLength(0);$s.stream.Write($before,0,$before.Length);$s.stream.Flush($true)
 $r=Request $h 'CandidateCommissioned' $commissioned 111
 Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} 'Whole valid older journal/head replay refused'
 Check $s.poisoned 'Rollback detection abandons custody'
 Close-SignedUpdateStore $h
 # Closed, missing, corrupted and clock/identity loss cannot regain custody.
 foreach($fault in @('closed','corrupt','expired','backward','identity')){
  $h=Fresh;$s=Get-SignedStoreLive $h;$r=Request $h 'PreviousRestored' $restored 120
  switch($fault){
   closed {$s.stream.Dispose()}
   corrupt {$s.stream.Position=0;$s.stream.WriteByte(0);$s.stream.Flush($true)}
   expired {$script:clock=4000}
   backward {$script:clock=999}
   identity {function Get-SignedStoreOwner {return [pscustomobject]@{changed=$true}}}
  }
  Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} ('Lost '+$fault+' refuses')
  $script:clock=1000;${function:Get-SignedStoreOwner}=$ownerImpl
  Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} ('Lost '+$fault+' never recovers automatically')
  Close-SignedUpdateStore $h
 }
 # Failed initial arm must retain even an empty marker and issue no live handle.
 $p=PrivateDirectory
 function Write-SignedStoreFrame($Store,[byte[]]$Bytes){throw 'Injected initial flush failure'}
 Reject {New-SignedUpdateStore $p (Json $binding)} 'Initial arm failure returns no live custody'
 ${function:Write-SignedStoreFrame}=$writeImpl
 Check ([IO.File]::Exists((Join-Path $p 'signed-update-armed.jsonl'))) 'Initial arm failure preserves denial marker'
 Reject {New-SignedUpdateStore $p (Json $binding)} 'Failed initial arm cannot retry'
 # Inject failure before write, mid-append, and after durable flush but before return.
 foreach($fault in @('before','partial','after')){
  $h=Fresh;$s=Get-SignedStoreLive $h;$r=Request $h 'PreviousRestored' $restored 130
  $script:faultMode=$fault
  function Write-SignedStoreFrame($Store,[byte[]]$Bytes){
   if($script:faultMode -eq 'partial'){$Store.stream.Position=$Store.stream.Length;$Store.stream.Write($Bytes,0,17);$Store.stream.Flush($true)}
   elseif($script:faultMode -eq 'after'){& $script:writeImpl $Store $Bytes}
   throw 'Injected inert storage failure'
  }
  Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} ('Storage '+$fault+' failure returns no receipt')
  ${function:Write-SignedStoreFrame}=$writeImpl
  Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} ('Storage '+$fault+' failure consumes/abandons without retry')
  $p=$s.directory;Close-SignedUpdateStore $h
  Reject {New-SignedUpdateStore $p (Json $binding)} ('Storage '+$fault+' failure persists armed denial')
 }
 # Opt-in operation stores use the same durable writer, owner and anti-replay
 # custody, with a monotonic event sequence independent of receipt count.
 function EventRequest($Store,[string]$Action,$Evidence) {
  $o=$Store.operations
  return [pscustomobject]@{session=$o.session.binding.session;sequence=@($o.events).Count+1;action=$Action;requestNonce=(Hash (500+@($o.events).Count));previousEventSha256=$o.headSha256;evidence=$Evidence}
 }
 function Event($Store,[string]$Handle,[string]$Action,$Evidence) {return Add-SignedUpdateStoreEvent $Handle (Json (EventRequest $Store $Action $Evidence))}
 $preparation=[pscustomobject]@{identity=$new;stateSha256=$commissioned.stateSha256;ownershipSha256=$commissioned.ownershipSha256;preparedSha256=$commissioned.preparedSha256;transactionDirectory='C:\private\runs\candidate';contextSha256=(Hash 201);planSha256=(Hash 202);inventorySha256=(Hash 203);quarantineSha256=(Hash 204);baselineSha256=$restored.profileSha256;sourceReviewSha256=$binding.release.sourceReviewSha256}
 $h=Fresh;$s=Get-SignedStoreLive $h
 Reject {Event $s $h PreviousRestored $restored} 'Legacy custody cannot silently opt into operations'
 Close-SignedUpdateStore $h
 $p=PrivateDirectory;$h=New-SignedUpdateStore $p (Json $binding) -OperationEvents;$s=Get-SignedStoreLive $h
 Reject {Add-SignedUpdateStoreReceipt $h (Json (Request $h PreviousRestored $restored 500))} 'Receipt-only API cannot bypass operation mode'
 $null=Event $s $h PreviousRestored $restored
 $r=EventRequest $s CandidatePreparationArmed $preparation
 $duplicate=([Text.Encoding]::UTF8.GetString((Json $r))).Replace('"sequence":2','"sequence":2,"Sequence":2')
 Reject {Add-SignedUpdateStoreEvent $h (Raw $duplicate)} 'Operation parser rejects case-folded duplicate head/sequence keys'
 $null=Add-SignedUpdateStoreEvent $h (Json $r)
 $denial=[pscustomobject]@{preparationEventSha256=$s.operations.events[1].eventSha256;denialSha256=(Hash 205)}
 $r=EventRequest $s DeniedBeforeLaunch $denial;$e=Add-SignedUpdateStoreEvent $h (Json $r)
 Reject {Add-SignedUpdateStoreEvent $h (Json $r)} 'Lost denied response cannot append twice'
 $e.evidence.denialSha256=Hash 999
 Check ($s.operations.events[-1].evidence.denialSha256 -ceq $denial.denialSha256) 'Returned operation cannot mutate live owner state'
 Reject {Event $s $h CandidateCommissioned $commissioned} 'Denied live store cannot return to candidate approval'
 $intent=[pscustomobject]@{identity=$new;stateSha256=$preparation.stateSha256;ownershipSha256=$preparation.ownershipSha256;preparedSha256=$preparation.preparedSha256;transactionDirectory=$preparation.transactionDirectory;preparationEventSha256=$s.operations.events[1].eventSha256;denialEventSha256=$s.operations.events[2].eventSha256;inspectionSha256=(Hash 206);currentProfileSha256=(Hash 207);targetProfileSha256=$preparation.baselineSha256;targetTreesSha256=(Hash 208);preservationSha256=$null}
 $null=Event $s $h DeniedRestoreIntent $intent
 $complete=[pscustomobject]@{identity=$new;stateSha256=$preparation.stateSha256;ownershipSha256=$preparation.ownershipSha256;preparedSha256=$preparation.preparedSha256;transactionDirectory=$preparation.transactionDirectory;restoreIntentEventSha256=$s.operations.events[-1].eventSha256;inspectionSha256=$intent.inspectionSha256;restoreCompletionSha256=(Hash 209);profileSha256=$intent.targetProfileSha256;treesSha256=$intent.targetTreesSha256;activeMarkerAbsent=$true}
 $null=Event $s $h CandidateDeniedRestored $complete
 Check ($s.session.receipts.Count -eq 1 -and $s.operations.events.Count -eq 5) 'Durable denied restoration does not forge commissioning/launch receipts'
 $bytes=Read-SignedStoreHeldBytes $s;$lines=([Text.Encoding]::UTF8.GetString($bytes)).TrimEnd("`n").Split("`n")
 Check ($lines.Count -eq 6) 'One durable frame per operation including non-receipt transitions'
 $prefix=[byte[]]@();$previous=('0'*64)
 for($i=0;$i -lt $lines.Count;$i++) {
  $f=ConvertFrom-SignedStoreJson (Raw $lines[$i])
  Check ($f.schema -eq 2 -and $f.sequence -eq $i -and $f.previousFrameSha256 -ceq $previous) 'Operation frame head/sequence coherent'
  $replay=New-SignedUpdateOperationSession $binding 1000
  foreach($e in $f.state.events){$request=[pscustomobject]@{session=$e.session;sequence=$e.sequence;action=$e.action;requestNonce=$e.requestNonce;previousEventSha256=$e.previousEventSha256;evidence=$e.evidence};$null=Add-SignedUpdateOperationEvent $replay $request $e.recordedUnix}
  Check ((Get-SignedUpdateReceiptHash $replay) -ceq (Get-SignedUpdateReceiptHash $f.state)) 'Durable operation state independently replays for inspection only'
  $prefix=[byte[]]($prefix+(Raw ($lines[$i]+"`n")));$previous=Get-SignedStoreDigest $prefix
 }
 Reject {Event $s $h CandidatePreparationArmed $preparation} 'Completed denied operation store stays terminal'
 Close-SignedUpdateStore $h
 Reject {Add-SignedUpdateStoreEvent $h (Json $r)} 'Closed operation owner cannot resume'
 Reject {New-SignedUpdateStore $p (Json $binding) -OperationEvents} 'Completed operation disk marker cannot rearm'
 # Exercise lost/failed spawn consumption across the actual append/flush boundary.
 foreach($fault in @('lost-response','before','partial','after','replay','identity','expired')) {
  $h=New-SignedUpdateStore (PrivateDirectory) (Json $binding) -OperationEvents;$s=Get-SignedStoreLive $h
  $null=Event $s $h PreviousRestored $restored;$null=Event $s $h CandidatePreparationArmed $preparation
  $null=Event $s $h CandidateCommissioned $commissioned
  $before=Read-SignedStoreHeldBytes $s
  $spawn=[pscustomobject]@{commissionReceiptSha256=$s.session.receipts[-1].receiptSha256;creationRequestSha256=(Hash 210);environmentSha256=$commissioned.environmentSha256}
  $r=EventRequest $s CandidateSpawnConsumed $spawn
  if($fault -cin @('before','partial','after')) {
   $script:faultMode=$fault
   function Write-SignedStoreFrame($Store,[byte[]]$Bytes){
    if($script:faultMode -eq 'partial'){$Store.stream.Position=$Store.stream.Length;$Store.stream.Write($Bytes,0,17);$Store.stream.Flush($true)}
    elseif($script:faultMode -eq 'after'){& $script:writeImpl $Store $Bytes}
    throw 'Injected operation append failure'
   }
   Reject {Add-SignedUpdateStoreEvent $h (Json $r)} ('Consumed spawn storage fault '+$fault)
   ${function:Write-SignedStoreFrame}=$writeImpl
  } else {
   $null=Add-SignedUpdateStoreEvent $h (Json $r) # Deliberately discard result.
   if($fault -ceq 'replay'){$s.stream.SetLength(0);$s.stream.Write($before,0,$before.Length);$s.stream.Flush($true)}
   elseif($fault -ceq 'identity'){function Get-SignedStoreOwner {return [pscustomobject]@{changed=$true}}}
   elseif($fault -ceq 'expired'){$script:clock=4000}
  }
  Reject {Add-SignedUpdateStoreEvent $h (Json $r)} ('No spawn replay after '+$fault)
  $script:clock=1000;${function:Get-SignedStoreOwner}=$ownerImpl
  $denial=[pscustomobject]@{preparationEventSha256=$s.operations.events[1].eventSha256;denialSha256=(Hash 211)}
  Reject {Event $s $h DeniedBeforeLaunch $denial} ('No false unlaunched recovery after '+$fault)
  $p=$s.directory;Close-SignedUpdateStore $h
  Reject {New-SignedUpdateStore $p (Json $binding) -OperationEvents} 'Uncertain operation custody cannot resume from disk'
 }
 if($native){
  # Actual Windows security and file-identity behavior, not portable substitutes.
  $p=PrivateDirectory;$acl=Get-Acl -LiteralPath $p
  $acl.AddAccessRule([Security.AccessControl.FileSystemAccessRule]::new([Security.Principal.SecurityIdentifier]::new('S-1-1-0'),[Security.AccessControl.FileSystemRights]::Read,[Security.AccessControl.AccessControlType]::Allow))
  Set-Acl -LiteralPath $p -AclObject $acl
  Reject {New-SignedUpdateStore $p (Json $binding)} 'Other-principal ACL refused'
  $p=PrivateDirectory;$h=New-SignedUpdateStore $p (Json $binding);$s=Get-SignedStoreLive $h
  Reject {[IO.File]::Delete($s.path)} 'Held native armed file cannot be deleted'
  Reject {[IO.Directory]::Move($p,($p+'-moved'))} 'Held ancestor cannot be renamed'
  $replacement=Join-Path $fixtureRoot 'replacement';[IO.File]::WriteAllText($replacement,'old state')
  Reject {[IO.File]::Replace($replacement,$s.path,$null)} 'Native atomic replacement refused'
  $acl=Get-Acl -LiteralPath $p;$acl.SetAccessRuleProtection($false,$true);Set-Acl -LiteralPath $p -AclObject $acl
  $r=Request $h 'PreviousRestored' $restored 140
  Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} 'ACL weakening during custody abandons store'
  Close-SignedUpdateStore $h
  # Validate hardlink detection against a separate inert file held by real native
  # handles. No need to relax the production armed file's exclusive sharing.
  $one=Join-Path $fixtureRoot 'one-link';$two=Join-Path $fixtureRoot 'two-links'
  [IO.File]::WriteAllText($one,'inert')
  $null=New-Item -ItemType HardLink -Path $two -Target $one
  $held=[IO.File]::Open($one,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try{Reject {[SignedUpdateStoreNative]::Check($held.SafeFileHandle,$false)} 'Native multiple-link handle refused'}finally{$held.Dispose()}
  [IO.File]::Delete($two)
  $held=[IO.File]::Open($one,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try{[SignedUpdateStoreNative]::Check($held.SafeFileHandle,$false);Check $true 'Native single-link regular file accepted'}finally{$held.Dispose()}
  foreach($nonlocal in @('relative','C:\\bad','\\server\share','C:\bad\..\path')){
   Reject {[SignedUpdateStoreNative]::PinDirectories($nonlocal)} 'Noncanonical/remote directory refused'
  }
  $target=PrivateDirectory;$link=Join-Path $fixtureRoot 'junction'
  $null=New-Item -ItemType Junction -Path $link -Target $target
  try{Reject {New-SignedUpdateStore $link (Json $binding)} 'Native reparse parent refused'}finally{[IO.Directory]::Delete($link)}
 } else {
  $h=Fresh;$s=Get-SignedStoreLive $h;$r=Request $h 'PreviousRestored' $restored 150
  # Unix permits unlink despite FileShare.None; fake native boundary detects it.
  [IO.File]::Delete($s.path)
  Reject {Add-SignedUpdateStoreReceipt $h (Json $r)} 'Missing armed marker fails closed'
  Close-SignedUpdateStore $h
 }
 Write-Output ('Signed update store: '+$checks+' inert checks passed; nativeWindows='+$native+'. No launch authority or resumable recovery claimed.')
} finally {
 ${function:Write-SignedStoreFrame}=$writeImpl;${function:Get-SignedStoreOwner}=$ownerImpl;${function:Assert-SignedStoreNative}=$nativeImpl;${function:New-SignedStoreFile}=$fileImpl
 foreach($h in @($script:SignedUpdateStores.Keys)){Close-SignedUpdateStore $h}
 if(Test-Path -LiteralPath $fixtureRoot){Remove-Item -LiteralPath $fixtureRoot -Recurse -Force}
}
