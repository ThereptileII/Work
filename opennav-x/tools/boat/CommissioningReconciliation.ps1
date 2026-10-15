# Explicit cold reconciliation only. Never select, launch or relabel a generation.
$script:ReconciliationOwner='OpenNavX.ColdCommissioningReconciliation.1'
# A complete observed profile/plugin/ACL inventory is already 4.53 MB. Keep
# bounded headroom for that owned record only; all generic records stay 4 MiB.
$script:ReconciliationInspectionMaxBytes=16777216
function Assert-ReconciliationInspectionPath($Context,[string]$Path) {
  $path=Assert-LocalPath $Path;$directory=[IO.Path]::GetDirectoryName($path)
  if([IO.Path]::GetFileName($path) -cne 'inspection.json' -or
     [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Context.workspace 'runs') -or
     [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-reconciliation-[a-f0-9]{8}$') {
    throw 'Owned cold reconciliation inspection path required.'
  }
  Assert-ColdPrivateEvidence $directory $Context.sid
  return $path
}
function Assert-ReconciliationInspectionRecord($Record,$Context) {
  $fields=@('schema','owner','status','parentPrepared','parentPreparedSha256','currentContext','activeMarkerSha256',
    'profileTree','currentPluginTrees','acls','currentIniSha256','connectionDisposition','diff','applicationLaunched')
  if((@($Record.PSObject.Properties.Name|Sort-Object)|ConvertTo-Json -Compress) -cne
     (@($fields|Sort-Object)|ConvertTo-Json -Compress) -or
     ($Record.schema -isnot [int] -and $Record.schema -isnot [long]) -or $Record.schema -ne 1 -or
     $Record.owner -cne $script:ReconciliationOwner -or $Record.status -cne 'inspected' -or
     $Record.applicationLaunched -isnot [bool] -or $Record.applicationLaunched -or
     $Record.currentContext.workspace -cne $Context.workspace -or $Record.currentContext.sid -cne $Context.sid -or
     $Record.currentContext.profile -cne $Context.profile -or $Record.profileTree.root -cne $Context.profile -or
     $Record.profileTree.exists -isnot [bool] -or -not $Record.profileTree.exists -or
     $Record.connectionDisposition -cnotin @('preserve-current-original-direction','restore-original-temporary-direction')) {
    throw 'Exact inspected reconciliation inventory kind and context required.'
  }
  foreach($hash in @($Record.parentPreparedSha256,$Record.activeMarkerSha256,$Record.currentIniSha256)) {
    if($hash -isnot [string] -or $hash -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact inspection identity hashes required.'}
  }
}
function Read-ReconciliationInspection($Context,[string]$Path,[string]$Hash) {
  $path=Assert-ReconciliationInspectionPath $Context $Path
  if($Hash -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact reconciliation inspection hash required.'}
  # Bind the hash and parsed document to the same bounded bytes. Deny concurrent
  # writers while reading, rather than hashing one open and parsing another.
  $stream=New-Object IO.FileStream($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try {
    if($stream.Length -le 0 -or $stream.Length -gt $script:ReconciliationInspectionMaxBytes){throw 'Reconciliation inspection size outside 16 MiB bound.'}
    $bytes=New-Object byte[] ([int]$stream.Length);$offset=0
    while($offset -lt $bytes.Length) {
      $read=$stream.Read($bytes,$offset,$bytes.Length-$offset)
      if($read -eq 0){throw 'Reconciliation inspection ended before its bounded length.'}
      $offset+=$read
    }
  } finally {$stream.Dispose()}
  if((Get-CommissioningHash $bytes) -cne $Hash){throw 'Reconciliation inspection bytes changed.'}
  $record=(New-Object Text.UTF8Encoding($false,$true)).GetString($bytes)|ConvertFrom-Json
  Assert-ReconciliationInspectionRecord $record $Context
  return $record
}
function Write-ReconciliationInspection($Context,[string]$Path,$Record) {
  $path=Assert-ReconciliationInspectionPath $Context $Path
  $text=$Record|ConvertTo-Json -Depth 16
  $bytes=(New-Object Text.UTF8Encoding($false,$true)).GetBytes($text)
  if($bytes.Length -le 0 -or $bytes.Length -gt $script:ReconciliationInspectionMaxBytes){throw 'Reconciliation inspection size outside 16 MiB bound.'}
  Assert-ReconciliationInspectionRecord ($text|ConvertFrom-Json) $Context
  Write-Record $path $Record
  $hash=Get-Digest $path
  # Inspect must not announce review-required for a document Plan cannot read.
  $null=Read-ReconciliationInspection $Context $path $hash
  return $hash
}
function Assert-ReconciliationContexts($Old,$Current) {
  foreach($key in @('sid','localAppData','profile','application','managed','workspace','executable')) {
    if($Old.$key -cne $Current.$key){throw 'Reconciliation requires the same actual account, profile and original roots.'}
  }
  if(-not $Old.installation -or -not $Current.installation -or $Old.installation.root -cne $Current.installation.root -or
     $Old.installation.generation -ceq $Current.installation.generation -or $Current.session -le 0){throw 'Two distinct owned generations and a current interactive session are required.'}
  foreach($context in @($Old,$Current)) {
    $i=$context.installation;$root=Assert-LocalPath $i.root;$generation=Assert-LocalPath $i.generation
    if([IO.Path]::GetDirectoryName($generation) -ine (Join-Path $root 'generations') -or
       [IO.Path]::GetFileName($generation) -cnotmatch '^[a-f0-9]{32}$' -or
       (Assert-LocalPath $i.executable) -ine (Join-Path $generation 'app/opencpn.exe') -or $i.commit -cnotmatch '^[a-f0-9]{40}$') {throw 'Canonical owned generation identity required.'}
    foreach($hash in @($i.stateSha256,$i.ownershipSha256,$i.executableSha256)){if($hash -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact generation identity hashes required.'}}
    $expected=@($context.managed,(Join-Path $context.application 'plugins'),(Join-Path $generation 'app/plugins')|Sort-Object -Unique)
    if(($expected|ConvertTo-Json -Compress) -cne (@($context.pluginRoots)|ConvertTo-Json -Compress)){throw 'Exact original and installed loader roots required.'}
  }
}
function Read-ReconciliationParent([string]$Workspace,[string]$Record,[string]$Hash,[int]$Depth=0) {
  $record=Assert-LocalPath $Record;$directory=[IO.Path]::GetDirectoryName($record)
  if([IO.Path]::GetFileName($record) -cne 'prepared.json' -or [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs')){throw 'Original workspace prepared record required.'}
  $parent=Read-SessionPreservationParent $record $Hash
  if($parent.context.workspace -ine $Workspace){throw 'Original transaction belongs to another workspace.'}
  $baseline=Get-PreparedCommissioningBaseline $parent $directory $Workspace $Depth
  $inventoryPath=Join-Path $directory 'inventory.json'
  if((Get-Digest $inventoryPath) -cne $parent.inventorySha256){throw 'Original inventory changed.'}
  $inventory=Read-Record $inventoryPath;Assert-CommissioningInventory $inventory $parent.context
  $plan=Read-Record (Join-Path $directory 'review-plan.json')
  $moves=@($plan.plugins|Where-Object {$_.decision -ceq 'quarantine'})
  if(@($parent.quarantine).Count -ne $moves.Count -or $moves.Count -lt 1 -or $moves.Count -gt 256){throw 'Complete bounded original quarantine required.'}
  $seen=@{};$destinations=@{};$backups=@{}
  foreach($item in @($parent.quarantine)) {
    $path=Assert-LocalPath $item.path;$destination=Assert-LocalPath $item.destination;$backup=Assert-LocalPath $item.backup
    $match=@($moves|Where-Object {$_.path -ieq $path -and $_.sha256 -ceq $item.sha256})
    $roots=@($inventory.trees|Where-Object {$path.StartsWith($_.root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)})
    $candidate=@($inventory.plugins|Where-Object {$_.path -ieq $path -and $_.sha256 -ceq $item.sha256})
    if($seen.ContainsKey($path) -or $destinations.ContainsKey($destination) -or $backups.ContainsKey($backup) -or
       $candidate.Count -ne 1 -or $match.Count -ne 1 -or $roots.Count -ne 1 -or [IO.Path]::GetFileName($path) -notlike '*_pi.dll' -or
       [IO.Path]::GetDirectoryName($destination) -ine (Join-Path $directory 'quarantine') -or [IO.Path]::GetDirectoryName($backup) -ine $directory){throw 'Each original quarantine path must have independent, exact ownership.'}
    $seen[$path]=$true;$destinations[$destination]=$true;$backups[$backup]=$true
  }
  return [pscustomobject]@{parent=$parent;inventory=$inventory;directory=$directory;record=$record;sha256=$Hash;baseline=$baseline}
}
function Restore-ReconciliationPlugin($Item) {
  # Caller already owns the plan/profile/transaction locks and exact original
  # path inventory. File.Move never overwrites an occupied destination.
  [IO.File]::Move($Item.destination,$Item.path)
  if((Get-Digest $Item.path) -cne $Item.sha256){throw 'Restored original plugin bytes changed.'}
  Assert-PreparationAcl $Item.acl (Get-PreparationAccessAcl $Item.path)
}
function Assert-ReconciliationLiveIdentity($Proof,$Current) {
  Assert-ReconciliationContexts $Proof.parent.context $Current
  $old=$Proof.parent.context.installation
  if(-not [IO.Directory]::Exists($old.generation) -or (Get-Digest $old.executable) -cne $old.executableSha256 -or
     (Get-Digest (Join-Path $old.generation 'ownership.json')) -cne $old.ownershipSha256){throw 'Original generation retired, missing or changed; no path recreation or plugin transplant.'}
  $owner=Read-Record (Join-Path $old.generation 'ownership.json')
  $exe=@($owner.managedFiles|Where-Object {$_.path -ceq 'app/opencpn.exe'})
  if($owner.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $owner.commit -cne $old.commit -or
     $exe.Count -ne 1 -or $exe[0].sha256 -cne $old.executableSha256){throw 'Original generation ownership does not authenticate its executable.'}
  foreach($item in @($Proof.parent.quarantine)) {
    if($item.path.StartsWith($Current.installation.generation+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)){throw 'Old plugins must never enter the current generation.'}
  }
  Assert-PreparationClosed (@($Proof.parent.context.pluginRoots)+@($Current.pluginRoots))
}
function Assert-ReconciliationActive($Proof,[string]$ExpectedMarkerHash) {
  $path=Join-Path $Proof.parent.context.workspace 'commissioning-active.json'
  if($ExpectedMarkerHash -and (Get-Digest $path) -cne $ExpectedMarkerHash){throw 'Active ownership bytes changed.'}
  $active=Read-Record $path
  if($active.schema -ne 1 -or $active.owner -cne $script:CommissioningOwner -or $active.record -ine $Proof.record -or
     $active.recordSha256 -cne $Proof.sha256){throw 'Exact original active transaction required.'}
}
function Assert-ReconciliationMoves($Proof,[string]$Directory,[switch]$Resume) {
  Assert-CommissioningTrees $Proof.inventory.trees $Proof.parent.quarantine -AllowMoved
  $index=0
  foreach($item in @($Proof.parent.quarantine)) {
    $journal=Join-Path $Directory ('plugin-'+$index+'.json');$index++
    $source=[IO.File]::Exists($item.path);$quarantined=[IO.File]::Exists($item.destination)
    if($source -eq $quarantined){throw 'Ambiguous original/quarantined plugin locations.'}
    if(Test-Path -LiteralPath $journal) {
      $intent=Read-Record $journal
      if(-not $Resume -or $intent.owner -cne $script:ReconciliationOwner -or $intent.parentPreparedSha256 -cne $Proof.sha256 -or
         $intent.source -ine $item.destination -or $intent.destination -ine $item.path -or $intent.sha256 -cne $item.sha256){throw 'Plugin restoration journal differs from its original path.'}
    } elseif($source){throw 'Occupied original plugin destination; no overwrite or inferred restoration.'}
    $actual=if($source){$item.path}else{$item.destination}
    Assert-PreparationAcl $item.acl (Get-PreparationAccessAcl $actual)
    if(-not [IO.Directory]::Exists([IO.Path]::GetDirectoryName($item.path))){throw 'Missing original destination directory; no recreation.'}
  }
}
function Get-ReconciliationProfileBytes($Proof,[string]$Saved) {
  $before=Read-ProfileForAudit (Join-Path $Proof.directory 'input-only.ini')
  $original=Read-ProfileForAudit (Join-Path $Proof.directory 'baseline.ini');$current=Read-ProfileForAudit $Saved
  $key='Settings/NMEADataSource/DataConnections';$bytes=[IO.File]::ReadAllBytes($Saved)
  if($current[$key] -ceq $original[$key]) {
    # The user has already restored the original direction. Preserve ALL live
    # bytes; normalization is a separate review copy, never a live mutation.
    return [pscustomobject]@{input=(Get-CommissioningInputBytes $bytes);baseline=$bytes;disposition='preserve-current-original-direction'}
  }
  if($current[$key] -ceq $before[$key]) {
    return [pscustomobject]@{input=$bytes;baseline=(Get-CommissioningOutputBytes $bytes);disposition='restore-original-temporary-direction'}
  }
  throw 'Connection values differ from both exact original and temporary forms; no schema guessing or network/output changes.'
}
function Test-ReconciliationRetentionReview($Review) {
  if($Review.schema -eq 2 -and $Review.owner -ceq 'OpenNavX.ColdReconciliationRetentionReview.2'){
    if($Review.mode -cne 'retain-existing-original-direction-no-grants'){throw 'Exact cold-retention review mode required.'}
    $fields=@('schema','owner','mode','beforeSha256','afterSha256','parentPreparedSha256','inspectionSha256','connectionDisposition','provenance','preservationOnly','launchPermission','reviewedUtc','changes','retentionAcknowledgments','wmmRetention')
    if((@($Review.PSObject.Properties.Name|Sort-Object)|ConvertTo-Json -Compress) -cne (@($fields|Sort-Object)|ConvertTo-Json -Compress)){throw 'Exact cold-retention review fields required.'}
    return $true
  }
  if($Review.schema -ne 1 -or $Review.owner -cne 'OpenNavX.SessionPreservationReview.1' -or
     $Review.PSObject.Properties['mode'] -or $Review.PSObject.Properties['retentionAcknowledgments'] -or
     $Review.PSObject.Properties['wmmRetention']){throw 'Unsupported reconciliation review; ordinary v1 preservation remains strict.'}
  return $false
}
function Assert-ReconciliationRetentionClaims($Value) {
  if($Value.retentionMode -cne 'retain-existing-original-direction-no-grants'){throw 'Exact retained-current-state lineage mode required.'}
  foreach($name in @('pilotBindingQualified','commandsAuthorized','profilePublished')){
    if($Value.$name -isnot [bool] -or $Value.$name){throw 'Retained state cannot claim pilot qualification, commands or profile publication.'}
  }
}
function Read-ReconciliationPilotSettings([string]$Value) {
  # Settings.cpp EncodeSettings/DecodeSettings and St4000Pilot.cpp: a bounded
  # base record plus these four exact pilot scalars. Never decode mappings,
  # calibration, an unknown field, or an escaped/multiline identity as a scalar.
  if($Value.Length -gt 65536 -or -not $Value.StartsWith('OpenNavXSettings 1\n') -or $Value -match '[\x00-\x1f\x7f]'){throw 'Unsupported retained settings encoding.'}
  $base=$Value;$pilot=@{}
  foreach($field in @('address','interface','name','permission')){
    $pattern='(?<=\\n)"pilot\.'+$field+'" "([^"\\\x00-\x1f]*)"\\n'
    $matches=[regex]::Matches($Value,$pattern)
    if($matches.Count -gt 1 -or $matches.Count -ne [regex]::Matches($Value,'(?<=\\n)"pilot\.'+$field+'" ').Count){throw 'Duplicate or malformed retained pilot scalar.'}
    if($matches.Count){$pilot[$field]=$matches[0].Groups[1].Value}
    $base=[regex]::Replace($base,$pattern,'')
  }
  Assert-PreservedAlphaSettings $base
  if($pilot.Count){
    if(-not $pilot.ContainsKey('interface') -or -not $pilot.ContainsKey('name') -or -not $pilot.ContainsKey('permission') -or
       $pilot.interface -cne 'COM8' -or $pilot.permission -cnotin @('display-only','manual') -or $pilot.name -cnotmatch '^(?:|[a-f0-9]{16})$'){throw 'Supported retained COM8 pilot encoding required.'}
    if($pilot.name){
      $name=[Convert]::ToUInt64($pilot.name,16)
      if((($name -shr 21) -band 0x7ff) -ne 1851 -or (($name -shr 40) -band 0xff) -ne 135 -or
         (($name -shr 49) -band 0x7f) -ne 40 -or (($name -shr 60) -band 7) -ne 4){throw 'Unsupported retained pilot NAME class.'}
    }
    if($pilot.ContainsKey('address') -and ($pilot.address -cnotmatch '^(?:0|[1-9][0-9]{0,2})$' -or [int]$pilot.address -gt 253)){throw 'Unsupported retained pilot source address.'}
    if($pilot.permission -ceq 'manual' -and -not $pilot.name -and -not $pilot.ContainsKey('address')){throw 'Saved manual permission lacks an explicit stored identity.'}
  }
  return [pscustomobject]@{base=$base;pilot=$pilot}
}
function Assert-ReconciliationAlphaRetention([string]$Before,[string]$After) {
  $old=Read-ReconciliationPilotSettings $Before;$current=Read-ReconciliationPilotSettings $After
  $remaining=@()
  foreach($record in @($old.base,$current.base)){
    $copy=$record
    foreach($key in @('capacity','reserve','draft','model_source','display.rail')){
      $copy=[regex]::Replace($copy,'(?<=\\n)"'+[regex]::Escape($key)+'" "[^"\\]*"(?=\\n)',('"'+$key+'" "<retained-known-field>"'))
    }
    $remaining+=,$copy
  }
  if($remaining[0] -cne $remaining[1]){throw 'Cold retention cannot admit changed mappings, calibration or other vessel settings.'}
}
function Assert-ReconciliationGenerationWmmProof($Context,$Value) {
  $i=$Context.installation
  if(@($Value.PSObject.Properties).Count -ne 7 -or $Value.schema -ne 1 -or $Value.generation -cne $i.generation -or $Value.commit -cne $i.commit -or
     $Value.ownershipSha256 -cne $i.ownershipSha256 -or $Value.location -cne (Get-CommissioningWmmLocation (Join-Path $i.generation 'app')) -or
     $Value.ownershipBase64 -isnot [string] -or $Value.ownershipBase64.Length -gt 1398104){throw 'Exact retained-generation WMM identity required.'}
  $raw=[Convert]::FromBase64String($Value.ownershipBase64)
  if($raw.Length -le 0 -or $raw.Length -gt 1048576 -or (Get-CommissioningHash $raw) -cne $i.ownershipSha256){throw 'Retained WMM ownership bytes changed.'}
  $owner=(New-Object Text.UTF8Encoding($false,$true)).GetString($raw)|ConvertFrom-Json
  if($owner.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $owner.commit -cne $i.commit){throw 'Unowned retained WMM generation.'}
  $pins=Get-CommissioningWmmResourcePins
  if(@($Value.resources).Count -ne $pins.Count){throw 'Complete pinned WMM resource inventory required.'}
  foreach($name in $pins.Keys){
    $file=@($Value.resources|Where-Object {$_.name -ceq $name});$path='app/plugins/wmm_pi/data/'+$name
    $owned=@($owner.managedFiles|Where-Object {$_.path -ieq $path})
    if($file.Count -ne 1 -or @($file[0].PSObject.Properties).Count -ne 3 -or $file[0].sha256 -cne $pins[$name] -or $file[0].bytes -le 0 -or $file[0].bytes -gt 1048576 -or
       $owned.Count -ne 1 -or $owned[0].path -cne $path -or $owned[0].sha256 -cne $pins[$name]){throw 'WMM resources lack exact source pins and unique generation ownership.'}
  }
}
function Get-ReconciliationGenerationWmmProof($Context) {
  $i=$Context.installation;$owner=Assert-LocalPath (Join-Path $i.generation 'ownership.json')
  if((Get-Item -LiteralPath $owner).Length -gt 1048576 -or (Get-Digest $owner) -cne $i.ownershipSha256){throw 'Live retained WMM generation ownership changed.'}
  $directory=Assert-LocalPath (Join-Path $i.generation 'app/plugins/wmm_pi/data');$items=@(Get-ChildItem -LiteralPath $directory -Force)
  $pins=Get-CommissioningWmmResourcePins;$resources=@()
  if($items.Count -ne $pins.Count){throw 'Unexpected retained WMM directory entries.'}
  foreach($name in $pins.Keys){
    $file=@($items|Where-Object {$_.Name -ceq $name})
    if($file.Count -ne 1 -or $file[0].PSIsContainer){throw 'Pinned retained WMM file required.'}
    $path=Assert-LocalPath $file[0].FullName
    $resources+=[pscustomobject]@{name=$name;bytes=$file[0].Length;sha256=(Get-Digest $path)}
  }
  $value=[pscustomobject]@{schema=1;generation=$i.generation;commit=$i.commit;ownershipSha256=$i.ownershipSha256;
    location=(Get-CommissioningWmmLocation (Join-Path $i.generation 'app'));ownershipBase64=[Convert]::ToBase64String([IO.File]::ReadAllBytes($owner));resources=$resources}
  Assert-ReconciliationGenerationWmmProof $Context $value
  return $value
}
function Get-ReconciliationWmmOriginalReferenceGeneration($Proof,$Current) {
  # The original input-only bytes, never a review-supplied path, select this
  # resource reference. It does not replace the prepared-parent identity.
  $values=Read-ProfileForAudit (Join-Path $Proof.directory 'input-only.ini')
  $location=$values['Directories/WMMDataLocation'];$old=$Proof.parent.context.installation
  if($location -ceq (Get-CommissioningWmmLocation (Join-Path $old.generation 'app'))){return $null}
  if($old.root -cne $Current.installation.root -or $location -isnot [string]){throw 'Original WMM reference requires the same owned installation root.'}
  $generations=Assert-LocalPath (Join-Path $old.root 'generations');$separator=[string][IO.Path]::DirectorySeparatorChar
  $prefix=$generations+$separator;$decoded=$location.Replace('\\','\')
  $suffix=$separator+'app'+$separator+'plugins'+$separator+'wmm_pi'+$separator+'data'+$separator
  if(-not $decoded.StartsWith($prefix,[StringComparison]::Ordinal) -or
     $decoded.Substring($prefix.Length) -cnotmatch ('^[a-f0-9]{32}'+[regex]::Escape($suffix)+'$')){throw 'Exact original WMM path within the owned generations directory required.'}
  $generation=Assert-LocalPath (Join-Path $generations $decoded.Substring($prefix.Length,32))
  if($generation -ceq $old.generation -or $generation -ceq $Current.installation.generation -or
     $location -cne (Get-CommissioningWmmLocation (Join-Path $generation 'app'))){throw 'Canonical distinct historical original WMM reference required.'}
  return $generation
}
function Get-ReconciliationWmmOriginalReferenceProof([string]$Generation) {
  $path=Assert-LocalPath (Join-Path $Generation 'ownership.json')
  $stream=New-Object IO.FileStream($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try {
    if($stream.Length -le 0 -or $stream.Length -gt 1048576){throw 'Bounded historical WMM ownership required.'}
    $bytes=New-Object byte[] ([int]$stream.Length);$offset=0
    while($offset -lt $bytes.Length){$read=$stream.Read($bytes,$offset,$bytes.Length-$offset);if($read -eq 0){throw 'Historical WMM ownership ended early.'};$offset+=$read}
  }finally{$stream.Dispose()}
  $owner=(New-Object Text.UTF8Encoding($false,$true)).GetString($bytes)|ConvertFrom-Json
  if($owner.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $owner.commit -isnot [string] -or $owner.commit -cnotmatch '^[a-f0-9]{40}$'){throw 'Owned historical WMM generation required.'}
  $context=[pscustomobject]@{installation=[pscustomobject]@{generation=$Generation;commit=$owner.commit;ownershipSha256=(Get-CommissioningHash $bytes)}}
  return Get-ReconciliationGenerationWmmProof $context
}
function Get-ReconciliationWmmRetention($Proof,$Current) {
  $old=Get-ReconciliationGenerationWmmProof $Proof.parent.context;$currentProof=Get-ReconciliationGenerationWmmProof $Current
  $reference=Get-ReconciliationWmmOriginalReferenceGeneration $Proof $Current
  if($reference){return [pscustomobject]@{schema=2;old=$old;current=$currentProof;originalReference=(Get-ReconciliationWmmOriginalReferenceProof $reference)}}
  return [pscustomobject]@{schema=1;old=$old;current=$currentProof}
}
function Assert-ReconciliationWmmRetention($Proof,$Current,$Value,[switch]$Live) {
  $reference=Get-ReconciliationWmmOriginalReferenceGeneration $Proof $Current
  if($reference){
    $fields=@('schema','old','current','originalReference')
    if(($Value.schema -isnot [int] -and $Value.schema -isnot [long]) -or $Value.schema -ne 2 -or (@($Value.PSObject.Properties.Name|Sort-Object)|ConvertTo-Json -Compress) -cne
       (@($fields|Sort-Object)|ConvertTo-Json -Compress)){throw 'Explicit historical original-reference WMM proof required.'}
  }elseif(@($Value.PSObject.Properties).Count -ne 3 -or $Value.schema -ne 1){throw 'Explicit two-generation WMM retention proof required.'}
  Assert-ReconciliationGenerationWmmProof $Proof.parent.context $Value.old
  Assert-ReconciliationGenerationWmmProof $Current $Value.current
  if(($Value.old.resources|ConvertTo-Json -Depth 8 -Compress) -cne ($Value.current.resources|ConvertTo-Json -Depth 8 -Compress)){throw 'Old and current WMM source bytes differ.'}
  if($reference){
    $r=$Value.originalReference
    if($r.commit -isnot [string] -or $r.commit -cnotmatch '^[a-f0-9]{40}$' -or
       $r.ownershipSha256 -isnot [string] -or $r.ownershipSha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact historical original-reference WMM identity required.'}
    $context=[pscustomobject]@{installation=[pscustomobject]@{generation=$reference;commit=$r.commit;ownershipSha256=$r.ownershipSha256}}
    Assert-ReconciliationGenerationWmmProof $context $r
    if(($Value.old.resources|ConvertTo-Json -Depth 8 -Compress) -cne ($r.resources|ConvertTo-Json -Depth 8 -Compress)){throw 'Historical original-reference WMM source bytes differ.'}
  }
  if($Live){
    $actual=Get-ReconciliationWmmRetention $Proof $Current
    if(($actual|ConvertTo-Json -Depth 12 -Compress) -cne ($Value|ConvertTo-Json -Depth 12 -Compress)){throw 'Live WMM retention generation proof changed.'}
  }
}
function Assert-ReconciliationCurrentRetention($Proof,[string]$Saved,[string]$Normalized,$Review,$Inspection,[datetime]$At) {
  if(-not $Inspection -or $Review.connectionDisposition -cne 'preserve-current-original-direction' -or
     $Inspection.connectionDisposition -cne $Review.connectionDisposition -or $Inspection.currentIniSha256 -cne (Get-Digest $Saved)){
    throw 'Current-state retention requires original-direction bytes; no profile publication or permission changes.'
  }
  $required=@('retainExistingBytes','originUnverified','pilotIdentityUnverified','savedPermissionIsNotAuthorization','noProfilePublication','noApplicationLaunch','noCommands','freshCommissioningRequired')
  if(@($Review.retentionAcknowledgments.PSObject.Properties).Count -ne $required.Count){throw 'Every explicit retention risk acknowledgment is required.'}
  foreach($name in $required){Assert-TrueBoolean $Review.retentionAcknowledgments.$name ('Cold retention '+$name)}
  $old=Read-ProfileForAudit (Join-Path $Proof.directory 'input-only.ini');$new=Read-ProfileForAudit $Normalized
  Assert-InputOnlyProfile $new
  if($old['Settings/PersistActiveRoute'] -cne '0' -or $new['Settings/PersistActiveRoute'] -cne '0'){throw 'Retained route persistence must remain OFF.'}
  $display=Get-RestartDisplayKeys;$wmm=$false
  foreach($change in @(Get-CommissioningIniDiff (Join-Path $Proof.directory 'input-only.ini') $Normalized)){
    $key=$change.key
    if($display.ContainsKey($key)){
      Assert-RestartScalar $display[$key] $change.after
      if($null -ne $change.before){Assert-RestartScalar $display[$key] $change.before}
    }elseif($key -ceq 'OpenNav/AlphaSettings'){
      if($null -eq $change.before -or $null -eq $change.after){throw 'Retained AlphaSettings must have both original and current records.'}
      Assert-ReconciliationAlphaRetention $change.before $change.after
    }elseif($key -ceq 'Directories/WMMDataLocation'){
      $wmm=$true;Assert-ReconciliationWmmRetention $Proof $Inspection.currentContext $Review.wmmRetention
      $originalLocation=if($Review.wmmRetention.schema -eq 2){$Review.wmmRetention.originalReference.location}else{$Review.wmmRetention.old.location}
      if($change.before -cne $originalLocation -or $change.after -cne $Review.wmmRetention.current.location){throw 'Only exact original-reference-to-current owned WMM locations can be retained.'}
    }elseif($key -ceq 'OpenNav/OnlineAIS/v1/RadiusNm'){
      foreach($value in @($change.before,$change.after)){
        if($null -ne $value -and ($value -cnotmatch '^[1-9][0-9]{0,2}$' -or [int]$value -gt 200)){throw 'Retained AIS radius outside product bounds.'}
      }
      if($null -eq $change.after){throw 'Retained AIS radius deletion is unsupported.'}
    }elseif($key -ceq 'OpenNav/Weather/v1/Enabled'){
      if($change.after -cnotin @('0','1') -or ($null -ne $change.before -and $change.before -cnotin @('0','1'))){throw 'Retained weather enable flag must be an exact boolean.'}
    }elseif($key -ceq 'PlugIns/Dashboard/SumLogNM'){
      if($null -eq $change.before -or $null -eq $change.after){throw 'Both retained Dashboard counters are required.'}
      Assert-RestartDashboardDelta $key $change.before $change.after
    }elseif($key -ceq 'OpenNav/VesselName'){
      if($null -eq $change.after){throw 'Retained vessel-name deletion is unsupported.'}
      Assert-PreservedVesselName $change.after
      if($null -ne $change.before){Assert-PreservedVesselName $change.before}
    }elseif($key -ceq 'Settings/ActiveRoute'){
      foreach($value in @($change.before,$change.after)){
        if($null -ne $value -and $value -cne '' -and $value -cnotmatch '^[a-fA-F0-9]{8}-(?:[a-fA-F0-9]{4}-){3}[a-fA-F0-9]{12}$'){throw 'Malformed retained inert route GUID.'}
      }
    }elseif($key -ceq 'Settings/ConfigVersionString'){
      foreach($value in @($change.before,$change.after)){
        if($value -cnotmatch '^Version 5\.12\.4(?:-0)?\+37fd0cd Build 20[0-9]{2}-[0-9]{2}-[0-9]{2}$' -or
           [datetime]::ParseExact($value.Substring($value.Length-10),'yyyy-MM-dd',[Globalization.CultureInfo]::InvariantCulture) -gt $At.Date){throw 'Unknown retained build marker.'}
      }
    }else{throw ('Cold retention has no source-backed key policy: '+$key)}
  }
  if(-not $wmm -and $null -ne $Review.wmmRetention){throw 'WMM proof supplied for a nonexistent resource-path change.'}
  # Validate unchanged pilot records too; a no-op or unrelated display edit
  # must not hide an unknown stored pilot field behind the changed-key loop.
  if($new.ContainsKey('OpenNav/AlphaSettings')){$null=Read-ReconciliationPilotSettings $new['OpenNav/AlphaSettings']}
}
function Assert-ReconciliationReview($Proof,[string]$Saved,[string]$Normalized,$Review,[datetime]$At,$Inspection=$null) {
  $before=Join-Path $Proof.directory 'input-only.ini'
  $bytes=Get-ReconciliationProfileBytes $Proof $Saved
  $retention=Test-ReconciliationRetentionReview $Review
  if($Review.beforeSha256 -cne (Get-Digest $before) -or $Review.afterSha256 -cne (Get-Digest $Saved) -or
     $Review.parentPreparedSha256 -cne $Proof.sha256 -or $Review.connectionDisposition -cne $bytes.disposition -or
     $Review.provenance -cne 'current-user-state;origin-unverified' -or $Review.preservationOnly -isnot [bool] -or -not $Review.preservationOnly -or
     $Review.launchPermission -isnot [bool] -or $Review.launchPermission -or
     (Get-Digest $Normalized) -cne (Get-CommissioningHash $bytes.input)){throw 'Exact explicit current-profile and connection-disposition review required.'}
  $reviewed=[datetime]::Parse($Review.reviewedUtc).ToUniversalTime()
  if($reviewed -gt $At -or ($At-$reviewed).TotalHours -gt 24){throw 'Preservation review expired or future-dated.'}
  $changes=@(Get-CommissioningIniDiff $before $Saved)
  if(@($Review.changes).Count -ne $changes.Count -or $changes.Count -gt 65){throw 'Every actual current-profile difference needs one decision.'}
  foreach($change in $changes) {
    $match=@($Review.changes|Where-Object {$_.key -ceq $change.key})
    if($match.Count -ne 1 -or $match[0].before -cne $change.before -or $match[0].after -cne $change.after -or
       $match[0].origin -cne 'unverified' -or $match[0].decision -cne 'preserve-current' -or
       [string]::IsNullOrWhiteSpace($match[0].reason) -or $match[0].reason.Length -gt 1024){throw 'Missing, duplicate or changed explicit preservation decision.'}
  }
  if($retention){
    Assert-ReconciliationCurrentRetention $Proof $Saved $Normalized $Review $Inspection $At
    return $changes
  }
  # Only the separately identified original direction byte is normalized for
  # existing policy checks. All other keys retain the bounded preservation policy.
  $policy=$Review|ConvertTo-Json -Depth 12|ConvertFrom-Json
  $policy.afterSha256=Get-Digest $Normalized
  $policy.changes=@($Review.changes|Where-Object {$_.key -cne 'Settings/NMEADataSource/DataConnections'})
  if((Get-Digest $before) -cne (Get-Digest $Normalized)) {$null=Assert-SessionPreservationReview $before $Normalized $policy $At}
  elseif($policy.changes.Count){throw 'Review lists nonexistent normalized profile differences.'}
  return $changes
}
function Read-ReconciliationPlan([string]$Workspace,[string]$Plan,[string]$Hash,[int]$Depth=0) {
  $plan=Assert-LocalPath $Plan;$directory=[IO.Path]::GetDirectoryName($plan)
  if($Hash -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $plan) -cne $Hash -or [IO.Path]::GetFileName($plan) -cne 'plan.json' -or
     [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
     [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-cold-reconciliation-[a-f0-9]{8}$'){throw 'Exact immutable cold reconciliation plan required.'}
  $value=Read-Record $plan
  if($value.schema -ne 1 -or $value.owner -cne $script:ReconciliationOwner -or $value.status -cne 'planned' -or
     $value.provenance -cne 'explicit-cross-generation-reconciliation;current-user-state-origin-unverified' -or
     $value.launchPermission -isnot [bool] -or $value.launchPermission){throw 'Plan cannot claim launch permission or another provenance.'}
  $proof=Read-ReconciliationParent $Workspace $value.parentPrepared $value.parentPreparedSha256 ($Depth+1)
  $inspection=Join-Path $directory 'inspection.json'
  $observed=Read-ReconciliationInspection $proof.parent.context $inspection $value.inspectionSha256
  if($observed.schema -ne 1 -or $observed.owner -cne $script:ReconciliationOwner -or $observed.status -cne 'inspected' -or
     $observed.parentPrepared -ine $proof.record -or $observed.parentPreparedSha256 -cne $proof.sha256 -or
     $observed.activeMarkerSha256 -cnotmatch '^[a-f0-9]{64}$' -or $observed.profileTree.root -ine $observed.currentContext.profile -or
     $observed.profileTree.exists -isnot [bool] -or -not $observed.profileTree.exists){throw 'Inspection belongs to another transaction or profile.'}
  Assert-ReconciliationContexts $proof.parent.context $observed.currentContext
  $newRoots=@($observed.currentContext.pluginRoots|Where-Object {$_ -cnotin $proof.parent.context.pluginRoots}|Sort-Object)
  if(($newRoots|ConvertTo-Json -Compress) -cne (@($observed.currentPluginTrees|ForEach-Object {$_.root}|Sort-Object)|ConvertTo-Json -Compress)){throw 'Inspection must cover every new current-generation loader root exactly once.'}
  Assert-ColdPrivateEvidence $directory $observed.currentContext.sid
  $backup=Get-PreparationTree (Join-Path $directory 'profile-backup');$saved=Join-Path $backup.root 'opencpn.ini';$baseline=Join-Path $directory 'baseline.ini'
  if(($backup.entries|ConvertTo-Json -Depth 8 -Compress) -cne ($observed.profileTree.entries|ConvertTo-Json -Depth 8 -Compress) -or
     (Get-Digest $saved) -cne $observed.currentIniSha256 -or (Get-Digest $baseline) -cne $value.baselineSha256 -or
     (Get-Item -LiteralPath $baseline).Length -ne $value.baselineBytes -or
     (Get-CommissioningHash (Get-ReconciliationProfileBytes $proof $saved).baseline) -cne $value.baselineSha256){throw 'Full current profile backup or exact connection disposition changed.'}
  if($observed.connectionDisposition -cne (Get-ReconciliationProfileBytes $proof $saved).disposition){throw 'Observed connection disposition differs from exact saved bytes.'}
  $review=Join-Path $directory 'preservation-review.json'
  if((Get-Digest $review) -cne $value.reviewSha256){throw 'Independent reconciliation review changed.'}
  $approval=Read-Record $review
  $retention=Test-ReconciliationRetentionReview $approval
  if($retention){Assert-ReconciliationRetentionClaims $value}
  elseif($value.PSObject.Properties['retentionMode']){throw 'Ordinary v1 plan cannot be relabelled as retained-current-state review.'}
  if($approval.inspectionSha256 -cne $value.inspectionSha256 -or
     @(Assert-ReconciliationReview $proof $saved (Join-Path $directory 'normalized-input.ini') $approval ([datetime]::Parse($value.createdUtc).ToUniversalTime()) $observed).Count -ne $value.changedKeys){throw 'Exact review/inspection lineage differs.'}
  return [pscustomobject]@{proof=$proof;value=$value;inspection=$observed;directory=$directory;path=$plan;sha256=$Hash;baseline=$baseline;retention=$retention;review=$approval}
}
function ConvertTo-ReconciliationMetadata($Metadata) {
  # Windows PowerShell 5.1 JSON dates retain only milliseconds. Persist exact
  # DateTime ticks as text; no date parser or JSON numeric rounding is admitted.
  return [pscustomobject]@{attributes=$Metadata.attributes;
    creationUtcTicks=$Metadata.creationUtc.Ticks.ToString([Globalization.CultureInfo]::InvariantCulture);
    acl=$Metadata.acl;auditAcl=$Metadata.auditAcl}
}
function ConvertFrom-ReconciliationMetadata($Value) {
  $ticks=[long]0
  if($Value.creationUtcTicks -isnot [string] -or $Value.creationUtcTicks -cnotmatch '^(0|[1-9][0-9]{0,18})$' -or
     -not [long]::TryParse($Value.creationUtcTicks,[Globalization.NumberStyles]::None,[Globalization.CultureInfo]::InvariantCulture,[ref]$ticks) -or
     $ticks -gt [datetime]::MaxValue.Ticks){throw 'Exact canonical profile creation ticks required.'}
  return [pscustomobject]@{attributes=$Value.attributes;creationUtc=(New-Object DateTime($ticks,[DateTimeKind]::Utc));acl=$Value.acl;auditAcl=$Value.auditAcl}
}
function Read-ReconciliationProfileStage($Plan) {
  $journal=Join-Path $Plan.directory 'profile-stage.json'
  if(-not(Test-Path -LiteralPath $journal)){return $null}
  $value=Read-Record $journal;$original=Join-Path $Plan.inspection.currentContext.profile 'opencpn.ini'
  # The name is derived from the pinned plan, not accepted from an arbitrary
  # matching recovery file. The journal must exist before this path is created.
  $stage=$original+'.opennav-recovery-'+$Plan.sha256.Substring(0,32)+'.partial'
  $native=[Environment]::OSVersion.Platform -eq 'Win32NT'
  if($value.schema -ne 1 -or $value.owner -cne $script:ReconciliationOwner -or $value.status -cne 'prepared' -or
     $value.planSha256 -cne $Plan.sha256 -or $value.parentPreparedSha256 -cne $Plan.proof.sha256 -or
     $value.original -cne $original -or $value.stage -cne $stage -or $value.candidate -cne $Plan.baseline -or
     $value.originalSha256 -cne $Plan.inspection.currentIniSha256 -or $value.candidateSha256 -cne $Plan.value.baselineSha256 -or
     $value.bytes -ne $Plan.value.baselineBytes -or $value.native -isnot [bool] -or $value.native -ne $native -or
     $value.originalSha256 -ceq $value.candidateSha256){throw 'Profile staging journal differs from its exact reconciliation plan.'}
  Assert-PreparationStagePath $original $stage
  if($native){
    $value.metadata=ConvertFrom-ReconciliationMetadata $value.metadata
    Assert-PreparationNativeMetadata $value.metadata (Get-PreparationNativeMetadata $original) -AllowDaclAutoInherited
  }else{Assert-PreparationAcl $value.acl (Get-PreparationAccessAcl $original)}
  if(Test-Path -LiteralPath $stage){
    if((Get-Digest $original) -cne $value.originalSha256 -or (Get-Digest $stage) -cne $value.candidateSha256 -or
       (Get-Item -LiteralPath $stage -Force).Length -ne $value.bytes){throw 'Journalled profile stage or original bytes changed; retain evidence without publication.'}
    if($native){Assert-PreparationNativeMetadata $value.metadata (Get-PreparationNativeMetadata $stage) -AllowDaclAutoInherited}
    else{Assert-PreparationAcl $value.acl (Get-PreparationAccessAcl $stage)}
  }
  return $value
}
function Publish-ReconciliationStage([string]$Stage,[string]$Original) {
  if([Environment]::OSVersion.Platform -eq 'Win32NT'){
    [OpenNavX.PreparationNative]::Publish($Stage,$Original)
  }else{[IO.File]::Replace($Stage,$Original,[NullString]::Value,$false)}
}
function Publish-ReconciliationProfile($Plan,$Current) {
  $original=Join-Path $Current.profile 'opencpn.ini';$journal=Join-Path $Plan.directory 'profile-stage.json'
  if(-not(Test-Path -LiteralPath $journal)){
    $native=[Environment]::OSVersion.Platform -eq 'Win32NT'
    $metadata=if($native){ConvertTo-ReconciliationMetadata (Get-PreparationNativeMetadata $original)}else{$null}
    Write-Record $journal @{schema=1;owner=$script:ReconciliationOwner;status='prepared';planSha256=$Plan.sha256;parentPreparedSha256=$Plan.proof.sha256;
      original=$original;stage=($original+'.opennav-recovery-'+$Plan.sha256.Substring(0,32)+'.partial');candidate=$Plan.baseline;
      originalSha256=$Plan.inspection.currentIniSha256;candidateSha256=$Plan.value.baselineSha256;bytes=$Plan.value.baselineBytes;
      native=$native;metadata=$metadata;acl=(Get-PreparationAccessAcl $original)}
  }
  $staged=Read-ReconciliationProfileStage $Plan
  if(-not(Test-Path -LiteralPath $staged.stage)){
    Copy-PreparationFile $Plan.baseline $staged.stage $staged.candidateSha256 $staged.bytes
    if($staged.native){
      Set-PreparationStageAcl $original $staged.stage $staged.metadata.acl
      [IO.File]::SetCreationTimeUtc($staged.stage,$staged.metadata.creationUtc)
      [IO.File]::SetAttributes($staged.stage,[IO.FileAttributes]$staged.metadata.attributes)
    }
  }
  # Recheck the complete trees, both contexts, exact marker, original bytes and
  # staged bytes/metadata immediately before the same atomic publication API.
  Assert-ReconciliationPlanLive $Plan (Get-CommissioningContext $Current.workspace) -Resume
  $staged=Read-ReconciliationProfileStage $Plan
  Publish-ReconciliationStage $staged.stage $original
  if((Get-Digest $original) -cne $Plan.value.baselineSha256 -or (Test-Path -LiteralPath $staged.stage)){throw 'Reconciliation profile publication postcondition failed.'}
  $null=Read-ReconciliationProfileStage $Plan
}
function Remove-ReconciliationActive($Plan) {
  Assert-ReconciliationActive $Plan.proof $Plan.inspection.activeMarkerSha256
  Remove-Item -LiteralPath (Join-Path $Plan.inspection.currentContext.workspace 'commissioning-active.json')
}
function Get-ReconciliationReceipt($Plan) {
  $completion=Join-Path $Plan.directory 'completion.json';$baseline=Join-Path $Plan.directory 'reconciled-baseline.json'
  return [pscustomobject]@{status='reconciled';completion=$completion;completionSha256=(Get-Digest $completion);baselineRecord=$baseline;
    baselineRecordSha256=(Get-Digest $baseline);applicationLaunched=$false;doNotAutoLaunch=$true}
}
function Assert-ReconciliationPlanLive($Plan,$Current,[switch]$Resume,[switch]$Completed) {
  $created=[datetime]::Parse($Plan.value.createdUtc).ToUniversalTime();$now=[datetime]::UtcNow
  if(-not $Resume -and ($created -gt $now -or ($now-$created).TotalHours -gt 24)){throw 'Unstarted reconciliation plan expired or future-dated.'}
  Assert-CommissioningContext $Plan.inspection.currentContext $Current
  Assert-ReconciliationLiveIdentity $Plan.proof $Current
  if($Plan.retention){
    if($Plan.value.baselineSha256 -cne $Plan.inspection.currentIniSha256 -or
       (Get-Digest (Join-Path $Current.profile 'opencpn.ini')) -cne $Plan.inspection.currentIniSha256 -or
       (Test-Path -LiteralPath (Join-Path $Plan.directory 'profile-stage.json'))){throw 'Retained current-state reconciliation never publishes or changes profile bytes.'}
    if($Plan.review.wmmRetention){Assert-ReconciliationWmmRetention $Plan.proof $Current $Plan.review.wmmRetention -Live}
  }
  if($Completed){
    if(-not $Resume -or (Test-Path -LiteralPath (Join-Path $Current.workspace 'commissioning-active.json'))){throw 'Completed reconciliation requires an absent active marker.'}
  }else{Assert-ReconciliationActive $Plan.proof $Plan.inspection.activeMarkerSha256}
  $tree=$Plan.inspection.profileTree|ConvertTo-Json -Depth 10|ConvertFrom-Json
  $ini=Join-Path $tree.root 'opencpn.ini';$hash=Get-Digest $ini
  if(($Completed -and $hash -cne $Plan.value.baselineSha256) -or ($hash -cne $Plan.inspection.currentIniSha256 -and (-not $Resume -or $hash -cne $Plan.value.baselineSha256))){throw 'Current profile drifted from the exact inspected bytes.'}
  $entry=@($tree.entries|Where-Object {$_.path -ceq 'opencpn.ini'})
  if($entry.Count -ne 1){throw 'Ambiguous current profile inventory.'}
  $entry[0].sha256=$hash;$entry[0].bytes=(Get-Item -LiteralPath $ini).Length
  $stage=Read-ReconciliationProfileStage $Plan
  if($Resume -and $hash -ceq $Plan.value.baselineSha256 -and $hash -cne $Plan.inspection.currentIniSha256 -and -not $stage){throw 'Published profile lacks its exact staging journal.'}
  $expectedTree=$tree|ConvertTo-Json -Depth 10|ConvertFrom-Json
  if($stage -and (Test-Path -LiteralPath $stage.stage)){
    if(-not $Resume -or $Completed){throw 'An unpublished stage requires the same unfinished commit.'}
    $expectedTree.entries=@(@($expectedTree.entries)+@([pscustomobject]@{path=[IO.Path]::GetFileName($stage.stage);directory=$false;bytes=$stage.bytes;sha256=$stage.candidateSha256})|Sort-Object path)
  }
  Assert-PreparationTree $expectedTree
  foreach($currentTree in @($Plan.inspection.currentPluginTrees)){Assert-PreparationTree $currentTree}
  Assert-ReconciliationMoves $Plan.proof $Plan.directory -Resume:$Resume
  $acls=Get-SessionPreservationAcls (@($tree)+@($Plan.proof.inventory.trees)+@($Plan.inspection.currentPluginTrees)) $Plan.proof.parent.quarantine
  Assert-SessionPreservationAcls $Plan.inspection.acls $acls $ini
}
function Read-ReconciledBaseline([string]$Workspace,[string]$Record,[string]$Hash,[int]$Depth) {
  $record=Assert-LocalPath $Record;$directory=[IO.Path]::GetDirectoryName($record)
  if((Get-Digest $record) -cne $Hash){throw 'Reconciled baseline changed.'}
  $value=Read-Record $record
  if($value.schema -ne 1 -or $value.owner -cne $script:ReconciliationOwner -or $value.status -cne 'reconciled' -or
     $value.provenance -cne 'explicit-cross-generation-reconciliation;current-user-state-origin-unverified' -or
     $value.launchPermission -isnot [bool] -or $value.launchPermission){throw 'Incomplete or authority-claiming reconciled baseline.'}
  $plan=Read-ReconciliationPlan $Workspace (Join-Path $directory 'plan.json') $value.planSha256 $Depth
  if($plan.retention){Assert-ReconciliationRetentionClaims $value}
  elseif($value.PSObject.Properties['retentionMode']){throw 'Ordinary baseline cannot claim another retention mode.'}
  $parent=$plan.proof.parent
  $completion=Join-Path $directory 'completion.json'
  if((Get-Digest $completion) -cne $value.completionSha256){throw 'Reconciliation completion changed.'}
  $done=Read-Record $completion
  if($plan.retention){Assert-ReconciliationRetentionClaims $done}
  elseif($done.PSObject.Properties['retentionMode']){throw 'Ordinary completion cannot claim another retention mode.'}
  if($done.schema -ne 1 -or $done.owner -cne $script:ReconciliationOwner -or $done.status -cne 'reconciled' -or $done.planSha256 -cne $plan.sha256 -or
     $done.parentPreparedSha256 -cne $plan.proof.sha256 -or $done.profileSha256 -cne $plan.value.baselineSha256 -or
     $value.baselineSha256 -cne $plan.value.baselineSha256 -or $value.baselineBytes -ne $plan.value.baselineBytes -or
     ($done.currentContext|ConvertTo-Json -Depth 8 -Compress) -cne ($plan.inspection.currentContext|ConvertTo-Json -Depth 8 -Compress)){throw 'Completed reconciliation differs from its two-context plan.'}
  foreach($key in @('originalPathsRestored','otherProfileFilesPreserved','noPluginTransplant','doNotAutoLaunch')){Assert-TrueBoolean $done.$key ('Reconciliation '+$key)}
  if($done.applicationLaunched -isnot [bool] -or $done.applicationLaunched){throw 'Reconciliation cannot claim an application launch.'}
  $index=0
  foreach($item in @($plan.proof.parent.quarantine)) {
    $journal=Join-Path $directory ('plugin-'+$index+'.json');$index++
    $move=Read-Record $journal
    if($move.owner -cne $script:ReconciliationOwner -or $move.parentPreparedSha256 -cne $plan.proof.sha256 -or
       $move.source -ine $item.destination -or $move.destination -ine $item.path -or $move.sha256 -cne $item.sha256){throw 'Original-path restoration provenance is incomplete.'}
  }
  # Historical lineage remains readable after a later legitimate installation.
  $result=[pscustomobject]@{sha256=$value.baselineSha256;bytes=$value.baselineBytes;record=$record;recordSha256=$Hash;sid=$parent.context.sid;profile=$parent.context.profile}
  if($plan.retention){
    $result|Add-Member -NotePropertyName pilotBindingQualified -NotePropertyValue $false
    $result|Add-Member -NotePropertyName retentionMode -NotePropertyValue $value.retentionMode
  }
  return Copy-CommissioningBaselineRestrictions $result $plan.proof.baseline
}
