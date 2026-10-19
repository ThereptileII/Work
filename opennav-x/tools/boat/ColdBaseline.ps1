# Cold, pre-existing user profile preservation. This policy neither attributes
# changes to OpenCPN nor authorizes a launch or a marine output.
$script:ColdBaselineOwner='OpenNavX.ColdProfileBaseline.1'
$script:ColdCaptureOwner='OpenNavX.ColdProfileCapture.1'
$script:ColdReviewOwner='OpenNavX.ColdProfileReview.1'
# A capture carries up to four bounded inventories and two frozen WMM ownership
# proofs. Native PowerShell's indentation can put it above generic Read-Record's
# 4 MiB limit. This exception is only for this typed, hash-pinned evidence.
$script:ColdCaptureMaxBytes=33554432

function Assert-ColdCaptureTree($Tree) {
  if($Tree -isnot [pscustomobject] -or @($Tree.PSObject.Properties).Count -ne 3 -or
     $Tree.root -isnot [string] -or $Tree.root.Length -lt 1 -or $Tree.root.Length -gt 32767 -or
     $Tree.exists -isnot [bool] -or $Tree.entries -isnot [array] -or $Tree.entries.Count -gt 30000 -or
     (-not $Tree.exists -and $Tree.entries.Count)){throw 'Cold capture tree shape or count outside bounds.'}
  $seen=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  foreach($entry in $Tree.entries) {
    if($entry -isnot [pscustomobject] -or @($entry.PSObject.Properties).Count -ne 4 -or
       $entry.path -isnot [string] -or $entry.path.Length -lt 1 -or $entry.path.Length -gt 32767 -or
       $entry.path -match '[\x00-\x1f]|^[\\/]|:|[\\/]{2}|[\\/]$|(^|[\\/])\.\.?([\\/]|$)' -or -not $seen.Add($entry.path.Replace('/','\')) -or
       $entry.directory -isnot [bool] -or ($entry.bytes -isnot [int] -and $entry.bytes -isnot [long]) -or
       $entry.bytes -lt 0 -or $entry.sha256 -isnot [string] -or
       ($entry.directory -and ($entry.bytes -ne 0 -or $entry.sha256 -cne '')) -or
       (-not $entry.directory -and $entry.sha256 -cnotmatch '^[a-f0-9]{64}$')){
      throw 'Cold capture inventory entry is malformed or duplicated.'
    }
  }
}

function Read-ColdCaptureRecord([string]$Path,[string]$Hash) {
  $path=Assert-LocalPath $Path
  if($Hash -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact cold capture hash required.'}
  # Hash and decode the same bounded bytes while denying concurrent writers.
  $stream=New-Object IO.FileStream($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
  try {
    if($stream.Length -le 0 -or $stream.Length -gt $script:ColdCaptureMaxBytes){throw 'Cold capture size outside 32 MiB bound.'}
    $bytes=New-Object byte[] ([int]$stream.Length);$offset=0
    while($offset -lt $bytes.Length) {
      $read=$stream.Read($bytes,$offset,$bytes.Length-$offset)
      if($read -eq 0){throw 'Cold capture ended before its bounded length.'}
      $offset+=$read
    }
  } finally {$stream.Dispose()}
  if((Get-CommissioningHash $bytes) -cne $Hash){throw 'Cold capture bytes changed.'}
  $text=(New-Object Text.UTF8Encoding($false,$true)).GetString($bytes).TrimStart([char]0xfeff)
  # ConvertFrom-Json can enumerate a one-element root array into one object.
  if(-not $text.TrimStart().StartsWith('{')){throw 'Cold capture must be one JSON object.'}
  # Windows PowerShell 5.1 has no ConvertFrom-Json -Depth parameter. Count only
  # structural tokens outside quoted strings before invoking its JSON parser.
  $depth=0
  foreach($token in [regex]::Matches($text,'"(?:\\.|[^"\\])*"|[{}\[\]]')) {
    if($token.Value -ceq '{' -or $token.Value -ceq '['){$depth++;if($depth -gt 16){throw 'Cold capture nesting exceeds 16 levels.'}}
    elseif($token.Value -ceq '}' -or $token.Value -ceq ']'){$depth--;if($depth -lt 0){throw 'Unbalanced cold capture JSON.'}}
  }
  if($depth -ne 0){throw 'Unbalanced cold capture JSON.'}
  $record=$text|ConvertFrom-Json
  $fields=@('schema','owner','status','createdUtc','predecessorRecord','predecessorSha256','context','profileTree','pluginTrees',
    'profileSha256','profileBytes','profileAcl','sourceIniLastWriteTicks','sourceIniCreationTicks','inputSha256','stockSha256',
    'provenance','profileChanged','applicationLaunched','launchPermission')
  if($record -isnot [pscustomobject]){throw 'Cold capture must be one JSON object.'}
  if($record.PSObject.Properties['wmmResourceProof']){$fields+='wmmResourceProof'}
  if((@($record.PSObject.Properties.Name|Sort-Object)|ConvertTo-Json -Compress) -cne (@($fields|Sort-Object)|ConvertTo-Json -Compress) -or
     ($record.schema -isnot [int] -and $record.schema -isnot [long]) -or $record.schema -ne 1 -or
     $record.owner -cne $script:ColdCaptureOwner -or $record.status -cne 'captured' -or
     $record.provenance -cne 'pre-existing-current-user-state;origin-unverified'){
    throw 'Exact cold capture record kind and fields required.'
  }
  foreach($field in @('profileChanged','applicationLaunched','launchPermission')){
    if($record.$field -isnot [bool] -or $record.$field){throw 'Cold capture cannot claim changes or launch authority.'}
  }
  foreach($field in @('predecessorSha256','profileSha256','inputSha256','stockSha256')){
    if($record.$field -isnot [string] -or $record.$field -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact cold capture identity hashes required.'}
  }
  if(($record.profileBytes -isnot [int] -and $record.profileBytes -isnot [long]) -or
     $record.profileBytes -le 0 -or $record.profileBytes -gt 4194304 -or $record.context -isnot [pscustomobject] -or
     $record.context.profile -isnot [string] -or $record.context.pluginRoots -isnot [array] -or
     $record.context.pluginRoots.Count -lt 2 -or $record.context.pluginRoots.Count -gt 3 -or
     $record.pluginTrees -isnot [array] -or $record.pluginTrees.Count -ne $record.context.pluginRoots.Count){
    throw 'Cold capture profile or plugin inventory bounds differ.'
  }
  Assert-ColdCaptureTree $record.profileTree
  if(-not $record.profileTree.exists -or $record.profileTree.root -cne $record.context.profile){throw 'Cold capture profile root differs.'}
  $roots=New-Object 'Collections.Generic.HashSet[string]' ([StringComparer]::OrdinalIgnoreCase)
  for($index=0;$index -lt $record.pluginTrees.Count;$index++){
    $tree=$record.pluginTrees[$index];Assert-ColdCaptureTree $tree
    if($tree.root -cne $record.context.pluginRoots[$index] -or -not $roots.Add($tree.root)){throw 'Cold capture plugin roots differ or repeat.'}
  }
  return $record
}

function Assert-ColdPrivateEvidence([string]$Directory,[string]$Sid) {
  $directory=Assert-LocalPath $Directory
  $acl=Get-Acl -LiteralPath $directory
  $allowed=@($Sid,'S-1-5-18','S-1-5-32-544')
  if(-not $acl.AreAccessRulesProtected -or $acl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cnotin $allowed){
    throw 'Cold evidence directory lost its private owner/protected ACL.'
  }
  $paths=New-Object 'Collections.Generic.List[string]';$queue=New-Object 'Collections.Generic.Queue[string]'
  $paths.Add($directory);$queue.Enqueue($directory)
  while($queue.Count) {
    foreach($child in @(Get-ChildItem -LiteralPath $queue.Dequeue() -Force)) {
      if($child.Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Cold evidence contains redirected path.'}
      if($paths.Count -ge 30010){throw 'Cold evidence ACL inventory exceeds bound.'}
      $paths.Add($child.FullName)
      if($child.PSIsContainer){$queue.Enqueue($child.FullName)}
    }
  }
  foreach($path in $paths) {
    $entry=Get-Item -LiteralPath $path -Force
    if($entry.Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Cold evidence contains redirected path.'}
    $pathAcl=Get-Acl -LiteralPath $path
    if($pathAcl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cnotin $allowed){throw 'Cold evidence child ownership changed.'}
    $rules=$pathAcl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])
    foreach($rule in $rules){
      if($rule.IdentityReference.Value -cnotin $allowed -or $rule.AccessControlType -ne [Security.AccessControl.AccessControlType]::Allow){
        throw 'Cold evidence has unreviewed access.'
      }
    }
  }
}

function Get-ColdWmmOriginalGeneration([string]$Location,$Context) {
  # Only the immutable predecessor value selects this path. No resource path
  # supplied by a review is accepted, and both generations share the owned root.
  if(-not $Context -or -not $Context.installation){throw 'Cold WMM preservation requires an owned current installation.'}
  $installation=$Context.installation
  $generations=Assert-LocalPath (Join-Path $installation.root 'generations')
  $separator=[string][IO.Path]::DirectorySeparatorChar
  $prefix=$generations+$separator
  $current=Assert-LocalPath $installation.generation
  if(-not $current.StartsWith($prefix,[StringComparison]::Ordinal) -or
     $current.Substring($prefix.Length) -cnotmatch '^[a-f0-9]{32}$'){throw 'Current WMM generation is outside its owned root.'}
  $decoded=$Location.Replace('\\','\')
  $suffix=$separator+'app'+$separator+'plugins'+$separator+'wmm_pi'+$separator+'data'+$separator
  if(-not $decoded.StartsWith($prefix,[StringComparison]::Ordinal) -or
     $decoded.Substring($prefix.Length) -cnotmatch ('^[a-f0-9]{32}'+[regex]::Escape($suffix)+'$')){
    throw 'Cold WMM predecessor must name an exact owned generation resource directory.'
  }
  $original=Assert-LocalPath (Join-Path $generations $decoded.Substring($prefix.Length,32))
  if($original -ceq $current -or $Location -cne (Get-CommissioningWmmLocation (Join-Path $original 'app'))){
    throw 'Cold WMM predecessor path is not canonical or distinct from the current generation.'
  }
  return $original
}

function Get-ColdWmmResourceProof([string]$Before,[string]$After,$Context) {
  $old=Read-ProfileForAudit $Before;$new=Read-ProfileForAudit $After
  if($old['Directories/WMMDataLocation'] -ceq $new['Directories/WMMDataLocation']){return $null}
  $generation=Get-ColdWmmOriginalGeneration $old['Directories/WMMDataLocation'] $Context
  if($new['Directories/WMMDataLocation'] -cne (Get-CommissioningWmmLocation (Join-Path $Context.installation.generation 'app'))){
    throw 'Cold WMM current path differs from the captured owned generation.'
  }
  $proof=[pscustomobject]@{schema=1;owner='OpenNavX.ColdWmmResourceReview.1';
    original=(Get-ReconciliationWmmOriginalReferenceProof $generation);
    current=(Get-ReconciliationGenerationWmmProof $Context)}
  Assert-ColdWmmResourceProof $Before $After $Context $proof
  return $proof
}

function Assert-ColdWmmResourceProof([string]$Before,[string]$After,$Context,$Proof,[switch]$Live) {
  $old=Read-ProfileForAudit $Before;$new=Read-ProfileForAudit $After
  if($old['Directories/WMMDataLocation'] -ceq $new['Directories/WMMDataLocation']){
    if($null -ne $Proof){throw 'Cold WMM proof supplied without a resource-path change.'}
    return
  }
  $generation=Get-ColdWmmOriginalGeneration $old['Directories/WMMDataLocation'] $Context
  if(-not $Proof -or (@($Proof.PSObject.Properties.Name|Sort-Object)|ConvertTo-Json -Compress) -cne
      (@('schema','owner','original','current'|Sort-Object)|ConvertTo-Json -Compress) -or
      $Proof.schema -ne 1 -or $Proof.owner -cne 'OpenNavX.ColdWmmResourceReview.1' -or
      $Proof.original.commit -isnot [string] -or $Proof.original.commit -cnotmatch '^[a-f0-9]{40}$' -or
      $Proof.original.ownershipSha256 -isnot [string] -or $Proof.original.ownershipSha256 -cnotmatch '^[a-f0-9]{64}$' -or
      $new['Directories/WMMDataLocation'] -cne (Get-CommissioningWmmLocation (Join-Path $Context.installation.generation 'app'))){
    throw 'Exact independent cold WMM ownership and source proof required.'
  }
  $originalContext=[pscustomobject]@{installation=[pscustomobject]@{generation=$generation;
    commit=$Proof.original.commit;ownershipSha256=$Proof.original.ownershipSha256}}
  # Frozen manifest bytes remain verifiable after a legitimate generation
  # retirement. Only Capture/Complete request the additional live-file checks.
  Assert-ReconciliationGenerationWmmProof $originalContext $Proof.original
  Assert-ReconciliationGenerationWmmProof $Context $Proof.current
  if(($Proof.original.resources|ConvertTo-Json -Depth 8 -Compress) -cne ($Proof.current.resources|ConvertTo-Json -Depth 8 -Compress)){
    throw 'Cold WMM generations do not contain identical pinned source resources.'
  }
  if($Live){
    $actual=Get-ColdWmmResourceProof $Before $After $Context
    if(($actual|ConvertTo-Json -Depth 12 -Compress) -cne ($Proof|ConvertTo-Json -Depth 12 -Compress)){
      throw 'Cold WMM ownership or resource bytes changed since capture.'
    }
  }
}

function Assert-ColdDataRailDelta([string]$Before,[string]$After) {
  # Settings.cpp/Assert-PreservedAlphaSettings define this six-item display
  # selection. Preserve every other encoded byte, including pilot permission,
  # identity, sources, calibration, presets and unknown future settings.
  $known=@('sog','cog','heading','stw','aws','awa','tws','twa','depth','water_temp','pressure','rudder','heel','soc','voltage','current','pack_power','motor_power','rpm','motor_temp','fresh_water','fuel','waste')
  $pattern='(?<=\\n)"display\.rail" "([^"\\\x00-\x1f]*)"(?=\\n)'
  $normalized=@()
  foreach($value in @($Before,$After)){
    if($value.Length -gt 131072 -or -not $value.StartsWith('OpenNavXSettings 1\n') -or $value -match '[\x00-\x1f\x7f]'){
      throw 'Unknown cold vessel settings encoding.'
    }
    $matches=[regex]::Matches($value,$pattern)
    if($matches.Count -ne 1 -or [regex]::Matches($value,'(?<=\\n)"display\.rail" ').Count -ne 1){
      throw 'Exactly one complete cold data-rail field is required.'
    }
    $items=$matches[0].Groups[1].Value.Split(',')
    if($items.Count -lt 1 -or $items.Count -gt 6 -or @($items|Sort-Object -Unique).Count -ne $items.Count){throw 'Cold data-rail selection exceeds source bounds or contains duplicates.'}
    foreach($item in $items){if($item -cnotin $known){throw 'Unknown cold data-rail item.'}}
    $normalized+=,[regex]::Replace($value,$pattern,'"display.rail" "<reviewed-display-only>"')
  }
  if($normalized[0] -cne $normalized[1]){throw 'Cold data-rail preservation cannot change other vessel, pilot or source bytes.'}
}

function Assert-ColdBaselineDelta([string]$Before,[string]$After,$Review,[datetime]$At=[datetime]::UtcNow,$WmmResourceProof=$null,$Context=$null) {
  $old=Read-ProfileForAudit $Before;$new=Read-ProfileForAudit $After
  if($Review.schema -ne 1 -or $Review.owner -cne $script:ColdReviewOwner -or
     $Review.beforeSha256 -cne (Get-Digest $Before) -or $Review.afterSha256 -cne (Get-Digest $After) -or
     $Review.provenance -cne 'pre-existing-current-user-state;origin-unverified' -or
     $Review.preservationOnly -isnot [bool] -or -not $Review.preservationOnly){throw 'Cold review must bind exact existing bytes as preservation only.'}
  $reviewed=[datetime]::Parse($Review.reviewedUtc).ToUniversalTime()
  if($reviewed -gt $At -or ($At-$reviewed).TotalHours -gt 24){throw 'Cold profile review is future-dated or expired.'}
  # This is deliberately narrower than the normal post-session migration policy.
  # A valid output-capable baseline is checked by the existing one-byte forward
  # transformation on an owned copy, never by editing the user's live profile.
  $null=Get-CommissioningInputBytes ([IO.File]::ReadAllBytes($After))
  Assert-ColdWmmResourceProof $Before $After $Context $WmmResourceProof
  $wmmReview=$Review.PSObject.Properties['wmmResourceProofSha256']
  if($WmmResourceProof){
    $proofHash=Get-CommissioningHash ((New-Object Text.UTF8Encoding($false)).GetBytes(($WmmResourceProof|ConvertTo-Json -Depth 12 -Compress)))
    if(-not $wmmReview -or $wmmReview.Value -isnot [string] -or $wmmReview.Value -cne $proofHash){throw 'Independent review must bind the exact captured cold WMM proof.'}
  }elseif($wmmReview){throw 'Cold review includes WMM evidence without a resource-path change.'}
  $preferences=@('OpenNav/ChartPresentationV1','OpenNav/DisplayPreferencesV1','OpenNav/OnlineAIS/v1/RadiusNm','OpenNav/AlphaSettings','Settings/ActiveRoute')
  foreach($key in @(@($old.Keys)+@($new.Keys)|Sort-Object -Unique)) {
    if($key -cmatch '^(Settings/NMEADataSource/|Directories/|ChartDirectories/|PlugIns/(?!Dashboard/)|OpenNav/(?!InterfaceMode$)|Settings/(?:CommPriority/|PersistActiveRoute$|ActiveRoute$))' -and $old[$key] -cne $new[$key] -and
       $key -cnotin $preferences -and -not ($key -ceq 'Directories/WMMDataLocation' -and $WmmResourceProof)){
      throw ('Protected marine or plugin configuration changed: '+$key)
    }
  }
  $changes=@(Get-CommissioningIniDiff $Before $After)
  if($changes.Count -lt 1 -or $changes.Count -gt 64 -or @($Review.changes).Count -ne $changes.Count){throw 'Every bounded cold profile change needs explicit review.'}
  $policy=Get-RestartDisplayKeys
  foreach($change in $changes) {
    $key=$change.key;$match=@($Review.changes|Where-Object {$_.key -ceq $key})
    if($match.Count -ne 1 -or $match[0].before -cne $change.before -or $match[0].after -cne $change.after -or
       $match[0].origin -cne 'unverified' -or [string]::IsNullOrWhiteSpace($match[0].reason) -or
       $match[0].reason.Length -gt 1024 -or
       $null -eq $change.after){throw 'Missing or inaccurate preservation decision.'}
    if($policy.ContainsKey($key)) {
      Assert-RestartScalar $policy[$key] $change.after
      if($null -ne $change.before){Assert-RestartScalar $policy[$key] $change.before}
    } elseif($key -ceq 'AUI/AUIPerspective') {
      if($null -eq $change.before){throw 'New AUI structure is outside cold policy.'}
      Assert-RestartAuiDelta $change.before $change.after
    } elseif($key -ceq 'OpenNav/InterfaceMode') {
      if($change.before -and $change.before -cnotin @('xnav','legacy')){throw 'Unknown previous interface mode.'}
      if($change.after -cnotin @('xnav','legacy')){throw 'Unknown current interface mode.'}
    } elseif($key -ceq 'OpenNav/DisplayPreferencesV1') {
      Assert-PreservedSetupPreference $key $change.after
      if($null -ne $change.before){Assert-PreservedSetupPreference $key $change.before}
    } elseif($key -ceq 'OpenNav/ChartPresentationV1') {
      foreach($v in @($change.before,$change.after)){
        if($null -ne $v -and $v -cnotin @('XNav','Standard')){throw 'Unknown cold chart palette preference.'}
      }
    } elseif($key -ceq 'OpenNav/OnlineAIS/v1/RadiusNm') {
      foreach($v in @($change.before,$change.after)){
        if($null -ne $v -and ($v -cnotmatch '^[1-9][0-9]{0,2}$' -or [int]$v -gt 200)){throw 'Cold AIS radius outside product bounds.'}
      }
    } elseif($key -ceq 'OpenNav/AlphaSettings') {
      if($null -eq $change.before){throw 'Cold data-rail preservation requires an existing vessel settings record.'}
      Assert-ColdDataRailDelta $change.before $change.after
    } elseif($key -ceq 'Settings/ActiveRoute') {
      if($old['Settings/PersistActiveRoute'] -cne '0' -or $new['Settings/PersistActiveRoute'] -cne '0'){
        throw 'Cold stored route preservation requires route persistence to remain disabled.'
      }
      foreach($v in @($change.before,$change.after)){
        if($null -ne $v -and $v -cne '' -and $v -cnotmatch '^[a-fA-F0-9]{8}-(?:[a-fA-F0-9]{4}-){3}[a-fA-F0-9]{12}$'){
          throw 'Unknown inert stored route identifier.'
        }
      }
    } elseif($key -ceq 'Directories/WMMDataLocation' -and $WmmResourceProof) {
      # Exact old/current owned paths, source bytes and independent review were
      # checked above. This is preservation, never permission to load a plugin.
    } elseif($key -ceq 'PlugIns/Dashboard/SumLogNM') {
      # Cold user state may include a deliberate counter reset; validate both
      # values without claiming the four-hour monotonic startup contract.
      foreach($v in @($change.before,$change.after)){
        if($v -cnotmatch '^(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$' -or $v.Length -gt 64 -or
           [double]::Parse($v,[Globalization.CultureInfo]::InvariantCulture) -gt 100000000){throw 'Invalid Dashboard counter.'}
      }
    } elseif($key -cmatch '^PlugIns/Dashboard/Dashboard[123]/(?:BestSize|PersistSize)[XY]$') {
      foreach($v in @($change.before,$change.after)){Assert-RestartScalar 'size' $v}
    } elseif($key -cmatch '^Settings/AutoTrackRaymarine/Pos[XY]$') {
      foreach($v in @($change.before,$change.after)){Assert-RestartScalar 'position' $v}
    } elseif($key -ceq 'Settings/ConfigVersionString') {
      foreach($v in @($change.before,$change.after)){
        if($v -cnotmatch '^Version 5\.12\.4(?:-0)?\+37fd0cd Build 20[0-9]{2}-[0-9]{2}-[0-9]{2}$' -or
           [datetime]::ParseExact($v.Substring($v.Length-10),'yyyy-MM-dd',[Globalization.CultureInfo]::InvariantCulture) -gt $At.Date){throw 'Unreviewed version marker.'}
      }
    } elseif($key -ceq 'Settings/MSWFonts/sv-00c6075a') {
      if($null -ne $change.before -or $new['Settings/Locale'] -cne 'sv' -or $new['Settings/LocaleOverride'] -cne 'sv_SE' -or
         $change.after.Length -gt 128 -or $change.after -cnotmatch '^(?:Menu|Meny):(?:-?[0-9]+;){15}[^:;\r\n]{1,64}:rgb\(([0-9]{1,3}), ?([0-9]{1,3}), ?([0-9]{1,3})\)$'){
        throw 'Only a bounded reviewed Swedish menu font entry may be preserved.'
      }
      foreach($component in @($Matches[1],$Matches[2],$Matches[3])){if([int]$component -gt 255){throw 'Font color component outside bounds.'}}
    } elseif($key -ceq 'Settings/MSWFonts/sv-6f52a406') {
      if($old['Settings/Locale'] -cne 'sv' -or $new['Settings/Locale'] -cne 'sv' -or
         $old['Settings/LocaleOverride'] -cne 'sv_SE' -or $new['Settings/LocaleOverride'] -cne 'sv_SE'){
        throw 'Cold menu font locale changed.'
      }
      foreach($v in @($change.before,$change.after)){
        if($null -eq $v -or $v.Length -gt 256 -or
           $v -cnotmatch '^[^:;\r\n]{1,64}:(?:-?[0-9]+;){15}[^:;\r\n]{1,64}:rgb\(([0-9]{1,3}), ?([0-9]{1,3}), ?([0-9]{1,3})\)$'){
          throw 'Unknown existing cold menu font encoding.'
        }
        foreach($component in @($Matches[1],$Matches[2],$Matches[3])){if([int]$component -gt 255){throw 'Font color component outside bounds.'}}
      }
    } else {throw ('Cold preservation needs a separate key policy: '+$key)}
  }
  return $changes
}
