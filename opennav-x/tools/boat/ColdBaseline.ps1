# Cold, pre-existing user profile preservation. This policy neither attributes
# changes to OpenCPN nor authorizes a launch or a marine output.
$script:ColdBaselineOwner='OpenNavX.ColdProfileBaseline.1'
$script:ColdCaptureOwner='OpenNavX.ColdProfileCapture.1'
$script:ColdReviewOwner='OpenNavX.ColdProfileReview.1'

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

function Assert-ColdBaselineDelta([string]$Before,[string]$After,$Review,[datetime]$At=[datetime]::UtcNow) {
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
  foreach($key in @(@($old.Keys)+@($new.Keys)|Sort-Object -Unique)) {
    if($key -cmatch '^(Settings/NMEADataSource/|Directories/|ChartDirectories/|PlugIns/(?!Dashboard/)|OpenNav/(?!InterfaceMode$)|Settings/(?:CommPriority/|PersistActiveRoute$|ActiveRoute$))' -and $old[$key] -cne $new[$key]){
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
    } elseif($key -ceq 'PlugIns/Dashboard/SumLogNM') {
      # Cold user state may include a deliberate counter reset; validate both
      # values without claiming the four-hour monotonic startup contract.
      foreach($v in @($change.before,$change.after)){
        if($v -cnotmatch '^(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?$' -or $v.Length -gt 64 -or
           [double]::Parse($v,[Globalization.CultureInfo]::InvariantCulture) -gt 100000000){throw 'Invalid Dashboard counter.'}
      }
    } elseif($key -cmatch '^PlugIns/Dashboard/Dashboard[23]/(?:BestSize|PersistSize)[XY]$') {
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
    } else {throw ('Cold preservation needs a separate key policy: '+$key)}
  }
  return $changes
}
