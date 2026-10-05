# Cold-only, stock chart-decoder recovery policy. This module never launches an
# application or changes the normal closed-process preparation guard.
. (Join-Path $PSScriptRoot 'ColdBaseline.ps1')
. (Join-Path $PSScriptRoot 'ChartHelperShutdown.ps1')
$script:ColdHelperOwner='OpenNavX.ColdChartHelper.1'
$script:ChartHelperHash='ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb'

function Assert-ColdHelperSet($Candidates,$Processes,$Context) {
  $candidates=@($Candidates);$all=@($Processes)
  if($candidates.Count -lt 1 -or $candidates.Count -gt 3){throw 'One to three exact cold helpers required.'}
  if(@($all|Where-Object {$_.Name -ieq 'opencpn.exe'}).Count){throw 'Every OpenCPN process must be absent.'}
  $seen=@{}
  foreach($helper in $candidates){
    $pidValue=[int]$helper.pid;$parent=[int]$helper.parentPid
    $path=Assert-LocalPath $helper.path
    $expected=Assert-LocalPath (Join-Path $Context.managed 'oexserverd.exe')
    $started=([datetime]($helper.startedUtc)).ToUniversalTime()
    $pipe='OCPN'+($parent%10000).ToString('D4',[Globalization.CultureInfo]::InvariantCulture)
    if($pidValue -le 0 -or $parent -le 0 -or $pidValue -eq $parent -or $seen.ContainsKey($pidValue) -or
       $helper.sid -cne $Context.sid -or $helper.sessionId -ne $Context.session -or $Context.session -le 0 -or
       $path -ine $expected -or $helper.sha256 -cne $script:ChartHelperHash -or
       $started.Ticks -le 0 -or $started -gt [datetime]::UtcNow -or
       $helper.commandLine -cne ('"'+$expected+'" -p '+$pipe)){
      throw 'Cold helper identity, exact source command or managed bytes differ.'
    }
    if(@($all|Where-Object {$_.ProcessId -eq $parent}).Count){throw 'Parent PID exists, including a reused PID.'}
    $seen[$pidValue]=$true
  }
  $actual=@($all|Where-Object {$_.Name -match '^(?i:oeserver\w*|oexserver\w*)\.exe$'})
  if($actual.Count -ne $candidates.Count -or @($actual|Where-Object {-not $seen.ContainsKey([int]$_.ProcessId)}).Count){
    throw 'Unknown or newly started chart helper refused.'
  }
  Assert-PreparationProcesses @($all|Where-Object {-not $seen.ContainsKey([int]$_.ProcessId)}) (@($Context.application,$Context.managed)+@($Context.pluginRoots))
}
function Get-ColdHelperSet($Context) {
  $all=@(Get-CimInstance Win32_Process)
  $helpers=@($all|Where-Object {$_.Name -ieq 'oexserverd.exe'})
  $records=@(foreach($helper in $helpers){
    $owner=Invoke-CimMethod -InputObject $helper -MethodName GetOwnerSid
    if($owner.ReturnValue -ne 0){throw 'Cannot verify chart-helper account.'}
    $process=Get-Process -Id $helper.ProcessId -ErrorAction Stop
    try {
      $path=Assert-LocalPath $process.Path
      if($process.HasExited -or $helper.ExecutablePath -ine $path){throw 'Chart helper changed during inventory.'}
      [pscustomobject]@{pid=[int]$process.Id;parentPid=[int]$helper.ParentProcessId;
        sessionId=[int]$process.SessionId;sid=$owner.Sid;
        startedUtc=$process.StartTime.ToUniversalTime().ToString('o');path=$path;
        sha256=(Get-Digest $path);commandLine=$helper.CommandLine}
    } finally {$process.Dispose()}
  })
  Assert-ColdHelperSet $records $all $Context
  return @($records|Sort-Object pid)
}
function Assert-ColdHelperRemaining($Captured,$Context,$Completed) {
  $remaining=@($Captured|Where-Object {$_.pid -notin @($Completed)})
  if($remaining.Count -eq 0){
    $all=@(Get-CimInstance Win32_Process)
    Assert-PreparationProcesses $all (@($Context.application,$Context.managed)+@($Context.pluginRoots))
    return
  }
  $current=@(Get-ColdHelperSet $Context)
  if($remaining.Count -ne $current.Count){throw 'Complete remaining chart-helper set changed since cold capture.'}
  $seen=@{}
  foreach($expected in $remaining){
    if($seen.ContainsKey([int]$expected.pid)){throw 'Duplicate captured chart-helper identity.'}
    $seen[[int]$expected.pid]=$true
    $match=@($current|Where-Object {$_.pid -eq $expected.pid})
    if($match.Count -ne 1 -or $match[0].parentPid -ne $expected.parentPid -or
       $match[0].sessionId -ne $expected.sessionId -or $match[0].sid -cne $expected.sid -or
       $match[0].path -ine $expected.path -or $match[0].sha256 -cne $expected.sha256 -or
       $match[0].commandLine -cne $expected.commandLine -or
       (([datetime]($match[0].startedUtc)).ToUniversalTime().Ticks) -ne (([datetime]($expected.startedUtc)).ToUniversalTime().Ticks)){
      throw 'Complete remaining chart-helper identity changed since cold capture.'
    }
  }
}
function Assert-ColdHelperReview($Review,$Capture,[string]$CaptureHash) {
  if($Review.schema -ne 1 -or $Review.owner -cne 'OpenNavX.ColdChartHelperReview.1' -or
     $Review.captureSha256 -cne $CaptureHash -or $Review.decision -cne 'approve-exact-cmd-exit-once' -or
     [string]::IsNullOrWhiteSpace($Review.reason) -or $Review.reason.Length -gt 1024 -or
     @($Review.candidates).Count -ne @($Capture.helpers).Count -or
     (@($Review.candidates)|ConvertTo-Json -Depth 5 -Compress) -cne (@($Capture.helpers)|ConvertTo-Json -Depth 5 -Compress)){
    throw 'Independent review must explicitly approve the exact captured candidate set.'
  }
  $at=([datetime]($Review.reviewedUtc)).ToUniversalTime()
  if($at -gt [datetime]::UtcNow -or ([datetime]::UtcNow-$at).TotalHours -gt 24){throw 'Cold helper review expired or is future dated.'}
}
function Assert-ColdHelperCapture($Capture,$Context) {
  if($Capture.schema -ne 1 -or $Capture.owner -cne $script:ColdHelperOwner -or $Capture.status -cne 'captured' -or
     $Capture.profileChanged -isnot [bool] -or $Capture.profileChanged -or
     $Capture.applicationLaunched -isnot [bool] -or $Capture.applicationLaunched -or
     $Capture.launchPermission -isnot [bool] -or $Capture.launchPermission -or
     $Capture.profile.exists -isnot [bool] -or -not $Capture.profile.exists -or
     (Assert-LocalPath $Capture.profile.root) -ine $Context.profile){throw 'Cold capture lacks exact preservation-only profile provenance.'}
  Assert-CommissioningContext $Capture.context $Context
  $seen=@{}
  foreach($tree in @($Capture.pluginTrees)){
    $root=Assert-LocalPath $tree.root
    if($root -inotin @($Context.pluginRoots) -or $seen.ContainsKey($root) -or $tree.exists -isnot [bool]){
      throw 'Cold capture has an unknown, duplicate or malformed plugin tree.'
    }
    $seen[$root]=$true
  }
  if($seen.Count -ne @($Context.pluginRoots).Count){throw 'Cold capture omitted a loader tree.'}
  $null=Assert-ColdHelperRemaining $Capture.helpers $Context @()
}
