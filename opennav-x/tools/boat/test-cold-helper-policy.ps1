# Pure cold chart-helper policy checks. No filesystem orchestration, real
# processes, vendor executable, OpenCPN, boat, or hardware access.
. (Join-Path $PSScriptRoot 'ColdChartHelper.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Value,[string]$Name){if(-not $Value){throw $Name};$checks.Add($Name)}
function Pass([string]$Name,[scriptblock]$Body){& $Body;$checks.Add($Name)}
function Refuse([string]$Name,[scriptblock]$Body){$failed=$false;try{$null=&$Body}catch{$failed=$true};Check $failed $Name}
function Clone($Value){return ($Value|ConvertTo-Json -Depth 12|ConvertFrom-Json)}

# The policy consumes Windows paths. On non-Windows PowerShell, retain their
# Windows spelling with a small path-only adapter; no path is opened or tested.
$native=[Environment]::OSVersion.Platform -eq 'Win32NT'
if(-not $native){
  function Assert-LocalPath([string]$Path){
    if($Path -notmatch '^[A-Za-z]:\\' -or $Path -match '[\x00-\x1f"]'){throw 'Use an absolute local Windows fixture path.'}
    return $Path.TrimEnd('\')
  }
  function Join-Path([string]$Path,[string]$ChildPath){
    if($Path -match '^[A-Za-z]:\\'){return ($Path.TrimEnd('\')+'\'+$ChildPath.TrimStart('\'))}
    return Microsoft.PowerShell.Management\Join-Path -Path $Path -ChildPath $ChildPath
  }
}
  $now=[datetime]::UtcNow
  $context=[pscustomobject]@{
    sid='S-1-5-21-223-1001';session=4;managed='C:\fixture\user-plugins'
    application='C:\fixture\stock';pluginRoots=@('C:\fixture\user-plugins','C:\fixture\stock\plugins','C:\fixture\installed\generations\g-0123456789abcdef0123456789abcdef\app\plugins')
    installation=[pscustomobject]@{root='C:\fixture\installed';generation='C:\fixture\installed\generations\g-0123456789abcdef0123456789abcdef';commit='c'*40}
  }
  $hash='ec27c947fc9ba4ae09345961780b885deddc2e4fcf094805a14cdf19504e0adb'
  $started=$now.AddMinutes(-5)
  $helper1=[pscustomobject]@{pid=28101;parentPid=1040;sessionId=4;sid=$context.sid;startedUtc=$started.ToString('o');path='C:\fixture\user-plugins\oexserverd.exe';sha256=$hash;commandLine='"C:\fixture\user-plugins\oexserverd.exe" -p OCPN1040'}
  $helper2=[pscustomobject]@{pid=28102;parentPid=2057;sessionId=4;sid=$context.sid;startedUtc=$started.AddSeconds(1).ToString('o');path='C:\fixture\user-plugins\oexserverd.exe';sha256=$hash;commandLine='"C:\fixture\user-plugins\oexserverd.exe" -p OCPN2057'}
  $helpers=@($helper1,$helper2)
  $processes=@(
    [pscustomobject]@{Name='oexserverd.exe';ProcessId=28101;ExecutablePath=$helper1.path},
    [pscustomobject]@{Name='oexserverd.exe';ProcessId=28102;ExecutablePath=$helper2.path},
    [pscustomobject]@{Name='explorer.exe';ProcessId=700;ExecutablePath='C:\Windows\explorer.exe'}
  )

  Pass 'Two exact known-hash helpers with matching SID/session/managed path/source command and absent parents are accepted under an installed-generation context' {
    Assert-ColdHelperSet $helpers $processes $context
  }

  foreach($field in @('sid','sessionId','path','sha256','commandLine','startedUtc')){
    Refuse "Refuses helper $field mismatch" {
      $bad=Clone $helpers
      switch($field){
        'sid' {$bad[0].sid+='-9'}
        'sessionId' {$bad[0].sessionId++}
        'path' {$bad[0].path='C:\fixture\other\oexserverd.exe'}
        'sha256' {$bad[0].sha256='f'*64}
        'commandLine' {$bad[0].commandLine+=' -d'}
        'startedUtc' {$bad[0].startedUtc=$now.AddMinutes(1).ToString('o')}
      }
      Assert-ColdHelperSet $bad $processes $context
    }
  }
  Refuse 'Refuses a parent PID that has been reused by a live process' {
    $reused=@($processes)+@([pscustomobject]@{Name='ordinary.exe';ProcessId=1040;ExecutablePath='C:\Windows\ordinary.exe'})
    Assert-ColdHelperSet $helpers $reused $context
  }
  Refuse 'Refuses duplicate captured helper PID' {
    Assert-ColdHelperSet @($helper1,(Clone $helper1)) $processes $context
  }
  Refuse 'Refuses a captured helper missing from the current process inventory' {
    Assert-ColdHelperSet $helpers @($processes|Where-Object {$_.ProcessId -ne 28102}) $context
  }
  Refuse 'Refuses an unreviewed extra chart helper' {
    $extra=[pscustomobject]@{Name='oexserverd.exe';ProcessId=28103;ExecutablePath=$helper1.path}
    Assert-ColdHelperSet $helpers (@($processes)+@($extra)) $context
  }
  Refuse 'Refuses live OpenCPN' {
    $live=[pscustomobject]@{Name='opencpn.exe';ProcessId=30001;ExecutablePath='C:\fixture\stock\opencpn.exe'}
    Assert-ColdHelperSet $helpers (@($processes)+@($live)) $context
  }
  Refuse 'Refuses an unrelated process from the installed generation plugin tree' {
    $plugin=[pscustomobject]@{Name='other.exe';ProcessId=30002;ExecutablePath='C:\fixture\installed\generations\g-0123456789abcdef0123456789abcdef\app\plugins\other.exe'}
    Assert-ColdHelperSet $helpers (@($processes)+@($plugin)) $context
  }
  Refuse 'Refuses duplicate candidate PIDs' {
    Assert-ColdHelperSet @($helper1,(Clone $helper1),$helper2) $processes $context
  }

  $capture=[pscustomobject]@{helpers=$helpers}
  $captureHash='a'*64
  $review=[pscustomobject]@{schema=1;owner='OpenNavX.ColdChartHelperReview.1';captureSha256=$captureHash;
    decision='approve-exact-cmd-exit-once';reason='Exact measured cold helper set, no commissioning active.';
    reviewedUtc=$now.AddMinutes(-2).ToString('o');candidates=$helpers}
  Pass 'Exact current review binds approval, capture hash, and both captured helpers' {Assert-ColdHelperReview $review $capture $captureHash}
  Refuse 'Refuses changed approval decision' {$bad=Clone $review;$bad.decision='approve';Assert-ColdHelperReview $bad $capture $captureHash}
  Refuse 'Refuses review bound to a changed capture hash' {$bad=Clone $review;$bad.captureSha256='b'*64;Assert-ColdHelperReview $bad $capture $captureHash}
  Refuse 'Refuses changed expected capture hash' {Assert-ColdHelperReview $review $capture ('b'*64)}
  Refuse 'Refuses expired review' {$bad=Clone $review;$bad.reviewedUtc=$now.AddHours(-25).ToString('o');Assert-ColdHelperReview $bad $capture $captureHash}
  Refuse 'Refuses future-dated review' {$bad=Clone $review;$bad.reviewedUtc=$now.AddMinutes(1).ToString('o');Assert-ColdHelperReview $bad $capture $captureHash}
  Refuse 'Refuses review that changes the exact helper set' {$bad=Clone $review;$bad.candidates[0].sha256='f'*64;Assert-ColdHelperReview $bad $capture $captureHash}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();filesystemOrchestration=$false;realProcesses=$false;vendorExecution=$false;boatAccess=$false;hardware=$false}|ConvertTo-Json -Depth 5
