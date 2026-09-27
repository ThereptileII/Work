# Standalone native protocol/process gate. Runs marker-only fixture executables
# in a unique temporary directory; no OpenCPN installation/profile or hardware.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Binaries,[Parameter(Mandatory=$true)][string]$Evidence)
$ErrorActionPreference='Stop';Set-StrictMode -Version Latest
if($PSVersionTable.PSEdition -ne 'Desktop') {
  & (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $PSCommandPath -Binaries $Binaries -Evidence $Evidence
  exit $LASTEXITCODE
}
$Binaries=[IO.Path]::GetFullPath($Binaries);$Evidence=[IO.Path]::GetFullPath($Evidence)
foreach($name in @('opencpn.exe','opennav-restart.exe','commissioning_restart_tests.exe')) {
  if(-not [IO.File]::Exists((Join-Path $Binaries $name))){throw ('Missing standalone test binary '+$name)}
}
$root=Split-Path $PSScriptRoot -Parent
Add-Type -Path (Join-Path $PSScriptRoot 'boat\RestartCommissioningNative.cs')
Add-Type -TypeDefinition @'
using System;
using System.Threading;
using System.Threading.Tasks;
public static class OpenNavRestartAsyncTest {
  static int faults;
  static void Fault(object sender,UnobservedTaskExceptionEventArgs args) {
    Interlocked.Increment(ref faults);args.SetObserved();
  }
  public static void Begin() {faults=0;TaskScheduler.UnobservedTaskException+=Fault;}
  public static int Finish() {
    GC.Collect();GC.WaitForPendingFinalizers();GC.Collect();
    TaskScheduler.UnobservedTaskException-=Fault;return faults;
  }
}
'@
$sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
$temporary=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav guarded restart & '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $temporary
$checks=New-Object 'Collections.Generic.List[string]'
$cases=New-Object 'Collections.Generic.List[object]'
function Require($Condition,[string]$Name){if(-not $Condition){throw $Name};$checks.Add($Name)}
function Digest([string]$Path){return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()}
function HashBytes([byte[]]$Bytes){$h=[Security.Cryptography.SHA256]::Create();try{return ([BitConverter]::ToString($h.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant()}finally{$h.Dispose()}}
function RandomHash{$b=New-Object byte[] 32;$r=[Security.Cryptography.RandomNumberGenerator]::Create();try{$r.GetBytes($b)}finally{$r.Dispose()};return ([BitConverter]::ToString($b)).Replace('-','').ToLowerInvariant()}
function WriteText([string]$Path,[string]$Text){[IO.File]::WriteAllText($Path,$Text,(New-Object Text.UTF8Encoding($false)))}
function WaitFile([string]$Path){$end=[DateTime]::UtcNow.AddSeconds(10);while(-not [IO.File]::Exists($Path)){if([DateTime]::UtcNow -ge $end){throw ('Marker absent: '+$Path)};Start-Sleep -Milliseconds 20}}
$status='failed';$failure=$null;$case=$null;$helperExit=$null;$receipt=$null;$currentHelperPid=$null;$currentParentPid=$null
try {
  $expired=[DateTime]::UtcNow.AddSeconds(-1)
  $memory=New-Object IO.MemoryStream
  try {
    $refused=$false;try{[OpenNavX.RestartCommissioningNative]::WriteFrame($memory,[byte[]]@(1),$expired)}catch{$refused=$true}
    Require ($refused -and $memory.Length -eq 0) 'Expired native verifier write performs no I/O'
  } finally {$memory.Dispose()}
  $memory=New-Object IO.MemoryStream(,[byte[]]@(1,0,0,0,42))
  try {
    $refused=$false;try{[OpenNavX.RestartCommissioningNative]::ReadFrame($memory,$expired)}catch{$refused=$true}
    Require ($refused -and $memory.Position -eq 0) 'Expired native verifier read consumes no bytes'
  } finally {$memory.Dispose()}
  # Native .NET Framework APM lifetime regression: exercise operations which
  # are genuinely pending, then cancel them. No marker/OpenCPN process or
  # external endpoint is involved; both pipe ends belong to this test process.
  [OpenNavRestartAsyncTest]::Begin()
  try {
    foreach($pending in @('connect','read')) {
      $token=RandomHash;$server=[OpenNavX.RestartCommissioningNative]::NewPipe($token,$sid)
      $serverHandle=$server.SafePipeHandle;$client=$null;$clientHandle=$null;$connected=$null
      try {
        if($pending -ceq 'read') {
          $client=[IO.Pipes.NamedPipeClientStream]::new('.',('OpenNavX-CommissioningRestart-'+$token),[IO.Pipes.PipeDirection]::InOut,[IO.Pipes.PipeOptions]::Asynchronous)
          $connected=$client.ConnectAsync(3000)
          [OpenNavX.RestartCommissioningNative]::Connect($server,[DateTime]::UtcNow.AddSeconds(3))
          Require ($connected.Wait(3000) -and $server.IsConnected -and $client.IsConnected) 'Pending-read fixture owns both connected local endpoints'
          $clientHandle=$client.SafePipeHandle
        }
        $watch=[Diagnostics.Stopwatch]::StartNew();$timedOut=$false
        try {
          if($pending -ceq 'connect'){[OpenNavX.RestartCommissioningNative]::Connect($server,[DateTime]::UtcNow.AddMilliseconds(250))}
          else{[OpenNavX.RestartCommissioningNative]::ReadFrame($server,[DateTime]::UtcNow.AddMilliseconds(250))}
        } catch {
          $timedOut=$_.Exception.GetBaseException() -is [TimeoutException]
          if(-not $timedOut){throw}
        } finally {$watch.Stop()}
        Require ($timedOut -and $watch.ElapsedMilliseconds -ge 150 -and $watch.ElapsedMilliseconds -lt 7500) ('Native pending '+$pending+' cancels within bounded deadline without closed-event failure')
      } finally {
        $server.Dispose();if($client){$client.Dispose()}
      }
      Require ($serverHandle.IsClosed -and ($null -eq $clientHandle -or $clientHandle.IsClosed)) ('Native pending '+$pending+' closes every owned pipe handle')
      $server=$null;$client=$null;$connected=$null
    }
  } finally {$unobserved=[OpenNavRestartAsyncTest]::Finish()}
  Require ($unobserved -eq 0) 'Pending native cancellation leaves no unobserved asynchronous fault'
  $capability=& (Join-Path $Binaries 'opennav-restart.exe') --commissioning-protocol-self-test
  Require ($LASTEXITCODE -eq 0) 'Actual helper capability probe exits without launching a process'
  $capability=$capability|ConvertFrom-Json
  Require ($capability.contract -ceq 'OpenNavX.RestartCapability.1' -and $capability.role -ceq 'restart-helper' -and
    ($capability.commissioning_restart_protocol -is [int] -or $capability.commissioning_restart_protocol -is [long]) -and $capability.commissioning_restart_protocol -eq 1 -and
    $capability.profile_accessed -is [bool] -and -not $capability.profile_accessed -and
    $capability.child_started -is [bool] -and -not $capability.child_started) 'Actual helper declares compiled guard capability without profile/child access'
  & (Join-Path $Binaries 'commissioning_restart_tests.exe')
  Require ($LASTEXITCODE -eq 0) 'Actual portable protocol executable passes under native MSVC'
  foreach($case in @('plain','partial','invalid','empty','both-empty','unsupported-mode','no-listener','parent-error',
                     'deny','wrong-nonce','wrong-record','wrong-request-hash','expired','overlong','extra-field',
                     'profile-changed','executable-changed','success','safe','xnav','parent-fast-exit','startup-binding','startup-environment','chain-no-listener')) {
    $directory=Join-Path $temporary $case;$null=New-Item -ItemType Directory -Path $directory
    foreach($name in @('opencpn.exe','opennav-restart.exe')){Copy-Item -LiteralPath (Join-Path $Binaries $name) -Destination (Join-Path $directory $name)}
    $exe=Join-Path $directory 'opencpn.exe';$helperPath=Join-Path $directory 'opennav-restart.exe';$profile=Join-Path $directory 'profile.ini'
    WriteText $profile "[Settings]`nFixture=1`n"
    $mode=if($case -ceq 'safe'){'--safe-mode'}elseif($case -ceq 'xnav'){'--xnav'}elseif($case -ceq 'unsupported-mode'){'--xnav-demo'}else{'--legacy'}
    WriteText (Join-Path $directory 'target-mode.txt') $mode
    if($case -ceq 'parent-error'){WriteText (Join-Path $directory 'parent-error.txt') 'yes'}
    if($case -ceq 'parent-fast-exit'){WriteText (Join-Path $directory 'parent-fast-exit.txt') 'yes'}
    if($case -ceq 'startup-binding'){WriteText (Join-Path $directory 'erase-environment-after-start.txt') 'yes'}
    if($case -ceq 'startup-environment'){WriteText (Join-Path $directory 'mutate-runtime-search-path.txt') 'yes'}
    if($case -ceq 'chain-no-listener'){WriteText (Join-Path $directory 'chain-without-listener.txt') 'yes'}
    $session=RandomHash;$record=RandomHash
    $cleanPath=$directory+';'+(Join-Path $env:WINDIR 'System32')+';'+$env:WINDIR
    $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=$exe;$start.Arguments='--parent';$start.WorkingDirectory=$directory;$start.UseShellExecute=$false
    $start.EnvironmentVariables.Remove('OPENNAV_COMMISSIONING_RESTART_SESSION');$start.EnvironmentVariables.Remove('OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256')
    $start.EnvironmentVariables['PATH']=$cleanPath
    $start.EnvironmentVariables['LOCALAPPDATA']=Join-Path $directory 'original-local'
    $start.EnvironmentVariables['APPDATA']=Join-Path $directory 'original-roaming'
    $start.EnvironmentVariables['OPENNAV_TEST_COLD_ENVIRONMENT']='unchanged cold value'
    $start.EnvironmentVariables.Remove('OPENNAV_TEST_RUNTIME_ADDITION')
    if($case -cne 'plain') {
      $start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_SESSION']=if($case -ceq 'invalid'){'invalid'}elseif($case -cin @('empty','both-empty')){''}else{$session}
      if($case -cne 'partial'){$start.EnvironmentVariables['OPENNAV_COMMISSIONING_RESTART_RECORD_SHA256']=if($case -ceq 'both-empty'){''}else{$record}}
    }
    $noBroker=$case -cin @('plain','partial','invalid','empty','both-empty','unsupported-mode','no-listener','parent-error')
    $allow=$case -cin @('success','safe','xnav','parent-fast-exit','startup-binding','startup-environment','chain-no-listener')
    $pipe=$null;$parent=$null;$helper=$null;$receipt=$null;$helperCreated=$null;$helperExit=$null;$currentHelperPid=$null;$currentParentPid=$null
    try {
      if(-not $noBroker){$pipe=[OpenNavX.RestartCommissioningNative]::NewPipe($session,$sid)}
      $parent=[Diagnostics.Process]::Start($start);$parentCreated=$parent.StartTime.ToUniversalTime().ToFileTimeUtc().ToString([Globalization.CultureInfo]::InvariantCulture)
      $currentParentPid=$parent.Id
      WaitFile (Join-Path $directory 'parent-armed.txt')
      if($case -cne 'parent-fast-exit') {
        Require (-not $parent.HasExited) ($case+': parent remains alive while helper is armed')
        Require (@(Get-ChildItem -LiteralPath $directory -Filter 'child*.txt').Count -eq 0) ($case+': replacement never overlaps parent')
      }
      if($noBroker) {
        $armed=[IO.File]::ReadAllLines((Join-Path $directory 'parent-armed.txt'))[0]
        $expectHelper=$case -cin @('plain','no-listener','parent-error')
        Require ($armed -ceq $(if($expectHelper){'yes'}else{'no'})) ($case+': arming result matches exact guard policy')
        # The parent is still alive here, so its normally waiting companion is
        # observable. Retain the actual Process handle, not just a reusable PID.
        $companions=@(Get-CimInstance Win32_Process -Filter ('ParentProcessId='+$parent.Id) | Where-Object {
          $_.ExecutablePath -and $_.ExecutablePath -ieq $helperPath
        })
        Require ($companions.Count -eq $(if($expectHelper){1}else{0})) ($case+': exact fixture helper count')
        if($expectHelper) {
          $helper=Get-Process -Id $companions[0].ProcessId
          $currentHelperPid=$helper.Id
          Require ($helper.Path -ieq $helperPath -and $helper.SessionId -eq [Diagnostics.Process]::GetCurrentProcess().SessionId) ($case+': helper identity is confined to fixture')
          $helperCreated=$helper.StartTime.ToUniversalTime().ToFileTimeUtc().ToString([Globalization.CultureInfo]::InvariantCulture)
          $null=$helper.Handle
        }
      }
      WriteText (Join-Path $directory 'parent-release.txt') 'fixture inspection complete'
      if(-not $noBroker) {
        $deadline=[DateTime]::UtcNow.AddSeconds(20)
        [OpenNavX.RestartCommissioningNative]::Connect($pipe,$deadline)
        $bytes=[OpenNavX.RestartCommissioningNative]::ReadFrame($pipe,$deadline)
        $request=[OpenNavX.RestartCommissioningNative]::Message($bytes)
        Require ($request['protocol'] -eq 1 -and $request['kind'] -ceq 'request') ($case+': exact request protocol')
        Require ($request['session'] -ceq $session -and $request['recordSha256'] -ceq $record) ($case+': immutable startup binding')
        Require ($parent.WaitForExit(10000) -and $parent.ExitCode -eq 0 -and $request['parentExitCode'] -ceq '0') ($case+': actual clean parent exit before request')
        Require ($request['parentPid'] -ceq $parent.Id.ToString() -and $request['parentCreatedFiletime'] -ceq $parentCreated) ($case+': genuine parent identity')
        $peer=[OpenNavX.RestartCommissioningNative]::ClientPid($pipe)
        Require ($request['helperPid'] -ceq $peer.ToString()) ($case+': OS peer PID matches helper request')
        $helper=Get-Process -Id $peer
        # Get-Process opens process queries lazily. Keep its genuine handle
        # before replying, while the helper is still waiting on our permit;
        # otherwise .NET Framework can return null ExitCode after PID exit.
        $null=$helper.Handle;$currentHelperPid=$helper.Id
        $helperCreated=$helper.StartTime.ToUniversalTime().ToFileTimeUtc().ToString([Globalization.CultureInfo]::InvariantCulture)
        Require ($helper.Path -ieq $helperPath -and $helper.SessionId -eq [Diagnostics.Process]::GetCurrentProcess().SessionId) ($case+': actual helper image and session')
        Require ($request['helperCreatedFiletime'] -ceq $helper.StartTime.ToUniversalTime().ToFileTimeUtc().ToString([Globalization.CultureInfo]::InvariantCulture)) ($case+': helper creation time')
        Require ($request['executable'] -ieq $exe -and $request['executableSha256'] -ceq (Digest $exe) -and $request['helperSha256'] -ceq (Digest $helperPath)) ($case+': exact native executable and helper hashes')
        Require ($request['workingDirectory'] -ieq $directory -and $request['path'] -ceq $cleanPath -and @($request['arguments']).Count -eq 1 -and $request['arguments'][0] -ceq $mode) ($case+': exact startup cwd PATH and target mode')
        $hash=HashBytes $bytes;$issued=[DateTime]::UtcNow.ToFileTimeUtc();$expires=$issued+100000000L;$permit=RandomHash
        [string[]]$fields=@('OpenNavX.CommissioningRestart.1','ALLOW',$session,$record,$request['nonce'],$hash,
          $issued.ToString([Globalization.CultureInfo]::InvariantCulture),$expires.ToString([Globalization.CultureInfo]::InvariantCulture),
          $request['executable'],$request['executableSha256'],$request['helper'],$request['helperSha256'],$profile,(Digest $profile),$directory,$cleanPath,$permit)
        switch -CaseSensitive ($case) {
          'deny' {$fields=@('OpenNavX.CommissioningRestart.1','DENY')}
          'wrong-nonce' {$fields[4]=RandomHash}
          'wrong-record' {$fields[3]=RandomHash}
          'wrong-request-hash' {$fields[5]=RandomHash}
          'expired' {$fields[6]=($issued-200000000L).ToString();$fields[7]=($issued-100000000L).ToString()}
          'overlong' {$fields[7]=($issued+100000001L).ToString()}
          'profile-changed' {WriteText $profile "[Settings]`nFixture=changed`n"}
          'executable-changed' {$f=[IO.File]::Open($exe,[IO.FileMode]::Append,[IO.FileAccess]::Write,[IO.FileShare]::None);try{$f.WriteByte(0)}finally{$f.Dispose()}}
        }
        $reply=[OpenNavX.RestartCommissioningNative]::Reply($fields)
        if($case -ceq 'extra-field'){$reply=[byte[]]($reply+@(1))}
        [OpenNavX.RestartCommissioningNative]::WriteFrame($pipe,$reply,[DateTime]::UtcNow.AddSeconds(5))
        if($allow) {
          $receipt=[OpenNavX.RestartCommissioningNative]::Message([OpenNavX.RestartCommissioningNative]::ReadFrame($pipe,[DateTime]::UtcNow.AddSeconds(10)))
          Require ($receipt['kind'] -ceq 'receipt' -and $receipt['status'] -ceq 'started' -and $receipt['requestSha256'] -ceq $hash -and $receipt['permitId'] -ceq $permit -and $receipt['nonce'] -ceq $request['nonce']) ($case+': one request-bound child receipt')
        }
        $pipe.Dispose();$pipe=$null
        Require ($helper.WaitForExit(10000)) ($case+': bounded helper exit')
        $helperExit=$helper.ExitCode
        $expectedExit=if($allow){0}elseif($case -cin @('profile-changed','executable-changed')){32}else{31}
        Require ($helperExit -is [int] -and $helperExit -eq $expectedExit) ($case+': retained native helper exit matches exact allow/refusal outcome')
      } else {
        Require ($parent.WaitForExit(10000)) ($case+': parent finishes normally')
        Require ($parent.ExitCode -eq $(if($case -ceq 'parent-error'){7}else{0})) ($case+': expected parent exit')
        if($helper) {
          Require ($helper.WaitForExit(35000)) ($case+': observed helper exits within guarded deadline')
          $helperExit=$helper.ExitCode
          $expectedExit=if($case -ceq 'plain'){0}elseif($case -ceq 'parent-error'){24}else{29}
          Require ($helperExit -is [int] -and $helperExit -eq $expectedExit) ($case+': actual helper exit reflects exact ordinary restart or guarded refusal')
        }
      }
      if($allow -or $case -ceq 'plain') {
        $marker=Join-Path $directory ('child'+$mode+'.txt');WaitFile $marker
        $lines=[IO.File]::ReadAllLines($marker)
        if($allow){Require ($lines[2] -ceq '1' -and $lines[3] -ceq $session -and $lines[4] -ceq $record -and $lines[5] -ceq $cleanPath) ($case+': child retains armed binding and verified environment')}
        else{Require ($lines[2] -ceq '0') 'Ordinary unarmed restart remains unarmed'}
        Require ($lines[6] -ceq (Join-Path $directory 'original-local') -and $lines[7] -ceq (Join-Path $directory 'original-roaming') -and
          $lines[8] -ceq '' -and $lines[9] -ceq 'unchanged cold value') ($case+': cold environment preserved; runtime additions and deletions do not propagate')
        $child=Get-Process -Id ([int]$lines[0]) -ErrorAction SilentlyContinue
        if($child){try{Require ($child.WaitForExit(10000)) ($case+': marker-only child exits')}finally{$child.Dispose()}}
        Start-Sleep -Milliseconds 500
        Require (@(Get-ChildItem -LiteralPath $directory -Filter 'child*.txt').Count -eq 1) ($case+': exactly one replacement; no second unguarded switch')
      } else {
        Start-Sleep -Milliseconds 800
        Require (@(Get-ChildItem -LiteralPath $directory -Filter 'child*.txt').Count -eq 0) ($case+': refusal creates no replacement')
      }
      $remaining=@(Get-CimInstance Win32_Process -Filter "Name='opennav-restart.exe'" | Where-Object {
        $_.ExecutablePath -and $_.ExecutablePath -ieq $helperPath
      })
      if($remaining.Count -and $case -ceq 'chain-no-listener') {
        Require ($remaining.Count -eq 1) ($case+': at most one second guarded helper')
        $second=Get-Process -Id $remaining[0].ProcessId -ErrorAction SilentlyContinue
        if($second){try{Require ($second.Path -ieq $helperPath -and $second.WaitForExit(10000)) ($case+': second helper refuses and exits without a listener')}finally{$second.Dispose()}}
        $remaining=@(Get-CimInstance Win32_Process -Filter "Name='opennav-restart.exe'" | Where-Object {$_.ExecutablePath -and $_.ExecutablePath -ieq $helperPath})
      }
      Require ($remaining.Count -eq 0) ($case+': no fixture helper remains after outcome')
      $cases.Add(@{name=$case;passed=$true;receipt=$receipt;parentPid=$parent.Id;parentCreatedFiletime=$parentCreated;parentExitCode=$parent.ExitCode;helperPid=$(if($helper){$helper.Id}else{$null});helperCreatedFiletime=$helperCreated;helperExitCode=$helperExit})
    } finally {
      if($pipe){$pipe.Dispose()};if($helper){$helper.Dispose()};if($parent){$parent.Dispose()}
    }
  }
  $status='passed'
} catch {$failure=@{message=$_.Exception.Message;stack=$_.ScriptStackTrace;case=$case;parentPid=$currentParentPid;helperPid=$currentHelperPid;helperExitCode=$helperExit;receipt=$receipt};throw}
finally {
  $null=New-Item -ItemType Directory -Force -Path $Evidence
  $report=@{status=$status;authority='Native Windows marker-only standalone process tests';noMarineCode=$true;checks=$checks.ToArray();cases=$cases.ToArray();failure=$failure;temporary=$temporary}
  WriteText (Join-Path $Evidence 'commissioning-restart-native.json') ($report|ConvertTo-Json -Depth 12)
}
Write-Output ('Native commissioning restart: '+$checks.Count+' checks across '+$cases.Count+' cases passed')
