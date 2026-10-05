# Inert disposable fixtures. No application launch, profile or installed state.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT -and $PSVersionTable.PSVersion.Major -gt 5) {
  & (Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $PSCommandPath
  exit $LASTEXITCODE
}
$module = Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/UpdateTransaction.ps1'
. $module
function Check([bool]$Condition,[string]$Message) { if (-not $Condition) { throw $Message } }
function Reject([scriptblock]$Action) {
  $rejected = $false
  try { & $Action } catch { $rejected = $true }
  Check $rejected 'Expected fail-closed rejection.'
}
function Identity([char]$Digit) {
  return [pscustomobject]@{generation=([string]$Digit)*32;commit=([string]$Digit)*40;packageSha256=([string]$Digit)*64;executableSha256=([string]$Digit)*64}
}
function NewRecord { New-UpdatePendingRecord (Identity 'a') (Identity 'b') }
function HashFile([string]$Path) {
  $sha=[Security.Cryptography.SHA256]::Create(); $stream=[IO.File]::OpenRead($Path)
  try { return ([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','').ToLowerInvariant() }
  finally { $stream.Dispose(); $sha.Dispose() }
}
$fixture = Join-Path ([IO.Path]::GetTempPath()) ('skager-update-fixture-' + [guid]::NewGuid().ToString('N'))
try {
  $null = New-Item -ItemType Directory -Path $fixture
  $path = Join-Path $fixture 'pending.json'
  $record = NewRecord
  Write-UpdatePendingRecord $path $record
  $read = Read-UpdatePendingRecord $path
  Check ($read.transaction -ceq $record.transaction) 'Atomic record read lost transaction identity.'
  Check ((Resolve-UpdatePendingRecovery $read $read.previous.generation) -ceq 'retain-previous') 'Interrupted precommit must retain previous.'
  Check ((Resolve-UpdatePendingRecovery $read $read.candidate.generation) -ceq 'restore-previous') 'Interrupted publication must restore previous.'
  Check ((Resolve-UpdatePendingRecovery $read ('c'*32)) -ceq 'manual-recovery') 'Unrelated generation must require manual recovery.'
  $read.attempts=1; $read.session='d'*32
  Write-UpdatePendingRecord $path $read
  Check ((Read-UpdatePendingRecord $path).attempts -eq 1) 'Consumed attempt was not durable.'
  Check ((Resolve-UpdatePendingRecovery $read $read.candidate.generation) -ceq 'restore-previous') 'Interrupted launch must never silently accept candidate.'
  Reject { New-UpdateStartupSession $read }
  $invalid = NewRecord; $invalid.candidate.commit='wrong'
  Reject { Write-UpdatePendingRecord $path $invalid }
  Check ((Read-UpdatePendingRecord $path).transaction -ceq $record.transaction) 'Invalid replacement changed durable pending record.'
  $invalid = NewRecord; $invalid.attempts=2
  Reject { Assert-UpdatePendingRecord $invalid }
  $invalid = NewRecord; $invalid.attempts=$true; $invalid.session='d'*32
  Reject { Assert-UpdatePendingRecord $invalid }
  Reject { New-UpdatePendingRecord (Identity 'a') (Identity 'a') }
  Reject { Assert-UpdateRecordPath 'relative.json' }
  [IO.File]::WriteAllText($path,'{"passed":true,"status":"success"}')
  Reject { Read-UpdatePendingRecord $path }
  [IO.File]::WriteAllText($path,('x'*4097))
  Reject { Read-UpdatePendingRecord $path }
  Write-Host 'PASS: exact identities, atomic replacement, bounded attempt, interruption recovery, malformed/oversized/untrusted JSON rejection.'

  $supervisor=Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/UpdateSupervisor.ps1'
  $Action='installer-action-sentinel'
  . $supervisor
  Check ($Action -ceq 'installer-action-sentinel') 'Dot-sourcing supervisor changed Lifecycle action.'
  $install=Join-Path $fixture 'installation'
  $null=New-Item -ItemType Directory -Path $install
  function FixtureJson([string]$Path,$Value) { [IO.File]::WriteAllText($Path,($Value|ConvertTo-Json -Depth 12)) }
  function FixtureGeneration([char]$Digit,[string]$Executable='') {
    $id=([string]$Digit)*32
    $directory=Join-Path (Join-Path $install 'generations') $id
    $null=New-Item -ItemType Directory -Path (Join-Path $directory 'app') -Force
    $exe=Join-Path $directory 'app/opencpn.exe'
    if ($Executable) { Copy-Item -LiteralPath $Executable -Destination $exe -Force }
    else { [IO.File]::WriteAllText($exe,'INERT FIXTURE; NOT AN EXECUTABLE') }
    Copy-Item -LiteralPath $module -Destination (Join-Path $directory 'UpdateTransaction.ps1') -Force
    Copy-Item -LiteralPath $supervisor -Destination (Join-Path $directory 'UpdateSupervisor.ps1') -Force
    # This inert fixture engine tests the same lock/guard/finalize contract. It
    # writes only this disposable root; never invokes the real installer shell.
    [IO.File]::WriteAllText((Join-Path $directory 'Lifecycle.ps1'),@'
param([string]$Action,[string]$UpdateTransaction)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'UpdateSupervisor.ps1')
if ($Action -cne 'Rollback') { throw 'Fixture permits guarded rollback only.' }
$root=Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$lock=[IO.File]::Open((Join-Path $root 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
try {
 $pending=Assert-SupervisedRollback $root $UpdateTransaction $lock
 Assert-NoUpdateApplication
 $state=Get-SupervisedInstallState $root
 $state.current=$pending.previous.generation; $state.previous=''
 Write-UpdateReceiptBytes (Join-Path $root 'state.json') ([Text.Encoding]::UTF8.GetBytes(($state|ConvertTo-Json -Depth 8)))
 Complete-SupervisedRollback $root $UpdateTransaction $lock
} finally { $lock.Dispose() }
'@)
    $records=@('app/opencpn.exe','Lifecycle.ps1','UpdateTransaction.ps1','UpdateSupervisor.ps1') | ForEach-Object { [pscustomobject]@{path=$_;sha256=(HashFile (Join-Path $directory $_))} }
    FixtureJson (Join-Path $directory 'ownership.json') ([pscustomobject]@{owner='OpenNavX.Alpha1.SideBySide.1';commit=([string]$Digit)*40;packageSha256=([string]$Digit)*64;xnavHardwareOutputPolicy='status-only';updateStartupHealth=1;files=@($records);managedFiles=@($records)})
    return Get-SupervisedGeneration $install $id
  }
  FixtureJson (Join-Path $install 'owner.json') @{owner='OpenNavX.Alpha1.SideBySide.1'}
  FixtureJson (Join-Path $install 'state.json') @{owner='OpenNavX.Alpha1.SideBySide.1';schema=1;current=('b'*32);previous=''}
  $candidate=FixtureGeneration 'a'
  $previous=FixtureGeneration 'b'
  $stateBefore=[IO.File]::ReadAllText((Join-Path $install 'state.json'))
  Reject { New-SupervisedUpdatePending $install ('a'*32) ('b'*32) $null }
  $held=[IO.File]::Open((Join-Path $install 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
  try {
    Assert-UpdateTransactionLock $install $held
    Reject { New-SupervisedUpdatePending $install ('a'*32) ('b'*32) $held }
    Check (-not (Test-Path -LiteralPath (Join-Path $install 'update-pending.json'))) 'Loader-only prior generation was accepted as known-good.'
    Check ([IO.File]::ReadAllText((Join-Path $install 'state.json')) -ceq $stateBefore) 'Missing known-good receipt changed installation state.'
  } finally { $held.Dispose() }
  [IO.File]::AppendAllText($candidate.executable,'tampered')
  Reject { Get-SupervisedGeneration $install ('a'*32) }
  $candidate=FixtureGeneration 'a'
  Reject { Get-UpdateOwnedPath $candidate.directory '../outside' }
  Reject { Get-UpdateOwnedPath $candidate.directory 'app/CON' }
  Check (-not (Test-UpdateIdentityEqual $candidate.identity $previous.identity)) 'Generation identity comparison ignored differences.'
  $forged=Join-Path $fixture 'forged.receipt'
  [IO.File]::WriteAllText($forged,'{"passed":true,"identity":"known-good"}')
  Reject { Assert-UpdateKnownGoodReceipt $forged $previous.identity }
  Write-Host 'PASS: real exclusive Lifecycle lock, owned inventory/hash validation, dot-source isolation, missing/forged known-good refusal without installation changes.'

  # Compile the exact embedded receiver even on Linux, without invoking Win32.
  $tokens=$null; $errors=$null
  $ast=[Management.Automation.Language.Parser]::ParseFile($module,[ref]$tokens,[ref]$errors)
  Check ($errors.Count -eq 0) 'Module parse failure.'
  $source=@($ast.FindAll({param($node) $node -is [Management.Automation.Language.StringConstantExpressionAst] -and $node.Value.StartsWith('using System;')},$true))
  Check ($source.Count -eq 1) 'Expected one exact native receiver source.'
  Add-Type -TypeDefinition $source[0].Value
  Write-Host 'PASS: exact embedded native receiver compiles.'
  # Test-only CREATE_SUSPENDED fixture: no installed application or equipment.
  Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
public sealed class SuspendedUpdateClient : IDisposable {
 [StructLayout(LayoutKind.Sequential,CharSet=CharSet.Unicode)]
 struct StartupInfo {
  public int cb; public string reserved,desktop,title;
  public int x,y,xSize,ySize,xCount,yCount,fill,flags;
  public short show,reservedBytes; public IntPtr reservedPointer,input,output,error;
 }
 [StructLayout(LayoutKind.Sequential)]
 struct ProcessInfo { public IntPtr process,thread; public int pid,tid; }
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
 static extern bool CreateProcess(string application,StringBuilder command,IntPtr processSecurity,IntPtr threadSecurity,
  bool inherit,uint flags,IntPtr environment,string directory,ref StartupInfo startup,out ProcessInfo info);
 [DllImport("kernel32.dll",SetLastError=true)] static extern uint ResumeThread(IntPtr thread);
 [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr handle);
 [DllImport("kernel32.dll")] static extern bool TerminateProcess(IntPtr process,uint exitCode);
 IntPtr thread; public Process Child { get; private set; }
 public Task<bool> Receiver { get; private set; }
 readonly ManualResetEvent entered=new ManualResetEvent(false);
 public SuspendedUpdateClient(string executable,string script,string pipe,string frame) {
  var startup=new StartupInfo();startup.cb=Marshal.SizeOf(typeof(StartupInfo));
  var command=new StringBuilder("\""+executable+"\" -NoProfile -NonInteractive -File \""+script+"\" -SuspendedPipe \""+pipe+"\" -SuspendedFrame \""+frame+"\"");
  ProcessInfo info;
  if(!CreateProcess(executable,command,IntPtr.Zero,IntPtr.Zero,false,0x08000004,IntPtr.Zero,null,ref startup,out info))
   throw new Win32Exception(Marshal.GetLastWin32Error());
  thread=info.thread;
  try { Child=Process.GetProcessById(info.pid);var held=Child.Handle; }
  catch { TerminateProcess(info.process,70);CloseHandle(thread);thread=IntPtr.Zero;entered.Dispose();if(Child!=null) Child.Dispose();throw; }
  finally { CloseHandle(info.process); }
 }
 public bool BeginReceive(object server,string image,string hash,string frame) {
  Receiver=Task<bool>.Factory.StartNew(delegate {
   entered.Set();
   return (bool)server.GetType().GetMethod("Receive",new Type[]{typeof(Process),typeof(string),typeof(string),typeof(string),typeof(int)}).Invoke(server,new object[]{Child,image,hash,frame,5000});
  });
  return entered.WaitOne(1000);
 }
 public void Resume() {
  if(ResumeThread(thread)!=1) throw new Win32Exception(Marshal.GetLastWin32Error());
 }
 public void Dispose() {
  try {
   // Exact newly created inert child only; retain its handle across cleanup.
   if(Child!=null) { if(!Child.HasExited) { Child.Kill();if(!Child.WaitForExit(5000)) throw new Exception("Inert child cleanup timed out."); } }
   if(Receiver!=null && !Receiver.Wait(5000)) throw new Exception("Inert receiver cleanup timed out.");
  } finally { if(Child!=null) Child.Dispose();if(thread!=IntPtr.Zero) CloseHandle(thread);entered.Dispose(); }
 }
}
'@
  Write-Host 'PASS: suspended-child native regression interop compiles.'
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    Write-Host 'SKIP: Windows authenticated pipe/PID/ACL fixtures require native Windows; Linux results do not qualify them.'
    return
  }
  $childFile=Join-Path $fixture 'inert-client.ps1'
  [IO.File]::WriteAllText($childFile,@'
param([string]$SuspendedPipe,[string]$SuspendedFrame)
$ErrorActionPreference='Stop'
if($SuspendedPipe) {
 $env:SKAGER_UPDATE_PIPE=$SuspendedPipe
 $env:SKAGER_FIXTURE_FRAME=[Text.Encoding]::ASCII.GetString([Convert]::FromBase64String($SuspendedFrame))
 $env:SKAGER_FIXTURE_WRITER='yes'
}
if ($env:SKAGER_FIXTURE_WRITER -eq 'yes') {
 $pipe=New-Object IO.Pipes.NamedPipeClientStream('.', $env:SKAGER_UPDATE_PIPE,[IO.Pipes.PipeDirection]::Out)
 try {
  $pipe.Connect(5000)
  if ($env:SKAGER_FIXTURE_PHASES) {
   foreach ($part in @($env:SKAGER_FIXTURE_PHASES | ConvertFrom-Json)) {
    if ($part.delayBefore) { Start-Sleep -Milliseconds $part.delayBefore }
    $bytes=[Text.Encoding]::ASCII.GetBytes($part.frame)
    $pipe.Write($bytes,0,$bytes.Length); $pipe.Flush()
   }
  } else {
   $bytes=[Text.Encoding]::ASCII.GetBytes($env:SKAGER_FIXTURE_FRAME)
   $pipe.Write($bytes,0,$bytes.Length); $pipe.Flush()
  }
 } finally { $pipe.Dispose() }
}
if ($env:SKAGER_FIXTURE_CLOSED) { [IO.File]::WriteAllText($env:SKAGER_FIXTURE_CLOSED,'pipe-closed') }
Start-Sleep -Seconds 10
'@)
  $executable=[Diagnostics.Process]::GetCurrentProcess().MainModule.FileName
  function StartClient($Session,[string]$Frame,[bool]$Writer=$true,[string]$Phases='') {
    $start=New-Object Diagnostics.ProcessStartInfo
    $start.FileName=$executable; $start.UseShellExecute=$false; $start.CreateNoWindow=$true
    $start.Arguments='-NoProfile -NonInteractive -File "'+$childFile+'"'
    $start.EnvironmentVariables['SKAGER_UPDATE_PIPE']=$Session.pipe
    $start.EnvironmentVariables['SKAGER_FIXTURE_FRAME']=$Frame
    $start.EnvironmentVariables['SKAGER_FIXTURE_PHASES']=$Phases
    $start.EnvironmentVariables['SKAGER_FIXTURE_WRITER']=$(if($Writer){'yes'}else{'no'})
    $start.EnvironmentVariables['SKAGER_FIXTURE_CLOSED']=Join-Path $fixture ($Session.session+'.closed')
    return [Diagnostics.Process]::Start($start)
  }
  foreach ($case in @('healthy','healthy-delayed-receive','wrong-challenge','wrong-commit','wrong-generation','wrong-pid','wrong-hash','oversized','timeout','dead-child')) {
    $pending=NewRecord; $pending.candidate.executableSha256=HashFile $executable
    $session=New-UpdateStartupSession $pending
    Write-UpdatePendingRecord $path $pending
    Reject { $duplicate=New-Object Skager.UpdateStartupPipe($session.pipe); $duplicate.Dispose() }
    $child=$null; $impostor=$null
    try {
      $challenge=$session.challenge
      if ($case -eq 'wrong-challenge') { $challenge='0'*64 }
      $frame='SKAGER-UPDATE-READY/1 '+$pending.candidate.generation+' '+$pending.candidate.commit+' '+$challenge+"`n"
      if ($case -eq 'wrong-commit') { $frame=$frame.Replace($pending.candidate.commit,('c'*40)) }
      if ($case -eq 'wrong-generation') { $frame=$frame.Replace($pending.candidate.generation,('c'*32)) }
      if ($case -eq 'oversized') { $frame+='x'*300 }
      $child=StartClient $session $frame ($case -notin @('wrong-pid','timeout','dead-child'))
      if ($case -eq 'wrong-pid') { $impostor=StartClient $session $frame }
      if ($case -eq 'wrong-hash') { $pending.candidate.executableSha256='0'*64 }
      if ($case -eq 'dead-child') { $child.Kill(); $child.WaitForExit() }
      if ($case -eq 'healthy-delayed-receive') {
        # Require the client to have closed its pipe before Receive begins.
        # This marker controls fixture ordering only; all normal OS/PID/hash/
        # exact-frame authentication still runs before accepting startup.
        $closed=Join-Path $fixture ($session.session+'.closed')
        $deadline=[DateTime]::UtcNow.AddSeconds(5)
        while (-not (Test-Path -LiteralPath $closed) -and [DateTime]::UtcNow -lt $deadline) { Start-Sleep -Milliseconds 10 }
        Check (Test-Path -LiteralPath $closed) 'Inert writer did not finish before delayed receive.'
      }
      $passed=Wait-UpdateStartupSuccess $pending $session $child $executable 5000
      Check ($passed -eq ($case -in @('healthy','healthy-delayed-receive'))) ('Unexpected authenticated startup result: '+$case+'; receiver: '+$session.server.FailureReason)
      if ($case -eq 'healthy') { Reject { Wait-UpdateStartupSuccess $pending $session $child $executable 100 } }
    } finally {
      $session.server.Dispose()
      foreach ($process in @($child,$impostor)) { if ($process) { if (-not $process.HasExited) { $process.Kill(); $process.WaitForExit() }; $process.Dispose() } }
    }
    Write-Host ('PASS: native '+$case)
  }
  function PhasePart([string]$Frame,[int]$Delay=0) {
    return [pscustomobject]@{frame=$Frame;delayBefore=$Delay}
  }
  foreach ($badHumanLimit in @(0,300001)) {
    $pending=NewRecord
    $session=New-UpdateStartupSession $pending
    try {
      $ready=Get-UpdateReadyFrame $pending.candidate $session
      Reject { $session.server.Receive([Diagnostics.Process]::GetCurrentProcess(),$executable,'0'*64,$ready,3000,$badHumanLimit) }
      Check ($null -eq $session.server.VerifiedFrame) 'Invalid human deadline produced startup proof.'
    } finally { $session.server.Dispose() }
  }
  Write-Host 'PASS: human-wait deadline is positive and capped at five minutes.'
  foreach ($case in @('wait-accept','wait-accept-after-initial-deadline','fragmented-phases',
      'wait-cancel','wait-eof','continue-eof','wait-ready','continue-first','cancel-first',
      'repeated-wait','repeated-continue','ready-trailing','cancel-trailing','unknown-phase',
      'spoof-wait-challenge','spoof-wait-generation','spoof-wait-commit','spoof-continue',
      'spoof-cancel','spoof-wait-pid','truncated-wait','oversized-phase','human-timeout',
      'health-timeout','ready-without-eof')) {
    $pending=NewRecord; $pending.candidate.executableSha256=HashFile $executable
    $session=New-UpdateStartupSession $pending
    Write-UpdatePendingRecord $path $pending
    $child=$null; $impostor=$null
    try {
      $ready=Get-UpdateReadyFrame $pending.candidate $session
      $wait=$ready.Replace('SKAGER-UPDATE-READY/1 ','SKAGER-UPDATE-WAIT/1 ')
      $continue=$ready.Replace('SKAGER-UPDATE-READY/1 ','SKAGER-UPDATE-CONTINUE/1 ')
      $cancel=$ready.Replace('SKAGER-UPDATE-READY/1 ','SKAGER-UPDATE-CANCEL/1 ')
      $parts=@((PhasePart $wait),(PhasePart $continue),(PhasePart $ready))
      $humanMs=1500; $expectedReason=''; $expectedPhase=''
      switch ($case) {
        'wait-accept-after-initial-deadline' {
          $humanMs=6000
          # WAIT may exceed startup's 3s test limit; CONTINUE grants a fresh
          # bounded health interval. Neither requires real-time 30s UI health
          # in this inert receiver fixture; the production sender tests do.
          $parts=@((PhasePart $wait),(PhasePart $continue 3500),(PhasePart $ready 2000))
        }
        'fragmented-phases' {
          $parts=@((PhasePart $wait.Substring(0,20)),(PhasePart $wait.Substring(20)),
            (PhasePart ($continue+$ready)))
        }
        'wait-cancel' { $parts=@((PhasePart $wait),(PhasePart $cancel));$expectedReason='human-cancelled';$expectedPhase='cancelled' }
        'wait-eof' { $parts=@((PhasePart $wait));$expectedReason='frame-length';$expectedPhase='awaiting-human' }
        'continue-eof' { $parts=@((PhasePart $wait),(PhasePart $continue));$expectedReason='frame-length';$expectedPhase='resuming' }
        'wait-ready' { $parts=@((PhasePart ($wait+$ready))) }
        'continue-first' { $parts=@((PhasePart $continue)) }
        'cancel-first' { $parts=@((PhasePart $cancel)) }
        'repeated-wait' { $parts=@((PhasePart $wait),(PhasePart $wait 50),(PhasePart $continue)) }
        'repeated-continue' { $parts=@((PhasePart ($wait+$continue+$continue+$ready))) }
        'ready-trailing' { $parts=@((PhasePart ($wait+$continue+$ready+'x')));$expectedReason='frame-trailing' }
        'cancel-trailing' { $parts=@((PhasePart ($wait+$cancel+$ready)));$expectedReason='frame-trailing' }
        'unknown-phase' { $parts=@((PhasePart $wait.Replace('WAIT/1','ALIVE/1'))) }
        'spoof-wait-challenge' { $parts=@((PhasePart $wait.Replace($session.challenge,('0'*64)))) }
        'spoof-wait-generation' { $parts=@((PhasePart $wait.Replace($pending.candidate.generation,('c'*32)))) }
        'spoof-wait-commit' { $parts=@((PhasePart $wait.Replace($pending.candidate.commit,('c'*40)))) }
        'spoof-continue' { $parts=@((PhasePart $wait),(PhasePart $continue.Replace($session.challenge,('0'*64)))) }
        'spoof-cancel' { $parts=@((PhasePart $wait),(PhasePart $cancel.Replace($session.challenge,('0'*64)))) }
        'spoof-wait-pid' { $parts=@((PhasePart $wait));$expectedReason='client-identity' }
        'truncated-wait' { $parts=@((PhasePart $wait.TrimEnd([char]10)));$expectedReason='frame-length' }
        'oversized-phase' { $parts=@((PhasePart ('x'*257)));$expectedReason='frame-oversized' }
        'human-timeout' { $humanMs=180;$parts=@((PhasePart $wait),(PhasePart $continue 400));$expectedReason='human-wait-timeout';$expectedPhase='awaiting-human' }
        'health-timeout' { $parts=@((PhasePart ($wait+$continue)),(PhasePart $ready 3500));$expectedReason='health-timeout';$expectedPhase='resuming' }
        'ready-without-eof' { $parts=@((PhasePart $ready),(PhasePart '' 3500));$expectedReason='read-timeout' }
      }
      $json=ConvertTo-Json -InputObject $parts -Compress
      $child=StartClient $session '' ($case -ne 'spoof-wait-pid') $json
      if ($case -eq 'spoof-wait-pid') { $impostor=StartClient $session '' $true $json }
      $passed=$session.server.Receive($child,$executable,$pending.candidate.executableSha256,$ready,3000,$humanMs)
      $success=$case -in @('wait-accept','wait-accept-after-initial-deadline','fragmented-phases')
      Check ($passed -eq $success) ('Unexpected phase result: '+$case+'; receiver: '+$session.server.FailureReason)
      if ($success) {
        Check ($session.server.Phase -ceq 'healthy' -and $session.server.VerifiedFrame -ceq $ready) 'Only final exact READY plus EOF may establish health.'
      } else {
        Check ($null -eq $session.server.VerifiedFrame -and $null -eq $session.server.VerifiedHash) 'Intermediate/rejected phase became health proof.'
        Reject { Write-UpdateKnownGoodReceipt (Join-Path $fixture ($case+'.receipt')) $pending.candidate $session $executable }
        Check (-not (Test-Path -LiteralPath (Join-Path $fixture ($case+'.receipt')))) 'Intermediate/rejected phase wrote a known-good receipt.'
        if (-not $expectedReason) { $expectedReason='frame-mismatch-or-order' }
        Check ($session.server.FailureReason -ceq $expectedReason) ('Wrong phase refusal category: '+$case+'; '+$session.server.FailureReason)
        if ($expectedPhase) { Check ($session.server.Phase -ceq $expectedPhase) ('Wrong observed phase: '+$case) }
      }
    } finally {
      $session.server.Dispose()
      foreach ($process in @($child,$impostor)) { if ($process) { if (-not $process.HasExited) { $process.Kill(); $process.WaitForExit() }; $process.Dispose() } }
    }
    Write-Host ('PASS: native bounded phase '+$case)
  }
  foreach($wrongHash in @($false,$true)) {
    $pending=NewRecord;$pending.candidate.executableSha256=HashFile $executable
    $session=New-UpdateStartupSession $pending;$suspended=$null
    try {
      $frame=Get-UpdateReadyFrame $pending.candidate $session
      $encoded=[Convert]::ToBase64String([Text.Encoding]::ASCII.GetBytes($frame))
      $suspended=New-Object SuspendedUpdateClient($executable,$childFile,$session.pipe,$encoded)
      $hash=if($wrongHash){'0'*64}else{$pending.candidate.executableSha256}
      Check ($suspended.BeginReceive($session.server,$executable,$hash,$frame)) 'Suspended receiver worker did not start.'
      Check (-not $suspended.Receiver.Wait(500)) 'Receiver rejected live suspended child before its loader could initialize.'
      Check ($null -eq $session.server.VerifiedFrame) 'Suspended child cannot already provide startup proof.'
      $suspended.Resume()
      Check ($suspended.Receiver.Wait(5000)) 'Resumed inert receiver did not finish within its existing deadline.'
      Check ($suspended.Receiver.Result -eq (-not $wrongHash)) ('Suspended startup result differs; receiver: '+$session.server.FailureReason)
      if($wrongHash) {
        Check ($session.server.FailureReason -ceq 'client-identity' -and $null -eq $session.server.VerifiedFrame) 'Deferred full image/hash identity must reject before accepting a frame.'
      } else {
        Check ($session.server.VerifiedFrame -ceq $frame -and $session.server.VerifiedHash -ceq $hash) 'Resumed child receipt did not bind exact authenticated identity.'
      }
    } finally {
      $session.server.Dispose()
      if($suspended){$suspended.Dispose()}
    }
    Write-Host ('PASS: native suspended-loader startup; wrongHash='+$wrongHash)
  }
  function StopInertFixtureApplications {
    foreach ($p in @(Get-Process -Name opencpn -ErrorAction SilentlyContinue)) {
      try {
        if ($p.MainModule.FileName.StartsWith($install+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) { $p.Kill(); $p.WaitForExit() }
      } finally { $p.Dispose() }
    }
  }
  function CompileInertApplication([char]$Digit,[bool]$Healthy,[int]$StayMilliseconds) {
    $output=Join-Path $fixture ('inert-'+$Digit+'-'+[guid]::NewGuid().ToString('N')+'.exe')
    $challenge=if($Healthy){'Environment.GetEnvironmentVariable("SKAGER_UPDATE_CHALLENGE")'}else{"new string('0',64)"}
    $source=@'
using System;
using System.IO;
using System.IO.Pipes;
using System.Text;
using System.Threading;
class InertStartupFixture {
 static void Main() {
  File.AppendAllText(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,"fixture-launches"),"launch\n");
  using(var pipe=new NamedPipeClientStream(".",Environment.GetEnvironmentVariable("SKAGER_UPDATE_PIPE"),PipeDirection.Out)) {
   pipe.Connect(5000);
   string frame="SKAGER-UPDATE-READY/1 "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_GENERATION")+" "+new string('__DIGIT__',40)+" "+__CHALLENGE__+"\n";
   byte[] bytes=Encoding.ASCII.GetBytes(frame); pipe.Write(bytes,0,bytes.Length); pipe.Flush();
  }
  Thread.Sleep(__STAY__);
 }
}
'@
    $source=$source.Replace('InertStartupFixture',('InertStartupFixture'+[guid]::NewGuid().ToString('N'))).Replace('__DIGIT__',[string]$Digit).Replace('__CHALLENGE__',$challenge).Replace('__STAY__',[string]$StayMilliseconds)
    Add-Type -TypeDefinition $source -OutputAssembly $output -OutputType ConsoleApplication
    return $output
  }
  function StageFixturePending {
    $held=[IO.File]::Open((Join-Path $install 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
    try {
      $pending=New-SupervisedUpdatePending $install ('a'*32) ('b'*32) $held
      $before=Get-SupervisedInstallState $install
      Check ($before.current -ceq ('b'*32)) 'Pending creation prematurely published candidate.'
      $before.current='a'*32; $before.previous='b'*32
      FixtureJson (Join-Path $install 'state.json') $before
      return $pending
    } finally { $held.Dispose() }
  }
  try {
    $previous=FixtureGeneration 'b' (CompileInertApplication 'b' $true 60000)
    Check ((Invoke-UpdateSupervision $install 'QualifyCurrent') -ceq 'current-qualified') 'Authenticated current startup was not qualified.'
    Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $install $previous.identity) $previous.identity
    StopInertFixtureApplications
    $emptyProof=New-UpdateHealthSession $previous.identity
    try { Reject { Write-UpdateKnownGoodReceipt (Join-Path $fixture 'unproven.receipt') $previous.identity $emptyProof $previous.executable } }
    finally { $emptyProof.server.Dispose() }
    $candidate=FixtureGeneration 'a' (CompileInertApplication 'a' $true 60000)
    Reject { Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $install $previous.identity) $candidate.identity }
    $pending=StageFixturePending
    Check ((Invoke-UpdateSupervision $install 'LaunchPending' $pending.transaction) -ceq 'candidate-qualified') 'Healthy candidate was not finalized.'
    Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $install $candidate.identity) $candidate.identity
    Check (-not (Test-Path -LiteralPath (Join-Path $install 'update-pending.json'))) 'Healthy pending transaction not finalized.'
    StopInertFixtureApplications
    Write-Host 'PASS: native supervisor known-good bootstrap, proof-only protected receipt, candidate one-shot startup, and healthy completion.'

    FixtureJson (Join-Path $install 'state.json') @{owner='OpenNavX.Alpha1.SideBySide.1';schema=1;current=('b'*32);previous=''}
    $candidate=FixtureGeneration 'a' (CompileInertApplication 'a' $false 1000)
    $pending=StageFixturePending
    Check ((Invoke-UpdateSupervision $install 'LaunchPending' $pending.transaction) -ceq 'previous-restored') 'Rejected candidate did not restore verified previous.'
    Check ((Get-SupervisedInstallState $install).current -ceq ('b'*32)) 'Rollback selected wrong generation.'
    $history=Read-UpdatePendingRecord (Join-Path $install ('update-history/'+$pending.transaction+'-restored.json'))
    Check ($history.attempts -eq 1) 'Failed candidate was not restricted to one attempt.'
    Write-Host 'PASS: native failed startup closes inert child and invokes separately locked guarded rollback.'

    $candidate=FixtureGeneration 'a' (CompileInertApplication 'a' $true 60000)
    $marker=Join-Path $candidate.directory 'app/fixture-launches'
    if (Test-Path -LiteralPath $marker) { Remove-Item -LiteralPath $marker }
    $pending=StageFixturePending
    $pending.attempts=1; $pending.session=[guid]::NewGuid().ToString('N')
    Write-UpdatePendingRecord (Join-Path $install 'update-pending.json') $pending
    Check ((Invoke-UpdateSupervision $install 'RecoverPending' $pending.transaction) -ceq 'previous-restored') 'Interrupted candidate did not restore previous.'
    Check (-not (Test-Path -LiteralPath $marker)) 'Interrupted recovery launched candidate again.'
    Write-Host 'PASS: native interrupted recovery restores previous without any candidate launch.'

    $candidate=FixtureGeneration 'a' (CompileInertApplication 'a' $true 60000)
    $pending=StageFixturePending
    [IO.File]::AppendAllText($candidate.executable,'corruption')
    Check ((Invoke-UpdateSupervision $install 'LaunchPending' $pending.transaction) -ceq 'previous-restored') 'Corrupt candidate blocked verified previous recovery.'
    Check ((Get-SupervisedInstallState $install).current -ceq ('b'*32)) 'Corrupt candidate fallback selected wrong generation.'
    Write-Host 'PASS: native candidate integrity failure uses known-good previous rollback engine without launching candidate.'

    $candidate=FixtureGeneration 'a' (CompileInertApplication 'a' $false 60000)
    $pending=StageFixturePending
    Reject { Invoke-UpdateSupervision $install 'LaunchPending' $pending.transaction }
    Check ((Get-SupervisedInstallState $install).current -ceq ('a'*32)) 'Live candidate triggered unsafe rollback.'
    Check (Test-Path -LiteralPath (Join-Path $install 'update-pending.json')) 'Refused graceful close lost pending recovery.'
    Check (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count -eq 1) 'Supervisor forcibly terminated inert live candidate.'
    StopInertFixtureApplications
    Check ((Invoke-UpdateSupervision $install 'RecoverPending' $pending.transaction) -ceq 'previous-restored') 'Recovery failed after inert process was independently closed.'
    Write-Host 'PASS: native failed graceful close preserves live process, selected generation and pending recovery; later recovery restores previous.'
  } finally { StopInertFixtureApplications }
} finally {
  if (Test-Path -LiteralPath $fixture) { Remove-Item -LiteralPath $fixture -Recurse -Force }
}
