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
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    Write-Host 'SKIP: Windows authenticated pipe/PID/ACL fixtures require native Windows; Linux results do not qualify them.'
    return
  }
  $childFile=Join-Path $fixture 'inert-client.ps1'
  [IO.File]::WriteAllText($childFile,@'
$ErrorActionPreference='Stop'
if ($env:SKAGER_FIXTURE_WRITER -eq 'yes') {
 $pipe=New-Object IO.Pipes.NamedPipeClientStream('.', $env:SKAGER_UPDATE_PIPE,[IO.Pipes.PipeDirection]::Out)
 try {
  $pipe.Connect(5000)
  $bytes=[Text.Encoding]::ASCII.GetBytes($env:SKAGER_FIXTURE_FRAME)
  $pipe.Write($bytes,0,$bytes.Length); $pipe.Flush()
 } finally { $pipe.Dispose() }
}
if ($env:SKAGER_FIXTURE_CLOSED) { [IO.File]::WriteAllText($env:SKAGER_FIXTURE_CLOSED,'pipe-closed') }
Start-Sleep -Seconds 10
'@)
  $executable=[Diagnostics.Process]::GetCurrentProcess().MainModule.FileName
  function StartClient($Session,[string]$Frame,[bool]$Writer=$true) {
    $start=New-Object Diagnostics.ProcessStartInfo
    $start.FileName=$executable; $start.UseShellExecute=$false; $start.CreateNoWindow=$true
    $start.Arguments='-NoProfile -NonInteractive -File "'+$childFile+'"'
    $start.EnvironmentVariables['SKAGER_UPDATE_PIPE']=$Session.pipe
    $start.EnvironmentVariables['SKAGER_FIXTURE_FRAME']=$Frame
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
