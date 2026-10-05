# Native disposable coverage of the actual Lifecycle transaction block. The
# production entrypoint's real user paths are never evaluated. Only its exact
# function definitions and final try/catch/finally execute with fixture roots.
param([switch]$Worker,[string]$Fixture='', [string]$Registry='',
      [string]$FailurePoint='', [string]$UpdateTransaction='')
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT') { throw 'Disposable native Windows CI only.' }
if ($PSVersionTable.PSVersion.Major -ne 5) { throw 'Run this native fixture with Windows PowerShell 5.1.' }
$engine=Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'
$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($engine,[ref]$null,[ref]$errors)
if ($errors) { throw ($errors | Out-String) }
$functions=@($ast.EndBlock.Statements | Where-Object { $_ -is [Management.Automation.Language.FunctionDefinitionAst] })
$transactions=@($ast.EndBlock.Statements | Where-Object { $_ -is [Management.Automation.Language.TryStatementAst] })
if ($functions.Count -lt 1 -or $transactions.Count -ne 1) { throw 'Unexpected Lifecycle AST; review the isolated execution boundary.' }
. (Join-Path $PSScriptRoot '../installer/windows/UpdateSupervisor.ps1')
foreach ($definition in $functions) { . ([scriptblock]::Create($definition.Extent.Text)) }
$Owner='OpenNavX.Alpha1.SideBySide.1'
$Utf8=New-Object Text.UTF8Encoding($false)
if ($Worker) {
  # Reject a hand-written invocation which could bind actual installation or
  # registry paths. The parent alone creates this uniquely named fixture.
  $temp=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')+'\'
  $Fixture=[IO.Path]::GetFullPath($Fixture)
  if (-not $Fixture.StartsWith($temp,[StringComparison]::OrdinalIgnoreCase) -or
      [IO.Path]::GetFileName($Fixture) -cnotmatch '^SkagerLifecycleTest-[a-f0-9]{32}$' -or
      $Registry -cnotmatch '^HKCU:\\Software\\SkagerLifecycleTests\\[a-f0-9]{32}$') { throw 'Invalid disposable fixture roots.' }
  $Root=Join-Path $Fixture 'installation';$Programs=Join-Path $Fixture 'Programs'
  $Action='Rollback';$OpenCpn='';$PackageDirectory='';$ManifestSha256='';$Report=Join-Path $Fixture 'worker-result.json'
  $ShortcutModes='';$SummaryPath='';$SupervisedUpdate=$false
  $SessionLog=New-Object 'Collections.Generic.List[string]'
  $TransactionLock=$null;$OwnsRoot=$false;$LastStartupHealth=0;$RecoveredSupervised=$false
  # Unmodified engine control flow, including failure injection, exit codes,
  # exclusive lock, self-test, publication, pending guard and finalization.
  & ([scriptblock]::Create($transactions[0].Extent.Text))
  throw 'Lifecycle transaction unexpectedly returned without an exit.'
}
if ($Fixture -or $Registry -or $FailurePoint -or $UpdateTransaction) { throw 'Fixture options are private to the child worker.' }
$Fixture=Join-Path ([IO.Path]::GetTempPath()) ('SkagerLifecycleTest-'+[guid]::NewGuid().ToString('N'))
$Registry='HKCU:\Software\SkagerLifecycleTests\'+[guid]::NewGuid().ToString('N')
$Root=Join-Path $Fixture 'installation';$Programs=Join-Path $Fixture 'Programs'
$SessionLog=New-Object 'Collections.Generic.List[string]';$Checks=0
$DiagnosticDirectory=Join-Path $PSScriptRoot '../build/updater-native'
$null=[IO.Directory]::CreateDirectory($DiagnosticDirectory)
$DiagnosticPath=Join-Path $DiagnosticDirectory ('installer-fixture-'+[IO.Path]::GetFileName($Fixture)+'.jsonl')
$RetainFixture=$false
function Diagnostic([string]$Phase,$Details) {
  $record=@{utc=[DateTime]::UtcNow.ToString('o');phase=$Phase;details=$Details}
  $line=$record | ConvertTo-Json -Depth 6 -Compress
  try {[IO.File]::AppendAllText($DiagnosticPath,$line+[Environment]::NewLine,$Utf8)}
  catch {Write-Host ('FIXTURE-DIAGNOSTIC-WRITE-FAILED: '+$_.Exception.GetType().Name)}
  Write-Host ('FIXTURE: '+$line)
}
$powerShell=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe'
function Check([bool]$Condition,[string]$Message) { if(-not $Condition){throw ('FAILED: '+$Message)};$script:Checks++;Write-Host ('PASS: '+$Message) }
function Reject([scriptblock]$Operation,[string]$Message) { $failed=$false;try{& $Operation | Out-Null}catch{$failed=$true};Check $failed $Message }
function RunLifecycle([string]$Fail='', [string]$Transaction='', [int]$Expected=0) {
  $result=Join-Path $Fixture 'worker-result.json'
  if(Test-Path -LiteralPath $result){Remove-Item -LiteralPath $result}
  $arguments=@('-NoProfile','-NonInteractive','-ExecutionPolicy','Bypass','-File',$PSCommandPath,'-Worker','-Fixture',$Fixture,'-Registry',$Registry)
  if($Fail){$arguments+=@('-FailurePoint',$Fail)}
  if($Transaction){$arguments+=@('-UpdateTransaction',$Transaction)}
  & $powerShell @arguments | ForEach-Object { Write-Host $_ }
  Check ($LASTEXITCODE -eq $Expected) ('Actual Lifecycle exit '+$Expected)
  $report=ReadJson $result
  Check ($report.status -ceq $(if($Expected -eq 0){'passed'}else{'failed'})) 'Actual Lifecycle report agrees with process result'
  return $report
}
function State([string]$Current,[string]$Previous='') {
  return @{owner=$Owner;schema=1;stock=$script:Stock;current=$Current;previous=$Previous;shortcutModes=@('xnav','legacy','safe')}
}
function CompileApplication([string]$Output,[string]$Commit,[int]$Health) {
  $source=@'
using System;
using System.IO;
using System.IO.Pipes;
using System.Text;
using System.Threading;
using System.Diagnostics;
using System.Runtime.InteropServices;
class InertLifecycleFixture {
 [DllImport("kernel32.dll")] static extern uint GetErrorMode();
 static readonly Stopwatch Clock=Stopwatch.StartNew();
 static void Note(string stage) {
  try { File.AppendAllText(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,"fixture-client.log"),
   DateTime.UtcNow.ToString("o")+" elapsedMs="+Clock.ElapsedMilliseconds+" pid="+Process.GetCurrentProcess().Id+" stage="+stage+Environment.NewLine); }
  catch { /* Diagnostics never change pipe authentication or process lifetime. */ }
 }
 static int Main(string[] args) {
  if(args.Length==2 && args[0]=="--opennav-self-test") {
   if((GetErrorMode() & 0x8003)!=0x8003) return 65;
   File.WriteAllText(args[1], "{\"passed\":true,\"commit\":\"__COMMIT__\",\"version\":\"0.4.0-beta2\",\"profile_initialized\":false,\"plugins_loaded\":false,\"xnav_hardware_output_policy\":\"status-only\",\"test_fixtures\":false,\"build_purpose\":\"INSTALLED PRODUCT\",\"update_startup_health\":__HEALTH__}");
   return 0;
  }
  if(args.Length!=1 || args[0]!="--xnav") return 64;
  Note("main");
  try {
  // Deterministically cross the previous fixture's five-second receiver
  // boundary. This inert delay is not an application startup-health claim.
  Note("deliberate-delay-start-6000ms");Thread.Sleep(6000);Note("deliberate-delay-finished");
  using(var pipe=new NamedPipeClientStream(".",Environment.GetEnvironmentVariable("SKAGER_UPDATE_PIPE"),PipeDirection.Out)) {
   Note("connect-start-5000ms");pipe.Connect(5000);Note("connected");
   string frame="SKAGER-UPDATE-READY/1 "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_GENERATION")+" __COMMIT__ "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_CHALLENGE")+"\n";
   byte[] bytes=Encoding.ASCII.GetBytes(frame);pipe.Write(bytes,0,bytes.Length);pipe.Flush();Note("frame-written");
  }
  // Only a fixture-local sentinel can stop this inert process. No profile,
  // plugin, chart, network or equipment code is present in this executable.
  Note("hold-start-10000ms");
  for(int i=0;i<1000 && !File.Exists(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,"fixture-stop"));i++) Thread.Sleep(10);
  Note("normal-exit");return 0;
  } catch(Exception error) {
   // An unhandled CLR exception can outlive the test under Windows Error
   // Reporting. Record a bounded nonsecret category and exit deterministically.
   Note("failed:"+error.GetType().Name+":0x"+error.HResult.ToString("X8"));return 70;
  }
 }
}
'@
  $source=$source.Replace('__COMMIT__',$Commit).Replace('__HEALTH__',[string]$Health)
  $inputPath=Join-Path $Fixture 'inert.cs';[IO.File]::WriteAllText($inputPath,$source,$Utf8)
  $compiler=Join-Path ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()) 'csc.exe'
  & $compiler /nologo /target:exe /platform:x86 "/out:$Output" $inputPath
  if($LASTEXITCODE -ne 0){throw 'Inert x86 fixture compilation failed.'}
}
function MakeGeneration([string]$Digit,[int]$Health) {
  $id=$Digit*32;$directory=Generation $id
  $null=[IO.Directory]::CreateDirectory((Join-Path $directory 'app'))
  CompileApplication (Join-Path $directory 'app/opencpn.exe') ($Digit*40) $Health
  foreach($name in @('Lifecycle.ps1','UpdateTransaction.ps1','UpdateSupervisor.ps1')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot ('../installer/windows/'+$name)) -Destination (Join-Path $directory $name)
  }
  foreach($name in @('Maintain.exe','app/skager-start.exe')){[IO.File]::WriteAllText((Join-Path $directory $name),'inert non-executable shortcut target',$Utf8)}
  $files=@(FileRecords $directory)
  AtomicJson (Join-Path $directory 'ownership.json') @{owner=$Owner;version='0.4.0-beta2';commit=($Digit*40);packageSha256=($Digit*64);xnavHardwareOutputPolicy='status-only';shellLayout='OpenNavX.SkagerStartMenu.1';updateStartupHealth=$Health;files=$files;managedFiles=$files;shortcutModes=@('xnav','legacy','safe')}
  return $id
}
function CheckLink([string]$Id,[string]$Target) {
  $shell=New-Object -ComObject WScript.Shell
  $link=$shell.CreateShortcut((Join-Path $Programs 'SKAGER/Skager.lnk'))
  Check ($link.TargetPath -ieq (Join-Path (Generation $Id) $Target) -and $link.Arguments -ceq '--xnav') 'Selected generation has exact startup target and arguments'
}
function Qualify([string]$Id) {
  $generation=$null;$session=$null;$process=$null;$primary=$null;$cleanup=$null
  $phase='generation-verification';$clock=[Diagnostics.Stopwatch]::StartNew();$ticks=0L;$expectedHash=''
  try {
    $generation=Get-SupervisedGeneration $Root $Id;$expectedHash=$generation.identity.executableSha256
    $phase='receiver-create';$session=New-UpdateHealthSession $generation.identity
    Diagnostic $phase @{generation=$Id;elapsedMs=$clock.ElapsedMilliseconds;receiverTimeoutMs=30000;deliberateClientDelayMs=6000;clientConnectTimeoutMs=5000;clientHoldMs=10000}
    $phase='process-start';$process=Start-SupervisedGeneration $generation $session
    $null=$process.get_Handle();$ticks=$process.StartTime.ToUniversalTime().Ticks
    Diagnostic $phase @{generation=$Id;elapsedMs=$clock.ElapsedMilliseconds;pid=$process.Id;startedUtcTicks=$ticks;executableSha256=$expectedHash}
    $phase='authenticated-receive'
    # Native probe 37329174360 measured CLR Main after the old five-second
    # deadline. Allow bounded cold startup plus the deliberate six-second
    # regression delay. Production's 90-second timeout remains unchanged.
    $accepted=Wait-UpdateGenerationStartupSuccess $generation.identity $session $process $generation.executable 30000
    Diagnostic $phase @{generation=$Id;elapsedMs=$clock.ElapsedMilliseconds;accepted=$accepted;receiverReason=$session.server.FailureReason;processExited=$process.HasExited}
    Check $accepted 'Actual receiver authenticates live inert generation'
    $phase='known-good-receipt'
    $path=Get-UpdateKnownGoodPath $Root $generation.identity
    $null=[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($path))
    Write-UpdateKnownGoodReceipt $path $generation.identity $session $generation.executable
    Assert-UpdateKnownGoodReceipt $path $generation.identity
  } catch {
    $primary=$_
    Diagnostic 'qualification-failed' @{at=$phase;generation=$Id;elapsedMs=$clock.ElapsedMilliseconds;error=$_.Exception.Message;receiverReason=$(if($session){$session.server.FailureReason}else{$null})}
  } finally {
    try {if($session){$session.server.Dispose()}} catch {$cleanup=$_}
    if($process){
      $stop=Join-Path $generation.directory 'app/fixture-stop'
      try {
        [IO.File]::WriteAllText($stop,'stop',$Utf8)
        # Cooperative cleanup remains bounded independently of the receiver.
        # Late CLR startup/deliberate delay may outlive it on a failure; never
        # let that replace the authenticated receiver's primary failure.
        if(-not $process.WaitForExit(15000)) {
          Diagnostic 'cooperative-stop-timeout' @{generation=$Id;pid=$process.Id;elapsedMs=$clock.ElapsedMilliseconds}
          $script:RetainFixture=$true
          $prefix=[IO.Path]::GetFullPath((Join-Path $Fixture 'installation/generations')).TrimEnd('\')+'\'
          $expected=[IO.Path]::GetFullPath($generation.executable)
          if($ticks -le 0 -or -not $expected.StartsWith($prefix,[StringComparison]::OrdinalIgnoreCase) -or
             $expected -ine (Join-Path $generation.directory 'app/opencpn.exe') -or
             $process.StartTime.ToUniversalTime().Ticks -ne $ticks -or
             [IO.Path]::GetFullPath($process.MainModule.FileName) -ine $expected -or (Hash $expected) -cne $expectedHash) {throw 'Inert process identity changed; fixture retained without signaling.'}
          # Only this retained handle to our freshly compiled inert temp EXE
          # can be terminated. No production cleanup or image-name killing.
          $process.Kill()
          if(-not $process.WaitForExit(5000)){throw 'Exact inert child did not terminate; fixture retained.'}
          $script:RetainFixture=$false
          Diagnostic 'inert-only-termination' @{generation=$Id;pid=$process.Id;startedUtcTicks=$ticks;executableSha256=$expectedHash;elapsedMs=$clock.ElapsedMilliseconds}
          throw 'Inert fixture required failure-only termination after cooperative deadline.'
        }
        $exit=$process.get_ExitCode()
        Diagnostic 'process-exit' @{generation=$Id;pid=$process.Id;exitCode=$exit;elapsedMs=$clock.ElapsedMilliseconds}
        if($exit -ne 0){throw ('Inert fixture returned '+$exit)}
      } catch {if(-not $cleanup){$cleanup=$_};Diagnostic 'qualification-cleanup-failed' @{error=$_.Exception.Message;elapsedMs=$clock.ElapsedMilliseconds}}
      finally {
        try {if(-not $process.HasExited){$script:RetainFixture=$true}} catch {$script:RetainFixture=$true}
        $process.Dispose()
        if(Test-Path -LiteralPath $stop){try{Remove-Item -LiteralPath $stop}catch{if(-not $cleanup){$cleanup=$_}}}
        $clientLog=Join-Path $generation.directory 'app/fixture-client.log'
        if(Test-Path -LiteralPath $clientLog){
          try {
            if((Get-Item -LiteralPath $clientLog).Length -gt 16384){throw 'Inert client diagnostic exceeds bound.'}
            $saved=Join-Path $DiagnosticDirectory ('installer-fixture-'+[IO.Path]::GetFileName($Fixture)+'-'+$Id+'-client.log')
            Copy-Item -LiteralPath $clientLog -Destination $saved
            Diagnostic 'client-log-retained' @{generation=$Id;path=$saved}
            Remove-Item -LiteralPath $clientLog
          } catch {if(-not $cleanup){$cleanup=$_}}
        } else {Diagnostic 'client-log-missing' @{generation=$Id;elapsedMs=$clock.ElapsedMilliseconds}}
      }
    }
  }
  if($primary){throw $primary}
  if($cleanup){throw $cleanup}
}
function Pending([string]$Candidate,[string]$Previous,[bool]$Publish=$true) {
  AtomicJson (Join-Path $Root 'state.json') (State $Previous)
  $lock=[IO.File]::Open((Join-Path $Root 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
  try{$record=New-SupervisedUpdatePending $Root $Candidate $Previous $lock}finally{$lock.Dispose()}
  if($Publish){$state=State $Candidate $Previous;AtomicJson (Join-Path $Root 'state.json') $state;PublishShell $state}
  return $record
}
function CheckRestored([string]$Previous,$Pending) {
  $state=ReadState
  Check ($state.current -ceq $Previous -and $state.previous -ceq '') 'Actual lifecycle restored the exact previous generation without another rollback target'
  Check (-not(Test-Path -LiteralPath (Join-Path $Root 'update-pending.json'))) 'Pending update removed only after restoration'
  Check (Test-Path -LiteralPath (Join-Path $Root ('update-history/'+$Pending.transaction+'-restored.json'))) 'Restored transaction durably archived'
  $archive=Read-UpdatePendingRecord (Join-Path $Root ('update-history/'+$Pending.transaction+'-restored.json'))
  Check ((Test-UpdateIdentityEqual $archive.candidate $Pending.candidate) -and (Test-UpdateIdentityEqual $archive.previous $Pending.previous)) 'Archived transaction preserves both exact generation identities'
  CheckLink $Previous 'app/skager-start.exe'
}
$primaryFailure=$null;$finalCleanup=$null
try {
  $null=[IO.Directory]::CreateDirectory($Root);$null=[IO.Directory]::CreateDirectory($Programs)
  AtomicJson (Join-Path $Root 'owner.json') @{owner=$Owner}
  $stockFile=Join-Path $Fixture 'untouched-stock.exe';[IO.File]::WriteAllText($stockFile,'inert stock sentinel',$Utf8)
  $Stock=@{path=$stockFile;sha256=(Hash $stockFile)}
  $profile=Join-Path $Fixture 'untouched-profile';[IO.File]::WriteAllText($profile,'navigation data sentinel',$Utf8);$profileHash=Hash $profile
  $old=MakeGeneration 'b' 1;$candidate=MakeGeneration 'a' 1;$historical=MakeGeneration 'c' 0
  AtomicJson (Join-Path $Root 'state.json') (State $old);PublishShell (State $old);CheckLink $old 'app/skager-start.exe'
  # A real authenticated pipe/DPAPI receipt is required; never fabricate proof.
  Reject {Pending $candidate $old} 'Pending creation refuses an unqualified previous generation'
  Qualify $old
  $record=Pending $candidate $old
  $bad=RunLifecycle -Transaction ('d'*32) -Expected 1
  Check ($bad.error -match 'transaction changed' -and (ReadState).current -ceq $candidate -and (Test-Path (Join-Path $Root 'update-pending.json'))) 'Wrong transaction cannot roll back or erase pending evidence'
  $null=RunLifecycle -Transaction $record.transaction
  CheckRestored $old $record
  # Candidate corruption cannot prevent restoration of the independently
  # authenticated previous identity. Restore the fixture bytes afterwards.
  $record=Pending $candidate $old
  $candidateExe=Join-Path (Generation $candidate) 'app/opencpn.exe'
  $candidateBytes=[IO.File]::ReadAllBytes($candidateExe)
  [IO.File]::AppendAllText($candidateExe,'corrupt candidate',$Utf8)
  $null=RunLifecycle -Transaction $record.transaction
  CheckRestored $old $record
  [IO.File]::WriteAllBytes($candidateExe,$candidateBytes)
  # A crash after the state commit leaves the journal and pending record.
  # Reentry must finish both, without treating Rollback as uninstall.
  $record=Pending $candidate $old
  $failed=RunLifecycle -Fail 'after-commit' -Transaction $record.transaction -Expected 1
  Check ($failed.error -ceq 'Injected interruption at after-commit' -and (ReadState).current -ceq $old) 'Failure point reaches the real rollback commit boundary'
  Check ((Test-Path (Join-Path $Root 'transaction.json')) -and (Test-Path (Join-Path $Root 'update-pending.json'))) 'Interrupted rollback retains journal and pending transaction'
  $null=RunLifecycle
  CheckRestored $old $record
  Check (-not(Test-Path (Join-Path $Root 'transaction.json'))) 'Recovery completes shell publication before clearing journal'
  # Pending may exist before candidate state publication. Manual recovery must
  # preserve current installation even though its previous field is empty.
  $record=Pending $candidate $old $false
  $null=RunLifecycle
  CheckRestored $old $record
  # Historical health0 rollback must remove the newer same-group launcher
  # before the state commit, so immutable old maintenance can recover it.
  $state=State $candidate $historical;AtomicJson (Join-Path $Root 'state.json') $state;PublishShell $state
  $null=RunLifecycle -Fail 'after-commit' -Expected 1
  Check ((ReadState).current -ceq $historical -and -not(Test-Path (Join-Path $Programs 'SKAGER'))) 'Health0 rollback commit cannot expose an unrecognized launcher to old maintenance'
  Recover;CheckLink $historical 'app/opencpn.exe'
  PublishShell (ReadState);CheckLink $historical 'app/opencpn.exe'
  foreach($id in @($old,$candidate,$historical)){VerifyFiles (Generation $id) (ReadGeneration $id).files}
  Check $true 'All immutable generation file inventories still verify after recovery'
  Check ((Hash $stockFile) -ceq $Stock.sha256 -and (Hash $profile) -ceq $profileHash) 'Stock and navigation-data sentinels remain byte-identical'
  Write-Host "$Checks actual-Lifecycle native integration checks passed. Inert fixture evidence only; no real application qualification."
} catch {
  $primaryFailure=$_
  Diagnostic 'fixture-primary-failure' @{error=$_.Exception.Message;position=$_.InvocationInfo.PositionMessage}
} finally {
  try {if(Test-Path -LiteralPath $Registry){Remove-Item -LiteralPath $Registry -Recurse -Force}} catch {$finalCleanup=$_}
  try {
    if($RetainFixture){Diagnostic 'fixture-retained' @{path=$Fixture;reason='unresolved exact inert child'}}
    elseif(Test-Path -LiteralPath $Fixture){Remove-Item -LiteralPath $Fixture -Recurse -Force}
  } catch {if(-not $finalCleanup){$finalCleanup=$_}}
  if($finalCleanup){Diagnostic 'fixture-final-cleanup-failed' @{error=$finalCleanup.Exception.Message}}
}
if($primaryFailure){throw $primaryFailure}
if($finalCleanup){throw $finalCleanup}
# Expected negative child runs leave LASTEXITCODE=1. Success is determined by
# every assertion and cleanup above, not the most recent injected child failure.
exit 0
