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
using System.Runtime.InteropServices;
class InertLifecycleFixture {
 [DllImport("kernel32.dll")] static extern uint GetErrorMode();
 static int Main(string[] args) {
  if(args.Length==2 && args[0]=="--opennav-self-test") {
   if((GetErrorMode() & 0x8003)!=0x8003) return 65;
   File.WriteAllText(args[1], "{\"passed\":true,\"commit\":\"__COMMIT__\",\"version\":\"0.4.0-beta2\",\"profile_initialized\":false,\"plugins_loaded\":false,\"xnav_hardware_output_policy\":\"status-only\",\"test_fixtures\":false,\"build_purpose\":\"INSTALLED PRODUCT\",\"update_startup_health\":__HEALTH__}");
   return 0;
  }
  if(args.Length!=1 || args[0]!="--xnav") return 64;
  using(var pipe=new NamedPipeClientStream(".",Environment.GetEnvironmentVariable("SKAGER_UPDATE_PIPE"),PipeDirection.Out)) {
   pipe.Connect(5000);
   string frame="SKAGER-UPDATE-READY/1 "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_GENERATION")+" __COMMIT__ "+Environment.GetEnvironmentVariable("SKAGER_UPDATE_CHALLENGE")+"\n";
   byte[] bytes=Encoding.ASCII.GetBytes(frame);pipe.Write(bytes,0,bytes.Length);pipe.Flush();
  }
  // Only a fixture-local sentinel can stop this inert process. No profile,
  // plugin, chart, network or equipment code is present in this executable.
  for(int i=0;i<1000 && !File.Exists(Path.Combine(AppDomain.CurrentDomain.BaseDirectory,"fixture-stop"));i++) Thread.Sleep(10);
  return 0;
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
  $generation=Get-SupervisedGeneration $Root $Id;$session=New-UpdateHealthSession $generation.identity;$process=$null
  try {
    $process=Start-SupervisedGeneration $generation $session
    Check (Wait-UpdateGenerationStartupSuccess $generation.identity $session $process $generation.executable 5000) 'Actual receiver authenticates live inert generation'
    $path=Get-UpdateKnownGoodPath $Root $generation.identity
    $null=[IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($path))
    Write-UpdateKnownGoodReceipt $path $generation.identity $session $generation.executable
    Assert-UpdateKnownGoodReceipt $path $generation.identity
  } finally {
    if($process){
      $stop=Join-Path $generation.directory 'app/fixture-stop';[IO.File]::WriteAllText($stop,'stop',$Utf8)
      try{if(-not $process.WaitForExit(12000)){throw 'Inert fixture did not exit.'}}finally{$process.Dispose();Remove-Item -LiteralPath $stop}
    }
    $session.server.Dispose()
  }
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
} finally {
  if(Test-Path -LiteralPath $Registry){Remove-Item -LiteralPath $Registry -Recurse -Force}
  if(Test-Path -LiteralPath $Fixture){Remove-Item -LiteralPath $Fixture -Recurse -Force}
}
# Expected negative child runs leave LASTEXITCODE=1. Success is determined by
# every assertion and cleanup above, not the most recent injected child failure.
exit 0
