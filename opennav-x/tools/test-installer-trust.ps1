param(
  [Parameter(Mandatory=$true)][string]$Launcher,
  [Parameter(Mandatory=$true)][string]$TrustFixture,
  [Parameter(Mandatory=$true)][string]$Evidence
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or
    $PSVersionTable.PSEdition -cne 'Desktop' -or $PSVersionTable.PSVersion.Major -ne 5 -or
    $PSVersionTable.PSVersion.Minor -ne 1) { throw 'Native Windows PowerShell 5.1 is required.' }
$Launcher = [IO.Path]::GetFullPath($Launcher)
$TrustFixture = [IO.Path]::GetFullPath($TrustFixture)
$Evidence = [IO.Path]::GetFullPath($Evidence)
if (Test-Path -LiteralPath $Evidence) { throw 'Evidence directory must be fresh.' }
$null = [IO.Directory]::CreateDirectory($Evidence)
$source = Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/Lifecycle.ps1'
$tokens=$null; $errors=$null
$ast = [Management.Automation.Language.Parser]::ParseFile($source,[ref]$tokens,[ref]$errors)
if ($errors.Count) { throw 'Lifecycle.ps1 does not parse.' }
$names = @('Hash','PlainPath','ReadJson','Generation','ReadState','ReadGeneration','ShortcutGroup',
  'InitializeCommittedTrust','Assert-LifecycleMutation','Assert-NoProtectedBrokerEnrollment','Get-ProtectedBrokerObservation')
$definitions = @($ast.FindAll({ param($node)
  $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -in $names
},$true))
if ($definitions.Count -ne $names.Count) { throw 'Trust handoff functions missing or duplicated.' }
foreach ($definition in $definitions) { . ([scriptblock]::Create($definition.Extent.Text)) }
. (Join-Path (Split-Path $source -Parent) 'UpdateSupervisor.ps1')
$Owner='OpenNavX.Alpha1.SideBySide.1'
$Utf8=New-Object Text.UTF8Encoding($false)
$fixture=Get-Content -LiteralPath $TrustFixture -Raw | ConvertFrom-Json
$temporary=Join-Path ([IO.Path]::GetTempPath()) ('skager-installer-trust-'+[guid]::NewGuid().ToString('N'))
$null=[IO.Directory]::CreateDirectory($temporary)
$Programs=Join-Path $temporary 'unused-shell'
$cases=New-Object 'System.Collections.Generic.List[object]'
$result=[ordered]@{status='failed';scope='Actual installer trust handoff on inert owned generations';
  powershellVersion=$PSVersionTable.PSVersion.ToString();powershellEdition=$PSVersionTable.PSEdition;
  processBits=([IntPtr]::Size*8);launcherSha256=(Hash $Launcher);lifecycleSha256=(Hash $source);
  transactionSha256=(Hash (Join-Path (Split-Path $source -Parent) 'UpdateTransaction.ps1'));
  fixtureSha256=(Hash $TrustFixture);cases=@();productAcceptance=$false;boatAcceptance=$false}
function Check([bool]$Condition,[string]$Message) { if (-not $Condition) { throw $Message } }
function WriteFixtureJson([string]$Path,$Value) {
  [IO.File]::WriteAllText($Path,($Value | ConvertTo-Json -Depth 32),$Utf8)
}
function NewFixture([string]$Name,[string]$Image,[bool]$Configured=$false) {
  $script:Root=Join-Path $temporary $Name
  $generation=Join-Path $Root ('generations\'+('a'*32))
  $app=Join-Path $generation 'app'
  $null=[IO.Directory]::CreateDirectory($app)
  $stock=Join-Path $Root 'stock\opencpn.exe';$null=[IO.Directory]::CreateDirectory((Split-Path $stock -Parent))
  [IO.File]::WriteAllBytes($stock,[Convert]::FromBase64String($fixture.stock))
  Copy-Item -LiteralPath $Image -Destination (Join-Path $app 'skager-start.exe')
  [IO.File]::WriteAllBytes((Join-Path $app 'opencpn.exe'),[Convert]::FromBase64String($fixture.stock))
  foreach($name in @('Lifecycle.ps1','UpdateTransaction.ps1','UpdateSupervisor.ps1','app/skager-update-prompt.exe')) {
    [IO.File]::WriteAllText((Join-Path $generation $name),'inert never-executed generation helper',$Utf8)
  }
  if($Configured) { WriteFixtureJson (Join-Path $app 'update-trust.json') $fixture.trust }
  $files=@(Get-ChildItem -LiteralPath $generation -File -Recurse | ForEach-Object {
    [pscustomobject]@{path=$_.FullName.Substring($generation.Length+1).Replace('\','/');sha256=(Hash $_.FullName)}
  })
  WriteFixtureJson (Join-Path $generation 'ownership.json') @{owner=$Owner;version='1.0.0';commit=('a'*40);
    packageSha256=('c'*64);xnavHardwareOutputPolicy='status-only';updateStartupHealth=1;
    shellLayout='OpenNavX.SkagerStartMenu.1';files=$files;managedFiles=$files}
  WriteFixtureJson (Join-Path $Root 'owner.json') @{owner=$Owner}
  $next=[pscustomobject]@{owner=$Owner;schema=1;current=('a'*32);previous='';
    stock=[pscustomobject]@{path=$stock;sha256=(Hash $stock);version='5.12.4';arch='x86'}}
  WriteFixtureJson (Join-Path $Root 'state.json') $next
  return $next
}
function OpenFixtureLock {
  return [IO.File]::Open((Join-Path $Root 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
}
function Handoff($Next,[bool]$Pass,[string]$FailurePattern='') {
  $lock=OpenFixtureLock;$errorText='';$clock=[Diagnostics.Stopwatch]::StartNew()
  try {
    try { InitializeCommittedTrust $Next ([ref]$lock) } catch { $errorText=$_.Exception.Message }
    if($Pass) { Check (-not $errorText) ('Expected trust success: '+$errorText) }
    else { Check ($errorText -match $FailurePattern) ('Expected explicit refusal: '+$errorText) }
    # Except an armed refusal, the parent must regain exact exclusive custody.
    if(-not (Test-Path -LiteralPath (Join-Path $Root 'signed-update-armed.jsonl'))) {
      Assert-UpdateTransactionLock $Root $lock
    }
    return [pscustomobject]@{elapsedMs=$clock.ElapsedMilliseconds;error=$errorText}
  } finally { if($lock) { $lock.Dispose() } }
}
try {
  # Compile only an inert parent-cleanup adversary. Positive trust, repeat and
  # contention cases below always run the separately built real Go launcher.
  $code=@'
using System;
using System.IO;
using System.Diagnostics;
using System.Threading;
public class InertTrustChild {
 public static int Main(string[] args) {
  try {
   if(args.Length!=1 || args[0]!="--initialize-trust") return 91;
   string app=AppDomain.CurrentDomain.BaseDirectory.TrimEnd('\\');
   string root=Path.GetFullPath(Path.Combine(app,"..","..",".."));
   string mode=File.ReadAllText(Path.Combine(app,"fixture-mode"));
   using(var held=new FileStream(Path.Combine(root,"transaction.lock"),FileMode.Open,FileAccess.ReadWrite,FileShare.None)) {
    bool imageRefused=false,renameRefused=false;
    try { File.WriteAllText(Path.Combine(app,"skager-start.exe"),"replacement"); } catch(IOException) { imageRefused=true; }
    try { Directory.Move(app,app+"-moved"); } catch(IOException) { renameRefused=true; }
    File.WriteAllText(Path.Combine(root,"child.json"),"{\"pid\":"+Process.GetCurrentProcess().Id+",\"lockAcquired\":true,\"imageWriteRefused\":"+imageRefused.ToString().ToLowerInvariant()+",\"directoryRenameRefused\":"+renameRefused.ToString().ToLowerInvariant()+"}");
    if(mode=="state") File.AppendAllText(Path.Combine(root,"state.json")," ");
    else if(mode=="owner") File.AppendAllText(Path.Combine(root,"owner.json")," ");
    else if(mode=="ownership") File.AppendAllText(Path.Combine(app,"..","ownership.json")," ");
    else if(mode=="stock") File.AppendAllText(Path.Combine(root,"stock","opencpn.exe"),"changed");
    else if(mode=="journal") File.WriteAllText(Path.Combine(root,"transaction.json"),"interrupted transaction");
    else if(mode=="armed") File.WriteAllText(Path.Combine(root,"signed-update-armed.jsonl"),"armed");
    else if(mode=="timeout") Thread.Sleep(Timeout.Infinite);
    else if(mode!="custody") return 92;
   }
   if(mode=="armed") File.Delete(Path.Combine(root,"transaction.lock"));
   return 0;
  } catch { return 93; }
 }
}
'@
  $inertSource=Join-Path $temporary 'inert.cs';$inert=Join-Path $temporary 'inert.exe'
  [IO.File]::WriteAllText($inertSource,$code,$Utf8)
  $compiler=Join-Path ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()) 'csc.exe'
  & $compiler /nologo /target:winexe /platform:x86 "/out:$inert" $inertSource
  if($LASTEXITCODE -ne 0) { throw 'Inert trust child compilation failed.' }

  $next=NewFixture 'owner-wait-lifecycle' $Launcher
  $waitImage=Join-Path (Generation $next.current) 'app\skager-start.exe'
  Initialize-UpdatePipeType
  $waitOwner=[Skager.UpdateImageOwner]::Acquire($waitImage,(Split-Path $waitImage -Parent),(Hash $waitImage))
  try { $waitOwner.WaitForOwnedExit() } finally { $waitOwner.Dispose() }
  $disposedRefused=$false
  try { $waitOwner.WaitForOwnedExit() }
  catch { $disposedRefused=$_.Exception.GetBaseException() -is [ObjectDisposedException] }
  Check $disposedRefused 'Disposed native custody owner accepted an exit wait.'
  $cases.Add(@{name='native-owner-wait-lifecycle';passed=$true;noChildWaitInert=$true;disposedWaitRefused=$true})

  $next=NewFixture 'configured' $Launcher $true
  $before=Hash (Join-Path $Root 'state.json')
  $first=Handoff $next $true
  $marker=Join-Path $Root 'update-trust-initialized-beta'
  Check (Test-Path -LiteralPath $marker) 'Real first initialization did not publish its once marker.'
  $trustState=Join-Path $Root 'update-trust\beta\state.json'
  Check (Test-Path -LiteralPath $trustState) 'Real first initialization did not publish retained trust state.'
  $markerHash=Hash $marker;$trustHash=Hash $trustState
  $repeat=Handoff $next $true
  Check ((Hash $marker) -ceq $markerHash -and (Hash $trustState) -ceq $trustHash -and
         (Hash (Join-Path $Root 'state.json')) -ceq $before) 'Repeated initialization reset retained state.'
  $cases.Add(@{name='actual-first-and-repeat';passed=$true;firstMs=$first.elapsedMs;repeatMs=$repeat.elapsedMs;onceUnchanged=$true;stateUnchanged=$true})

  $next=NewFixture 'missing-config' $Launcher
  $null=Handoff $next $true
  Check (-not (Test-Path -LiteralPath (Join-Path $Root 'update-trust'))) 'Absent config initialized trust.'
  $cases.Add(@{name='actual-missing-config';passed=$true})

  $next=NewFixture 'held-lock' $Launcher $true
  $lock=OpenFixtureLock;$process=$null
  try {
    $start=New-Object Diagnostics.ProcessStartInfo
    $start.FileName=Join-Path (Generation $next.current) 'app\skager-start.exe'
    $start.Arguments='--initialize-trust';$start.UseShellExecute=$false;$start.CreateNoWindow=$true
    $start.RedirectStandardError=$true
    $process=[Diagnostics.Process]::Start($start)
    Check ($process.WaitForExit(5000)) 'Exact initialization failure showed a modal or stalled under the held lock.'
    Check ($process.ExitCode -eq 1) 'Held-lock initialization did not return failure.'
    Check ($process.StandardError.ReadToEnd() -match 'installation transaction is busy') 'Held-lock failure did not report its cause.'
    Check (-not (Test-Path -LiteralPath (Join-Path $Root 'update-trust-initialized-beta'))) 'Held-lock child mutated trust.'
    $cases.Add(@{name='actual-held-lock-noninteractive';passed=$true;exitCode=$process.ExitCode})
  } finally {
    if($process) { try { if(-not $process.HasExited) { $process.Kill() };$process.WaitForExit() } finally { $process.Dispose() } }
    $lock.Dispose()
  }

  $next=NewFixture 'bad-config' $Launcher $true
  # Update the declared hash to exercise the actual trust parser failure, not a
  # tampered-managed-file rejection. Neither creates a marker or a modal.
  $generation=Generation $next.current;$config=Join-Path $generation 'app\update-trust.json'
  [IO.File]::WriteAllText($config,'{}',$Utf8)
  $record=ReadGeneration $next.current
  foreach($entry in @($record.files)+@($record.managedFiles)) { if($entry.path -ceq 'app/update-trust.json') {$entry.sha256=Hash $config} }
  WriteFixtureJson (Join-Path $generation 'ownership.json') $record
  $bad=Handoff $next $false 'explicit recovery'
  Check ($bad.elapsedMs -lt 10000 -and -not (Test-Path -LiteralPath (Join-Path $Root 'update-trust-initialized-beta'))) 'Malformed trust blocked or mutated state.'
  $cases.Add(@{name='actual-malformed-trust-noninteractive';passed=$true;elapsedMs=$bad.elapsedMs})

  foreach($mode in @('custody','state','owner','ownership','stock','journal','armed','timeout')) {
    $next=NewFixture ('parent-'+$mode) $inert
    [IO.File]::WriteAllText((Join-Path (Generation $next.current) 'app\fixture-mode'),$mode,$Utf8)
    $pattern=if($mode -eq 'timeout'){'did not finish'}elseif($mode -eq 'armed'){'armed'}else{'changed during trust initialization'}
    $observed=Handoff $next ($mode -eq 'custody') $pattern
    $child=Get-Content -LiteralPath (Join-Path $Root 'child.json') -Raw | ConvertFrom-Json
    Check ($child.lockAcquired -and $child.imageWriteRefused -and $child.directoryRenameRefused) 'Released transaction lost verified image/ancestor custody.'
    if($mode -eq 'armed') { Check (-not (Test-Path -LiteralPath (Join-Path $Root 'transaction.lock'))) 'Armed refusal recreated the lock before guarding mutation.' }
    if($mode -eq 'timeout') {
      Check ($observed.elapsedMs -ge 15000 -and $observed.elapsedMs -lt 30000) 'Owned timeout did not terminate and drain in the fixture.'
      $probe=OpenFixtureLock;$probe.Dispose()
    }
    $cases.Add(@{name=('inert-parent-'+$mode);passed=$true;elapsedMs=$observed.elapsedMs;ownedChildExitObserved=$true;imageAndAncestorCustody=$true})
  }
  $result.status='passed'
} catch {
  $result.failure=$_.Exception.Message.Replace($temporary,'<fixture>')
  throw
} finally {
  $result.cases=@($cases.ToArray())
  # Fixture roots contain only generated public trust data and inert test files.
  # The product helper has already retained custody until its exact child exits.
  # Preserve the first assertion failure if cleanup also fails.
  $cleanupFailure=$null
  try { Remove-Item -LiteralPath $temporary -Recurse -Force }
  catch { $cleanupFailure=$_; $result.cleanupFailure=$_.Exception.Message.Replace($temporary,'<fixture>') }
  if($cleanupFailure) { $result.status='failed' }
  [IO.File]::WriteAllText((Join-Path $Evidence 'results.json'),($result | ConvertTo-Json -Depth 16),$Utf8)
  if($cleanupFailure -and -not $result.Contains('failure')) { throw $cleanupFailure }
}
