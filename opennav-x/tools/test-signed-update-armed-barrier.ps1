# Inert policy checks plus native disposable-root presence checks. No app launch.
[CmdletBinding()]
param([switch]$PolicyOnly)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$windows=[Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT
if ($windows -and $PSVersionTable.PSVersion.Major -gt 5 -and -not $PolicyOnly) {
  & (Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $PSCommandPath
  exit $LASTEXITCODE
}
$installer=Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows'
$count=0
function Check([bool]$Condition,[string]$Message) {
  if (-not $Condition) { throw $Message }; $script:count++; Write-Host ('PASS: '+$Message)
}
function Reject([scriptblock]$Operation,[string]$Message) {
  $denied=$false
  try { & $Operation } catch { $denied=$true }
  Check $denied $Message
}
$trees=@{}
foreach ($name in @('UpdateTransaction','UpdateSupervisor','Lifecycle')) {
  $tokens=$null;$errors=$null
  $trees[$name]=[Management.Automation.Language.Parser]::ParseFile((Join-Path $installer ($name+'.ps1')),[ref]$tokens,[ref]$errors)
  Check ($errors.Count -eq 0) ($name+' parses')
}
$source=@($trees.UpdateTransaction.FindAll({param($node) $node -is [Management.Automation.Language.StringConstantExpressionAst] -and $node.Value.StartsWith('using System;')},$true))
Check ($source.Count -eq 1) 'One exact production native source selected'
if (-not ('Skager.UpdateArmedBarrier' -as [type])) { Add-Type -TypeDefinition $source[0].Value }
Check ($null -ne ('Skager.UpdateArmedBarrier' -as [type])) 'Exact production barrier and owner compile without native invocation'
Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Reflection;
public static class ArmedBarrierProbe {
 public static Type Barrier;
 public static readonly Dictionary<string,int> Values=new Dictionary<string,int>(StringComparer.OrdinalIgnoreCase);
 public static readonly List<string> Seen=new List<string>();
 static uint Read(string path) {
  Seen.Add(path); int value;
  if(!Values.TryGetValue(path,out value)) value=16;
  if(value<0) throw new Win32Exception(-value);
  return (uint)value;
 }
 public static void Check(string root,bool allowMissingRoot) {
  Seen.Clear();
  var method=Barrier.GetMethod("CheckAbsent",BindingFlags.Static|BindingFlags.NonPublic);
  try { method.Invoke(null,new object[]{root,allowMissingRoot,new Func<string,uint>(Read),new Func<string,uint>(Read)}); }
  catch(TargetInvocationException error) { throw error.InnerException; }
 }
}
'@
[ArmedBarrierProbe]::Barrier=[Skager.UpdateArmedBarrier]
$root='C:\fixture\installation';$leaf=$root+'\signed-update-armed.jsonl'
[ArmedBarrierProbe]::Values[$leaf]=-2
[ArmedBarrierProbe]::Check($root,$false)
Check ([ArmedBarrierProbe]::Seen[[ArmedBarrierProbe]::Seen.Count-1] -ceq $leaf) 'Only exact marker under checked root is queried'
foreach ($attributes in @(0,1,16,32,1024,1040)) {
  [ArmedBarrierProbe]::Values[$leaf]=$attributes
  Reject { [ArmedBarrierProbe]::Check($root,$false) } ('Any present leaf denies (attributes '+$attributes+')')
}
foreach ($errorCode in @(3,5,32,123,206)) {
  [ArmedBarrierProbe]::Values[$leaf]=-$errorCode
  Reject { [ArmedBarrierProbe]::Check($root,$false) } ('Uncertain leaf lookup denies (Win32 '+$errorCode+')')
}
[ArmedBarrierProbe]::Values[$leaf]=-2
foreach ($attributes in @(0,1024,1040,-2,-3,-5,-32)) {
  [ArmedBarrierProbe]::Values['C:\fixture']=$attributes
  Reject { [ArmedBarrierProbe]::Check($root,$false) } ('Non-plain or inaccessible ancestor denies ('+$attributes+')')
}
[ArmedBarrierProbe]::Values['C:\fixture']=16
[ArmedBarrierProbe]::Values[$root]=-2
[ArmedBarrierProbe]::Check($root,$true)
Check (-not [ArmedBarrierProbe]::Seen.Contains($leaf)) 'Fresh install permits proven missing root without claiming a leaf lookup succeeded'
foreach ($errorCode in @(3,5,32)) {
  [ArmedBarrierProbe]::Values[$root]=-$errorCode
  Reject { [ArmedBarrierProbe]::Check($root,$true) } ('Fresh install still denies uncertain root (Win32 '+$errorCode+')')
}
[ArmedBarrierProbe]::Values[$root]=16
foreach ($ambiguous in @('', 'relative', 'C:fixture', 'C:\', '\\server\share\installation', '\\?\C:\installation', 'C:\fixture\..\installation', 'C:\fixture\.\installation', 'C:\fixture\\installation', 'C:\fixture\installation\\', 'C:\fixture\installation.', 'C:\fixture\installation ', 'C:\fixture\installation:stream', 'C:\fixture\CON')) {
  Reject { [ArmedBarrierProbe]::Check($ambiguous,$true) } 'Ambiguous installation root denies before lookup'
  Check ([ArmedBarrierProbe]::Seen.Count -eq 0) 'Ambiguous root made no attribute queries'
}
if ($PolicyOnly -or -not $windows) {
  Write-Host ('PASS: '+$count+' portable checks. SKIP: native presence, entry and transaction-lock gates require Windows; no native APIs invoked.')
  exit 0
}
. (Join-Path $installer 'UpdateSupervisor.ps1')
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('skager-armed-barrier-'+[guid]::NewGuid().ToString('N'))
$lock=$null
try {
  Assert-NoSignedUpdateArmed $fixture $true
  Check $true 'Fresh native install permits a proven missing root'
  Reject { Assert-NoSignedUpdateArmed $fixture } 'Ordinary startup refuses a missing installation root'
  $null=New-Item -ItemType Directory -Path $fixture
  Assert-NoSignedUpdateArmed $fixture
  Check $true 'Actual native lookup permits absent marker'
  $marker=Join-Path $fixture 'signed-update-armed.jsonl'
  # State need not parse: marker rejection must precede installed-state parsing.
  [IO.File]::WriteAllText((Join-Path $fixture 'state.json'),'INERT STATE SENTINEL')
  [IO.File]::WriteAllText((Join-Path $fixture 'update-pending.json'),'INERT ATTEMPT SENTINEL')
  foreach ($bytes in @('', 'malformed', '{"receipt":"not authority"}')) {
    [IO.File]::WriteAllText($marker,$bytes)
    Reject { Assert-NoSignedUpdateArmed $fixture } 'Actual native marker presence denies independent of content'
    foreach ($action in @('LaunchPending','RecoverPending','QualifyCurrent')) {
      $failure=''
      try { Invoke-UpdateSupervision $fixture $action } catch { $failure=$_.Exception.Message }
      Check ($failure -match 'Signed update session is armed') ($action+' denied before installed-state parsing')
    }
    Check (-not [IO.File]::Exists((Join-Path $fixture 'transaction.lock'))) 'Denied entry created no transaction lock'
  }
  Remove-Item -LiteralPath $marker
  $null=New-Item -ItemType Directory -Path $marker
  Reject { Assert-NoSignedUpdateArmed $fixture } 'Marker directory also denies'
  Remove-Item -LiteralPath $marker
  $lock=[IO.File]::Open((Join-Path $fixture 'transaction.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
  Assert-NoSignedUpdateArmed $fixture
  [IO.File]::WriteAllText($marker,'armed after initial check')
  foreach ($operation in @(
    { New-SupervisedUpdatePending $fixture ('a'*32) ('b'*32) $lock },
    { Assert-SupervisedRollback $fixture ('c'*32) $lock },
    { Complete-SupervisedRollback $fixture ('c'*32) $lock },
    { Complete-UpdatePending $fixture $null 'healthy' },
    { Stop-InterruptedUpdateCandidate $fixture $null },
    { Invoke-UpdateGuardedRollback $fixture $null },
    { Start-SupervisedGeneration $null $null $fixture }
  )) {
    $failure=''
    try { & $operation } catch { $failure=$_.Exception.Message }
    Check ($failure -match 'Signed update session is armed') 'Mutation/recovery/spawn helper denies newly present marker before using arguments'
  }
  # Execute the unchanged Lifecycle entry guard and mutating helper definitions.
  $entry=@($trees.Lifecycle.EndBlock.Statements | Where-Object { $_ -is [Management.Automation.Language.TryStatementAst] })[0].Body.Statements[0]
  function Log([string]$Message) { $script:notice=$Message }
  $Root=$fixture;$SignedUpdateRestriction=''
  foreach ($Action in @('Install','Update','Repair','Rollback','Uninstall')) {
    Reject { & ([scriptblock]::Create($entry.Extent.Text)) } ('Lifecycle '+$Action+' refuses before any installed-state or package access')
  }
  foreach ($Action in @('Preflight','Diagnostics')) {
    $script:notice=''
    . ([scriptblock]::Create($entry.Extent.Text))
    Check ($script:notice -match 'startup and maintenance restricted') ($Action+' explicitly reports read-only restriction')
  }
  foreach ($name in @('Assert-LifecycleMutation','AtomicJson','SelfTest','Recover','RemoveShell','PublishShell','RemoveOwnedGenerations','ExtractPayload','PreserveAdditions')) {
    $function=@($trees.Lifecycle.FindAll({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq $name},$false))[0]
    . ([scriptblock]::Create($function.Extent.Text))
  }
  $Action='Update'
  foreach ($operation in @({AtomicJson 'unused' $null},{SelfTest 'unused' '' ''},{Recover},{RemoveShell},{PublishShell $null},{RemoveOwnedGenerations},{ExtractPayload 'unused' '' @()},{PreserveAdditions 'unused' '' @()})) {
    Reject $operation 'Lifecycle direct mutation helper refuses armed installation'
  }
  Check ([IO.File]::ReadAllText((Join-Path $fixture 'state.json')) -ceq 'INERT STATE SENTINEL') 'Restriction preserves selected state byte-for-byte'
  Check ([IO.File]::ReadAllText((Join-Path $fixture 'update-pending.json')) -ceq 'INERT ATTEMPT SENTINEL') 'Restriction preserves pending attempt byte-for-byte'
  Check ([IO.File]::ReadAllText($marker) -ceq 'armed after initial check') 'Restriction never edits or disarms marker'
} finally {
  if ($lock) { $lock.Dispose() }
  if ([IO.Directory]::Exists($fixture)) { Remove-Item -LiteralPath $fixture -Recurse -Force }
}
Write-Host ('PASS: '+$count+' armed-barrier checks; no application or installer mutation was launched.')
exit 0
