# Inert disposable fixtures. No application launch, profile or installed state.
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
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
    return [Diagnostics.Process]::Start($start)
  }
  foreach ($case in @('healthy','wrong-challenge','wrong-commit','wrong-generation','wrong-pid','wrong-hash','oversized','timeout','dead-child')) {
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
      $passed=Wait-UpdateStartupSuccess $pending $session $child $executable 5000
      Check ($passed -eq ($case -eq 'healthy')) ('Unexpected authenticated startup result: '+$case)
      if ($case -eq 'healthy') { Reject { Wait-UpdateStartupSuccess $pending $session $child $executable 100 } }
    } finally {
      $session.server.Dispose()
      foreach ($process in @($child,$impostor)) { if ($process) { if (-not $process.HasExited) { $process.Kill(); $process.WaitForExit() }; $process.Dispose() } }
    }
    Write-Host ('PASS: native '+$case)
  }
} finally {
  if (Test-Path -LiteralPath $fixture) { Remove-Item -LiteralPath $fixture -Recurse -Force }
}
