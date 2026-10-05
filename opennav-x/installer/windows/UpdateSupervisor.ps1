# Windows PowerShell 5.1. Dot-source for Lifecycle hooks; direct invocation is
# the supervisor. Only this process's authenticated startup can create health
# evidence. All changes are confined to the existing per-user installation.
[CmdletBinding()]
param(
  [Alias('Action')][ValidateSet('LaunchPending','RecoverPending','QualifyCurrent')][string]$SupervisorAction='RecoverPending',
  [Alias('Transaction')][ValidatePattern('^(|[a-f0-9]{32})$')][string]$SupervisorTransaction='',
  [Alias('InstallationRoot')][string]$SupervisorInstallationRoot=(Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'OpenNavXAlpha1')
)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'UpdateTransaction.ps1')

function Read-UpdateOwnedJson([string]$Path, [long]$Limit=4194304) {
  $Path=Assert-UpdateRecordPath $Path
  if (-not [IO.File]::Exists($Path)) { throw 'Required installation record missing.' }
  $file=Get-Item -LiteralPath $Path -Force
  if ($file.Length -le 0 -or $file.Length -gt $Limit) { throw 'Invalid installation record size.' }
  return [IO.File]::ReadAllText($Path) | ConvertFrom-Json
}
function Get-UpdateFileHash([string]$Path) {
  $Path=Assert-UpdateRecordPath $Path
  $sha=[Security.Cryptography.SHA256]::Create(); $file=$null
  try { $file=[IO.File]::OpenRead($Path); return ([BitConverter]::ToString($sha.ComputeHash($file))).Replace('-','').ToLowerInvariant() }
  finally { if ($file) { $file.Dispose() }; $sha.Dispose() }
}
function Get-UpdateOwnedPath([string]$Base, [string]$Relative) {
  if ($Relative -cnotmatch '^[A-Za-z0-9_ .()&@/+-]+$' -or $Relative.StartsWith('/') -or $Relative.Contains('//') -or
      $Relative -match '(^|/)\.{1,2}(/|$)' -or $Relative -match '[. ](/|$)' -or
      $Relative -match '(^|/)(CON|PRN|AUX|NUL|COM[0-9]|LPT[0-9])(\.|/|$)') { throw 'Unsafe owned generation path.' }
  return Assert-UpdateRecordPath (Join-Path $Base $Relative)
}
function Get-SupervisedInstallState([string]$InstallationRoot) {
  $null=Assert-UpdateRecordPath $InstallationRoot
  if (-not [IO.Directory]::Exists($InstallationRoot) -or (Read-UpdateOwnedJson (Join-Path $InstallationRoot 'owner.json') 4096).owner -cne 'OpenNavX.Alpha1.SideBySide.1') { throw 'Unknown installation owner.' }
  $state=Read-UpdateOwnedJson (Join-Path $InstallationRoot 'state.json') 32768
  if ($state.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $state.schema -ne 1 -or
      $state.current -cnotmatch '^[a-f0-9]{32}$' -or $state.previous -cnotmatch '^(|[a-f0-9]{32})$') { throw 'Invalid supervised installation state.' }
  return $state
}
function Get-SupervisedGeneration([string]$InstallationRoot, [string]$Generation) {
  if ($Generation -cnotmatch '^[a-f0-9]{32}$') { throw 'Invalid supervised generation.' }
  $directory=Assert-UpdateRecordPath (Join-Path (Join-Path $InstallationRoot 'generations') $Generation)
  $owned=Read-UpdateOwnedJson (Join-Path $directory 'ownership.json')
  if ($owned.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or $owned.xnavHardwareOutputPolicy -cne 'status-only') { throw 'Supervised startup requires an owned status-only product.' }
  if (-not $owned.PSObject.Properties['updateStartupHealth'] -or $owned.updateStartupHealth -ne 1) { throw 'This generation does not support authenticated startup health.' }
  $files=@($owned.files); $managed=@($owned.managedFiles)
  if ($files.Count -lt 1 -or $files.Count -gt 12000 -or $managed.Count -lt 1 -or $managed.Count -gt 12000) { throw 'Invalid generation inventory.' }
  $seen=@{}
  foreach ($entry in $files) {
    $path=Get-UpdateOwnedPath $directory $entry.path
    if ($entry.sha256 -cnotmatch '^[a-f0-9]{64}$' -or $seen.ContainsKey($entry.path)) { throw 'Invalid generation file declaration.' }
    $seen[$entry.path]=$entry.sha256
    if (-not [IO.File]::Exists($path) -or (Get-UpdateFileHash $path) -cne $entry.sha256) { throw ('Generation file changed: '+$entry.path) }
  }
  $managedSeen=@{}
  foreach ($entry in $managed) {
    $null=Get-UpdateOwnedPath $directory $entry.path
    if ($managedSeen.ContainsKey($entry.path) -or -not $seen.ContainsKey($entry.path) -or $seen[$entry.path] -cne $entry.sha256) { throw 'Managed file lacks matching immutable inventory.' }
    $managedSeen[$entry.path]=$entry.sha256
  }
  foreach ($required in @('app/opencpn.exe','Lifecycle.ps1','UpdateTransaction.ps1','UpdateSupervisor.ps1')) {
    if (-not $managedSeen.ContainsKey($required)) { throw ('Supervised generation lacks owned helper: '+$required) }
  }
  $identity=[pscustomobject]@{generation=$Generation;commit=$owned.commit;packageSha256=$owned.packageSha256;executableSha256=$managedSeen['app/opencpn.exe']}
  Assert-UpdateIdentity $identity
  return [pscustomobject]@{identity=$identity; directory=$directory; executable=(Get-UpdateOwnedPath $directory 'app/opencpn.exe'); lifecycle=(Get-UpdateOwnedPath $directory 'Lifecycle.ps1')}
}
function Get-UpdateKnownGoodPath([string]$InstallationRoot,$Identity) {
  Assert-UpdateIdentity $Identity
  return Assert-UpdateRecordPath (Join-Path (Join-Path $InstallationRoot 'known-good') ($Identity.generation+'.receipt'))
}
function Assert-UpdateTransactionLock([string]$InstallationRoot,[IO.FileStream]$Lock) {
  $path=Assert-UpdateRecordPath (Join-Path $InstallationRoot 'transaction.lock')
  if (-not $Lock -or -not $Lock.CanWrite -or -not [string]::Equals([IO.Path]::GetFullPath($Lock.Name),$path,[StringComparison]::OrdinalIgnoreCase)) { throw 'Lifecycle transaction lock is required.' }
  $probe=$null; $exclusive=$false
  try { $probe=[IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None) }
  catch [IO.IOException] { $exclusive=$true }
  finally { if ($probe) { $probe.Dispose() } }
  if (-not $exclusive) { throw 'Lifecycle transaction lock is not exclusive.' }
}
function New-SupervisedUpdatePending([string]$InstallationRoot,[string]$CandidateGeneration,[string]$PreviousGeneration,[IO.FileStream]$Lock) {
  Assert-UpdateTransactionLock $InstallationRoot $Lock
  $state=Get-SupervisedInstallState $InstallationRoot
  if ($state.current -cne $PreviousGeneration) { throw 'Previous generation changed before supervised update.' }
  $path=Assert-UpdateRecordPath (Join-Path $InstallationRoot 'update-pending.json')
  if (Test-Path -LiteralPath $path) { throw 'Resolve the pending supervised update first.' }
  $previous=Get-SupervisedGeneration $InstallationRoot $PreviousGeneration
  Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $InstallationRoot $previous.identity) $previous.identity
  $candidate=Get-SupervisedGeneration $InstallationRoot $CandidateGeneration
  $record=New-UpdatePendingRecord $candidate.identity $previous.identity
  Write-UpdatePendingRecord $path $record
  return $record
}
function Get-ValidatedUpdatePending([string]$InstallationRoot,[string]$Transaction='') {
  $state=Get-SupervisedInstallState $InstallationRoot
  $pending=Read-UpdatePendingRecord (Join-Path $InstallationRoot 'update-pending.json')
  if (-not $pending -or ($Transaction -and $pending.transaction -cne $Transaction)) { throw 'Supervised transaction changed or is unavailable.' }
  $previous=Get-SupervisedGeneration $InstallationRoot $pending.previous.generation
  if (-not (Test-UpdateIdentityEqual $previous.identity $pending.previous)) { throw 'Previous generation identity changed.' }
  Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $InstallationRoot $previous.identity) $previous.identity
  $decision=Resolve-UpdatePendingRecovery $pending $state.current
  if ($decision -ceq 'manual-recovery') { throw 'Current generation is unrelated to the pending update; explicit recovery required.' }
  if ($decision -ceq 'restore-previous' -and $state.previous -cne $pending.previous.generation) { throw 'Recorded rollback target changed.' }
  return [pscustomobject]@{state=$state; pending=$pending; previous=$previous; decision=$decision}
}
function Assert-SupervisedRollback([string]$InstallationRoot,[string]$Transaction,[IO.FileStream]$Lock) {
  Assert-UpdateTransactionLock $InstallationRoot $Lock
  if ($Transaction -cnotmatch '^[a-f0-9]{32}$') { throw 'Guarded rollback requires the pending transaction.' }
  $context=Get-ValidatedUpdatePending $InstallationRoot $Transaction
  if ($context.decision -cne 'restore-previous') { throw 'Guarded rollback requires the current candidate and exact previous generation.' }
  # Candidate corruption must not prevent recovery. Authenticate the stored
  # identity and known-good target; Lifecycle re-verifies previous owned files.
  return $context.pending
}
function Complete-UpdatePending([string]$InstallationRoot,$Pending,[string]$Outcome) {
  if ($Outcome -notin @('healthy','restored','unpublished')) { throw 'Unknown supervised update outcome.' }
  $history=Assert-UpdateRecordPath (Join-Path $InstallationRoot 'update-history')
  $null=New-Item -ItemType Directory -Path $history -Force
  $archive=Join-Path $history ($Pending.transaction+'-'+$Outcome+'.json')
  if (Test-Path -LiteralPath $archive) {
    $existing=Read-UpdatePendingRecord $archive
    if ($existing.transaction -cne $Pending.transaction -or -not (Test-UpdateIdentityEqual $existing.candidate $Pending.candidate) -or -not (Test-UpdateIdentityEqual $existing.previous $Pending.previous)) { throw 'Conflicting supervised update history.' }
  } else { Write-UpdatePendingRecord $archive $Pending }
  Remove-Item -LiteralPath (Assert-UpdateRecordPath (Join-Path $InstallationRoot 'update-pending.json'))
}
function Complete-SupervisedRollback([string]$InstallationRoot,[string]$Transaction,[IO.FileStream]$Lock) {
  Assert-UpdateTransactionLock $InstallationRoot $Lock
  $context=Get-ValidatedUpdatePending $InstallationRoot $Transaction
  if ($context.decision -cne 'retain-previous') { throw 'Previous generation was not restored.' }
  Complete-UpdatePending $InstallationRoot $context.pending 'restored'
}
function Assert-NoUpdateApplication {
  if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'Close OpenCPN, SKAGER, Legacy and Safe Mode before supervised startup.' }
}
function Start-SupervisedGeneration($Generation,$Session) {
  Assert-NoUpdateApplication
  if ((Get-UpdateFileHash $Generation.executable) -cne $Generation.identity.executableSha256) { throw 'Executable changed before launch.' }
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$Generation.executable; $start.Arguments='--xnav'
  $start.WorkingDirectory=Join-Path $Generation.directory 'app'; $start.UseShellExecute=$false
  $start.EnvironmentVariables['SKAGER_UPDATE_PIPE']=$Session.pipe
  $start.EnvironmentVariables['SKAGER_UPDATE_GENERATION']=$Generation.identity.generation
  $start.EnvironmentVariables['SKAGER_UPDATE_CHALLENGE']=$Session.challenge
  # Ordinary installed XNav startup: no alternate profile, demo, hardware, or
  # connection arguments. Existing application status-only gates remain active.
  return [Diagnostics.Process]::Start($start)
}
function Stop-SupervisedProcess([Diagnostics.Process]$Process,[string]$Executable,[string]$StartTicks='') {
  if ($Process.HasExited) { return }
  if ($StartTicks -and $Process.StartTime.ToUniversalTime().Ticks.ToString() -cne $StartTicks) { throw 'Candidate PID was reused; no process was signaled.' }
  if (-not [string]::Equals([IO.Path]::GetFullPath($Process.MainModule.FileName),[IO.Path]::GetFullPath($Executable),[StringComparison]::OrdinalIgnoreCase)) { throw 'Candidate process image changed; no process was signaled.' }
  $null=$Process.CloseMainWindow()
  if (-not $Process.WaitForExit(10000)) { throw 'Candidate is still running. Close it before recovery; no forced termination or rollback was performed.' }
}
function Stop-InterruptedUpdateCandidate([string]$InstallationRoot,$Pending) {
  $candidatePath=Get-UpdateOwnedPath (Join-Path (Join-Path $InstallationRoot 'generations') $Pending.candidate.generation) 'app/opencpn.exe'
  # Include the spawn-before-PID-persist crash window. Only the exact candidate
  # image may be asked to close; other OpenCPN processes merely block recovery.
  foreach ($process in @(Get-Process -Name opencpn -ErrorAction SilentlyContinue)) {
    try {
      if ([string]::Equals([IO.Path]::GetFullPath($process.MainModule.FileName),$candidatePath,[StringComparison]::OrdinalIgnoreCase)) {
        Stop-SupervisedProcess $process $candidatePath
      }
    } finally { $process.Dispose() }
  }
  Assert-NoUpdateApplication
}
function Invoke-UpdateGuardedRollback([string]$InstallationRoot,$Pending) {
  # Called ONLY after releasing transaction.lock. Lifecycle acquires it again
  # and uses -UpdateTransaction to recheck the exact current/previous identities.
  $previous=Get-SupervisedGeneration $InstallationRoot $Pending.previous.generation
  if (-not (Test-UpdateIdentityEqual $previous.identity $Pending.previous)) { throw 'Rollback engine generation changed.' }
  Assert-UpdateKnownGoodReceipt (Get-UpdateKnownGoodPath $InstallationRoot $previous.identity) $previous.identity
  $engine=$previous.lifecycle
  $powerShell=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe'
  $start=New-Object Diagnostics.ProcessStartInfo
  $start.FileName=$powerShell; $start.UseShellExecute=$false
  $start.Arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$engine+'" -Action Rollback -UpdateTransaction '+$Pending.transaction
  $process=[Diagnostics.Process]::Start($start)
  try {
    if (-not $process.WaitForExit(120000)) { throw 'Guarded rollback has not completed; its process retains the installation lock. Inspect recovery before retrying.' }
    if ($process.ExitCode -ne 0) { throw 'Guarded rollback failed; pending recovery evidence retained.' }
  } finally { $process.Dispose() }
  $state=Get-SupervisedInstallState $InstallationRoot
  if ($state.current -cne $Pending.previous.generation -or (Test-Path -LiteralPath (Join-Path $InstallationRoot 'update-pending.json'))) { throw 'Guarded rollback did not restore and finalize the recorded generation.' }
}
function Invoke-UpdateSupervision([string]$InstallationRoot,[string]$Action,[string]$Transaction='') {
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Supervised application startup requires native Windows.' }
  $state=Get-SupervisedInstallState $InstallationRoot
  $lockPath=Assert-UpdateRecordPath (Join-Path $InstallationRoot 'transaction.lock')
  $lock=[IO.File]::Open($lockPath,[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
  $session=$null; $process=$null; $rollback=$null
  try {
    Assert-UpdateTransactionLock $InstallationRoot $lock
    $state=Get-SupervisedInstallState $InstallationRoot
    if ($Action -ceq 'QualifyCurrent') {
      if (Test-Path -LiteralPath (Join-Path $InstallationRoot 'update-pending.json')) { throw 'Recover the pending update before qualifying current startup.' }
      $generation=Get-SupervisedGeneration $InstallationRoot $state.current
      $session=New-UpdateHealthSession $generation.identity
      $process=Start-SupervisedGeneration $generation $session
      if (-not (Wait-UpdateGenerationStartupSuccess $generation.identity $session $process $generation.executable)) {
        throw ('Current generation did not provide authenticated healthy startup; no known-good receipt was created. Receiver: '+$session.server.FailureReason)
      }
      $known=Get-UpdateKnownGoodPath $InstallationRoot $generation.identity
      $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($known)) -Force
      Write-UpdateKnownGoodReceipt $known $generation.identity $session $generation.executable
      return 'current-qualified'
    }
    $context=Get-ValidatedUpdatePending $InstallationRoot $Transaction
    $pending=$context.pending
    if ($Action -ceq 'RecoverPending' -or $pending.attempts -ne 0 -or $context.decision -ceq 'retain-previous') {
      Stop-InterruptedUpdateCandidate $InstallationRoot $pending
      if ($context.decision -ceq 'retain-previous' -and -not (Test-Path -LiteralPath (Join-Path $InstallationRoot 'transaction.json'))) {
        Complete-UpdatePending $InstallationRoot $pending $(if ($pending.attempts -eq 0) { 'unpublished' } else { 'restored' })
        return 'previous-retained'
      }
      $rollback=$pending
    } elseif ($Action -ceq 'LaunchPending') {
      if (-not $Transaction) { throw 'Candidate launch requires an explicit pending transaction.' }
      try {
        $generation=Get-SupervisedGeneration $InstallationRoot $pending.candidate.generation
        if (-not (Test-UpdateIdentityEqual $generation.identity $pending.candidate)) { throw 'Candidate identity changed before launch.' }
        $session=New-UpdateStartupSession $pending
        Write-UpdatePendingRecord (Join-Path $InstallationRoot 'update-pending.json') $pending
        $process=Start-SupervisedGeneration $generation $session
        $pending.processId=$process.Id; $pending.processStartTicks=$process.StartTime.ToUniversalTime().Ticks.ToString()
        Write-UpdatePendingRecord (Join-Path $InstallationRoot 'update-pending.json') $pending
        if (-not (Wait-UpdateStartupSuccess $pending $session $process $generation.executable)) { throw ('Candidate startup health was not authenticated. Receiver: '+$session.server.FailureReason) }
        $known=Get-UpdateKnownGoodPath $InstallationRoot $generation.identity
        $null=New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($known)) -Force
        Write-UpdateKnownGoodReceipt $known $generation.identity $session $generation.executable
        Complete-UpdatePending $InstallationRoot $pending 'healthy'
        return 'candidate-qualified'
      } catch {
        Write-Warning ('Supervised candidate failed: '+$_.Exception.Message)
        if ($process) { Stop-SupervisedProcess $process $generation.executable $pending.processStartTicks }
        Assert-NoUpdateApplication
        $rollback=$pending
      }
    } else { throw 'Unknown supervision action.' }
  } catch {
    if ($Action -ceq 'QualifyCurrent' -and $process) { Stop-SupervisedProcess $process $generation.executable }
    throw
  } finally {
    if ($session) { $session.server.Dispose() }
    if ($process) { $process.Dispose() }
    $lock.Dispose()
  }
  if ($rollback) { Invoke-UpdateGuardedRollback $InstallationRoot $rollback; return 'previous-restored' }
}
if ($MyInvocation.InvocationName -ne '.') {
  try { Write-Output (Invoke-UpdateSupervision $SupervisorInstallationRoot $SupervisorAction $SupervisorTransaction); exit 0 }
  catch { Write-Error -ErrorRecord $_; exit 1 }
}
