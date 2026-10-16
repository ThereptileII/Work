[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][ValidateSet('Inspect','Hold','Restore')][string]$Action,
  [Parameter(Mandatory=$true)][string]$Workspace,
  [string]$Record,
  [string]$ExpectedRecordSha256
)
# One known incompatible stock entry DLL only. Never launch, install, edit a
# profile, remove a helper directory, or relax the installer's PE closure.
. (Join-Path $PSScriptRoot 'Preparation.ps1')

function Assert-RtlsdrBytes([string]$Path) {
  $path=Assert-LocalPath $Path
  $item=Get-Item -LiteralPath $path -Force
  if ($item.PSIsContainer -or $item.Length -ne 208384 -or
      (Get-Digest $path) -cne '404e69931034140fbda0179964d0a29077a46b78a441c05fc68a4ee0616a64e5') {
    throw 'Only the exact reviewed incompatible RTL-SDR entry DLL may be held.'
  }
  Initialize-PreparationNative
  if ([OpenNavX.PreparationNative]::LinkCount($path) -ne 1) { throw 'Hard-linked RTL-SDR input refused.' }
}
function Get-RtlsdrPaths($Context,$Installed) {
  $source=Assert-LocalPath (Join-Path $Context.application 'plugins\rtlsdr_pi.dll')
  Assert-PreparationSeparate $Context.workspace @($Context.profile,$Context.application,$Context.managed,$Installed.root)
  foreach ($entry in @($env:PATH -split ';')) {
    if ($entry -match '^[A-Za-z]:\\') { Assert-PreparationSeparate $Context.workspace @($entry.Trim('"')) }
  }
  if ((Assert-LocalPath $Installed.state.stock.path) -ine $Context.executable) { throw 'Installed stock identity differs from validated stock.' }
  return [pscustomobject]@{source=$source;root=$Installed.root}
}
function Assert-RtlsdrColdRecords($Context,$Installed) {
  Assert-NoActiveCommissioning $Context.workspace
  Assert-NoManualPilotChild $Context.workspace
  foreach ($name in @('update-pending.json','transaction.json','signed-update-armed.jsonl')) {
    $path=Assert-LocalPath (Join-Path $Installed.root $name)
    if (Test-Path -LiteralPath $path) { throw ('Pending/armed installation requires recovery before RTL-SDR hold: '+$name) }
  }
  $values=Read-ProfileForAudit (Join-Path $Context.profile 'opencpn.ini')
  if ($values['Directories/pluginInstallDir']) { throw 'Custom plugin import roots are outside this hold.' }
}
function Get-RtlsdrTaskCommand($Action) {
  # Task Scheduler exposes different native CIM action subclasses. A COM
  # handler has no Execute property; missing/unknown shapes are not Exec actions.
  try { $class=$Action.CimClass.CimClassName } catch { throw 'Unknown scheduled action class.' }
  if ($class -ceq 'MSFT_TaskExecAction') { $fields=@('Id','Execute','Arguments','WorkingDirectory');$forbidden=@('ClassId','Data') }
  elseif ($class -ceq 'MSFT_TaskComHandlerAction') { $fields=@('Id','ClassId','Data');$forbidden=@('Execute','Arguments','WorkingDirectory') }
  else { throw 'Unknown scheduled action class.' }
  foreach ($field in $forbidden) {
    if ($null -ne $Action.PSObject.Properties[$field]) { throw 'Malformed scheduled action shape.' }
  }
  foreach ($field in $fields) {
    $property=$Action.PSObject.Properties[$field]
    if ($null -eq $property -or ($null -ne $property.Value -and $property.Value -isnot [string])) { throw 'Malformed scheduled action shape.' }
  }
  if ($class -ceq 'MSFT_TaskExecAction') {
    if ([string]::IsNullOrWhiteSpace($Action.Execute)) { throw 'Malformed scheduled Exec action.' }
    return ([string]$Action.Execute+' '+[string]$Action.Arguments+' '+[string]$Action.WorkingDirectory)
  }
  $classId=[guid]::Empty
  if (-not [guid]::TryParse([string]$Action.ClassId,[ref]$classId) -or $classId -eq [guid]::Empty) { throw 'Malformed scheduled COM handler identity.' }
  # SKAGER registers Exec actions only. Known COM action data can be empty;
  # inspect any explicit path text, without loading COM or inspecting its registry.
  return [string]$Action.Data
}
function Assert-RtlsdrActors($Processes,$Tasks,[string[]]$Roots) {
  Assert-PreparationProcesses $Processes $Roots
  foreach ($process in @($Processes)) {
    if ($process.Name -match '^(?i:skager-start|skager-update-prompt)\.exe$') { throw 'Close SKAGER launch/maintenance processes normally first.' }
    if ($process.ProcessId -eq $PID) { continue }
    if ($process.CommandLine -and $process.CommandLine -match '(?i)(Lifecycle|UpdateSupervisor|commission-read-only|manual-pilot|hold-incompatible-rtlsdr)\.ps1') {
      throw 'Concurrent maintenance/commissioning helper refused.'
    }
  }
  foreach ($task in @($Tasks)) {
    if ($task.TaskName -match '^(?i:OpenNavX|SKAGER)') { throw 'Remove or resolve application scheduled tasks before this cold hold.' }
    foreach ($action in @($task.Actions)) {
      $command=Get-RtlsdrTaskCommand $action
      if ($command -match '(?i)(opencpn|skager-start|rtl_ais|rtl_fm|aisdecoder|ais-?catcher|oeserver\w*|oexserver\w*)\.exe') { throw 'Scheduled application/helper launch is incompatible with a cold hold.' }
      foreach ($root in $Roots) {
        if ($command.IndexOf($root,[StringComparison]::OrdinalIgnoreCase) -ge 0) { throw 'Scheduled action can access a protected application/profile tree.' }
      }
    }
  }
}
function Assert-RtlsdrCold($Context,$Installed) {
  Assert-PreparationStock $Context
  $null=Get-RtlsdrPaths $Context $Installed
  Assert-RtlsdrColdRecords $Context $Installed
  Assert-RtlsdrActors @(Get-CimInstance Win32_Process) @(Get-ScheduledTask -ErrorAction Stop) @($Context.application,$Context.managed,$Context.profile,$Installed.root,$Context.workspace)
}
function Get-RtlsdrTreeDigest([string]$Root,[string]$Excluded='') {
  $tree=Get-PreparationTree $Root
  $entries=@($tree.entries | Where-Object { $_.path -ine $Excluded })
  $json=ConvertTo-Json -InputObject ([ordered]@{exists=$tree.exists;entries=$entries}) -Depth 8 -Compress
  $sha=[Security.Cryptography.SHA256]::Create()
  try { return ([BitConverter]::ToString($sha.ComputeHash([Text.Encoding]::UTF8.GetBytes($json)))).Replace('-','').ToLowerInvariant() }
  finally { $sha.Dispose() }
}
function Get-RtlsdrPreserved($Context,[string]$Generation) {
  # Hash complete inventories, retaining compact digests rather than personal
  # file lists in the journal. Only this one stock DLL is excluded.
  return [ordered]@{
    profile=(Get-RtlsdrTreeDigest $Context.profile)
    stock=(Get-RtlsdrTreeDigest $Context.application 'plugins\rtlsdr_pi.dll')
    managed=(Get-RtlsdrTreeDigest $Context.managed)
    generation=(Get-RtlsdrTreeDigest $Generation)
  }
}
function Assert-RtlsdrPreserved($Expected,$Context,[string]$Generation) {
  $actual=Get-RtlsdrPreserved $Context $Generation
  foreach ($key in @('profile','stock','managed','generation')) {
    if ($Expected.$key -cnotmatch '^[a-f0-9]{64}$' -or $Expected.$key -cne $actual.$key) { throw ('Protected tree changed; retain RTL-SDR journal for review: '+$key) }
  }
}
function Assert-RtlsdrPrivateDirectory([string]$Directory,[string]$Sid) {
  $acl=Get-Acl -LiteralPath (Assert-LocalPath $Directory)
  if (-not $acl.AreAccessRulesProtected) { throw 'Hold journal must retain its private protected ACL.' }
  foreach ($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
    if ($rule.AccessControlType -ne 'Allow' -or $rule.IdentityReference.Value -notin @($Sid,'S-1-5-18','S-1-5-32-544')) { throw 'Hold journal ACL grants access outside its owner/system/administrators.' }
  }
}
function Read-RtlsdrPlan($Context,$Installed,[string]$Record,[string]$Digest) {
  $record=Assert-LocalPath $Record
  $directory=[IO.Path]::GetDirectoryName($record)
  if ([IO.Path]::GetFileName($record) -cne 'plan.json' -or
      [IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Context.workspace 'runs') -or
      [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-rtlsdr-hold-[a-f0-9]{8}$' -or
      $Digest -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $record) -cne $Digest) { throw 'Exact private RTL-SDR plan and its digest are required.' }
  Assert-RtlsdrPrivateDirectory $directory $Context.sid
  $plan=Read-Record $record
  $paths=Get-RtlsdrPaths $Context $Installed
  if ($plan.schema -ne 1 -or $plan.owner -cne 'SKAGER.ExactRtlsdrHold.1' -or $plan.sid -cne $Context.sid -or
      $plan.source -ine $paths.source -or $plan.root -ine $Installed.root -or $plan.profile -ine $Context.profile -or
      $plan.generation -cnotmatch '^[a-f0-9]{32}$') { throw 'RTL-SDR plan does not bind this stock/user/installation.' }
  $oldGeneration=Assert-LocalPath (Join-Path (Join-Path $Installed.root 'generations') $plan.generation)
  Assert-RtlsdrPreserved $plan.preserved $Context $oldGeneration
  Assert-RtlsdrBytes (Join-Path $directory 'backup.bin')
  return [pscustomobject]@{plan=$plan;directory=$directory;source=$paths.source;held=(Join-Path $directory 'held.bin');digest=$Digest}
}
function Write-RtlsdrPhase($State,[string]$Phase) {
  Write-Record (Join-Path $State.directory ($Phase+'.json')) ([ordered]@{schema=1;owner='SKAGER.ExactRtlsdrHold.1';planSha256=$State.digest;phase=$Phase;utc=[DateTime]::UtcNow.ToString('o')})
}
function Assert-RtlsdrPhase($State,[string]$Phase) {
  $record=Read-Record (Join-Path $State.directory ($Phase+'.json'))
  if ($record.schema -ne 1 -or $record.owner -cne 'SKAGER.ExactRtlsdrHold.1' -or $record.planSha256 -cne $State.digest -or $record.phase -cne $Phase) { throw 'Invalid RTL-SDR recovery phase.' }
}
function Move-RtlsdrExact([string]$Source,[string]$Destination,[string]$Acl) {
  $source=Assert-LocalPath $Source;$destination=Assert-LocalPath $Destination
  if ([IO.Path]::GetPathRoot($source) -ine [IO.Path]::GetPathRoot($destination) -or (Test-Path -LiteralPath $destination)) { throw 'Only a same-volume move to an absent exact destination is permitted.' }
  Assert-RtlsdrBytes $source
  Assert-PreparationAcl $Acl (Get-PreparationAccessAcl $source)
  # File.Move never overwrites or copies across volumes. The durable intent is
  # already published; a crash at either side is resolved by exact byte presence.
  [IO.File]::Move($source,$destination)
  Assert-RtlsdrBytes $destination
  Assert-PreparationAcl $Acl (Get-PreparationAccessAcl $destination)
  if (Test-Path -LiteralPath $source) { throw 'Original RTL-SDR path unexpectedly remains after move.' }
}
function Invoke-RtlsdrPhase([string]$Action,$Context,$Installed,[string]$Record,[string]$Digest) {
  $action=$Action.ToLowerInvariant()
  if ($action -notin @('inspect','hold','restore')) { throw 'Unknown RTL-SDR phase.' }
  Assert-RtlsdrCold $Context $Installed
  $paths=Get-RtlsdrPaths $Context $Installed
  if ($action -eq 'inspect') {
    if ($Record -or $Digest) { throw 'Inspect creates a fresh plan; it cannot consume one.' }
    Assert-RtlsdrBytes $paths.source
    if (Test-Path -LiteralPath (Join-Path $Installed.generation 'app\plugins\rtlsdr_pi.dll')) { throw 'Current generation contains an RTL-SDR entry DLL; this stock-only hold cannot exclude it.' }
    $preserved=Get-RtlsdrPreserved $Context $Installed.generation
    $acl=Get-PreparationAccessAcl $paths.source
    $directory=New-PreparationDirectory $Context 'rtlsdr-hold'
    Assert-RtlsdrPrivateDirectory $directory $Context.sid
    if ([IO.Path]::GetPathRoot($directory) -ine [IO.Path]::GetPathRoot($paths.source)) { throw 'Hold workspace must be on the stock installation volume.' }
    Copy-PreparationFile $paths.source (Join-Path $directory 'backup.bin') (Get-Digest $paths.source) 208384
    Assert-RtlsdrCold $Context $Installed
    Assert-RtlsdrPreserved $preserved $Context $Installed.generation
    Assert-PreparationAcl $acl (Get-PreparationAccessAcl $paths.source)
    $record=Join-Path $directory 'plan.json'
    Write-Record $record ([ordered]@{schema=1;owner='SKAGER.ExactRtlsdrHold.1';sid=$Context.sid;source=$paths.source;root=$Installed.root;profile=$Context.profile;generation=$Installed.state.current;acl=$acl;preserved=$preserved})
    return [pscustomobject]@{status='inspected';record=$record;recordSha256=(Get-Digest $record)}
  }
  $state=Read-RtlsdrPlan $Context $Installed $Record $Digest
  if ($action -eq 'hold') {
    if ($Installed.state.current -cne $state.plan.generation) { throw 'Selected generation changed since Inspect.' }
    foreach ($name in @('hold-intent.json','held.json','restore-intent.json','restored.json','held.bin')) {
      if (Test-Path -LiteralPath (Join-Path $state.directory $name)) { throw 'Hold has already been attempted; inspect the existing journal and use Restore.' }
    }
    Assert-RtlsdrBytes $state.source
    Assert-PreparationAcl $state.plan.acl (Get-PreparationAccessAcl $state.source)
    Assert-RtlsdrCold $Context $Installed
    Write-RtlsdrPhase $state 'hold-intent'
    Move-RtlsdrExact $state.source $state.held $state.plan.acl
    Assert-RtlsdrPreserved $state.plan.preserved $Context $Installed.generation
    Write-RtlsdrPhase $state 'held'
    return [pscustomobject]@{status='held';recordSha256=$Digest;restoreRequired=$true}
  }
  Assert-RtlsdrPhase $state 'hold-intent'
  if (Test-Path -LiteralPath (Join-Path $state.directory 'held.json')) { Assert-RtlsdrPhase $state 'held' }
  if (Test-Path -LiteralPath (Join-Path $state.directory 'restored.json')) { throw 'Restore already completed; retain its original receipt.' }
  $restoreIntent=Test-Path -LiteralPath (Join-Path $state.directory 'restore-intent.json')
  if ($restoreIntent) { Assert-RtlsdrPhase $state 'restore-intent' }
  $recoveryMode='moved-back'
  if (Test-Path -LiteralPath $state.source) {
    # A move may not have started after hold-intent was persisted, or may have
    # completed after restore-intent. Both require the exact original and no
    # parked file; a held receipt rules out the pre-move state. Never overwrite.
    if ((Test-Path -LiteralPath $state.held) -or (-not $restoreIntent -and (Test-Path -LiteralPath (Join-Path $state.directory 'held.json')))) { throw 'Restore destination is occupied; never overwrite or delete it.' }
    $recoveryMode=if($restoreIntent){'restore-move-already-completed'}else{'hold-move-not-started'}
    Assert-RtlsdrBytes $state.source
    Assert-PreparationAcl $state.plan.acl (Get-PreparationAccessAcl $state.source)
  } else {
    Assert-RtlsdrBytes $state.held
    Assert-RtlsdrCold $Context $Installed
    if (-not $restoreIntent) { Write-RtlsdrPhase $state 'restore-intent' }
    Move-RtlsdrExact $state.held $state.source $state.plan.acl
  }
  Assert-RtlsdrPreserved $state.plan.preserved $Context (Join-Path (Join-Path $Installed.root 'generations') $state.plan.generation)
  Write-RtlsdrPhase $state 'restored'
  return [pscustomobject]@{status='restored';recordSha256=$Digest;stockDllRestored=$true;recoveryMode=$recoveryMode;freshCommissioningInventoryRequired=$true}
}

function Invoke-RtlsdrLocked([string]$Action,$Context,$Installed,[string]$Record,[string]$Digest) {
  Assert-RtlsdrCold $Context $Installed
  # Cooperate with commissioning's existing profile lock and the installer's
  # existing lock. Never create/alter an installed lock or commissioning marker.
  if (-not ('OpenNavX.CommissioningProfileLock' -as [type])) { Add-Type -Path (Join-Path $PSScriptRoot 'CommissioningProfileLock.cs') }
  $profileLock=[OpenNavX.CommissioningProfileLock]::Acquire($Context.profile)
  $transaction=$null
  try {
    $lockPath=Assert-LocalPath (Join-Path $Installed.root 'transaction.lock')
    $transaction=[IO.File]::Open($lockPath,[IO.FileMode]::Open,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None)
    $installed=Read-InstalledIdentity $Installed.root
    return Invoke-RtlsdrPhase $Action $Context $installed $Record $Digest
  } finally { if ($transaction) { $transaction.Dispose() };$profileLock.Dispose() }
}

$context=Get-PreparationIdentityContext $Workspace
$installed=Get-Installed
Invoke-RtlsdrLocked $Action $context $installed $Record $ExpectedRecordSha256 | ConvertTo-Json -Depth 6
