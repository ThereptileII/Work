# Explicit bootstrap/no-config boat launch. No trust provisioning, profile edit,
# update offer, retry, capture, close or force termination is performed here.
. (Join-Path $PSScriptRoot 'StartupLog.ps1')

function Get-StartupLauncherContext($Installed) {
  if (-not $Installed.ownership.PSObject.Properties['updateStartupHealth'] -or
      $Installed.ownership.updateStartupHealth -isnot [int] -or $Installed.ownership.updateStartupHealth -ne 1) { throw 'Authenticated startup health version 1 is required.' }
  if (Test-Path -LiteralPath (Join-Path $Installed.root 'update-pending.json')) { throw 'Pending update requires separate recovery; this boat helper cannot launch it.' }
  if (Test-Path -LiteralPath (Join-Path $Installed.generation 'app/update-trust.json')) { throw 'Configured update trust is outside bootstrap-only boat qualification.' }
  $required=@('app/opencpn.exe','app/skager-start.exe','UpdateTransaction.ps1','UpdateSupervisor.ps1','Lifecycle.ps1')
  $files=@{}
  foreach ($relative in $required) {
    $entries=@($Installed.ownership.managedFiles | Where-Object { $_.path -ceq $relative })
    $path=Assert-LocalPath (Join-Path $Installed.generation $relative)
    if ($entries.Count -ne 1 -or $entries[0].sha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $path) -cne $entries[0].sha256) { throw ('Startup launcher ownership mismatch: '+$relative) }
    $files[$relative]=[pscustomobject]@{path=$path;sha256=$entries[0].sha256}
  }
  $identity=[pscustomobject]@{generation=$Installed.state.current;commit=$Installed.ownership.commit;packageSha256=$Installed.ownership.packageSha256;executableSha256=$files['app/opencpn.exe'].sha256}
  if ($identity.generation -cnotmatch '^[a-f0-9]{32}$' -or $identity.commit -cnotmatch '^[a-f0-9]{40}$' -or $identity.packageSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Invalid installed startup identity.' }
  return [pscustomobject]@{files=$files;identity=$identity;stateSha256=(Get-Digest (Join-Path $Installed.root 'state.json'));ownershipSha256=(Get-Digest (Join-Path $Installed.generation 'ownership.json'))}
}

function Assert-StartupObservedIdentity($Observation,[string]$Image,[string]$Hash,[string]$Sid,[int]$Session,[long]$NotBefore) {
  if (-not $Observation -or $Observation.pid -le 0 -or $Observation.startedTicks -lt $NotBefore -or
      $Observation.sid -cne $Sid -or $Observation.session -ne $Session -or $Observation.image -ine $Image -or $Observation.sha256 -cne $Hash) { throw 'Actual startup process identity does not match the audited launch.' }
}
function Assert-StartupParentChain($App,$Launcher,$Bridge,[bool]$HadReceipt) {
  $parent=$Launcher
  if (-not $HadReceipt) {
    if (-not $Bridge -or $Bridge.parentPid -ne $Launcher.pid -or $Bridge.startedTicks -lt $Launcher.startedTicks -or
        ($Launcher.exitTicks -and $Bridge.startedTicks -gt $Launcher.exitTicks)) { throw 'Bootstrap supervisor ancestry was missed or changed.' }
    $parent=$Bridge
  } elseif ($Bridge) { throw 'Unexpected supervisor on the no-config known-good path.' }
  if ($App.parentPid -ne $parent.pid -or $App.startedTicks -lt $parent.startedTicks -or
      ($parent.exitTicks -and $App.startedTicks -gt $parent.exitTicks)) { throw 'Actual application is not the exact retained launcher/supervisor child.' }
}
function Assert-StartupCommandArguments([string[]]$Arguments,[string]$Image,[string]$Root,[string]$Supervisor,[bool]$Bridge,[string[]]$CommandImages=@()) {
  $permitted=@($Image)
  if($Bridge -and $CommandImages.Count){$permitted=@($CommandImages)}
  if ($Arguments.Count -lt 1 -or $permitted -inotcontains $Arguments[0].Replace('/','\')) { throw 'Observed startup command image changed.' }
  if (-not $Bridge) {
    if ($Arguments.Count -ne 2 -or $Arguments[1] -cne '--xnav') { throw 'Unexpected actual application startup arguments.' }
    return
  }
  if ($Arguments.Count -ne 11 -or $Arguments[1] -cne '-NoProfile' -or $Arguments[2] -cne '-NonInteractive' -or
      $Arguments[3] -cne '-ExecutionPolicy' -or $Arguments[4] -cne 'Bypass' -or $Arguments[5] -cne '-File' -or
      $Arguments[6].Replace('/','\') -ine $Supervisor -or $Arguments[7] -cne '-InstallationRoot' -or
      $Arguments[8].Replace('/','\') -ine $Root -or $Arguments[9] -cne '-Action' -or $Arguments[10] -cne 'QualifyCurrent') { throw 'Unexpected startup supervisor command.' }
}
function Initialize-StartupArguments {
  if ('OpenNavX.StartupArguments' -as [type]) { return }
  Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
namespace OpenNavX {
 public static class StartupArguments {
  [DllImport("shell32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern IntPtr CommandLineToArgvW(string command,out int count);
  [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr memory);
  public static string[] Parse(string command) {
   if(String.IsNullOrEmpty(command)||command.Length>32768)throw new ArgumentException("Invalid process command length.");
   int count; IntPtr data=CommandLineToArgvW(command,out count);
   if(data==IntPtr.Zero)throw new System.ComponentModel.Win32Exception();
   try { if(count<1||count>16)throw new ArgumentException("Unexpected argument count.");
    var result=new string[count];for(int i=0;i<count;i++)result[i]=Marshal.PtrToStringUni(Marshal.ReadIntPtr(data,i*IntPtr.Size));return result;
   } finally {LocalFree(data);}
  }
 }
}
'@
}
function Get-StartupObservedProcess($Native,[string]$Sid,[int]$Session) {
  $process=Get-Process -Id $Native.ProcessId -ErrorAction Stop
  try {
    $null=$process.get_Handle() # Retain this process object through final binding.
    $ticks=$process.StartTime.ToUniversalTime().Ticks
    $created=([datetime]$Native.CreationDate).ToUniversalTime().Ticks
    # CIM creation time is microsecond precision; the retained native handle
    # supplies the full FILETIME identity before and after the owner query.
    if ($ticks-($ticks%10) -ne $created-($created%10) -or $process.HasExited) { throw 'Observed process creation identity changed.' }
    $owner=Invoke-CimMethod -InputObject $Native -MethodName GetOwnerSid -OperationTimeoutSec 3
    $image=Assert-LocalPath $process.Path
    if ($owner.ReturnValue -ne 0 -or $owner.Sid -cne $Sid -or $process.SessionId -ne $Session -or
        (Assert-LocalPath $Native.ExecutablePath) -ine $image -or $Native.SessionId -ne $Session) { throw 'Observed process account/session/image changed.' }
    $hash=Get-Digest $image
    $arguments=[OpenNavX.StartupArguments]::Parse($Native.CommandLine)
    $process.Refresh()
    if ($process.HasExited -or $process.StartTime.ToUniversalTime().Ticks -ne $ticks -or (Assert-LocalPath $process.Path) -ine $image) { throw 'Observed process exited or changed during verification.' }
    return [pscustomobject]@{process=$process;pid=$process.Id;parentPid=[int]$Native.ParentProcessId;startedTicks=$ticks;exitTicks=0L;sid=$owner.Sid;session=$process.SessionId;image=$image;sha256=$hash;arguments=$arguments}
  } catch { $process.Dispose(); throw }
}

function Invoke-StartupLauncher($Job) {
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Native interactive Windows is required.' }
  if ($Job.action -cne 'LaunchStartup' -or $Job.mode -cne '--xnav' -or
      $Job.PSObject.Properties['restartSessionRecord'] -or $Job.PSObject.Properties['restartSessionSha256']) { throw 'Bootstrap-only launch cannot combine modes or restart commissioning.' }
  $workspace=Assert-LocalPath $Job.workspace
  $target=Join-Path $workspace 'boat-target.json';$targetHash=Get-Digest $target
  $config=Get-Target $workspace;$installed=Get-Installed
  if ($installed.executable -ine $Job.executable -or (Get-Digest $installed.executable) -cne $Job.executableSha256) { throw 'Installed application changed after dispatch.' }
  # Same complete audit used by the existing direct path, repeated inside the
  # interactive session immediately before process creation.
  $environment=Assert-ReadOnlyAudit $config $installed $workspace
  $context=Get-StartupLauncherContext $installed
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $session=[Diagnostics.Process]::GetCurrentProcess().SessionId
  if ((Assert-LocalPath $environment.workingDirectory) -ine [IO.Path]::GetDirectoryName($installed.executable)) { throw 'Audited application directory changed.' }
  $held=New-Object 'Collections.Generic.List[IDisposable]'
  $launcher=$null;$bridge=$null;$app=$null
  try {
    # Keep selected state, ownership, target and executable/helper bytes stable.
    # No lifecycle lock is held: the real launcher/supervisor owns that lock.
    $paths=@((Join-Path $installed.root 'state.json'),(Join-Path $installed.generation 'ownership.json'),$target)+@($context.files.Values | ForEach-Object {$_.path})
    foreach ($path in $paths) { $held.Add([IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)) }
    $again=Get-StartupLauncherContext (Get-Installed)
    if ($again.stateSha256 -cne $context.stateSha256 -or $again.ownershipSha256 -cne $context.ownershipSha256 -or (Get-Digest $target) -cne $targetHash) { throw 'Audited launch context changed before creation.' }
    $receipt=Assert-LocalPath (Join-Path (Join-Path $installed.root 'known-good') ($context.identity.generation+'.receipt'))
    # Import only the hash-verified installed receipt verifier, not installer entrypoints.
    . ($context.files['UpdateTransaction.ps1'].path)
    $hadReceipt=Test-Path -LiteralPath $receipt
    if ($hadReceipt) { Assert-UpdateKnownGoodReceipt $receipt $context.identity }
    Initialize-StartupArguments
    $log=Assert-LocalPath (Join-Path $config.profileDirectory 'opencpn.log')
    $before=Read-StartupLogBytes $log
    if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'An OpenCPN instance is already running. Close it normally first.' }
    if ((Get-Digest $target) -cne $targetHash) { throw 'Audited target changed before launcher creation.' }
    $start=New-Object Diagnostics.ProcessStartInfo
    $start.FileName=$context.files['app/skager-start.exe'].path;$start.Arguments='--xnav';$start.UseShellExecute=$false
    $start.WorkingDirectory=$environment.workingDirectory;$start.EnvironmentVariables['PATH']=$environment.path
    $started=[Diagnostics.Process]::Start($start);$null=$started.get_Handle()
    $launcher=[pscustomobject]@{process=$started;pid=$started.Id;startedTicks=$started.StartTime.ToUniversalTime().Ticks;exitTicks=0L;sid=$sid;session=$session;image=$start.FileName;sha256=$context.files['app/skager-start.exe'].sha256}
    $deadline=[DateTime]::UtcNow.AddSeconds(690) # Receiver phases plus bounded recovery; never auto-acknowledge.
    $systems=@([Environment]::GetFolderPath('System'),[Environment]::GetFolderPath('SystemX86')) | Select-Object -Unique
    $engines=@{};foreach ($system in $systems) {if($system){$path=Assert-LocalPath (Join-Path $system 'WindowsPowerShell/v1.0/powershell.exe');$engines[$path]=Get-Digest $path}}
    while (-not $app -and [DateTime]::UtcNow -lt $deadline) {
      if (-not $hadReceipt -and -not $bridge) {
        $parents=@(Get-CimInstance Win32_Process -Filter ('ParentProcessId='+$launcher.pid) -OperationTimeoutSec 3)
        if ($parents.Count -gt 1) { throw 'Ambiguous launcher children; bootstrap ancestry refused.' }
        if ($parents.Count -eq 1) {
          $bridge=Get-StartupObservedProcess $parents[0] $sid $session
          if (-not $engines.ContainsKey($bridge.image)) { throw 'Bootstrap did not launch verified system PowerShell.' }
          Assert-StartupObservedIdentity $bridge $bridge.image $engines[$bridge.image] $sid $session $launcher.startedTicks
          Assert-StartupCommandArguments $bridge.arguments $bridge.image $installed.root $context.files['UpdateSupervisor.ps1'].path $true @($engines.Keys)
        }
      }
      $children=@(Get-CimInstance Win32_Process -Filter "Name='opencpn.exe'" -OperationTimeoutSec 3)
      if ($children.Count -gt 1) { throw 'Ambiguous application children; startup refused.' }
      if ($children.Count -eq 1) { $app=Get-StartupObservedProcess $children[0] $sid $session; break }
      Start-Sleep -Milliseconds 100
    }
    if (-not $app) { throw 'No exact application child observed; no automatic retry or termination.' }
    Assert-StartupObservedIdentity $app $installed.executable $Job.executableSha256 $sid $session $launcher.startedTicks
    Assert-StartupCommandArguments $app.arguments $installed.executable '' '' $false
    if ($launcher.process.HasExited) { $launcher.exitTicks=$launcher.process.ExitTime.ToUniversalTime().Ticks }
    if ($bridge -and $bridge.process.HasExited) { $bridge.exitTicks=$bridge.process.ExitTime.ToUniversalTime().Ticks }
    Assert-StartupParentChain $app $launcher $bridge $hadReceipt
    $remaining=[int][Math]::Max(1,($deadline-[DateTime]::UtcNow).TotalMilliseconds)
    if (-not $launcher.process.WaitForExit($remaining) -or $launcher.process.get_ExitCode() -ne 0) { throw 'Installed launcher did not complete successfully; inspect desktop without retry.' }
    if ($bridge -and (-not $bridge.process.HasExited -or $bridge.process.get_ExitCode() -ne 0)) { throw 'Bootstrap supervisor did not finish successfully.' }
    Assert-UpdateKnownGoodReceipt $receipt $context.identity
    do {
      $app.process.Refresh()
      if ($app.process.HasExited) { throw 'Audited application exited during startup.' }
      if ($app.process.MainWindowHandle -and $app.process.Responding -and (Test-StartupInitializedSince $before (Read-StartupLogBytes $log))) { break }
      Start-Sleep -Milliseconds 100
    } while ([DateTime]::UtcNow -lt $deadline)
    if (-not $app.process.MainWindowHandle -or -not $app.process.Responding -or -not (Test-StartupInitializedSince $before (Read-StartupLogBytes $log))) { throw 'Actual application readiness was not established.' }
    if ($app.process.StartTime.ToUniversalTime().Ticks -ne $app.startedTicks -or (Get-Digest $installed.executable) -cne $Job.executableSha256) { throw 'Actual application identity changed after startup.' }
    $last=Get-StartupLauncherContext (Get-Installed)
    if ($last.stateSha256 -cne $context.stateSha256 -or $last.ownershipSha256 -cne $context.ownershipSha256 -or (Get-Digest $target) -cne $targetHash) { throw 'Selected audited generation changed during startup.' }
    return @{status='passed';action='LaunchStartup';scope='bootstrap/no-config only; signed offers and rollback not qualified';pid=$app.pid;processStartedUtc=$app.process.StartTime.ToUniversalTime().ToString('o');sid=$sid;sessionId=$session;mode='--xnav';generation=$context.identity.generation;buildCommit=$context.identity.commit;executableSha256=$Job.executableSha256;targetSha256=$targetHash;commissioning=$config.readOnlyAudit.commissioning;launcherPid=$launcher.pid;launcherStartedUtc=$launcher.process.StartTime.ToUniversalTime().ToString('o');startupReceiptSha256=(Get-Digest $receipt);bootstrapped=(-not $hadReceipt);startupMarkerPresent=$true;responding=$true;utc=[DateTime]::UtcNow.ToString('o')}
  } finally {
    foreach ($entry in @($app,$bridge,$launcher)) { if ($entry) { $entry.process.Dispose() } }
    foreach ($handle in $held) { $handle.Dispose() }
  }
}
