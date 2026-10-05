# SCRUM-312: one existing launcher failure dialog only. Never starts/closes the app.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$ExplicitHelperDir,[string]$Request)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
function Assert-BootstrapFailureIdentity($Identity,[string]$Image,[string]$Hash,[string]$Sid,[int]$Session) {
  $ticks=([datetime]::Parse('2026-10-05T20:01:22.9834203Z',[Globalization.CultureInfo]::InvariantCulture,[Globalization.DateTimeStyles]::RoundtripKind)).ToUniversalTime().Ticks
  if($Identity.pid -ne 1228 -or $Identity.startedTicks -ne $ticks -or $Identity.exited -or
      $Identity.image -ine $Image -or $Identity.sha256 -cne $Hash -or $Hash -cnotmatch '^[a-f0-9]{64}$' -or
      $Identity.sid -cne $Sid -or $Sid -cnotmatch '^S-1-5-[0-9-]+$' -or $Session -le 0 -or $Identity.session -ne $Session) {
    throw 'Exact incident launcher PID, creation time, image, owner and session required.'
  }
}
function Assert-BootstrapFailureDialog($Dialog) {
  $text="SKAGER could not complete startup.`n`nAn update may need recovery. Open SKAGER Maintenance diagnostics before trying again.`n`nYou can try the Legacy or Safe Mode shortcut while checking the problem."
  if($Dialog.processId -ne 1228 -or -not $Dialog.visible -or $Dialog.className -cne '#32770' -or
      $Dialog.caption -cne 'SKAGER startup' -or $Dialog.text.Replace("`r`n","`n") -cne $text -or
      $Dialog.buttonCount -ne 1 -or $Dialog.buttonText -cne 'OK' -or -not $Dialog.buttonEnabled) {
    throw 'Exact visible startup failure dialog with its sole OK button required.'
  }
}
if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Native interactive Windows required.'}
$helpers=[IO.Path]::GetFullPath($ExplicitHelperDir)
# The operator supplies the already-qualified immutable helper directory.
. (Join-Path $helpers 'Common.ps1')
$helpers=Assert-LocalPath $helpers
$self=Assert-LocalPath $PSCommandPath;$common=Assert-LocalPath (Join-Path $helpers 'Common.ps1')
$workspace=Assert-LocalPath 'C:\XNav'
$intent=Join-Path $workspace 'runs/bootstrap-failure-1228-20261005T2001229834203Z.intent.json'
$sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
if(-not $Request) {
  if(Test-Path -LiteralPath $intent){throw 'Incident already has a close intent; inspect evidence, never retry automatically.'}
  $desktops=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" -OperationTimeoutSec 3 | Where-Object {
    (Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid -OperationTimeoutSec 3).Sid -ceq $sid
  })
  if($desktops.Count -ne 1 -or $desktops[0].SessionId -le 0){throw 'One same-user interactive desktop required.'}
  $directory=New-RunDirectory $workspace 'dismiss-bootstrap-failure'
  $requestPath=Join-Path $directory 'request.json';$resultPath=Join-Path $directory 'result.json'
  Write-Record $requestPath @{schema=1;owner='OpenNavX.BootstrapFailureDismissal.1';sid=$sid;session=[int]$desktops[0].SessionId;
    script=$self;scriptSha256=(Get-Digest $self);helperDir=$helpers;commonSha256=(Get-Digest $common);result=$resultPath}
  $name='OpenNavX-BootstrapFailure-'+[guid]::NewGuid().ToString('N');$task=$null
  $powerShell=Assert-LocalPath (Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe')
  $action=New-ScheduledTaskAction -Execute $powerShell -Argument ('-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$self+'" -ExplicitHelperDir "'+$helpers+'" -Request "'+$requestPath+'"')
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 2) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  try {
    $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
    Start-ScheduledTask -TaskName $name
    $deadline=[datetime]::UtcNow.AddSeconds(60)
    while(-not(Test-Path -LiteralPath $resultPath) -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 200}
    if(-not(Test-Path -LiteralPath $resultPath)){throw 'Interactive dismissal did not finish; inspect without retry or termination.'}
    $result=Read-Record $resultPath
    $result|ConvertTo-Json -Depth 6
    if($result.status -cne 'dismissed-known-startup-failure'){throw 'Dismissal requires inspection; startup success is not claimed.'}
  } finally {if($task){Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
  return
}
$requestPath=Assert-LocalPath $Request;$job=Read-Record $requestPath
$session=[Diagnostics.Process]::GetCurrentProcess().SessionId
if($job.schema -ne 1 -or $job.owner -cne 'OpenNavX.BootstrapFailureDismissal.1' -or $job.sid -cne $sid -or $job.session -ne $session -or
    $job.script -ine $self -or $job.scriptSha256 -cne (Get-Digest $self) -or $job.helperDir -ine $helpers -or $job.commonSha256 -cne (Get-Digest $common) -or
    [IO.Path]::GetFileName($requestPath) -cne 'request.json' -or [IO.Path]::GetDirectoryName([IO.Path]::GetDirectoryName($requestPath)) -ine (Join-Path $workspace 'runs') -or
    $job.result -ine (Join-Path ([IO.Path]::GetDirectoryName($requestPath)) 'result.json')){throw 'Interactive incident request changed.'}
$held=New-Object 'Collections.Generic.List[IDisposable]';$process=$null
$result=@{schema=1;status='refused';processId=1228;closeRequested=$false;exitCode=$null;applicationLaunched=$false;startupSuccess=$false}
try {
  $installed=Get-Installed
  if($installed.ownership.commit -cne 'c0d8d85fb602e86d40e2f3f1be32307919702408'){throw 'Only the selected incident product is allowed.'}
  $image=Assert-LocalPath (Join-Path $installed.generation 'app/skager-start.exe')
  $entries=@($installed.ownership.managedFiles|Where-Object{$_.path -ceq 'app/skager-start.exe'})
  if($entries.Count -ne 1 -or $entries[0].sha256 -cnotmatch '^[a-f0-9]{64}$'){throw 'Exact managed launcher hash required.'}
  $hash=$entries[0].sha256
  $stateHash=Get-Digest (Join-Path $installed.root 'state.json');$ownershipHash=Get-Digest (Join-Path $installed.generation 'ownership.json')
  foreach($path in @((Join-Path $installed.root 'state.json'),(Join-Path $installed.generation 'ownership.json'),$image,$self,$common)){
    $held.Add([IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read))
  }
  if((Get-Digest $image) -cne $hash -or (Get-Digest (Join-Path $installed.root 'state.json')) -cne $stateHash -or
      (Get-Digest (Join-Path $installed.generation 'ownership.json')) -cne $ownershipHash){throw 'Installed incident identity changed.'}
  $process=Get-Process -Id 1228 -ErrorAction Stop;$null=$process.get_Handle()
  function Confirm-IncidentProcess {
    $process.Refresh()
    $native=Get-CimInstance Win32_Process -Filter 'ProcessId=1228' -OperationTimeoutSec 3
    if(-not $native){throw 'Incident launcher exited.'}
    $owner=Invoke-CimMethod -InputObject $native -MethodName GetOwnerSid -OperationTimeoutSec 3
    if($owner.ReturnValue -ne 0 -or (Assert-LocalPath $native.ExecutablePath) -ine $image -or $native.SessionId -ne $session){throw 'Actual incident owner/image/session changed.'}
    Assert-BootstrapFailureIdentity ([pscustomobject]@{pid=$process.Id;startedTicks=$process.StartTime.ToUniversalTime().Ticks;
      exited=$process.HasExited;image=(Assert-LocalPath $process.Path);sha256=(Get-Digest $image);sid=$owner.Sid;session=$process.SessionId}) $image $hash $sid $session
  }
  Confirm-IncidentProcess
  Add-Type -TypeDefinition @'
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
namespace OpenNavX {
 public static class BootstrapFailureDialog {
  public sealed class Info { public long handle; public int processId,buttonCount; public bool visible,buttonEnabled; public string className,caption,text,buttonText; }
  delegate bool Callback(IntPtr h,IntPtr p);
  [DllImport("user32.dll")] static extern bool EnumWindows(Callback f,IntPtr p);
  [DllImport("user32.dll")] static extern bool EnumChildWindows(IntPtr h,Callback f,IntPtr p);
  [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h,out uint pid);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] static extern bool IsWindowEnabled(IntPtr h);
  [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetWindowTextW(IntPtr h,StringBuilder b,int n);
  [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetClassNameW(IntPtr h,StringBuilder b,int n);
  [DllImport("user32.dll",SetLastError=true)] static extern IntPtr SendMessageTimeoutW(IntPtr h,uint m,UIntPtr w,IntPtr l,uint flags,uint timeout,out UIntPtr result);
  static string Text(IntPtr h) {var b=new StringBuilder(2048);GetWindowTextW(h,b,b.Capacity);return b.ToString();}
  static string Class(IntPtr h) {var b=new StringBuilder(128);GetClassNameW(h,b,b.Capacity);return b.ToString();}
  public static Info Inspect() {
   var windows=new List<IntPtr>();
   EnumWindows((h,p)=>{uint id;GetWindowThreadProcessId(h,out id);if(id==1228 && IsWindowVisible(h))windows.Add(h);return true;},IntPtr.Zero);
   if(windows.Count!=1)throw new InvalidOperationException("One exact visible incident window required.");
   var window=windows[0];var info=new Info{handle=window.ToInt64(),processId=1228,visible=IsWindowVisible(window),className=Class(window),caption=Text(window)};
   var texts=new List<string>();bool foreign=false;
   EnumChildWindows(window,(h,p)=>{uint id;GetWindowThreadProcessId(h,out id);if(id!=1228){foreign=true;return false;}
    string kind=Class(h),text=Text(h);if(kind=="Button"){info.buttonCount++;info.buttonText=text;info.buttonEnabled=IsWindowEnabled(h)&&IsWindowVisible(h);}
    else if(kind=="Static"){if(text.Length>0){if(!IsWindowVisible(h)){foreign=true;return false;}texts.Add(text);}}
    else {foreign=true;return false;}return true;},IntPtr.Zero);
   if(foreign || texts.Count!=1)throw new InvalidOperationException("One exact owned static failure text required.");info.text=texts[0];return info;
  }
  public static void CloseOnce(long expected) {
   var i=Inspect();const string text="SKAGER could not complete startup.\n\nAn update may need recovery. Open SKAGER Maintenance diagnostics before trying again.\n\nYou can try the Legacy or Safe Mode shortcut while checking the problem.";
   if(i.handle!=expected || i.className!="#32770" || i.caption!="SKAGER startup" || i.text.Replace("\r\n","\n")!=text || i.buttonCount!=1 || i.buttonText!="OK" || !i.buttonEnabled)throw new InvalidOperationException("Incident dialog changed before normal close.");
   UIntPtr result;if(SendMessageTimeoutW(new IntPtr(expected),0x10,UIntPtr.Zero,IntPtr.Zero,0x2,2000,out result)==IntPtr.Zero)throw new InvalidOperationException("Normal WM_CLOSE not confirmed; no retry.");
  }
 }
}
'@
  $dialog=[OpenNavX.BootstrapFailureDialog]::Inspect();Assert-BootstrapFailureDialog $dialog
  # Exclusive immutable intent also prevents concurrent dispatches from closing twice.
  Write-Record $intent @{schema=1;owner='OpenNavX.BootstrapFailureDismissal.1';processId=1228;startedUtc='2026-10-05T20:01:22.9834203Z';
    executableSha256=$hash;sid=$sid;session=$session;requestSha256=(Get-Digest $requestPath);dialogHandle=$dialog.handle;action='single-normal-WM_CLOSE';expectedExitCode=1}
  Confirm-IncidentProcess
  $result.closeRequested=$true
  [OpenNavX.BootstrapFailureDialog]::CloseOnce($dialog.handle)
  if(-not $process.WaitForExit(10000)){throw 'Launcher did not exit normally within the bound; no retry or force termination.'}
  $result.exitCode=$process.get_ExitCode()
  if($result.exitCode -isnot [int] -or $result.exitCode -ne 1){throw 'Launcher exit differs from the known startup failure.'}
  $result.status='dismissed-known-startup-failure'
} catch {$result.failure=$_.Exception.Message} finally {
  if($process){$process.Dispose()};foreach($file in $held){$file.Dispose()}
  Write-Record $job.result $result
}
if($result.status -cne 'dismissed-known-startup-failure'){throw 'Dismissal requires inspection; see its result. No retry performed.'}
