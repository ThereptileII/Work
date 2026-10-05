# Read-only, exact-process observation. No focus, input, visibility, close or kill APIs.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string]$ReviewedHelperDirectory,
  [string]$Workspace='C:\XNav',
  [string]$Request,
  [string]$ExpectedSelfSha256,
  [string]$ExpectedCommonSha256,
  [string]$ExpectedRequestSha256,
  [switch]$TopWindowsOnly
)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
function Get-InspectionHash([string]$Name) {
  $hash=[Security.Cryptography.SHA256]::Create();$stream=[IO.File]::OpenRead($Name)
  try {return ([BitConverter]::ToString($hash.ComputeHash($stream))).Replace('-','').ToLowerInvariant()}
  finally {$stream.Dispose();$hash.Dispose()}
}
$self=[IO.Path]::GetFullPath($PSCommandPath)
$common=[IO.Path]::Combine($ReviewedHelperDirectory,'Common.ps1')
if ($Request) {
  foreach($pin in @($ExpectedSelfSha256,$ExpectedCommonSha256,$ExpectedRequestSha256)) {if($pin -cnotmatch '^[a-f0-9]{64}$'){throw 'Missing diagnostic dispatch hash.'}}
  if((Get-InspectionHash $self) -cne $ExpectedSelfSha256 -or (Get-InspectionHash $common) -cne $ExpectedCommonSha256 -or (Get-InspectionHash $Request) -cne $ExpectedRequestSha256){throw 'Diagnostic dispatch bytes changed.'}
}
. $common
$self=Assert-LocalPath $self;$common=Assert-LocalPath $common
$Workspace=Assert-LocalPath $Workspace
$held=New-Object 'Collections.Generic.List[IDisposable]'
try {
  foreach($path in @($self,$common)) {$held.Add([IO.File]::Open($path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read))}
  $selfHash=Get-Digest $self;$commonHash=Get-Digest $common
  if($Request -and ($selfHash -cne $ExpectedSelfSha256 -or $commonHash -cne $ExpectedCommonSha256)){throw 'Diagnostic helper changed before custody.'}
  if(-not $Request) {
    if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Native Windows required.'}
    $installed=Get-Installed
    if($installed.ownership.commit -cne 'c0d8d85fb602e86d40e2f3f1be32307919702408'){throw 'Unexpected installed candidate.'}
    $appHash='8ed5cc1fad45bfc9cda13f98ec0b55cf18270b45a348764673aa7702c12c192c'
    if((Get-Digest $installed.executable) -cne $appHash){throw 'Candidate bytes changed.'}
    $launcher=Assert-LocalPath (Join-Path $installed.generation 'app/skager-start.exe')
    $entry=@($installed.ownership.managedFiles|Where-Object {$_.path -ceq 'app/skager-start.exe'})
    if($entry.Count -ne 1 -or (Get-Digest $launcher) -cne $entry[0].sha256){throw 'Launcher ownership mismatch.'}
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    $explorer=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" -OperationTimeoutSec 3|Where-Object {(Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid -OperationTimeoutSec 3).Sid -ceq $sid})
    if($explorer.Count -ne 1 -or $explorer[0].SessionId -le 0){throw 'One interactive desktop for this account required.'}
    $directory=New-RunDirectory $Workspace 'inspect-startup-windows'
    $requestPath=Join-Path $directory 'request.json';$resultPath=Join-Path $directory 'result.json'
    Write-Record $requestPath @{schema=1;owner='SKAGER.StartupWindowObservation.1';sid=$sid;session=[int]$explorer[0].SessionId;resultPath=$resultPath;generation=$installed.state.current;topWindowsOnly=[bool]$TopWindowsOnly;targets=@(
      @{pid=7196;startedUtc='2026-10-05T20:01:57.2931063Z';image=$installed.executable;sha256=$appHash},
      @{pid=1228;startedUtc='2026-10-05T20:01:22.9834203Z';image=$launcher;sha256=$entry[0].sha256})}
    $held.Add([IO.File]::Open($requestPath,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read))
    $requestHash=Get-Digest $requestPath
    $arguments='-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "'+$self+'" -ReviewedHelperDirectory "'+(Assert-LocalPath $ReviewedHelperDirectory)+'" -Workspace "'+$Workspace+'" -Request "'+$requestPath+'" -ExpectedSelfSha256 '+$selfHash+' -ExpectedCommonSha256 '+$commonHash+' -ExpectedRequestSha256 '+$requestHash
    $name='OpenNavX-StartupObservation-'+[guid]::NewGuid().ToString('N')
    $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument $arguments
    $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
    $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Seconds 90) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
    $task=$null
    try {
      $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
      Start-ScheduledTask -TaskName $name
      $deadline=[DateTime]::UtcNow.AddSeconds(90)
      while(-not [IO.File]::Exists($resultPath) -and [DateTime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 200}
      if(-not [IO.File]::Exists($resultPath)){throw ('Observation timed out; no application action performed. Evidence directory: '+$directory)}
      Read-Record $resultPath|ConvertTo-Json -Depth 16
    } finally {if($task){Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
    return
  }
  $Request=Assert-LocalPath $Request
  $held.Add([IO.File]::Open($Request,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read))
  if((Get-Digest $Request) -cne $ExpectedRequestSha256){throw 'Diagnostic request changed before custody.'}
  $job=Read-Record $Request
  if($job.schema -ne 1 -or $job.owner -cne 'SKAGER.StartupWindowObservation.1' -or @($job.targets).Count -ne 2 -or $job.topWindowsOnly -isnot [bool]){throw 'Invalid diagnostic request.'}
  $resultPath=Assert-LocalPath $job.resultPath
  if([IO.Path]::GetDirectoryName($resultPath) -ine [IO.Path]::GetDirectoryName($Request)){throw 'Diagnostic output directory changed.'}
  $result=@{schema=1;status='failed';scope='read-only exact startup processes';selfSha256=$selfHash;commonSha256=$commonHash;requestSha256=$ExpectedRequestSha256;utc=[DateTime]::UtcNow.ToString('o')}
  $processes=New-Object 'Collections.Generic.List[Diagnostics.Process]'
  try {
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;$session=[Diagnostics.Process]::GetCurrentProcess().SessionId
    if($sid -cne $job.sid -or $session -ne $job.session -or $session -le 0){throw 'Diagnostic is not in the dispatched interactive account/session.'}
    $installed=Get-Installed
    if($installed.state.current -cne $job.generation -or $installed.ownership.commit -cne 'c0d8d85fb602e86d40e2f3f1be32307919702408'){throw 'Selected installation changed.'}
    foreach($target in $job.targets) {
      $process=Get-Process -Id $target.pid -ErrorAction Stop;$processes.Add($process);$null=$process.get_Handle()
      $expectedTicks=[DateTimeOffset]::Parse($target.startedUtc).UtcDateTime.Ticks
      $native=Get-CimInstance Win32_Process -Filter ('ProcessId='+[int]$target.pid) -OperationTimeoutSec 3
      $owner=Invoke-CimMethod -InputObject $native -MethodName GetOwnerSid -OperationTimeoutSec 3
      if($process.HasExited -or $process.StartTime.ToUniversalTime().Ticks -ne $expectedTicks -or $process.SessionId -ne $session -or $owner.ReturnValue -ne 0 -or $owner.Sid -cne $sid -or (Assert-LocalPath $process.Path) -ine $target.image -or (Get-Digest $target.image) -cne $target.sha256){throw 'Exact observed process identity refused.'}
    }
    Add-Type -TypeDefinition @'
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
namespace SkagerObservation {
 public class Window {
  public long hwnd; public uint pid; public string title, className; public bool visible,enabled,child,textTimedOut; public int left,top,right,bottom;
 }
 public static class Windows {
  delegate bool EnumProc(IntPtr hwnd,IntPtr value);
  [StructLayout(LayoutKind.Sequential)] struct Rect {public int left,top,right,bottom;}
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc callback,IntPtr value);
  [DllImport("user32.dll")] static extern bool EnumChildWindows(IntPtr parent,EnumProc callback,IntPtr value);
  [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr hwnd,out uint pid);
  [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetClassName(IntPtr hwnd,StringBuilder text,int count);
  [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetWindowText(IntPtr hwnd,StringBuilder text,int count);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr hwnd);
  [DllImport("user32.dll")] static extern bool IsWindowEnabled(IntPtr hwnd);
  [DllImport("user32.dll")] static extern bool GetWindowRect(IntPtr hwnd,out Rect rect);
  [DllImport("user32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern IntPtr SendMessageTimeout(IntPtr hwnd,uint message,UIntPtr count,StringBuilder text,uint flags,uint timeout,out UIntPtr result);
  public static Window[] Inspect(uint first,uint second,bool topOnly) {
   var found=new List<Window>();var clock=Stopwatch.StartNew();bool truncated=false;
   EnumProc collect=(hwnd,unused)=> {
    if(clock.ElapsedMilliseconds>15000 || found.Count>=256){truncated=true;return false;}
    uint pid;GetWindowThreadProcessId(hwnd,out pid);if(pid!=first && pid!=second)return true;
    var cls=new StringBuilder(128);GetClassName(hwnd,cls,cls.Capacity);
    // All descendant labels are limited to native static/button controls.
    var text=new StringBuilder(513);bool timeout=false;
    if(cls.ToString()=="Static" || cls.ToString()=="Button") {
     UIntPtr ignored;timeout=SendMessageTimeout(hwnd,13,new UIntPtr(513),text,3,50,out ignored)==IntPtr.Zero;
    }
    Rect r;GetWindowRect(hwnd,out r);
    found.Add(new Window{hwnd=hwnd.ToInt64(),pid=pid,className=cls.ToString(),title=text.ToString(),child=true,textTimedOut=timeout,visible=IsWindowVisible(hwnd),enabled=IsWindowEnabled(hwnd),left=r.left,top=r.top,right=r.right,bottom=r.bottom});return true;
   };
   EnumWindows((hwnd,unused)=> {
    if(clock.ElapsedMilliseconds>15000 || found.Count>=256){truncated=true;return false;}
    uint pid;GetWindowThreadProcessId(hwnd,out pid);if(pid!=first && pid!=second)return true;
    var cls=new StringBuilder(128);GetClassName(hwnd,cls,cls.Capacity);var text=new StringBuilder(513);
    if(cls.ToString()!="Edit" && !cls.ToString().StartsWith("RichEdit",StringComparison.OrdinalIgnoreCase))GetWindowText(hwnd,text,text.Capacity);
    Rect r;GetWindowRect(hwnd,out r);
    found.Add(new Window{hwnd=hwnd.ToInt64(),pid=pid,className=cls.ToString(),title=text.ToString(),visible=IsWindowVisible(hwnd),enabled=IsWindowEnabled(hwnd),left=r.left,top=r.top,right=r.right,bottom=r.bottom});
    if(!topOnly)EnumChildWindows(hwnd,collect,IntPtr.Zero);return !truncated;
   },IntPtr.Zero);
   if(truncated)throw new InvalidOperationException("Window observation limit reached; incomplete data refused.");
   return found.ToArray();
  }
 }
}
'@
    $windows=[SkagerObservation.Windows]::Inspect([uint32]$job.targets[0].pid,[uint32]$job.targets[1].pid,$job.topWindowsOnly)
    foreach($process in $processes){$process.Refresh();$target=@($job.targets|Where-Object {$_.pid -eq $process.Id})[0];if($process.HasExited -or $process.StartTime.ToUniversalTime().Ticks -ne [DateTimeOffset]::Parse($target.startedUtc).UtcDateTime.Ticks -or (Assert-LocalPath $process.Path) -ine $target.image -or (Get-Digest $target.image) -cne $target.sha256){throw 'Observed process changed during inspection.'}}
    $result.status='passed';$result.sid=$sid;$result.session=$session;$result.targets=$job.targets;$result.windows=@($windows)
  } catch {$result.error=$_.Exception.Message}
  finally {foreach($process in $processes){$process.Dispose()}}
  Write-Record $resultPath $result
} finally {foreach($handle in $held){$handle.Dispose()}}
