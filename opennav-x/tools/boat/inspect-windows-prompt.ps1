# Read-only inspection of the Windows Security prompt in the current user's
# interactive desktop. Captures only that window; never clicks or grants access.
[CmdletBinding()]
param([string]$Workspace='C:\XNav',[string]$OutputDirectory='', [switch]$Interactive,
      [int]$ApplicationProcessId=0)
. (Join-Path $PSScriptRoot 'Preparation.ps1')
if (-not $Interactive) {
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $explorers=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {
    (Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -eq $sid
  })
  if ($explorers.Count -ne 1) { throw 'One interactive desktop for this account required.' }
  $directory=New-PreparationDirectory ([pscustomobject]@{workspace=$Workspace;sid=$sid}) 'windows-prompt'
  $name='OpenNavX-Prompt-Inspect-'+[guid]::NewGuid().ToString('N')
  $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument ('-NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File "'+$PSCommandPath+'" -Interactive -OutputDirectory "'+$directory+'" -ApplicationProcessId '+$ApplicationProcessId)
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 2) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  $task=$null
  try {
    $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
    Start-ScheduledTask -TaskName $name
    $result=Join-Path $directory 'inspection.json';$deadline=[datetime]::UtcNow.AddSeconds(55)
    while (-not [IO.File]::Exists($result) -and [datetime]::UtcNow -lt $deadline) {Start-Sleep -Milliseconds 250}
    if (-not [IO.File]::Exists($result)) {throw ('Inspection timed out; inspect '+$directory)}
    Write-Output $result
  } finally {if ($task) {Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
  exit
}
$directory=Assert-LocalPath $OutputDirectory
if ([IO.Path]::GetDirectoryName($directory) -ine (Join-Path $Workspace 'runs') -or
    [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-windows-prompt-[a-f0-9]{8}$' -or
    @(Get-ChildItem -LiteralPath $directory -Force).Count) {throw 'New private prompt inspection directory required.'}
$result=@{status='failed';utc=[datetime]::UtcNow.ToString('o');readOnly=$true;actionsSent=0;sourceSha256=(Get-Digest $PSCommandPath)}
try {
  Add-Type -AssemblyName UIAutomationClient,UIAutomationTypes,System.Drawing,WindowsBase
  Add-Type -TypeDefinition @'
using System;using System.Runtime.InteropServices;using System.Text;using System.Collections.Generic;
public static class OpenNavPromptInspect {
 [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr value);
 [DllImport("user32.dll")] public static extern IntPtr GetForegroundWindow();
 [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
 delegate bool EnumProc(IntPtr h,IntPtr p);
 [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc callback,IntPtr p);
 [DllImport("user32.dll")] static extern bool EnumDesktopWindows(IntPtr d,EnumProc callback,IntPtr p);
 [DllImport("user32.dll",SetLastError=true)] static extern IntPtr OpenInputDesktop(uint flags,bool inherit,uint access);
 [DllImport("user32.dll")] static extern bool CloseDesktop(IntPtr d);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern bool GetUserObjectInformation(IntPtr h,int n,StringBuilder b,uint length,out uint needed);
 [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetWindowText(IntPtr h,StringBuilder s,int n);
 [DllImport("user32.dll",CharSet=CharSet.Unicode)] static extern int GetClassName(IntPtr h,StringBuilder s,int n);
 [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h,out uint p);
 public class Window {public long Handle;public uint ProcessId;public string Title,ClassName;}
 public static string InputDesktop(){var d=OpenInputDesktop(0,false,0x41);if(d==IntPtr.Zero)return "Unavailable "+Marshal.GetLastWin32Error();try{var name=new StringBuilder(256);uint n;return GetUserObjectInformation(d,2,name,512,out n)?name.ToString():"Unknown";}finally{CloseDesktop(d);}}
 public static Window[] InputWindows(){var items=new List<Window>();var d=OpenInputDesktop(0,false,0x41);if(d==IntPtr.Zero)return items.ToArray();try{EnumDesktopWindows(d,(h,p)=>{if(IsWindowVisible(h)){
  var text=new StringBuilder(512);var cl=new StringBuilder(256);uint pid;GetWindowThreadProcessId(h,out pid);GetWindowText(h,text,512);GetClassName(h,cl,256);
  items.Add(new Window{Handle=h.ToInt64(),ProcessId=pid,Title=text.ToString(),ClassName=cl.ToString()});}return items.Count<80;},IntPtr.Zero);return items.ToArray();}finally{CloseDesktop(d);}}
 public static Window[] InspectVisible(){var items=new List<Window>();EnumWindows((h,p)=>{if(IsWindowVisible(h)){
  var text=new StringBuilder(512);var cl=new StringBuilder(256);uint pid;GetWindowThreadProcessId(h,out pid);GetWindowText(h,text,512);GetClassName(h,cl,256);
  items.Add(new Window{Handle=h.ToInt64(),ProcessId=pid,Title=text.ToString(),ClassName=cl.ToString()});}return items.Count<80;},IntPtr.Zero);return items.ToArray();}
}
'@
  $old=[OpenNavPromptInspect]::SetThreadDpiAwarenessContext([IntPtr](-4))
  try {
    if ($ApplicationProcessId -gt 0) {
      $installed=Get-Installed;$application=Get-Process -Id $ApplicationProcessId
      if ($application.Path -ine $installed.executable -or $application.SessionId -ne [Diagnostics.Process]::GetCurrentProcess().SessionId -or -not $application.MainWindowHandle) {throw 'Exact installed application in this interactive session required for focus.'}
      $result.applicationFocused=[OpenNavPromptInspect]::SetForegroundWindow($application.MainWindowHandle)
      Start-Sleep -Milliseconds 500
    }
    $result.inputDesktop=[OpenNavPromptInspect]::InputDesktop()
    $result.inputWindows=[OpenNavPromptInspect]::InputWindows()
    # Read-only accessibility probes at the reviewed screenshot's prompt area.
    # No pointer movement or click. Useful for modern composited system sheets
    # whose proxy HWND exposes no automation children.
    $result.promptProbes=@(foreach($point in @(@(1110,755),@(810,755),@(900,265))) {
      $node=[Windows.Automation.AutomationElement]::FromPoint((New-Object Windows.Point($point[0],$point[1])))
      $chain=@();for($i=0;$i -lt 7 -and $node;$i++) {
        $c=$node.Current;$chain+=@{name=$c.Name;id=$c.AutomationId;className=$c.ClassName;processId=$c.ProcessId;handle=$c.NativeWindowHandle;type=$c.ControlType.ProgrammaticName;bounds=$c.BoundingRectangle.ToString();patterns=@($node.GetSupportedPatterns()|ForEach-Object {$_.ProgrammaticName})}
        $node=[Windows.Automation.TreeWalker]::ControlViewWalker.GetParent($node)
      }
      @{x=$point[0];y=$point[1];ancestors=$chain}
    })
    $condition=New-Object Windows.Automation.PropertyCondition([Windows.Automation.AutomationElement]::NameProperty,'Windows Security')
    $windows=[Windows.Automation.AutomationElement]::RootElement.FindAll([Windows.Automation.TreeScope]::Children,$condition)
    $window=$null
    if ($windows.Count -eq 1) {$window=$windows[0]}
    else {
      $result.nativeWindows=[OpenNavPromptInspect]::InspectVisible()
      $proxy=@($result.nativeWindows | Where-Object {$_.Title -ceq 'Windows Security' -and $_.ClassName -ceq 'Shell_SystemDialogProxy'})
      if ($proxy.Count -eq 1) {$window=[Windows.Automation.AutomationElement]::FromHandle([IntPtr]$proxy[0].Handle)}
    }
    if ($null -eq $window) {
      # Private metadata only, to identify an OS-version-specific wrapper. Do
      # not capture unrelated desktop pixels or traverse unrelated app trees.
      $tops=[Windows.Automation.AutomationElement]::RootElement.FindAll([Windows.Automation.TreeScope]::Children,[Windows.Automation.Condition]::TrueCondition)
      $result.topWindows=@($tops | Select-Object -First 80 | ForEach-Object {
        $c=$_.Current;[pscustomobject]@{name=$c.Name;className=$c.ClassName;processId=$c.ProcessId;handle=$c.NativeWindowHandle;offscreen=$c.IsOffscreen}
      })
      throw 'Expected one Windows Security window; no desktop-wide capture.'
    }
    $info=$window.Current
    $process=Get-Process -Id $info.ProcessId -ErrorAction Stop
    $signature=Get-AuthenticodeSignature -LiteralPath $process.Path
    $result.process=@{id=$process.Id;startedUtc=$process.StartTime.ToUniversalTime().ToString('o');path=$process.Path;sha256=(Get-Digest $process.Path);
      signatureStatus=$signature.Status.ToString();signer=$signature.SignerCertificate.Subject;session=$process.SessionId}
    $result.window=@{title=$info.Name;className=$info.ClassName;handle=$info.NativeWindowHandle;automationId=$info.AutomationId;foreground=[OpenNavPromptInspect]::GetForegroundWindow().ToInt64()}
    $nodes=$window.FindAll([Windows.Automation.TreeScope]::Descendants,[Windows.Automation.Condition]::TrueCondition)
    if ($nodes.Count -gt 300) {throw 'Prompt tree exceeds inspection bound.'}
    $result.controls=@(foreach ($node in $nodes) {
      $c=$node.Current
      [pscustomobject]@{name=$c.Name;id=$c.AutomationId;className=$c.ClassName;type=$c.ControlType.ProgrammaticName;enabled=$c.IsEnabled;offscreen=$c.IsOffscreen;
        processId=$c.ProcessId;handle=$c.NativeWindowHandle;bounds=$c.BoundingRectangle.ToString();patterns=@($node.GetSupportedPatterns()|ForEach-Object {$_.ProgrammaticName})}
    })
    $r=$info.BoundingRectangle
    if ($r.Width -lt 100 -or $r.Height -lt 100 -or $r.Width -gt 1920 -or $r.Height -gt 1080 -or $info.IsOffscreen) {throw 'Prompt is not visibly bounded.'}
    $image=Join-Path $directory 'prompt.png';$bitmap=New-Object Drawing.Bitmap([int]$r.Width,[int]$r.Height);$graphics=[Drawing.Graphics]::FromImage($bitmap)
    try {$graphics.CopyFromScreen([int]$r.X,[int]$r.Y,0,0,$bitmap.Size);$bitmap.Save($image,[Drawing.Imaging.ImageFormat]::Png)} finally {$graphics.Dispose();$bitmap.Dispose()}
    $result.image=$image;$result.imageSha256=Get-Digest $image;$result.status='passed'
  } finally {$null=[OpenNavPromptInspect]::SetThreadDpiAwarenessContext($old)}
} catch {$result.error=$_.Exception.Message}
Write-Record (Join-Path $directory 'inspection.json') $result
