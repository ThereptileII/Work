# Disposable marker window only. No stock/product application or marine input.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Directory)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$config=Get-Content -LiteralPath (Join-Path $Directory 'fixture.json') -Raw|ConvertFrom-Json
if($config.owner -cne 'OpenNavX.StockResize.Fixture.1' -or $config.case -cnotin @('offscreen','maximized','rename-on-restore','wrong-title','disabled')){throw 'Unknown fixture case.'}
Add-Type -AssemblyName System.Windows.Forms,System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class StockResizeFixture {
 [StructLayout(LayoutKind.Sequential)] public struct Rect {public int Left,Top,Right,Bottom;}
 [StructLayout(LayoutKind.Sequential)] public struct Point {public int X,Y;}
 [StructLayout(LayoutKind.Sequential)] public struct Placement {public int Length,Flags,Show;public Point Minimum,Maximum;public Rect Normal;}
 [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr value);
 [DllImport("user32.dll")] private static extern bool SetWindowPlacement(IntPtr window,ref Placement value);
 [DllImport("user32.dll")] private static extern bool GetWindowPlacement(IntPtr window,ref Placement value);
 [DllImport("user32.dll")] private static extern bool EnableWindow(IntPtr window,bool enabled);
 public static Placement Seed(IntPtr window,int left,int top,int width,int height,bool maximum) {
  var p=new Placement();p.Length=Marshal.SizeOf(typeof(Placement));
  if(!GetWindowPlacement(window,ref p))throw new InvalidOperationException("GetWindowPlacement failed");
  p.Normal=new Rect{Left=left,Top=top,Right=left+width,Bottom=top+height};p.Show=maximum?3:1;
  if(!SetWindowPlacement(window,ref p)||!GetWindowPlacement(window,ref p))throw new InvalidOperationException("Window placement seeding failed");
  return p;
 }
 public static void Disable(IntPtr window){EnableWindow(window,false);}
}
'@
$oldDpi=[StockResizeFixture]::SetThreadDpiAwarenessContext([IntPtr](-4));if($oldDpi -eq [IntPtr]::Zero){throw 'Physical DPI fixture unavailable.'}
$form=New-Object Windows.Forms.Form;$form.Text='OpenCPN - disposable resize marker';$form.StartPosition='Manual'
$form.Location=New-Object Drawing.Point(50,50);$form.Size=New-Object Drawing.Size(900,650)
$script:armed=$false;$script:seed=$null;$script:ready=$false
$form.Add_Resize({if($script:armed -and $config.case -ceq 'rename-on-restore' -and $form.WindowState -eq 'Normal'){$form.Text='Changed marker identity';$script:armed=$false}})
$form.Add_Shown({
 $screen=[Windows.Forms.Screen]::FromHandle($form.Handle);$work=$screen.WorkingArea
 if($work.Width -lt 1280 -or $work.Height -lt 800){throw 'Fixture desktop needs existing1280x800work area; display not changed.'}
 $maximum=$config.case -cin @('maximized','rename-on-restore')
 $script:seed=[StockResizeFixture]::Seed($form.Handle,$work.Left-400,$work.Top+20,$work.Width+300,$work.Height+100,$maximum)
 if($config.case -ceq 'wrong-title'){$form.Text='Different application marker'}
 if($config.case -ceq 'disabled'){[StockResizeFixture]::Disable($form.Handle)}
 $script:armed=$true
})
$timer=New-Object Windows.Forms.Timer;$timer.Interval=100;$started=[datetime]::UtcNow
$timer.Add_Tick({
 if(-not $script:ready -and $null -ne $script:seed){
  $process=[Diagnostics.Process]::GetCurrentProcess()
  $value=@{pid=$PID;createdFiletime=$process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString();handle=$form.Handle.ToInt64();seed=$script:seed;case=$config.case}
  $tmp=Join-Path $Directory 'ready.partial';[IO.File]::WriteAllText($tmp,($value|ConvertTo-Json -Depth 5));[IO.File]::Move($tmp,(Join-Path $Directory 'ready.json'));$script:ready=$true
 }
 if((Test-Path -LiteralPath (Join-Path $Directory 'release')) -or ([datetime]::UtcNow-$started).TotalSeconds -gt 45){$form.Close()}
})
try{$timer.Start();[Windows.Forms.Application]::Run($form)}finally{$timer.Stop();$timer.Dispose();$form.Dispose();$null=[StockResizeFixture]::SetThreadDpiAwarenessContext($oldDpi)}
