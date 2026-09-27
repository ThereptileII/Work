# One image-reviewed Cancel click for the Win11 composited firewall sheet.
# This fallback is used only when the normal HWND close removed the proxy but
# left its sheet visible. Fixed reviewed pixels; no Allow/keyboard/other action.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Review,[Parameter(Mandatory=$true)][string]$ReviewSha256,
      [string]$OutputDirectory='', [switch]$Interactive)
. (Join-Path $PSScriptRoot 'Preparation.ps1')
if (-not $Interactive) {
  $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
  $explorers=@(Get-CimInstance Win32_Process -Filter "Name='explorer.exe'" | Where-Object {(Invoke-CimMethod -InputObject $_ -MethodName GetOwnerSid).Sid -eq $sid})
  if($explorers.Count -ne 1){throw 'One interactive desktop required.'}
  $directory=New-PreparationDirectory ([pscustomobject]@{workspace='C:\XNav';sid=$sid}) 'visible-firewall-cancel'
  $name='OpenNavX-Visible-Cancel-'+[guid]::NewGuid().ToString('N')
  $action=New-ScheduledTaskAction -Execute (Join-Path $env:WINDIR 'System32\WindowsPowerShell\v1.0\powershell.exe') -Argument ('-NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File "'+$PSCommandPath+'" -Interactive -OutputDirectory "'+$directory+'" -Review "'+(Assert-LocalPath $Review)+'" -ReviewSha256 '+$ReviewSha256)
  $principal=New-ScheduledTaskPrincipal -UserId $sid -LogonType Interactive -RunLevel Limited
  $settings=New-ScheduledTaskSettingsSet -ExecutionTimeLimit (New-TimeSpan -Minutes 2) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
  $task=$null
  try {
    $task=Register-ScheduledTask -TaskName $name -Action $action -Principal $principal -Settings $settings
    Start-ScheduledTask -TaskName $name
    $result=Join-Path $directory 'result.json';$deadline=[datetime]::UtcNow.AddSeconds(55)
    while(-not [IO.File]::Exists($result) -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 250}
    if(-not [IO.File]::Exists($result)){throw ('Read '+$directory+' before further action; do not retry.')}
    Write-Output $result
  }finally{if($task){Unregister-ScheduledTask -TaskName $name -Confirm:$false}}
  exit
}
$directory=Assert-LocalPath $OutputDirectory
if([IO.Path]::GetDirectoryName($directory) -ine 'C:\XNav\runs' -or [IO.Path]::GetFileName($directory) -cnotmatch '^\d{8}-\d{6}-visible-firewall-cancel-[a-f0-9]{8}$' -or @(Get-ChildItem -LiteralPath $directory -Force).Count){throw 'New private evidence directory required.'}
$result=@{status='failed';utc=[datetime]::UtcNow.ToString('o');clickSent=$false;allowInvoked=$false;reviewSha256=$ReviewSha256;sourceSha256=(Get-Digest $PSCommandPath)}
try {
  if($ReviewSha256 -cnotmatch '^[a-f0-9]{64}$' -or (Get-Digest $Review) -cne $ReviewSha256){throw 'Exact reviewed image binding required.'}
  $r=Read-Record $Review;$installed=Get-Installed;$p=Get-Process -Id $r.processId
  if($p.Path -ine $installed.executable -or $r.executableSha256 -cne (Get-Digest $installed.executable) -or $p.SessionId -ne [Diagnostics.Process]::GetCurrentProcess().SessionId -or
     $p.StartTime.ToUniversalTime().Ticks -ne ([datetime]::Parse($r.startedUtc)).ToUniversalTime().Ticks -or (Get-Digest $r.image) -cne $r.imageSha256){throw 'Installed PID/start/build/image changed.'}
  $at=[datetime]::Parse($r.reviewedUtc).ToUniversalTime()
  if($at -gt [datetime]::UtcNow -or ([datetime]::UtcNow-$at).TotalMinutes -gt 30 -or $r.action -cne 'Cancel Windows Firewall prompt for opencpn'){throw 'Recent specific visual review required.'}
  Add-Type -AssemblyName System.Drawing
  Add-Type -ReferencedAssemblies System.Drawing -TypeDefinition @'
using System;using System.Drawing;using System.Drawing.Imaging;using System.Runtime.InteropServices;using System.Security.Cryptography;
public static class OpenNavVisibleCancel {
 [StructLayout(LayoutKind.Sequential)] struct RECT{public int L,T,R,B;}
 [StructLayout(LayoutKind.Sequential)] struct MOUSE{public int X,Y;public uint Data,Flags,Time;public UIntPtr Extra;}
 [StructLayout(LayoutKind.Sequential)] struct INPUT{public uint Type;public MOUSE Mouse;}
 [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr p);
 [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
 [DllImport("user32.dll")] static extern IntPtr GetForegroundWindow();
 [DllImport("user32.dll")] static extern int GetSystemMetrics(int n);
 [DllImport("user32.dll",SetLastError=true)] static extern uint SendInput(uint n,INPUT[] data,int size);
 [DllImport("dwmapi.dll")] static extern int DwmGetWindowAttribute(IntPtr h,uint a,out RECT r,int size);
 [DllImport("dwmapi.dll")] static extern int DwmFlush();
 public static void Bound(IntPtr h){RECT r;if(GetForegroundWindow()!=h || DwmGetWindowAttribute(h,9,out r,16)!=0 || r.L!=0||r.T!=0||r.R!=1280||r.B!=800||GetSystemMetrics(0)!=1920||GetSystemMetrics(1)!=1080)throw new InvalidOperationException("Reviewed frame/foreground/desktop changed");}
 static string Region(Bitmap b,int x,int y,int w,int h){byte[] rgb=new byte[w*h*3];int at=0;for(int row=y;row<y+h;row++)for(int col=x;col<x+w;col++){var c=b.GetPixel(col,row);rgb[at++]=c.R;rgb[at++]=c.G;rgb[at++]=c.B;}using(var hash=SHA256.Create())return BitConverter.ToString(hash.ComputeHash(rgb)).Replace("-","").ToLowerInvariant();}
 public static void Capture(string path){DwmFlush();using(var b=new Bitmap(1280,800)){using(var g=Graphics.FromImage(b)){g.CopyFromScreen(0,0,0,0,b.Size);}b.Save(path,ImageFormat.Png);}}
 public static void Verify(IntPtr h){Bound(h);DwmFlush();using(var b=new Bitmap(1280,800)){using(var g=Graphics.FromImage(b)){g.CopyFromScreen(0,0,0,0,b.Size);}if(Region(b,650,245,620,390)!="1d3269c56cc0bac08e5ee75c650c599161392c8ae1f195eb0375fe5a141a9437"||Region(b,974,735,270,38)!="536fba3f3ecb98eec29eff8ba3e93c5a61915d6ac1ac352a4bdcc14c3220bc66"||Region(b,650,200,205,33)!="a48823e90f6726ba1e52272b5f1cbbbfb4bf1e79d18c35d967f1f6761068b4eb")throw new InvalidOperationException("Reviewed Windows Firewall heading, opencpn body or Cancel pixels changed; no input");}Bound(h);}
 public static uint Cancel(IntPtr h){Verify(h);var input=new INPUT[3];input[0].Mouse.X=(int)Math.Round(1110.0*65535/1919);input[0].Mouse.Y=(int)Math.Round(755.0*65535/1079);input[0].Mouse.Flags=0x8001;input[1].Mouse.Flags=2;input[2].Mouse.Flags=4;return SendInput(3,input,Marshal.SizeOf(typeof(INPUT)));}
}
'@
  $old=[OpenNavVisibleCancel]::SetThreadDpiAwarenessContext([IntPtr](-4))
  try {
    $null=[OpenNavVisibleCancel]::SetForegroundWindow($p.MainWindowHandle);Start-Sleep -Milliseconds 500
    [OpenNavVisibleCancel]::Verify($p.MainWindowHandle)
    [OpenNavVisibleCancel]::Capture((Join-Path $directory 'before.png'))
    Write-Record (Join-Path $directory 'intent.json') @{action=$r.action;reviewSha256=$ReviewSha256;processId=$p.Id;startedUtc=$r.startedUtc;target='Reviewed Cancel button only'}
    $result.eventsSent=[OpenNavVisibleCancel]::Cancel($p.MainWindowHandle);$result.clickSent=$result.eventsSent -eq 3
    Start-Sleep -Seconds 2
    [OpenNavVisibleCancel]::Capture((Join-Path $directory 'after.png'))
    $result.beforeSha256=Get-Digest (Join-Path $directory 'before.png');$result.afterSha256=Get-Digest (Join-Path $directory 'after.png')
    $result.status='attention';$result.review='One Cancel click sent; inspect actual after pixels. No successful dismissal inferred from input delivery.'
  }finally{$null=[OpenNavVisibleCancel]::SetThreadDpiAwarenessContext($old)}
}catch{$result.error=$_.Exception.Message}
Write-Record (Join-Path $directory 'result.json') $result
