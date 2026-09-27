# Actual Win32 placement tests on owned inert windows, never a boat/application.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Evidence,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or ($env:GITHUB_ACTIONS -cne 'true' -and -not $IsolatedLocal)){throw 'Native CI or explicit disposable local desktop required.'}
if(@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count){throw 'Resize fixtures refuse coexistence with a navigation application.'}
. (Join-Path $PSScriptRoot 'boat\StockReview.ps1');Initialize-StockReviewNative
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class StockResizeObservation {
 [StructLayout(LayoutKind.Sequential)] public struct Rect {public int Left,Top,Right,Bottom;}
 [DllImport("user32.dll")] private static extern bool GetWindowRect(IntPtr window,out Rect value);
 public static Rect Read(IntPtr window){Rect value;if(!GetWindowRect(window,out value))throw new InvalidOperationException("Fixture bounds unavailable");return value;}
}
'@
$fixture=Join-Path ([IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))) 'tests\stock-resize\window-fixture.ps1'
$destination=[IO.Path]::GetFullPath($Evidence);$null=New-Item -ItemType Directory -Path $destination -Force
$results=New-Object 'Collections.Generic.List[object]';$cleanup=New-Object 'Collections.Generic.List[string]';$failure=$null
$oldDpi=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext([IntPtr](-4));if($oldDpi -eq [IntPtr]::Zero){throw 'Physical DPI context unavailable.'}
try {
 foreach($case in @('offscreen','maximized','rename-on-restore','wrong-title','disabled','wrong-pid','reported-failure')) {
  $directory=Join-Path ([IO.Path]::GetTempPath()) ('opennav-stock-resize-'+[guid]::NewGuid().ToString('N'));$null=New-Item -ItemType Directory -Path $directory
  $variant=if($case -ceq 'wrong-pid'){'offscreen'}else{$case}
  [IO.File]::WriteAllText((Join-Path $directory 'fixture.json'),(@{owner='OpenNavX.StockResize.Fixture.1';case=$variant}|ConvertTo-Json -Compress))
  $start=New-Object Diagnostics.ProcessStartInfo;$start.FileName=Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell\v1.0\powershell.exe'
  $start.Arguments='-NoProfile -STA -ExecutionPolicy Bypass -File "'+$fixture+'" -Directory "'+$directory+'"';$start.UseShellExecute=$false;$start.CreateNoWindow=$true
  $process=[Diagnostics.Process]::Start($start);$null=$process.Handle
  try {
   $readyPath=Join-Path $directory 'ready.json';$failedPath=Join-Path $directory 'failure.json';$deadline=[datetime]::UtcNow.AddSeconds(10)
   while(-not (Test-Path -LiteralPath $readyPath) -and -not (Test-Path -LiteralPath $failedPath) -and -not $process.HasExited -and [datetime]::UtcNow -lt $deadline){Start-Sleep -Milliseconds 50}
   if(Test-Path -LiteralPath $failedPath) {
    $failed=Get-Content -LiteralPath $failedPath -Raw|ConvertFrom-Json
    if($failed.pid -ne $process.Id -or $failed.status -cne 'failed'){throw 'Unexpected fixture failure identity.'}
    if($case -ceq 'reported-failure' -and $failed.error -ceq 'Expected disposable readiness failure.') {
     $results.Add(@{case=$case;refused=$true;reason=$failed.error;nativeActionCalled=$false})
     continue
    }
    throw ('Owned native resize fixture refused: '+$failed.error)
   }
   if($case -ceq 'reported-failure'){throw 'Expected callback error did not publish bounded failure metadata.'}
   if(-not (Test-Path -LiteralPath $readyPath)){throw 'Owned native resize fixture failed to become ready.'}
   $ready=Get-Content -LiteralPath $readyPath -Raw|ConvertFrom-Json
   if($ready.pid -ne $process.Id -or $ready.createdFiletime -cne $process.StartTime.ToUniversalTime().ToFileTimeUtc().ToString()){throw 'Fixture process creation identity changed.'}
   $frame=[IntPtr]$ready.handle;$before=[StockResizeObservation]::Read($frame);$refused=$false;$reason=$null;$resized=$null
   if($case -ceq 'offscreen') {
    $strictRejected=$false
    try{[OpenNavX.StockReviewNative]::Foreground($frame,$process.Id)}catch{$strictRejected=$_.Exception.Message -like '*Complete reviewed frame must fit*'}
    if(-not $strictRejected){throw 'Fixture did not reproduce strict capture/foreground containment refusal.'}
   }
   try {$resized=[OpenNavX.StockReviewNative]::Resize1280x800($frame,$(if($case -ceq 'wrong-pid'){$process.Id+1}else{$process.Id}))}catch{$refused=$true;$reason=$_.Exception.Message}
   $after=[StockResizeObservation]::Read($frame)
   $results.Add(@{case=$case;pid=$process.Id;createdFiletime=$ready.createdFiletime;seed=$ready.seed;before=$before;after=$after;resize=$resized;refused=$refused;reason=$reason})
   if($case -cin @('offscreen','maximized')) {
    if($refused -or $null -eq $resized -or $resized.After.Bounds.Width -ne 1280 -or $resized.After.Bounds.Height -ne 800 -or $resized.RestoreRequested -ne ($case -ceq 'maximized')){throw ('Fixed native resize failed: '+$case+': '+$reason)}
    if($case -ceq 'maximized' -and $resized.Restored.Bounds.Left -ge $resized.MonitorBounds.Left -and $resized.Restored.Bounds.Top -ge $resized.MonitorBounds.Top -and $resized.Restored.Bounds.Right -le $resized.MonitorBounds.Right -and $resized.Restored.Bounds.Bottom -le $resized.MonitorBounds.Bottom){throw 'Maximized fixture did not restore an offscreen/oversized normal rectangle.'}
    $info=[OpenNavX.StockReviewNative]::AssertFrame($frame,$process.Id)
    $bitmap=New-Object Drawing.Bitmap(1280,800);$graphics=[Drawing.Graphics]::FromImage($bitmap)
    try{
     [OpenNavX.StockReviewNative]::AssertCapture($frame,$process.Id,$info)
     $graphics.CopyFromScreen($info.Bounds.Left,$info.Bounds.Top,0,0,$bitmap.Size)
     [OpenNavX.StockReviewNative]::AssertCapture($frame,$process.Id,$info)
     $bitmap.Save((Join-Path $destination ($case+'.png')),[Drawing.Imaging.ImageFormat]::Png)
    }finally{$graphics.Dispose();$bitmap.Dispose()}
   } else {
    if(-not $refused){throw ('Unsafe resize fixture accepted: '+$case)}
    if($case -cne 'rename-on-restore' -and ($before|ConvertTo-Json -Compress) -cne ($after|ConvertTo-Json -Compress)){throw 'Refused identity changed the native frame bounds.'}
    if($case -ceq 'rename-on-restore' -and $reason -notlike '*Expected the official stock OpenCPN main frame*'){throw 'Restored identity was not rechecked before placement.'}
   }
  }finally{
   try{[IO.File]::WriteAllText((Join-Path $directory 'release'),'normal marker close');if(-not $process.WaitForExit(30000)){throw 'Marker did not close normally; no force termination.'};$process.Dispose();Remove-Item -LiteralPath $directory -Recurse -Force}catch{$cleanup.Add($_.Exception.Message)}
  }
 }
 # Exercise the same pre-mutation guard with insufficient work areas. The real
 # desktop resolution/work area is observed above and is never modified.
 $guard=[OpenNavX.StockReviewNative].GetMethod('ValidateResizeWorkArea',[Reflection.BindingFlags]'NonPublic,Static')
 foreach($size in @(@(1279,800),@(1280,799))) {
  $rect=New-Object OpenNavX.StockReviewNative+Rect;$rect.Right=$size[0];$rect.Bottom=$size[1];$refused=$false
  try{$null=$guard.Invoke($null,[object[]]@($rect))}catch{$refused=$true}
  if(-not $refused){throw 'Insufficient work-area guard accepted fixed resize.'}
  $results.Add(@{case='insufficient-work-area';width=$size[0];height=$size[1];refused=$true;desktopChanged=$false})
 }
 if($cleanup.Count){throw 'Native fixture cleanup failed.'}
}catch{$failure=$_.Exception.Message;throw}
finally{
 $null=[OpenNavX.StockReviewNative]::SetThreadDpiAwarenessContext($oldDpi)
 [IO.File]::WriteAllText((Join-Path $destination 'native-stock-resize.json'),(@{status=$(if($failure -or $cleanup.Count){'failed'}else{'passed'});error=$failure;cleanupErrors=$cleanup.ToArray();cases=$results.ToArray();nativeHelperSha256=(Get-Digest (Join-Path $PSScriptRoot 'boat\StockReviewNative.cs'));fixtureSha256=(Get-Digest $fixture);navigationApplicationLaunched=$false;boatAccess=$false;desktopSettingsChanged=$false}|ConvertTo-Json -Depth 10))
}
