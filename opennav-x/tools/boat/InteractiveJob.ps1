# Runs only as the already logged-in user. This is not an elevated service.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Request)
. (Join-Path $PSScriptRoot 'Common.ps1')
$job=Read-Record $Request
$result=@{status='failed';action=$job.action;utc=[DateTime]::UtcNow.ToString('o')}
try {
  if ($job.action -notin @('Launch','Close','Capture')) { throw 'Unsupported interactive action.' }
  $exe=Assert-LocalPath $job.executable
  if ((Get-Digest $exe) -cne $job.executableSha256) { throw 'Application changed between dispatch and interactive execution.' }
  if ($job.action -eq 'Launch') {
    # Repeat the complete guard inside the interactive session, immediately
    # before launch. A queued task is not a reusable safety approval.
    $config=Get-Target $job.workspace;$installed=Get-Installed
    if ((Assert-LocalPath $installed.executable) -ine $exe) { throw 'Installed generation changed since dispatch.' }
    Assert-ReadOnlyAudit $config $installed
    if (@(Get-Process -Name opencpn -ErrorAction SilentlyContinue).Count) { throw 'An OpenCPN instance is already running. Close it normally first.' }
    if ($job.mode -notin @('--xnav','--legacy','--safe-mode')) { throw 'Only real-data navigation/recovery modes are permitted.' }
    $start=New-Object Diagnostics.ProcessStartInfo
    $start.FileName=$exe; $start.Arguments=$job.mode; $start.WorkingDirectory=[IO.Path]::GetDirectoryName($exe);$start.UseShellExecute=$false
    $process=[Diagnostics.Process]::Start($start)
    try {
      $deadline=[DateTime]::UtcNow.AddSeconds(45)
      do { Start-Sleep -Milliseconds 250;$process.Refresh() } while (-not $process.HasExited -and -not $process.MainWindowHandle -and [DateTime]::UtcNow -lt $deadline)
      if ($process.HasExited -or -not $process.MainWindowHandle) { throw 'Application failed to expose its normal window; inspect logs and Safe Mode.' }
      $result.pid=$process.Id;$result.mode=$job.mode;$result.status='passed'
    } finally {$process.Dispose()}
  } else {
    $process=Get-Process -Id $job.processId -ErrorAction Stop
    try {
      if ((Assert-LocalPath $process.Path) -ine $exe -or -not $process.MainWindowHandle) { throw 'Process identity/window mismatch.' }
      if ($job.action -eq 'Close') {
        if (-not $process.CloseMainWindow()) { throw 'Application did not accept normal window close.' }
        if (-not $process.WaitForExit(30000)) { throw 'Application close needs attention; no process termination attempted.' }
        $result.exitCode=$process.ExitCode
        if ($process.ExitCode -ne 0) { throw 'Application exited with an error.' }
      } else {
        Add-Type -AssemblyName System.Drawing
        Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class OpenNavCapture {
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left,Top,Right,Bottom; }
  [DllImport("user32.dll")] public static extern IntPtr SetThreadDpiAwarenessContext(IntPtr value);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern uint GetDpiForWindow(IntPtr h);
}
'@
        $null=[OpenNavCapture]::SetForegroundWindow($process.MainWindowHandle)
        Start-Sleep -Milliseconds 500
        $oldDpi=[OpenNavCapture]::SetThreadDpiAwarenessContext([IntPtr](-4))
        $rect=New-Object OpenNavCapture+RECT
        if (-not [OpenNavCapture]::GetWindowRect($process.MainWindowHandle,[ref]$rect)) { throw 'Window rectangle unavailable.' }
        $width=$rect.Right-$rect.Left;$height=$rect.Bottom-$rect.Top
        if ($width -lt 100 -or $height -lt 100 -or $width -gt 7680 -or $height -gt 4320) { throw 'Window size outside capture bound.' }
        $path=Assert-LocalPath $job.imagePath
        if ([IO.File]::Exists($path)) { throw 'Capture requires a new output path.' }
        $bitmap=New-Object Drawing.Bitmap($width,$height)
        $graphics=[Drawing.Graphics]::FromImage($bitmap)
        try {
          $graphics.CopyFromScreen($rect.Left,$rect.Top,0,0,$bitmap.Size)
          $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
        } finally {$graphics.Dispose();$bitmap.Dispose();$null=[OpenNavCapture]::SetThreadDpiAwarenessContext($oldDpi)}
        $result.image=$path;$result.sha256=Get-Digest $path;$result.width=$width;$result.height=$height
        $result.dpi=[OpenNavCapture]::GetDpiForWindow($process.MainWindowHandle)
        $result.review='Native window pixels captured; human chart/content review still required.'
      }
      $result.status='passed'
    } finally {$process.Dispose()}
  }
} catch {$result.error=$_.Exception.Message}
Write-Record $job.resultPath $result
