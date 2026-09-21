param([string]$Name = '11-legacy-mode')
$ErrorActionPreference = 'Stop'
$Root = Split-Path $PSScriptRoot -Parent
$Build = Join-Path $Root 'build/pristine-windows'
$Evidence = Join-Path $Root 'evidence/local'
$Profile = Join-Path $Root ('build/profiles/' + [Guid]::NewGuid().ToString())
& python (Join-Path $PSScriptRoot 'prepare-test-profile.py') --build $Build --profile $Profile
if ($LASTEXITCODE -ne 0) { throw 'Cannot create isolated profile' }
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class OpenNavCapture {
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
  [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr h, IntPtr after, int x, int y, int w, int height, uint flags);
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr dc, uint flags);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out Rect rect);
  [StructLayout(LayoutKind.Sequential)] public struct Rect { public int left, top, right, bottom; }
}
'@
[OpenNavCapture]::SetProcessDPIAware() | Out-Null
$Exe = Join-Path $Root 'build/pristine-install/opencpn.exe'
$Proc = Start-Process -FilePath $Exe -ArgumentList @("--configdir=`"$Profile`"", '--no_opengl') -PassThru
try {
    $Deadline = (Get-Date).AddSeconds(60)
    do {
        Start-Sleep -Milliseconds 500
        $Proc.Refresh()
        if ($Proc.HasExited) { throw "OpenCPN exited during startup: $($Proc.ExitCode)" }
    } until ($Proc.MainWindowHandle -ne 0 -or (Get-Date) -gt $Deadline)
    if ($Proc.MainWindowHandle -eq 0) { throw 'OpenCPN window not found' }
    Start-Sleep -Seconds 8
    $Proc.Refresh()
    [OpenNavCapture]::ShowWindow($Proc.MainWindowHandle, 9) | Out-Null
    if (-not [OpenNavCapture]::SetWindowPos($Proc.MainWindowHandle, [IntPtr]::Zero, 0, 0, 1280, 800, 4)) {
        throw 'Cannot size baseline window'
    }
    Start-Sleep -Seconds 2
    $Rect = New-Object OpenNavCapture+Rect
    [OpenNavCapture]::GetWindowRect($Proc.MainWindowHandle, [ref]$Rect) | Out-Null
    if ($Rect.right - $Rect.left -ne 1280 -or $Rect.bottom - $Rect.top -ne 800) {
        throw 'Native window is not 1280x800'
    }
    $Bitmap = New-Object System.Drawing.Bitmap(1280, 800)
    $Graphics = [System.Drawing.Graphics]::FromImage($Bitmap)
    $Dc = $Graphics.GetHdc()
    try {
        if (-not [OpenNavCapture]::PrintWindow($Proc.MainWindowHandle, $Dc, 2)) {
            throw 'Native window capture failed'
        }
    } finally { $Graphics.ReleaseHdc($Dc) }
    $Bitmap.Save((Join-Path $Evidence "$Name.png"), [System.Drawing.Imaging.ImageFormat]::Png)
    $Graphics.Dispose()
    $Bitmap.Dispose()
    "Native Windows baseline; software chart rendering (--no_opengl); no charts or sensor fixtures loaded; visual review required." |
        Out-File (Join-Path $Evidence "$Name.txt")
    if (-not $Proc.CloseMainWindow()) { throw 'Graceful close request failed' }
    if (-not $Proc.WaitForExit(30000)) { throw 'OpenCPN did not close gracefully' }
    if ($Proc.ExitCode -ne 0) { throw "OpenCPN exit code $($Proc.ExitCode)" }
} finally {
    $Proc.Refresh()
    if (-not $Proc.HasExited) { Stop-Process -Id $Proc.Id }
    $ProfileEvidence = Join-Path $Evidence 'baseline-profile'
    New-Item -ItemType Directory -Force $ProfileEvidence | Out-Null
    Copy-Item (Join-Path $Profile '*') $ProfileEvidence -Recurse -Force
}
