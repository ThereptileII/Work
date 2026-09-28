# CI-only exercise of the actual boat capture guard against the built native UI.
# Never authorizes a boat launch, input action or change to an installed profile.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][int]$ProcessId,
      [Parameter(Mandatory=$true)][long]$Handle,
      [Parameter(Mandatory=$true)][string]$Output)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -ne 'Win32NT' -or $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable native CI only.'}
$root=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$path=[IO.Path]::GetFullPath($Output)
$evidence=Join-Path $root 'evidence\local\'
if(-not $path.StartsWith($evidence,[StringComparison]::OrdinalIgnoreCase) -or (Test-Path -LiteralPath $path) -or (Test-Path -LiteralPath ($path+'.json'))){throw 'Fresh CI evidence path required.'}
$process=Get-Process -Id $ProcessId
$executable=Join-Path $root 'build\production-install\opencpn.exe'
if($process.Path -ine $executable -or $process.SessionId -ne (Get-Process -Id $PID).SessionId){throw 'Exact disposable CI executable and session required.'}
Add-Type -Path (Join-Path $root 'tools\boat\ReviewWindowNative.cs')
Add-Type -AssemblyName System.Drawing
$oldDpi=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext([IntPtr](-4))
if($oldDpi -eq [IntPtr]::Zero){throw 'Physical-pixel DPI context unavailable.'}
$bitmap=$null;$graphics=$null
try {
 $info=[OpenNavX.ReviewWindowNative]::AssertFrame([IntPtr]$Handle,$ProcessId)
 $bitmap=New-Object Drawing.Bitmap($info.Bounds.Width,$info.Bounds.Height)
 $graphics=[Drawing.Graphics]::FromImage($bitmap)
 [OpenNavX.ReviewWindowNative]::AssertCapture([IntPtr]$Handle,$ProcessId,$info)
 $graphics.CopyFromScreen($info.Bounds.Left,$info.Bounds.Top,0,0,$bitmap.Size)
 [OpenNavX.ReviewWindowNative]::AssertCapture([IntPtr]$Handle,$ProcessId,$info)
 $bitmap.Save($path,[Drawing.Imaging.ImageFormat]::Png)
 [IO.File]::WriteAllText(($path+'.json'),($info|ConvertTo-Json -Depth 8))
} finally {
 if($graphics){$graphics.Dispose()};if($bitmap){$bitmap.Dispose()}
 $null=[OpenNavX.ReviewWindowNative]::SetThreadDpiAwarenessContext($oldDpi)
}
