# Tests only. This tree is never a release input.
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$Root=Split-Path $PSScriptRoot -Parent
$Vswhere="${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VS=& $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VS) {throw 'Native MSVC runtime not found'}
$Runtime=Get-ChildItem "$VS\VC\Redist\MSVC\*\x86\Microsoft.VC143.CRT" -Directory | Sort-Object FullName -Descending | Select-Object -First 1
if (-not $Runtime) {throw 'Native x86 runtime not found'}
python (Join-Path $PSScriptRoot 'smoke-preview.py') --install "$Root/build/xnav-install" --runtime $Runtime.FullName
if ($LASTEXITCODE -ne 0) {throw 'Fixture UI regression failed'}
