$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Root = Split-Path $PSScriptRoot -Parent
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VS = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VS) { throw 'Native licensed MSVC toolchain not found' }
$Runtime = Get-ChildItem "$VS\VC\Redist\MSVC\*\x86\Microsoft.VC143.CRT" -Directory |
    Sort-Object FullName -Descending | Select-Object -First 1
if (-not $Runtime) { throw 'App-local x86 MSVC redistributable directory not found' }
$Output = Join-Path $Root 'build/developer-preview'
python (Join-Path $PSScriptRoot 'package-preview.py') --install "$Root/build/production-install" `
    --build "$Root/build/production-windows" --runtime $Runtime.FullName --output $Output
if ($LASTEXITCODE -ne 0) { throw 'Recovery assembly failed' }
python (Join-Path $PSScriptRoot 'verify-preview-pe.py') "$Output/OpenNavX-Beta2-Portable-Recovery/app" `
    --report "$Root/evidence/local/preview-dll-audit.json"
if ($LASTEXITCODE -ne 0) { throw 'Recovery dependency closure failed' }
python (Join-Path $PSScriptRoot 'smoke-portable-production.py') --package "$Output/OpenNavX-Beta2-Portable-Recovery.zip"
if ($LASTEXITCODE -ne 0) { throw 'Extracted production recovery smoke test failed' }
