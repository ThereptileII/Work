param([switch]$DeferRuntimeQualification)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ($DeferRuntimeQualification -and $env:GITHUB_ACTIONS -cne 'true') {
    throw 'Deferred portable qualification requires the explicit CI build job'
}
$Root = Split-Path $PSScriptRoot -Parent
$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$VS = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VS) { throw 'Native licensed MSVC toolchain not found' }
$Runtime = Get-ChildItem "$VS\VC\Redist\MSVC\*\x86\Microsoft.VC143.CRT" -Directory |
    Sort-Object FullName -Descending | Select-Object -First 1
if (-not $Runtime) { throw 'App-local x86 MSVC redistributable directory not found' }
$Output = Join-Path $Root 'build/developer-preview'
python (Join-Path $PSScriptRoot 'package-preview.py') --install "$Root/build/production-install" `
    --build "$Root/build/production-windows" --runtime $Runtime.FullName --output $Output `
    --openssl-source-cache "$Root/build/dependency-downloads/openssl-3.5.9.tar.gz" `
    --dependency-source-cache "$Root/build/dependency-downloads"
$AssemblyExit = $LASTEXITCODE
$Evidence = Join-Path $Root 'evidence/local'
$null = New-Item -ItemType Directory -Path $Evidence -Force
foreach ($Name in @('production-package-selftest.json', 'production-restart-selftest.json')) {
    $Report = Join-Path $Output $Name
    if (Test-Path -LiteralPath $Report -PathType Leaf) {
        Copy-Item -LiteralPath $Report -Destination (Join-Path $Evidence $Name)
    } elseif ($AssemblyExit -eq 0) {
        throw ('Successful assembly omitted executed capability evidence: ' + $Name)
    }
}
if ($AssemblyExit -ne 0) { throw 'Recovery assembly failed' }
python (Join-Path $PSScriptRoot 'verify-preview-pe.py') "$Output/SKAGER-Beta2-Portable-Recovery/app" `
    --report "$Root/evidence/local/preview-dll-audit.json"
if ($LASTEXITCODE -ne 0) { throw 'Recovery dependency closure failed' }
if (-not $DeferRuntimeQualification) {
    python (Join-Path $PSScriptRoot 'smoke-portable-production.py') --package "$Output/SKAGER-Beta2-Portable-Recovery.zip"
    if ($LASTEXITCODE -ne 0) { throw 'Extracted production recovery smoke test failed' }
}
