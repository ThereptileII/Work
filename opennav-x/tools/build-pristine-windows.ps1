param([ValidateSet('Win32', 'x64')][string]$Architecture = 'Win32', [switch]$Integration)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Root = Split-Path $PSScriptRoot -Parent
$Source = Join-Path $Root 'upstream/OpenCPN'
$Variant = if ($Integration) { 'xnav' } else { 'pristine' }
$Evidence = Join-Path $Root 'evidence/local'
New-Item -ItemType Directory -Force $Evidence | Out-Null
Start-Transcript -Path (Join-Path $Evidence "windows-$Variant-$Architecture.log")
function Run([string]$Program, [string[]]$Arguments) {
    & $Program @Arguments 2>&1 | Tee-Object -FilePath (Join-Path $Evidence 'windows-native-output.log') -Append
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
try {
    Run python @((Join-Path $PSScriptRoot 'verify-upstream.py'))
    if (-not [Environment]::Is64BitOperatingSystem) { throw 'Windows x64 host required' }
    if ($Architecture -eq 'x64') {
        throw 'OpenCPN 5.12.4 ships Win32 dependencies. An x64 dependency and plugin ABI port is not validated; refusing to mislabel Win32 as x64.'
    }
    if ($Integration) {
        Run python @((Join-Path $PSScriptRoot 'prepare-integration.py'))
        $Source = Join-Path $Root 'build/integration-source'
    }
    Push-Location $Source
    try {
        Run cmd @('/c', 'buildwin\win_deps.bat')
    } finally { Pop-Location }
    $Wx = Join-Path $Source 'cache/wxWidgets-3.2.8'
    $Build = Join-Path $Root "build/$Variant-windows"
    $Install = Join-Path $Root "build/$Variant-install"
    $Gettext = @(
        "$env:ProgramFiles\Poedit\Gettexttools\bin",
        "${env:ProgramFiles(x86)}\Poedit\Gettexttools\bin"
    ) | Where-Object { Test-Path (Join-Path $_ 'msgfmt.exe') } | Select-Object -First 1
    if (-not $Gettext) { throw 'Poedit gettext tools not found after dependency installation' }
    $env:PATH += ";$Gettext;$Wx\lib\vc14x_dll;$Source\cache\buildwin"
    $OpenNavArgs = @()
    if ($Integration) { $OpenNavArgs = @("-DOPENNAV_ROOT=$Root") }
    Run cmake (@('-S', $Source, '-B', $Build, '-G', 'Visual Studio 17 2022',
        '-A', $Architecture, '-DCMAKE_POLICY_VERSION_MINIMUM=3.5', '-DCMAKE_BUILD_TYPE=Release',
        "-DwxWidgets_ROOT_DIR=$Wx", "-DwxWidgets_LIB_DIR=$Wx/lib/vc14x_dll",
        '-DwxWidgets_CONFIGURATION=mswu', '-DOCPN_CI_BUILD=ON',
        '-DOCPN_BUILD_TEST=ON', '-DOCPN_BUNDLE_WXDLLS=ON',
        '-DOCPN_BUNDLE_DOCS=OFF', '-DOCPN_BUNDLE_GSHHS=ON',
        '-DOCPN_BUNDLE_TCDATA=ON', "-DCMAKE_INSTALL_PREFIX=$Install") + $OpenNavArgs)
    Run cmake @('--build', $Build, '--config', 'Release', '--parallel', '2')
    Run cmake @('--install', $Build, '--config', 'Release')
    Run ctest @('--test-dir', (Join-Path $Build 'test'), '-C', 'Release', '--output-on-failure', '--no-tests=error',
        '--timeout', '90', '--output-junit', (Join-Path $Evidence 'windows-tests.xml'))
    Get-FileHash (Join-Path $Build 'Release/opencpn.exe') -Algorithm SHA256 |
        Format-List | Out-File (Join-Path $Evidence 'windows-executable-sha256.txt')
    Run python @((Join-Path $PSScriptRoot 'verify-upstream.py'))
    if ($Integration) {
        Run python @((Join-Path $PSScriptRoot 'smoke-modes-windows.py'))
        & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1') -Variant xnav -Mode legacy -Name '11-legacy-mode'
        & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1') -Variant xnav -Mode safe-mode -Name '12-safe-mode'
    } else {
        & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1')
    }
} finally { Stop-Transcript }
