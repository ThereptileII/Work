param([string]$ProductCommit = '', [switch]$ExtendedTests, [switch]$CompiledRetest)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ($env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted') {
    throw 'Staging qualification is restricted to disposable native CI desktops'
}
$Root = Split-Path $PSScriptRoot -Parent
if (-not $ProductCommit) { $ProductCommit = $env:GITHUB_SHA }
if ($ProductCommit -cnotmatch '^[0-9a-f]{40}$') { throw 'Exact product commit required' }
function Check([string]$Script, [string[]]$Arguments = @()) {
    & python (Join-Path $PSScriptRoot $Script) @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Staging qualification failed: $Script" }
}
# Run the same thirteen compiled components only after immutable input restore.
# The runner verifies the original manifest and each fixed executable against the
# restore receipt, rebasing paths while keeping producer/harness identities distinct.
Check 'test-boat-feedback-widgets.py' @('--manifest', "$Root/build/xnav-windows/Release/boat-feedback-tests.json",
    '--output', "$Root/evidence/local/boat-feedback-windows", '--runtime-dir', "$Root/build/xnav-install",
    '--expected-commit', $ProductCommit, '--compiled-input-receipt', "$Root/evidence/local/staging-inputs.json")
# These are the existing native application gates, moved after immutable
# compile/package retention. This script contains no compiler/package producer.
Check 'smoke-installer-selftest.py' @('--expect-test-loopback', '--app', "$Root/build/xnav-install/opencpn.exe")
Check 'smoke-modes-windows.py'
Check 'smoke-navigation.py'
Check 'smoke-navigation.py' @('--route-fixture')
Check 'smoke-navigation.py' @('--route-fixture-standard')
Check 'smoke-navigation.py' @('--instruments')
Check 'smoke-navigation.py' @('--n2k')
Check 'smoke-navigation.py' @('--boat')
Check 'smoke-signalk.py'
Check 'smoke-recording.py'
Check 'smoke-pilot.py'
Check 'smoke-navigation.py' @('--objects')
Check 'smoke-recovery.py'
Check 'smoke-user-flows.py'
Check 'smoke-pilot.py' @('--production')
Check 'smoke-portable-production.py' @('--package', "$Root/build/developer-preview/SKAGER-Beta2-Portable-Recovery.zip", '--expected-commit', $ProductCommit)
$InstallerArguments = @('--mode', 'staging')
if ($CompiledRetest) { $InstallerArguments += @('--compiled-input-receipt', "$Root/evidence/local/staging-inputs.json") }
Check 'smoke-installer-windows.py' $InstallerArguments
Check 'smoke-charts.py'
if ($ExtendedTests) {
    foreach ($Attempt in 1..2) {
        $Capture = "$Root/evidence/local/recovery-repeat-$Attempt"
        New-Item -ItemType Directory -Path $Capture | Out-Null
        Get-ChildItem "$Root/evidence/local" -Filter 'recovery-*' | Where-Object { $_.Name -notlike 'recovery-repeat-*' } | Move-Item -Destination $Capture
        Check 'smoke-recovery.py'
    }
    & (Join-Path $PSScriptRoot 'test-preview-windows.ps1')
    if ($LASTEXITCODE -ne 0) { throw 'Extended fixture checks failed' }
}
if ($env:SKAGER_DESIGN_VALIDATION -ceq 'true') {
    $OriginalPath = $env:PATH
    try {
    foreach ($Variant in @('xnav', 'production')) {
        $Build = "$Root/build/$Variant-windows/Release"
        $env:PATH = "$Root/build/$Variant-install;$env:WINDIR/System32;$env:WINDIR"
        foreach ($Painter in @('chart_name_text_test','chart_light_label_test','skager_wordmark_test','chart_route_label_test','onboard_ais_body_test')) {
            & "$Build/$Painter.exe" "$Root/evidence/local/$Painter-$Variant.png"
            if ($LASTEXITCODE -ne 0) { throw "Requested design painter failed: $Painter" }
        }
        & "$Build/ui_font_resolution_test.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Requested font-resolution check failed' }
    }
    } finally { $env:PATH = $OriginalPath }
    & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1') -Variant xnav -Mode legacy -Name '11-legacy-mode'
    & (Join-Path $PSScriptRoot 'capture-pristine-windows.ps1') -Variant xnav -Mode safe-mode -Name '12-safe-mode'
    Check 'smoke-dpi-windows.py'
}
