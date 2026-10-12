# CI-only actual SCM enrollment smoke. The caller must use the retained broker
# built earlier in this exact component workflow on a disposable hosted runner.
# It never removes fixed files, registry keys, fences or services after failure.
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Install,
    [Parameter(Mandatory = $true)][string]$ExpectedCommit,
    [Parameter(Mandatory = $true)][string]$RunnerEnvironment,
    [Parameter(Mandatory = $true)][switch]$ConfirmDisposableHostedRunner,
    [string]$RetainedProducerReceipt
)

$ErrorActionPreference = 'Stop'
if (-not $ConfirmDisposableHostedRunner -or
    $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or
    $RunnerEnvironment -cne 'github-hosted' -or
    $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work' -or
    $ExpectedCommit -cnotmatch '^[0-9a-f]{40}$' -or
    $env:GITHUB_RUN_ID -cnotmatch '^[1-9][0-9]*$') {
    throw 'Explicit same-commit disposable GitHub-hosted Windows fixture required'
}

$checkoutCommit = $ExpectedCommit
if ($RetainedProducerReceipt) {
    $checkoutCommit = $env:GITHUB_SHA
    $selection = Join-Path $PSScriptRoot 'retained-broker-enrollment-selection.json'
    & python (Join-Path $PSScriptRoot 'retained_broker_enrollment.py') --selection $selection --receipt $RetainedProducerReceipt --validate-only --expected-producer-commit $ExpectedCommit
    if ($LASTEXITCODE -ne 0) { throw 'Actual original retained producer authentication refused' }
    $env:SKAGER_ENROLLMENT_RETAINED_PRODUCER = 'SCRUM-311-retained-original-broker'
} elseif ($ExpectedCommit -cne $env:GITHUB_SHA) {
    throw 'Same-commit producer required without independently authenticated retained selection'
}

$module = Join-Path $PSScriptRoot 'update-verifier'
$installPath = (Resolve-Path -LiteralPath $Install).ProviderPath
$producerPath = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../build/updater-native/broker-install')).ProviderPath
if ($installPath -ne $producerPath) {
    throw 'Fixture must use the retained broker-install directory from this component workflow'
}
$brokerPath = Join-Path $installPath 'skager-update-broker.exe'
$recordPath = Join-Path $installPath 'opennav/third-party/update-broker/build.json'
if (-not (Test-Path -LiteralPath $brokerPath -PathType Leaf) -or
    -not (Test-Path -LiteralPath $recordPath -PathType Leaf)) {
    throw 'Retained same-commit broker producer outputs required'
}
$head = (& git -C (Join-Path $PSScriptRoot '..') rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or $head -cne $checkoutCommit) {
    throw 'Fixture checkout differs from its actual helper commit'
}

# Existing source-package verifier checks exact Go build identity, Win32 GUI PE,
# source archive and producer-record hash before any machine creation.
& python -c 'import sys; from pathlib import Path; sys.path.insert(0, sys.argv[1]); from broker_package import verify_broker_package; verify_broker_package(Path(sys.argv[2]), sys.argv[3])' $PSScriptRoot $installPath $ExpectedCommit
if ($LASTEXITCODE -ne 0) { throw 'Retained broker producer verification failed' }
$record = Get-Content -LiteralPath $recordPath -Raw | ConvertFrom-Json
if ($record.productCommit -cne $ExpectedCommit -or
    $record.binary.path -cne 'skager-update-broker.exe' -or
    $record.binary.sha256 -cnotmatch '^[0-9a-f]{64}$') {
    throw 'Retained broker producer record differs'
}

$env:SKAGER_ENROLLMENT_NATIVE_OPT_IN = 'SCRUM-311-disposable-hosted-enrollment'
$env:SKAGER_ENROLLMENT_RUNNER_ENVIRONMENT = $RunnerEnvironment
$env:SKAGER_ENROLLMENT_PRODUCER_COMMIT = $ExpectedCommit
$env:SKAGER_ENROLLMENT_BROKER_PATH = $brokerPath
$env:SKAGER_ENROLLMENT_BROKER_SHA256 = $record.binary.sha256
$env:GOARCH = '386'
$env:GOTOOLCHAIN = 'local'
$env:GOWORK = 'off'
Push-Location $module
try {
    & go test -count=1 -v ./brokerenroll -run '^(TestNativeFreshEnrollmentServiceSmoke|TestServiceMutationDiagnosticPreservesOriginalDescriptorAndBoundary|TestServiceManagerAndServiceMutationRightsRemainDistinct|TestServiceRegistryCreatorOwnerTemplateIsNotPrincipalAuthority)$'
    if ($LASTEXITCODE -ne 0) { throw 'Native fresh enrollment acceptance failed; fixed machine state retained for review' }
} finally {
    Pop-Location
}
