param(
    [Parameter(Mandatory=$true)][string]$TestExecutable,
    [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedSha256,
    [Parameter(Mandatory=$true)][string]$EvidenceDirectory
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# Only the bounded store-lock fixtures run. They use disposable directories and
# an inert copy of this test executable, never the application or SCM service.
$Executable = Get-Item -LiteralPath $TestExecutable -Force
if ($Executable.PSIsContainer -or ($Executable.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
    throw 'Plain retained test executable required'
}
$Hash = (Get-FileHash -LiteralPath $Executable.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
if ($Hash -cne $ExpectedSha256) { throw 'Retained test executable hash mismatch' }
if (Test-Path -LiteralPath $EvidenceDirectory) { throw 'Fresh fixture evidence directory required' }
$Evidence = New-Item -ItemType Directory -Path $EvidenceDirectory
$Scratch = New-Item -ItemType Directory -Path (Join-Path $Evidence.FullName 'temporary-files')
$OriginalTemp, $OriginalTmp = $env:TEMP, $env:TMP
try {
    $env:TEMP, $env:TMP = $Scratch.FullName, $Scratch.FullName
    $Selection = '^TestNativeStoreLifetimeLock(ExcludesOtherOwnerAndReacquires|ExcludesAnotherProcess|RefusesUnsafeExistingObjects|RechecksHeldMetadataAndReleasesAfterCleanupError)$'
    # Windows PowerShell 5.1 splits bare dotted native arguments differently.
    # Keep every Go flag in the string array used by both PowerShell versions.
    $Arguments = @("-test.run=$Selection", '-test.count=1', '-test.v', '-test.timeout=90s')
    & $Executable.FullName @Arguments 2>&1 |
        Tee-Object -FilePath (Join-Path $Evidence.FullName 'native-tests.log')
    $TestExit = $LASTEXITCODE
    [ordered]@{
        schema = 1
        executableSha256 = $Hash
        selection = $Selection
        exitCode = $TestExit
        os64 = [Environment]::Is64BitOperatingSystem
        powershellVersion = $PSVersionTable.PSVersion.ToString()
        scope = 'Native lock primitive only; no application, service, profile or hardware operation'
    } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $Evidence.FullName 'receipt.json') -Encoding UTF8
    if ($TestExit -ne 0) { throw ('Native fixture failed: ' + $TestExit) }
} finally {
    $env:TEMP, $env:TMP = $OriginalTemp, $OriginalTmp
}
