param(
    [Parameter(Mandatory=$true)][string]$TestExecutable,
    [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedSha256,
    [Parameter(Mandatory=$true)][string]$EvidenceDirectory,
    [Parameter(Mandatory=$true)][ValidateSet('ClientStore','ClientRequest','StockCleanup')][string]$Group
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# SCRUM-311: exact reviewed inert native fixtures only. The supplied hash must
# independently identify the reviewed test binary; this runner cannot establish
# its provenance. Never provision/start a service, application, profile, network
# session or physical output. Native fixture success is not release acceptance.
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'Native Windows fixture execution required'
}
if ($ExpectedSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Exact lowercase test SHA-256 required' }

switch ($Group) {
    'ClientStore' {
        $Group = 'ClientStore'
        $CaseNames = @(
            'TestNativeClientRegistrationFinalDescriptorsAbsentStateAndCompletionReadback',
            'TestNativeClientPartialCreationRetainsStartedMarkerAndRefusesRetry',
            'TestNativeClientCompleteRefusesMissingOrUnsafeChannel',
            'TestNativeClientCompletedBytesTamperingAndHardLinksRefuse',
            'TestNativeClientExistingRegistrationNeverTruncatedOrAdopted',
            'TestNativeClientStartedAndMalformedCompletionRefuse',
            'TestNativeClientExistingSIDTreeNeverAdopted',
            'TestNativeClientSIDRequiresCanonicalActualUser',
            'TestNativeClientCancellationDuringCommittedMarkerRemainsReadableAfterReopen'
        )
    }
    'ClientRequest' {
        $Group = 'ClientRequest'
        $CaseNames = @(
            'TestClientRequestNativeReadonlyCustody',
            'TestClientRequestDescriptorExcludesClientAndBrokerMutation',
            'TestClientRequestNativeRefusesOperatorConsoleFallback',
            'TestClientServiceArgumentsRefuseBeforeNativeStart'
        )
    }
    'StockCleanup' {
        $Group = 'StockCleanup'
        $CaseNames = @('TestNativeStockCloseFailureIsRetained')
    }
}
$Selection = '^(' + ($CaseNames -join '|') + ')$'
$Utf8 = New-Object System.Text.UTF8Encoding($false)

function Assert-LocalPlainAncestry([string]$Path) {
    if ($Path -cnotmatch '^[A-Za-z]:\\' -or $Path.Substring(2).Contains(':')) {
        throw 'Local DOS path without alternate streams required'
    }
    $Root = [IO.Path]::GetPathRoot($Path)
    $Drive = New-Object System.IO.DriveInfo($Root)
    if ($Drive.DriveType -ne [IO.DriveType]::Fixed) { throw 'Local fixed disk required' }
    $Current = $Path
    while ($Current) {
        $Item = Get-Item -LiteralPath $Current -Force
        if (-not $Item.PSIsContainer -or ($Item.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
            throw 'Plain existing directory ancestry required'
        }
        if ($Current.TrimEnd('\') -ieq $Root.TrimEnd('\')) { break }
        $Current = [IO.Path]::GetDirectoryName($Current.TrimEnd('\'))
    }
}

function Assert-ExactNames([string[]]$Observed, [string[]]$Expected, [string]$Phase) {
    if (@($Observed).Count -ne @($Expected).Count) { throw ($Phase + ': exact nonempty case count required') }
    foreach ($Name in $Expected) {
        if ($Observed -cnotcontains $Name) { throw ($Phase + ': missing exact case ' + $Name) }
    }
    foreach ($Name in $Observed) {
        if ($Expected -cnotcontains $Name) { throw ($Phase + ': unexpected case ' + $Name) }
    }
}

function Write-JsonEvidence([string]$Path, $Value) {
    [IO.File]::WriteAllText($Path, (($Value | ConvertTo-Json -Depth 8) + [Environment]::NewLine), $Utf8)
}

function Invoke-OwnedTest([string]$Phase, [string[]]$Arguments, [int]$Milliseconds) {
    $Out = Join-Path $Evidence.FullName ($Phase + '-stdout.log')
    $Err = Join-Path $Evidence.FullName ($Phase + '-stderr.log')
    $Proof = [ordered]@{
        arguments = $Arguments; processId = $null; startedUtc = [DateTime]::UtcNow.ToString('o')
        exitCode = $null; timedOut = $false; error = $null; stdout = $Out; stderr = $Err
    }
    $Process = New-Object System.Diagnostics.Process
    $Process.StartInfo.FileName = $Executable.FullName
    # Every argument is a fixed Go flag or a regex built only from the named
    # allowlist. There are no spaces/shell syntax supplied by a caller.
    $Process.StartInfo.Arguments = $Arguments -join ' '
    $Process.StartInfo.WorkingDirectory = $Scratch.FullName
    $Process.StartInfo.UseShellExecute = $false
    $Process.StartInfo.CreateNoWindow = $true
    $Process.StartInfo.RedirectStandardOutput = $true
    $Process.StartInfo.RedirectStandardError = $true
    $Started = $false
    $Watch = [Diagnostics.Stopwatch]::StartNew()
    try {
        if (-not $Process.Start()) { throw 'Owned fixture process could not start' }
        $Started = $true
        # Retain this exact process handle before waiting, including fast exits.
        # Never reopen by PID/name, terminate a process tree or retry a test.
        $null = $Process.Handle
        $Proof.processId = $Process.Id
        $OutputTask = $Process.StandardOutput.ReadToEndAsync()
        $ErrorTask = $Process.StandardError.ReadToEndAsync()
        if (-not $Process.WaitForExit($Milliseconds)) {
            $Proof.timedOut = $true
            $Process.Kill() # Only this runner's retained inert test process.
            if (-not $Process.WaitForExit(5000)) { throw 'Owned timed-out fixture did not exit within cleanup bound' }
        }
        $Proof.exitCode = $Process.ExitCode
        # Pipe draining also has a deadline; never use an unbounded Wait/Result
        # on an incomplete task or accept missing exit status as success.
        if (-not [Threading.Tasks.Task]::WaitAll([Threading.Tasks.Task[]]@($OutputTask, $ErrorTask), 5000)) {
            throw 'Owned fixture output did not drain within bound'
        }
        [IO.File]::WriteAllText($Out, $OutputTask.GetAwaiter().GetResult(), $Utf8)
        [IO.File]::WriteAllText($Err, $ErrorTask.GetAwaiter().GetResult(), $Utf8)
    } catch {
        $Proof.error = $_.Exception.Message
    } finally {
        try {
            if ($Started -and -not $Process.HasExited) {
                $Remaining = [Math]::Max(0, $Milliseconds - [int]$Watch.ElapsedMilliseconds)
                if ($Remaining -gt 0) { $null = $Process.WaitForExit($Remaining) }
                if (-not $Process.HasExited) {
                    $Proof.timedOut = $true
                    $Process.Kill()
                    if (-not $Process.WaitForExit(5000)) { $Proof.error = 'Owned fixture exit remains uncertain after deadline' }
                }
            }
        } catch {
            $Proof.error = 'Owned fixture deadline cleanup uncertain: ' + $_.Exception.Message
        }
        $Watch.Stop()
        $Proof.elapsedMilliseconds = $Watch.ElapsedMilliseconds
        $Process.Dispose()
    }
    return $Proof
}

$ExecutablePath = [IO.Path]::GetFullPath($TestExecutable)
if ($ExecutablePath -cnotmatch '^[A-Za-z]:\\' -or $ExecutablePath.Substring(2).Contains(':')) {
    throw 'Plain local test executable path required'
}
Assert-LocalPlainAncestry ([IO.Path]::GetDirectoryName($ExecutablePath))
$Executable = Get-Item -LiteralPath $ExecutablePath -Force
if ($Executable.PSIsContainer -or ($Executable.Attributes -band [IO.FileAttributes]::ReparsePoint) -or
    [IO.Path]::GetExtension($Executable.FullName) -ine '.exe') { throw 'Plain retained test executable required' }
$EvidencePath = [IO.Path]::GetFullPath($EvidenceDirectory)
if ($EvidencePath -cnotmatch '^[A-Za-z]:\\' -or $EvidencePath.Substring(2).Contains(':')) {
    throw 'Plain local evidence directory path required'
}
Assert-LocalPlainAncestry ([IO.Path]::GetDirectoryName($EvidencePath))
if (Test-Path -LiteralPath $EvidencePath) { throw 'Fresh fixture evidence directory required' }

# Hold a read owner denying write/delete throughout hash verification and both
# invocations. This is fixture custody, not native broker installation authority.
$HeldExecutable = [IO.File]::Open($Executable.FullName, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read)
$OriginalTemp = [Environment]::GetEnvironmentVariable('TEMP', 'Process')
$OriginalTmp = [Environment]::GetEnvironmentVariable('TMP', 'Process')
$Receipt = $null
$Evidence = $null
try {
    $Hasher = [Security.Cryptography.SHA256]::Create()
    try { $Hash = [BitConverter]::ToString($Hasher.ComputeHash($HeldExecutable)).Replace('-', '').ToLowerInvariant() }
    finally { $Hasher.Dispose() }
    if ($Hash -cne $ExpectedSha256) { throw 'Retained test executable hash mismatch' }
    $Evidence = New-Item -ItemType Directory -Path $EvidencePath
    $Scratch = New-Item -ItemType Directory -Path (Join-Path $Evidence.FullName 'temporary-files')
    $Cases = New-Item -ItemType Directory -Path (Join-Path $Evidence.FullName 'cases')
    $Receipt = [ordered]@{
        schema = 1; group = $Group; executable = $Executable.FullName; executableSha256 = $Hash
        expectedCases = $CaseNames; selection = $Selection; status = 'not-completed'; failure = $null
        startedUtc = [DateTime]::UtcNow.ToString('o'); os = [Environment]::OSVersion.VersionString
        os64 = [Environment]::Is64BitOperatingSystem; process64 = [Environment]::Is64BitProcess
        powershellVersion = $PSVersionTable.PSVersion.ToString(); listing = $null; execution = $null
        scope = 'Exact inert native fixtures only; no application/service/profile/network/physical output; not enrollment or release qualification'
    }
    [Environment]::SetEnvironmentVariable('TEMP', $Scratch.FullName, 'Process')
    [Environment]::SetEnvironmentVariable('TMP', $Scratch.FullName, 'Process')

    $ListArguments = @("-test.list=$Selection")
    $Listing = Invoke-OwnedTest -Phase 'list' -Arguments $ListArguments -Milliseconds 15000
    $Receipt.listing = $Listing
    if ($Listing.timedOut -or $Listing.error -or $null -eq $Listing.exitCode -or $Listing.exitCode -ne 0) {
        throw 'Native fixture discovery did not complete successfully'
    }
    $Listed = @([IO.File]::ReadAllLines($Listing.stdout) | Where-Object { $_ -cne '' })
    if ([IO.File]::ReadAllText($Listing.stderr).Trim() -cne '') { throw 'Unexpected native fixture discovery stderr' }
    Assert-ExactNames $Listed $CaseNames 'Discovery'

    $RunArguments = @("-test.run=$Selection", '-test.count=1', '-test.v', '-test.timeout=90s')
    $Execution = Invoke-OwnedTest -Phase 'tests' -Arguments $RunArguments -Milliseconds 100000
    $Receipt.execution = $Execution
    $Lines = @()
    $StandardOutput, $StandardError = '', ''
    if ([IO.File]::Exists($Execution.stdout)) { $Lines = [IO.File]::ReadAllLines($Execution.stdout); $StandardOutput = [IO.File]::ReadAllText($Execution.stdout) }
    if ([IO.File]::Exists($Execution.stderr)) { $StandardError = [IO.File]::ReadAllText($Execution.stderr) }
    $Combined = $StandardOutput + [Environment]::NewLine + $StandardError
    $Passed = @([regex]::Matches($Combined, '(?m)^--- PASS: ([A-Za-z0-9_]+) \([0-9]+(?:\.[0-9]+)?s\)\r?$') | ForEach-Object { $_.Groups[1].Value })
    $Ran = @([regex]::Matches($Combined, '(?m)^=== RUN   ([A-Za-z0-9_]+)\r?$') | ForEach-Object { $_.Groups[1].Value })
    # Preserve per-case observations even when another case fails or the owned
    # process times out. Only the final group receipt can assert full success.
    foreach ($Name in $CaseNames) {
        $CaseLines = New-Object 'System.Collections.Generic.List[string]'
        $Inside = $false
        foreach ($Line in $Lines) {
            if ($Line -cmatch '^=== RUN   ([A-Za-z0-9_]+)$') { $Inside = ($Matches[1] -ceq $Name) }
            if ($Inside) { $CaseLines.Add($Line) }
        }
        [IO.File]::WriteAllLines((Join-Path $Cases.FullName ($Name + '.log')), $CaseLines.ToArray(), $Utf8)
        Write-JsonEvidence (Join-Path $Cases.FullName ($Name + '.json')) ([ordered]@{
            schema = 1; group = $Group; name = $Name; selection = '^' + $Name + '$'
            runMarkers = @($Ran | Where-Object { $_ -ceq $Name }).Count
            passMarkers = @($Passed | Where-Object { $_ -ceq $Name }).Count
            executableSha256 = $Hash; exitCode = $Execution.exitCode; os = $Receipt.os
            timedOut = $Execution.timedOut; groupReceipt = (Join-Path $Evidence.FullName 'receipt.json')
            executionLog = $Execution.stdout; caseLog = $Name + '.log'
        })
    }
    if ($Execution.timedOut -or $Execution.error -or $null -eq $Execution.exitCode -or $Execution.exitCode -ne 0) {
        throw 'Native fixture execution did not complete successfully'
    }
    if ($Combined -cmatch '(?m)^[\t ]*--- (SKIP|FAIL):|^FAIL(?:\s|$)|^testing: warning: no tests to run') {
        throw 'Skipped, failed or empty native fixture execution refused'
    }
    if ($StandardError.Trim() -cne '') { throw 'Unexpected native fixture execution stderr' }
    Assert-ExactNames $Ran $CaseNames 'Execution RUN'
    Assert-ExactNames $Passed $CaseNames 'Execution PASS'
    if ([regex]::Matches($Combined, '(?m)^PASS\r?$').Count -ne 1) { throw 'Exact successful Go test summary required' }
    # The executable remained open without write/delete sharing. Recheck the
    # selected spelling/hash too; a changed artifact never earns a passed receipt.
    $After = Get-Item -LiteralPath $Executable.FullName -Force
    if (($After.Attributes -band [IO.FileAttributes]::ReparsePoint) -or
        (Get-FileHash -LiteralPath $After.FullName -Algorithm SHA256).Hash.ToLowerInvariant() -cne $Hash) {
        throw 'Retained native fixture executable changed'
    }
    $Receipt.status = 'passed'
} catch {
    if ($null -ne $Receipt) { $Receipt.status = 'failed'; $Receipt.failure = $_.Exception.Message }
    throw
} finally {
    [Environment]::SetEnvironmentVariable('TEMP', $OriginalTemp, 'Process')
    [Environment]::SetEnvironmentVariable('TMP', $OriginalTmp, 'Process')
    try {
        $HeldExecutable.Dispose()
    } catch {
        if ($null -ne $Receipt) { $Receipt.status = 'failed'; $Receipt.failure = 'Retained test executable cleanup uncertain' }
        throw
    } finally {
        if ($null -ne $Receipt) {
            $Receipt.completedUtc = [DateTime]::UtcNow.ToString('o')
            Write-JsonEvidence (Join-Path $Evidence.FullName 'receipt.json') $Receipt
        }
    }
}
