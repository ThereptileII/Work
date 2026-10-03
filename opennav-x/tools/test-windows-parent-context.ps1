param([string]$Evidence)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or $env:GITHUB_ACTIONS -cne 'true') {
    throw 'This bounded native parent-context proof requires a disposable Windows Actions runner'
}
$Root = Split-Path $PSScriptRoot -Parent
if (-not $Evidence) { $Evidence = Join-Path $Root 'evidence/local/windows-parent-context' }
$null = New-Item -ItemType Directory -Force -Path $Evidence
$Evidence = (Resolve-Path -LiteralPath $Evidence).Path
$InitialPath = $env:PATH
$Report = [ordered]@{
    schemaVersion = 1; status = 'failed'; stages = @(); inputs = @()
    scope = 'Native OpenSSL parent context only; no producer build, AIS configure/link/TLS, or x86 child acceptance'
}
function Digest([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function TextDigest([string]$Text) {
    $Hasher = [Security.Cryptography.SHA256]::Create()
    try { ([BitConverter]::ToString($Hasher.ComputeHash([Text.Encoding]::UTF8.GetBytes($Text)))).Replace('-','').ToLowerInvariant() }
    finally { $Hasher.Dispose() }
}
function FileFact([string]$Path) {
    [ordered]@{ path = (Resolve-Path -LiteralPath $Path).Path; bytes = (Get-Item -LiteralPath $Path).Length; sha256 = Digest $Path }
}
function Stage([string]$Name) {
    $Report.stages += $Name
    Write-Output $Name
}
function Get-ProducerSetup([string]$Text) {
    # Exact maintained producer code, excluding source acquisition and all build/
    # capture/child paths. LF normalization is solely for checkout portability.
    $Text = $Text.Replace("`r`n", "`n")
    $Start = "foreach (`$Directory in @('C:\Program Files\NASM','C:\Strawberry\perl\bin')) {"
    $End = "if (-not `$VerifyToolFactsOnly) {`n    & `$ToolFacts -Mode Capture"
    if ([regex]::Matches($Text, [regex]::Escape($Start)).Count -ne 1 -or
        [regex]::Matches($Text, [regex]::Escape($End)).Count -ne 1) {
        throw 'Producer-local setup boundaries changed'
    }
    $First = $Text.IndexOf($Start, [StringComparison]::Ordinal)
    $Last = $Text.IndexOf($End, [StringComparison]::Ordinal)
    if ($Last -le $First) { throw 'Producer-local setup boundaries reversed' }
    $Span = $Text.Substring($First, $Last - $First)
    if ((TextDigest $Span) -cne 'dc6b184a18f728b98db4cf0ac522b4edb5495cffc8a037eacc3194483fd59749') {
        throw 'Producer-local setup differs from the reviewed bounded span'
    }
    return $Span
}
function Assert-OnlyPathDifference([string]$Captured, [string]$Observed) {
    $Before = ($Captured | ConvertFrom-Json).environment.PATHSha256
    $After = ($Observed | ConvertFrom-Json).environment.PATHSha256
    if ($Before -cnotmatch '^[0-9a-f]{64}$' -or $After -cnotmatch '^[0-9a-f]{64}$' -or $Before -ceq $After) {
        throw 'Negative control did not change a valid PATHSha256'
    }
    $Needle = '"PATHSha256":"' + $After + '"'
    if ([regex]::Matches($Observed, [regex]::Escape($Needle)).Count -ne 1 -or
        $Observed.Replace($Needle, ('"PATHSha256":"' + $Before + '"')) -cne $Captured) {
        throw 'Negative control changed facts besides PATHSha256'
    }
}
try {
    $Producer = Join-Path $PSScriptRoot 'build-openssl-windows.ps1'
    $ToolFacts = Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1'
    $Initializer = Join-Path $PSScriptRoot 'windows-parent-environment.ps1'
    $LockPath = Join-Path $PSScriptRoot 'windows-openssl.lock.json'
    $Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
    $Downloads = Join-Path $Root 'build/dependency-downloads'
    # The exact producer span may only verify already acquired NASM, never fetch.
    $VerifyToolFactsOnly = $true
    $Span = Get-ProducerSetup ([IO.File]::ReadAllText($Producer))
    [IO.File]::WriteAllText((Join-Path $Evidence 'producer-local-setup.ps1'), $Span, (New-Object Text.UTF8Encoding($false)))
    $Setup = [scriptblock]::Create($Span)
    $Report.producerSetupSha256 = TextDigest $Span
    $Workflow = Join-Path $Root '.github/workflows/skager-parent-context.yml'
    if (-not (Test-Path -LiteralPath $Workflow -PathType Leaf)) {
        $Workflow = Join-Path (Split-Path $Root -Parent) '.github/workflows/skager-parent-context.yml'
    }
    foreach ($InputPath in @($PSCommandPath,$Producer,$ToolFacts,$Initializer,$LockPath,
        (Join-Path $PSScriptRoot 'windows_gettext.py'),$Workflow,(Join-Path $Evidence 'nasm-source.json'))) {
        $Report.inputs += FileFact $InputPath
    }
    $Commit = (& git -C $Root rev-parse HEAD | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or $Commit -cne $env:GITHUB_SHA) { throw 'Proof checkout differs from the dispatched commit' }
    $Report.commit = $Commit
    $Python = (Get-Command python.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    $Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    $VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $VisualStudio) { throw 'Actual MSVC installation selection failed' }
    $Receipt = Join-Path $Evidence 'windows-gettext-xnav.json'
    $Capture = Join-Path $Evidence 'captured-parent.json'
    if (Test-Path -LiteralPath $Capture) { throw 'Refusing to replace an earlier parent capture' }
    $FactsArguments = @{ Kind='openssl-parent'; Output=$Capture; ProducerScript=$Producer; Vswhere=$Vswhere; VisualStudio=$VisualStudio }
    . $Initializer
    Initialize-WindowsNativePerl
    $NativePrefix = Split-Path (Resolve-Path -LiteralPath $env:SKAGER_NATIVE_PERL).Path -Parent
    $Gettext = Initialize-WindowsGettext -Python $Python -Receipt $Receipt -Mode Ensure -Log (Join-Path $Evidence 'gettext.log')
    if ($env:PATH -cne "$Gettext;$NativePrefix;$InitialPath") { throw 'Shared initialization changed more than its ordered prefixes' }
    $ReceiptHash = Digest $Receipt
    . $Setup
    $CapturedPath = $env:PATH
    & $ToolFacts -Mode Capture @FactsArguments
    $CapturedText = [IO.File]::ReadAllText($Capture)
    $CaptureHash = Digest $Capture
    $Report.captured = FileFact $Capture
    $Report.gettext = FileFact $Receipt
    $Report.selectedPython = FileFact $Python
    $Report.selectedNativePerl = FileFact $env:SKAGER_NATIVE_PERL
    $Report.selectedCurlTestPerl = FileFact $env:SKAGER_CURL_TEST_PERL
    $Report.sharedPrefixes = @($Gettext,$NativePrefix)
    $Report.inheritedPathSha256 = TextDigest $InitialPath
    & $ToolFacts -Mode Verify @FactsArguments
    Stage 'Original initialized context captured once and verified with actual tools'

    # Reproduce the original omission only; producer-local resolution is identical.
    $env:PATH = $InitialPath
    . $Setup
    $Rejected = $false
    try { & $ToolFacts -Mode Verify @FactsArguments }
    catch {
        if ($_.Exception.Message -cne 'Native tool facts changed: openssl-parent') { throw }
        $Rejected = $true
        [IO.File]::WriteAllText((Join-Path $Evidence 'expected-rejection.txt'), $_.Exception.Message)
    }
    if (-not $Rejected) { throw 'Missing shared prefixes were unexpectedly accepted' }
    $Observed = "$Capture.observed.json"
    Assert-OnlyPathDifference $CapturedText ([IO.File]::ReadAllText($Observed))
    if ((Digest $Capture) -cne $CaptureHash -or (Digest $Receipt) -cne $ReceiptHash) { throw 'Original capture or Gettext receipt changed during rejection' }
    $ObservedHash = Digest $Observed
    $Report.negative = FileFact $Observed
    $Report.changedFields = @('environment.PATHSha256')
    Stage 'Omitted-prefix control rejected; only PATHSha256 differs, including identical selected tool paths and hashes'

    $env:PATH = $InitialPath
    Initialize-WindowsNativePerl
    $RestoredGettext = Initialize-WindowsGettext -Python $Python -Receipt $Receipt -Mode Verify -Log (Join-Path $Evidence 'gettext.log')
    if ($RestoredGettext -cne $Gettext -or $env:PATH -cne "$Gettext;$NativePrefix;$InitialPath") { throw 'Shared restoration differs from the original prefix context' }
    . $Setup
    if ($env:PATH -cne $CapturedPath) { throw 'Restored producer parent PATH is not exactly the captured PATH' }
    & $ToolFacts -Mode Verify @FactsArguments
    if ((Digest $Capture) -cne $CaptureHash -or (Digest $Observed) -cne $ObservedHash -or
        (Digest $Receipt) -cne $ReceiptHash) { throw 'Retained original, negative, or Gettext receipt changed during restoration' }
    Stage 'Shared Verify initialization restored exact original facts without replacing either retained JSON'
    $Report.status = 'passed'
} catch {
    $Report.error = $_.Exception.Message
    [IO.File]::WriteAllText((Join-Path $Evidence 'failure.txt'), ($_ | Out-String))
    throw
} finally {
    $env:PATH = $InitialPath
    $Report.pathRestoredOnExit = $env:PATH -ceq $InitialPath
    $Report | ConvertTo-Json -Depth 16 | Set-Content -LiteralPath (Join-Path $Evidence 'result.json') -Encoding utf8
}
