# Inert evidence-only fixture: execute the driver's actual setup statements,
# never the build driver, native tools, dependency producers or network checks.
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Driver = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'build-pristine-windows.ps1'))
$Probe = [IO.File]::ReadAllText((Join-Path $PSScriptRoot 'verify-curl-bundle.ps1'))
function Section([string]$Text, [string]$First, [string]$Last) {
    $Start = $Text.IndexOf($First, [StringComparison]::Ordinal)
    $End = $Text.IndexOf($Last, $Start, [StringComparison]::Ordinal)
    if ($Start -lt 0 -or $End -le $Start) { throw 'Production evidence setup boundary changed' }
    $Text.Substring($Start, $End - $Start)
}
function Statement([string]$Text, [string]$Prefix) {
    $Lines = @($Text -split '\r?\n' | Where-Object { $_.TrimStart().StartsWith($Prefix, [StringComparison]::Ordinal) })
    if ($Lines.Count -ne 1) { throw "Ambiguous production statement: $Prefix" }
    $Lines[0]
}
$Setup = Section $Driver '$Evidence = Join-Path' '# Capture the entry interpreter'
$RuntimeSetup = Section $Probe '$Evidence = Join-Path' '$OpenSslPrefix ='
$PythonWrite = Statement $Driver '$PythonIdentityJson | Set-Content'
$GettextPath = Statement $Driver '$GettextReceipt ='
$PreflightPath = Statement $Driver '$CurlPreflight ='
$FactsPath = Statement $Probe '$Facts ='
$Root = Join-Path ([IO.Path]::GetTempPath()) ('reprobe-evidence-test-' + [guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $Root
function Exercise([bool]$VerifyDependencyBundleOnly, [string]$Variant) {
    $Architecture = 'Win32'
    $PythonIdentityJson = '{"inert":"interpreter fixture"}'
    Invoke-Expression $Setup | Out-Null
    try {
        Invoke-Expression $PythonWrite
        Invoke-Expression $GettextPath
        [IO.File]::WriteAllText($GettextReceipt, '{"inert":"gettext fixture"}')
        Invoke-Expression $PreflightPath
        if (Test-Path -LiteralPath $CurlPreflight) { throw 'Preflight diagnostic would overwrite earlier evidence' }
        $null = New-Item -ItemType Directory -Path $CurlPreflight
        [IO.File]::WriteAllText((Join-Path $CurlPreflight 'source-preflight.json'), '{"inert":true}')
        $DriverEvidence = $Evidence
        $Runtime = & {
            $RuntimeEvidenceDirectory = $DriverEvidence
            Invoke-Expression $RuntimeSetup
            Invoke-Expression $FactsPath
            if ($Facts -cne (Join-Path $Root 'evidence/local/windows-curl-parent-tool-facts.json')) {
                throw 'Producer tool-fact read was relocated'
            }
            if (Split-Path $RuntimeReport -Parent | Where-Object { $_ -cne $DriverEvidence }) {
                throw 'Consumer runtime observation escaped invocation evidence'
            }
            [IO.File]::WriteAllText($RuntimeReport, '{"inert":"runtime observation"}')
            $RuntimeReport
        }
        [pscustomobject]@{evidence=$DriverEvidence;runtime=$Runtime;preflight=$CurlPreflight}
    } finally { Stop-Transcript | Out-Null }
}
try {
    $Initial = Exercise $false 'xnav'
    $Facts = Join-Path $Root 'evidence/local/windows-curl-parent-tool-facts.json'
    [IO.File]::WriteAllText($Facts, '{"immutable":"original producer facts"}')
    $Original = @{}
    Get-ChildItem -LiteralPath (Join-Path $Root 'evidence/local') -File -Recurse | ForEach-Object {
        $Original[$_.FullName] = (Get-FileHash -LiteralPath $_.FullName).Hash
    }
    $First = Exercise $true 'xnav'
    $Second = Exercise $true 'xnav'
    $Production = Exercise $false 'production'
    if ($Initial.evidence -cne $Production.evidence -or
        $First.evidence -ceq $Second.evidence -or $First.evidence -ceq $Initial.evidence) {
        throw 'Reprobe namespace was reused or normal producer path changed'
    }
    foreach ($Path in $Original.Keys) {
        if ((Get-FileHash -LiteralPath $Path).Hash -cne $Original[$Path]) {
            throw "Original producer/application evidence changed: $Path"
        }
    }
    Write-Output 'Passed: initial application evidence, two fresh reprobes, normal production evidence, unchanged original bytes and tool-fact read path'
} finally { Remove-Item -LiteralPath $Root -Recurse -Force }
