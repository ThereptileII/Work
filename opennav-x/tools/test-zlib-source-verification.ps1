param([string]$ConsumerScript,[string]$ObservedNativeManifest)
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$Script = Join-Path $PSScriptRoot 'build-zlib-windows.ps1'
$Lock = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'windows-zlib.lock.json') -Raw | ConvertFrom-Json
if ($Lock.version -cne '1.3.2' -or $Lock.configuration -cne 'Win32 shared' -or
    $Lock.runtime -cne 'MultiThreadedDLL (/MD)' -or
    $Lock.archive -cne 'zlib-1.3.2.tar.gz' -or
    $Lock.url -cne 'https://github.com/madler/zlib/releases/download/v1.3.2/zlib-1.3.2.tar.gz' -or
    $Lock.bytes -ne 1502830 -or
    $Lock.sha256 -cne 'bb329a0a2cd0274d05519d61c667c062e06990d72e125ee2dfa8de64f0119d16' -or
    $Lock.signingPrimaryFingerprint -cne '5ED46A6721D365587791E2AA783FCD8E58BCAFBA') {
    throw 'zlib source lock no longer matches the reviewed upstream release asset'
}
$ProducerLock = $Lock
$Errors = $null
$Ast = [Management.Automation.Language.Parser]::ParseFile($Script,[ref]$null,[ref]$Errors)
if ($Errors) { throw ($Errors | Out-String) }
foreach ($Name in @('Digest','SourceRecord','Assert-SourceArchive')) {
    $Nodes = @($Ast.FindAll({param($Node)
        $Node -is [Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -eq $Name
    },$true))
    if ($Nodes.Count -ne 1) { throw "Expected one production source verifier function: $Name" }
    . ([scriptblock]::Create($Nodes[0].Extent.Text))
}
$Fixture = Join-Path ([IO.Path]::GetTempPath()) ('zlib-source-guard-'+[guid]::NewGuid().ToString('N'))
$null = New-Item -ItemType Directory -Path $Fixture
try {
    $Archive = Join-Path $Fixture 'source.tar.gz'
    $Report = Join-Path $Fixture 'source-verification.json'
    [IO.File]::WriteAllBytes($Archive,[byte[]](1,2,3,4,5,6))
    $Lock = [pscustomobject]@{archive='source.tar.gz';bytes=6;sha256=(Digest $Archive)}
    Assert-SourceArchive $Archive $Lock $Report 'source-only'
    $Result = Get-Content -LiteralPath $Report -Raw | ConvertFrom-Json
    if ($Result.status -cne 'verified' -or $Result.observed.bytes -ne 6 -or
        $Result.observed.sha256 -cne $Lock.sha256) { throw 'Exact source guard did not accept matching bytes and record evidence' }
    foreach ($Case in @('truncated','same-length-corrupt','missing')) {
        switch ($Case) {
            'truncated' { [IO.File]::WriteAllBytes($Archive,[byte[]](1,2,3)) }
            'same-length-corrupt' { [IO.File]::WriteAllBytes($Archive,[byte[]](1,2,3,4,5,7)) }
            'missing' { Remove-Item -LiteralPath $Archive }
        }
        $Rejected = $false
        try { Assert-SourceArchive $Archive $Lock $Report 'source-only' } catch { $Rejected=$true }
        $Result = Get-Content -LiteralPath $Report -Raw | ConvertFrom-Json
        if (-not $Rejected -or $Result.status -cne 'rejected' -or
            $Result.expected.bytes -ne $Lock.bytes -or $Result.expected.sha256 -cne $Lock.sha256 -or
            $Result.observed.exists -ne ($Case -cne 'missing')) {
            throw "Source guard failed to preserve rejection evidence for $Case"
        }
        if ($Case -eq 'truncated' -and $Result.observed.bytes -ne 3) { throw 'Truncated byte count absent' }
        if ($Case -eq 'same-length-corrupt' -and
            ($Result.observed.bytes -ne 6 -or $Result.observed.sha256 -ceq $Lock.sha256)) {
            throw 'Corrupt same-length digest absent'
        }
    }
    Write-Output 'Actual zlib source guard accepts exact bytes and rejects truncated, corrupt, and missing archives with evidence.'
} finally {
    Remove-Item -LiteralPath $Fixture -Recurse -Force
}

# Execute the actual curl consumer's source-identity conditional, without
# entering its dependency build or downloading anything. This catches a stale
# consumer URL before a three-dependency native build reaches curl.
$CurlScript = if ($ConsumerScript) { $ConsumerScript } else { Join-Path $PSScriptRoot 'build-curl-windows.ps1' }
$CurlErrors = $null
$CurlAst = [Management.Automation.Language.Parser]::ParseFile($CurlScript,[ref]$null,[ref]$CurlErrors)
if ($CurlErrors) { throw ($CurlErrors | Out-String) }
$Guards = @($CurlAst.FindAll({param($Node)
    $Node -is [Management.Automation.Language.IfStatementAst] -and
    $Node.Extent.Text.Contains("throw 'zlib manifest does not identify the reviewed zlib 1.3.2 source'")
},$true))
if ($Guards.Count -ne 1) { throw 'Expected one production curl zlib source-identity guard' }
$CurlSourceGuard = [scriptblock]::Create($Guards[0].Extent.Text)
$Source = [pscustomobject]@{
    url=$ProducerLock.url; archive=$ProducerLock.archive; sha256=$ProducerLock.sha256
    bytes=$ProducerLock.bytes; signingPrimaryFingerprint=$ProducerLock.signingPrimaryFingerprint
}
function Test-CurlSourceGuard([object]$Record,[bool]$ShouldAccept,[string]$Case) {
    $Zlib = [pscustomobject]@{source=$Record}
    $ZlibSourceKeys = @($Zlib.source.PSObject.Properties.Name | Sort-Object)
    $Accepted = $true
    try { & $CurlSourceGuard } catch { $Accepted = $false }
    if ($Accepted -ne $ShouldAccept) { throw "Curl zlib source guard gave wrong result for $Case" }
}
Test-CurlSourceGuard $Source $true 'reviewed producer lock'
if ($ObservedNativeManifest) {
    $Zlib = Get-Content -LiteralPath $ObservedNativeManifest -Raw | ConvertFrom-Json
    $ZlibKeys = @($Zlib.PSObject.Properties.Name | Sort-Object)
    $ZlibSourceKeys = @($Zlib.source.PSObject.Properties.Name | Sort-Object)
    $SchemaGuards = @($CurlAst.FindAll({param($Node)
        $Node -is [Management.Automation.Language.IfStatementAst] -and
        $Node.Extent.Text.Contains("throw 'zlib manifest does not satisfy the reviewed Win32 shared /MD schema'")
    },$true))
    if ($SchemaGuards.Count -ne 1) { throw 'Expected one production curl zlib manifest-schema guard' }
    & ([scriptblock]::Create($SchemaGuards[0].Extent.Text))
    & $CurlSourceGuard
    Test-CurlSourceGuard $Zlib.source $true 'observed native manifest'
    Write-Output 'Actual native zlib manifest satisfies the exact curl schema and source guards.'
}
foreach ($Case in @('url','sha256','bytes','extra','missing')) {
    $Changed = $Source | ConvertTo-Json -Compress | ConvertFrom-Json
    switch ($Case) {
        'url' { $Changed.url = 'https://zlib.net/zlib-1.3.2.tar.gz' }
        'sha256' { $Changed.sha256 = '0' * 64 }
        'bytes' { $Changed.bytes = $Changed.bytes - 1 }
        'extra' { $Changed | Add-Member -NotePropertyName unexpected -NotePropertyValue 'unreviewed' }
        'missing' { $Changed.PSObject.Properties.Remove('url') }
    }
    Test-CurlSourceGuard $Changed $false $Case
}
Write-Output 'Actual curl zlib source guard accepts the producer lock and rejects changed URL, hash, bytes, and schema.'
