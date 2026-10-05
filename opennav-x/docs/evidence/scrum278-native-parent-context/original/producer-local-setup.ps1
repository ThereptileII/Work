foreach ($Directory in @('C:\Program Files\NASM','C:\Strawberry\perl\bin')) {
    if (Test-Path -LiteralPath $Directory -PathType Container) { $env:PATH = "$Directory;$env:PATH" }
}
if (-not (Get-Command nasm.exe -CommandType Application -ErrorAction SilentlyContinue) -or
    (& nasm.exe -v 2>&1 | Out-String) -notmatch 'version 3\.02(?:\s|$)') {
    $NasmLock = $Lock.buildTools.nasm
    $NasmArchive = Join-Path $Downloads $NasmLock.archive
    if (-not (Test-Path -LiteralPath $NasmArchive -PathType Leaf) -or (Digest $NasmArchive) -cne $NasmLock.sha256) {
        if ($VerifyToolFactsOnly) { throw 'Verified NASM archive unavailable for tool-facts reprobe' }
        Invoke-Checked curl.exe @('--fail','--location','--silent','--show-error','--retry','3','--retry-all-errors',
            '--connect-timeout','20','--max-time','180','--output',$NasmArchive,$NasmLock.url)
    }
    if ((Digest $NasmArchive) -cne $NasmLock.sha256 -or (Get-Item -LiteralPath $NasmArchive).Length -ne $NasmLock.bytes) {
        throw 'NASM host-tool archive digest or size differs from the reviewed lock'
    }
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $Zip = [IO.Compression.ZipFile]::OpenRead($NasmArchive)
    try { $Entries = @($Zip.Entries | ForEach-Object FullName | Sort-Object) } finally { $Zip.Dispose() }
    $ExpectedEntries = @('nasm-3.02/LICENSE','nasm-3.02/nasm.exe','nasm-3.02/ndisasm.exe')
    if (@(Compare-Object $ExpectedEntries $Entries).Count) { throw 'NASM archive has an unexpected entry or path' }
    $NasmRoot = Join-Path $Root 'build/dependency-tools/nasm-3.02'
    if (-not $VerifyToolFactsOnly) {
        if (Test-Path -LiteralPath $NasmRoot) { Remove-Item -LiteralPath $NasmRoot -Recurse -Force }
        Expand-Archive -LiteralPath $NasmArchive -DestinationPath (Split-Path $NasmRoot -Parent) -Force
    }
    if (-not (Test-Path -LiteralPath (Join-Path $NasmRoot 'nasm.exe') -PathType Leaf)) { throw 'Pinned NASM executable missing' }
    $env:PATH = "$NasmRoot;$env:PATH"
}
foreach ($Tool in @('perl.exe','nasm.exe','tar.exe','cmd.exe')) {
    if (-not (Get-Command $Tool -CommandType Application -ErrorAction SilentlyContinue)) {
        throw "OpenSSL build prerequisite missing after pinned local-tool resolution: $Tool"
    }
}
