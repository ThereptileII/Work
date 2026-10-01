param(
    [Parameter(Mandatory=$true)][string]$IntegrationSource,
    [switch]$VerifyToolFactsOnly
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$Root = Split-Path $PSScriptRoot -Parent
$LockPath = Join-Path $PSScriptRoot 'windows-openssl.lock.json'
$Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
if ($Lock.version -cne '3.5.9' -or $Lock.configuration -cne 'VC-WIN32 shared') {
    throw 'Unexpected OpenSSL lock identity; refusing an unreviewed build'
}
$IntegrationSource = (Resolve-Path -LiteralPath $IntegrationSource).Path
$BuildRoot = Join-Path $Root 'build/windows-openssl-3.5.9'
$Downloads = Join-Path $Root 'build/dependency-downloads'
$Archive = Join-Path $Downloads $Lock.archive
$Source = Join-Path $BuildRoot "openssl-$($Lock.version)"
$Prefix = Join-Path $BuildRoot 'install'
$Evidence = Join-Path $Root 'evidence/local'
if (-not $VerifyToolFactsOnly) {
    $null = New-Item -ItemType Directory -Force -Path $Downloads,$BuildRoot,$Evidence
}

function Invoke-Checked([string]$Program,[string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
function Digest([string]$Path) {
    $Stream = [IO.File]::Open($Path,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    $Hasher = [Security.Cryptography.SHA256]::Create()
    try { ([BitConverter]::ToString($Hasher.ComputeHash($Stream))).Replace('-','').ToLowerInvariant() }
    finally { $Hasher.Dispose(); $Stream.Dispose() }
}
function FileRecord([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "Required OpenSSL output missing: $Path" }
    [ordered]@{ sha256 = Digest $Path; bytes = (Get-Item -LiteralPath $Path).Length }
}
function Assert-Win32Image([string]$Path) {
    $Bytes = [IO.File]::ReadAllBytes($Path)
    if ($Bytes.Length -lt 64 -or $Bytes[0] -ne 0x4d -or $Bytes[1] -ne 0x5a) { throw "Not a PE DLL: $Path" }
    $Pe = [BitConverter]::ToInt32($Bytes,0x3c)
    if ($Pe -lt 0 -or $Pe + 6 -gt $Bytes.Length -or [BitConverter]::ToUInt32($Bytes,$Pe) -ne 0x00004550 -or
        [BitConverter]::ToUInt16($Bytes,$Pe + 4) -ne 0x014c) { throw "OpenSSL image is not the supported Win32 machine type: $Path" }
}

if (-not (Test-Path -LiteralPath $Archive -PathType Leaf) -or (Digest $Archive) -cne $Lock.sha256) {
    if ($VerifyToolFactsOnly) { throw 'Verified OpenSSL archive unavailable for tool-facts reprobe' }
    Invoke-Checked curl.exe @('--fail','--location','--silent','--show-error','--retry','3','--retry-all-errors',
        '--connect-timeout','20','--max-time','300','--output',$Archive,$Lock.url)
}
if ((Digest $Archive) -cne $Lock.sha256 -or (Get-Item -LiteralPath $Archive).Length -ne $Lock.bytes) {
    throw 'OpenSSL source archive digest or size differs from the reviewed lock'
}

$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $Vswhere -PathType Leaf)) { throw 'Visual Studio locator missing' }
$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VisualStudio) { throw 'Licensed MSVC x86 toolchain missing' }
$VcVars = Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat'
if (-not (Test-Path -LiteralPath $VcVars -PathType Leaf)) { throw 'MSVC vcvarsall.bat missing' }
$ToolFacts = Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1'
$ChildPowerShell = [Diagnostics.Process]::GetCurrentProcess().MainModule.FileName
if (-not (Test-Path -LiteralPath $ChildPowerShell -PathType Leaf) -or
    [IO.Path]::GetFileName($ChildPowerShell) -notin @('powershell.exe','pwsh.exe')) {
    throw 'Current PowerShell interpreter cannot be reproduced after vcvarsall'
}
$ParentFacts = Join-Path $Evidence 'windows-openssl-parent-tool-facts.json'
$ChildFacts = Join-Path $Evidence 'windows-openssl-child-tool-facts.json'
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
if (-not $VerifyToolFactsOnly) {
    & $ToolFacts -Mode Capture -Kind openssl-parent -Output $ParentFacts -ProducerScript $PSCommandPath `
        -Vswhere $Vswhere -VisualStudio $VisualStudio
}
& $ToolFacts -Mode Verify -Kind openssl-parent -Output $ParentFacts -ProducerScript $PSCommandPath `
    -Vswhere $Vswhere -VisualStudio $VisualStudio
# These paths are embedded in an owned cmd file. Double quotes protect spaces
# and command separators, and delayed expansion remains disabled for literal !.
# Percent expansion and characters which break a quoted line are unsupported.
foreach ($BatchPath in @($VcVars,$Source,$Prefix,$ToolFacts,$ChildFacts,$Vswhere,$VisualStudio,$PSCommandPath,$ChildPowerShell)) {
    if ($BatchPath.Contains('%') -or $BatchPath.Contains('"') -or
        $BatchPath.Contains([char]10) -or $BatchPath.Contains([char]13)) {
        throw "Unsupported character in OpenSSL build path: $BatchPath"
    }
}
if ($VerifyToolFactsOnly) {
    # The original child captured facts only after vcvarsall initialized x86.
    # Recreate that child environment without invoking any producer build step.
    $VerifyCmd = Join-Path ([IO.Path]::GetTempPath()) ("xnav-openssl-tool-facts-$([guid]::NewGuid().ToString('N')).cmd")
    if ($VerifyCmd.Contains('%') -or $VerifyCmd.Contains('"') -or
        $VerifyCmd.Contains([char]10) -or $VerifyCmd.Contains([char]13)) {
        throw 'Unsupported temporary tool-facts command path'
    }
    $VerifyLines = @(
        '@echo off',
        'setlocal DisableDelayedExpansion',
        "call `"$VcVars`" x86 || exit /b 1",
        'where cl || exit /b 1',
        'where nmake || exit /b 1',
        'where perl || exit /b 1',
        'where nasm || exit /b 1',
        "`"$ChildPowerShell`" -NoProfile -ExecutionPolicy Bypass -File `"$ToolFacts`" -Mode Verify -Kind openssl-child -Output `"$ChildFacts`" -ProducerScript `"$PSCommandPath`" -Vswhere `"$Vswhere`" -VisualStudio `"$VisualStudio`" -VcVars `"$VcVars`" || exit /b 1"
    )
    try {
        [IO.File]::WriteAllLines($VerifyCmd,$VerifyLines,(New-Object Text.UTF8Encoding($false)))
        Invoke-Checked cmd.exe @('/d','/s','/c',"`"$VerifyCmd`"")
    } finally {
        if (Test-Path -LiteralPath $VerifyCmd) { Remove-Item -LiteralPath $VerifyCmd -Force }
    }
    Write-Output 'Reprobed captured OpenSSL parent and x86 child tool facts'
    return
}

if (Test-Path -LiteralPath $Source) { Remove-Item -LiteralPath $Source -Recurse -Force }
if (Test-Path -LiteralPath $Prefix) { Remove-Item -LiteralPath $Prefix -Recurse -Force }
Invoke-Checked tar.exe @('-xzf',$Archive,'-C',$BuildRoot)
if (-not (Test-Path -LiteralPath (Join-Path $Source 'Configure') -PathType Leaf)) {
    throw 'Verified OpenSSL archive did not extract the expected source root'
}

$BuildCmd = Join-Path $BuildRoot 'build-openssl.cmd'
$CommandLines = @(
    '@echo off',
    'setlocal DisableDelayedExpansion',
    "call `"$VcVars`" x86 || exit /b 1",
    'where cl || exit /b 1',
    'where nmake || exit /b 1',
    'where perl || exit /b 1',
    'where nasm || exit /b 1',
    "`"$ChildPowerShell`" -NoProfile -ExecutionPolicy Bypass -File `"$ToolFacts`" -Mode Capture -Kind openssl-child -Output `"$ChildFacts`" -ProducerScript `"$PSCommandPath`" -Vswhere `"$Vswhere`" -VisualStudio `"$VisualStudio`" -VcVars `"$VcVars`" || exit /b 1",
    "cd /d `"$Source`" || exit /b 1",
    "perl Configure VC-WIN32 shared --libdir=lib --prefix=`"$Prefix`" --openssldir=`"$Prefix\ssl`" || exit /b 1",
    'nmake || exit /b 1',
    'nmake test || exit /b 1',
    'nmake install_sw install_ssldirs || exit /b 1',
    'cl /Bv 2>&1',
    'nmake /? 2>&1',
    'perl -V 2>&1',
    'nasm -v 2>&1',
    "`"$ChildPowerShell`" -NoProfile -ExecutionPolicy Bypass -File `"$ToolFacts`" -Mode Verify -Kind openssl-child -Output `"$ChildFacts`" -ProducerScript `"$PSCommandPath`" -Vswhere `"$Vswhere`" -VisualStudio `"$VisualStudio`" -VcVars `"$VcVars`" || exit /b 1"
)
[IO.File]::WriteAllLines($BuildCmd,$CommandLines,(New-Object Text.UTF8Encoding($false)))
Invoke-Checked cmd.exe @('/d','/s','/c',"`"$BuildCmd`"")

$Expected = [ordered]@{
    'include/openssl/opensslv.h' = Join-Path $Prefix 'include/openssl/opensslv.h'
    'lib/libssl.lib' = Join-Path $Prefix 'lib/libssl.lib'
    'lib/libcrypto.lib' = Join-Path $Prefix 'lib/libcrypto.lib'
    'bin/libssl-3.dll' = Join-Path $Prefix 'bin/libssl-3.dll'
    'bin/libcrypto-3.dll' = Join-Path $Prefix 'bin/libcrypto-3.dll'
    'bin/openssl.exe' = Join-Path $Prefix 'bin/openssl.exe'
}
$Outputs = [ordered]@{}
foreach ($Name in $Expected.Keys) { $Outputs[$Name] = FileRecord $Expected[$Name] }
Assert-Win32Image $Expected['bin/libssl-3.dll']
Assert-Win32Image $Expected['bin/libcrypto-3.dll']
Assert-Win32Image $Expected['bin/openssl.exe']
$VersionHeader = Get-Content -LiteralPath $Expected['include/openssl/opensslv.h'] -Raw
if ($VersionHeader -notmatch '#\s*define\s+OPENSSL_VERSION_MAJOR\s+3' -or
    $VersionHeader -notmatch '#\s*define\s+OPENSSL_VERSION_MINOR\s+5' -or
    $VersionHeader -notmatch '#\s*define\s+OPENSSL_VERSION_PATCH\s+9') {
    throw 'Installed OpenSSL headers do not identify version 3.5.9'
}
$VersionText = & $Expected['bin/openssl.exe'] version -a 2>&1 | Out-String
if ($LASTEXITCODE -ne 0 -or $VersionText -notmatch 'OpenSSL 3\.5\.9') { throw 'Built OpenSSL executable has the wrong version' }

$Cache = Join-Path $IntegrationSource 'cache/buildwin'
$CacheHeaders = Join-Path $Cache 'include/openssl'
if (Test-Path -LiteralPath $CacheHeaders) { Remove-Item -LiteralPath $CacheHeaders -Recurse -Force }
$null = New-Item -ItemType Directory -Force -Path $CacheHeaders
Copy-Item -Path (Join-Path $Prefix 'include/openssl/*') -Destination $CacheHeaders -Recurse -Force
Copy-Item -LiteralPath $Expected['lib/libssl.lib'] -Destination (Join-Path $Cache 'libssl.lib') -Force
Copy-Item -LiteralPath $Expected['lib/libcrypto.lib'] -Destination (Join-Path $Cache 'libcrypto.lib') -Force
Copy-Item -LiteralPath $Expected['bin/libssl-3.dll'] -Destination (Join-Path $Cache 'libssl-3.dll') -Force
Copy-Item -LiteralPath $Expected['bin/libcrypto-3.dll'] -Destination (Join-Path $Cache 'libcrypto-3.dll') -Force

$Mappings = [ordered]@{}
$CacheSources = [ordered]@{
    'include/openssl/opensslv.h'='include/openssl/opensslv.h'; 'lib/libssl.lib'='libssl.lib';
    'lib/libcrypto.lib'='libcrypto.lib'; 'bin/libssl-3.dll'='libssl-3.dll';
    'bin/libcrypto-3.dll'='libcrypto-3.dll'
}
foreach ($Pair in $CacheSources.GetEnumerator()) {
    $Destination = Join-Path $Cache $Pair.Value
    if ((Digest $Destination) -cne $Outputs[$Pair.Key].sha256) { throw "OpenSSL cache mapping differs: $($Pair.Value)" }
    $Mappings[$Pair.Value] = [ordered]@{ source=$Pair.Key; sha256=Digest $Destination; bytes=(Get-Item $Destination).Length }
}
$Manifest = [ordered]@{
    schemaVersion = 1; library = 'OpenSSL'; version = $Lock.version
    configuration = $Lock.configuration; architecture = 'Win32'; abi = 'x86'
    source = [ordered]@{ url=$Lock.url; archive=$Lock.archive; sha256=$Lock.sha256; bytes=$Lock.bytes; signingPrimaryFingerprint=$Lock.signingPrimaryFingerprint }
    toolchain = [ordered]@{ visualStudioInstallation=$VisualStudio; vcvarsall=$VcVars; compiler='cl /Bv (captured in Windows native log)'; nmake='nmake /? (captured in Windows native log)'; perl='perl -V (captured in Windows native log)'; nasm='NASM 3.02 Win64 host tool; nasm -v captured in Windows native log'; nasmProvenance=$Lock.buildTools.nasm.provenance; nasmArchiveSha256=$Lock.buildTools.nasm.sha256 }
    buildSteps = [ordered]@{ configure='passed'; compile='passed'; test='passed'; install='passed'; log='evidence/local/windows-openssl-native-output.log' }
    versionOutput = $VersionText.Trim(); outputs = $Outputs; cacheBuildwin = $Mappings
}
$Json = $Manifest | ConvertTo-Json -Depth 8
$Json | Set-Content -LiteralPath (Join-Path $Prefix 'openssl-build.json') -Encoding UTF8
$Json | Set-Content -LiteralPath (Join-Path $Cache 'openssl-build.json') -Encoding UTF8
$Json | Set-Content -LiteralPath (Join-Path $Evidence 'windows-openssl-build.json') -Encoding UTF8
Write-Output "Built and verified OpenSSL $($Lock.version) $($Lock.configuration) for the supported x86 ABI"
