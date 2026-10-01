param()
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$Root = Split-Path $PSScriptRoot -Parent
$LockPath = Join-Path $PSScriptRoot 'windows-zlib.lock.json'
$Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
if ($Lock.version -cne '1.3.2' -or $Lock.configuration -cne 'Win32 shared' -or
    $Lock.runtime -cne 'MultiThreadedDLL (/MD)') {
    throw 'Unexpected zlib lock identity; refusing an unreviewed build'
}

$BuildRoot = Join-Path $Root 'build/windows-zlib-1.3.2'
$Downloads = Join-Path $Root 'build/dependency-downloads'
$Source = Join-Path $BuildRoot 'zlib-1.3.2'
$Wrapper = Join-Path $BuildRoot 'cmake-wrapper'
$CMakeBuild = Join-Path $BuildRoot 'cmake-build'
$Prefix = Join-Path $BuildRoot 'install'
$Evidence = Join-Path $Root 'evidence/local/windows-zlib-1.3.2'
$Archive = Join-Path $Downloads $Lock.archive
$ManifestPath = Join-Path $Prefix 'zlib-build.json'
$null = New-Item -ItemType Directory -Force -Path $Downloads,$BuildRoot,$Evidence

function Invoke-Checked([string]$Program,[string[]]$Arguments) {
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) { throw "$Program failed with exit code $LASTEXITCODE" }
}
function Digest([string]$Path) {
    (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}
function FileRecord([string]$Path) {
    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) { throw "Required zlib output missing: $Path" }
    [ordered]@{ sha256 = Digest $Path; bytes = (Get-Item -LiteralPath $Path).Length }
}
function Assert-Win32Dll([string]$Path) {
    $Bytes = [IO.File]::ReadAllBytes($Path)
    if ($Bytes.Length -lt 64 -or $Bytes[0] -ne 0x4d -or $Bytes[1] -ne 0x5a) { throw "Not a PE DLL: $Path" }
    $Pe = [BitConverter]::ToInt32($Bytes,0x3c)
    if ($Pe -lt 0 -or $Pe + 24 -gt $Bytes.Length -or [BitConverter]::ToUInt32($Bytes,$Pe) -ne 0x00004550 -or
        [BitConverter]::ToUInt16($Bytes,$Pe + 4) -ne 0x014c -or
        ([BitConverter]::ToUInt16($Bytes,$Pe + 22) -band 0x2000) -eq 0) {
        throw "zlib DLL is not a Win32 x86 PE DLL: $Path"
    }
}

if (-not (Test-Path -LiteralPath $Archive -PathType Leaf) -or (Digest $Archive) -cne $Lock.sha256) {
    Invoke-Checked curl.exe @('--fail','--location','--silent','--show-error','--retry','3','--retry-all-errors',
        '--connect-timeout','20','--max-time','300','--output',$Archive,$Lock.url)
}
# The archive's identity and byte count are checked before any extraction is allowed.
if ((Digest $Archive) -cne $Lock.sha256 -or (Get-Item -LiteralPath $Archive).Length -ne $Lock.bytes) {
    throw 'zlib source archive digest or size differs from the reviewed lock'
}

$Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $Vswhere -PathType Leaf)) { throw 'Visual Studio locator missing' }
$VisualStudio = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VisualStudio) { throw 'Licensed MSVC x86 toolchain missing' }
$VcVars = Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat'
if (-not (Test-Path -LiteralPath $VcVars -PathType Leaf)) { throw 'MSVC vcvarsall.bat missing' }
$CMake = Get-Command cmake.exe -CommandType Application -ErrorAction SilentlyContinue
if (-not $CMake) { throw 'CMake missing' }
$Tar = Get-Command tar.exe -CommandType Application -ErrorAction SilentlyContinue
if (-not $Tar) { throw 'tar missing' }

function Assert-SafeNativePath([string]$Path,[bool]$ForCMake = $false) {
    if ($Path.Contains('%') -or $Path.Contains('"') -or $Path.Contains([char]10) -or $Path.Contains([char]13)) {
        throw "Unsupported character in native build path: $Path"
    }
    if ($ForCMake -and ($Path.Contains(';') -or $Path.Contains('#') -or $Path.Contains('$'))) {
        throw "Unsupported CMake expansion character in build path: $Path"
    }
}
foreach ($Path in @($VcVars,$Source,$Wrapper,$CMakeBuild,$Prefix,$Evidence,$BuildRoot)) {
    Assert-SafeNativePath $Path
}
foreach ($Path in @($Source,$Wrapper,$CMakeBuild,$Prefix)) {
    Assert-SafeNativePath $Path $true
}

foreach ($Path in @($Source,$Wrapper,$CMakeBuild,$Prefix)) {
    if (Test-Path -LiteralPath $Path) { Remove-Item -LiteralPath $Path -Recurse -Force }
}
Invoke-Checked tar.exe @('-xzf',$Archive,'-C',$BuildRoot)
if (-not (Test-Path -LiteralPath (Join-Path $Source 'CMakeLists.txt') -PathType Leaf)) {
    throw 'Verified zlib archive did not extract the expected source root'
}

# This generated wrapper is the reviewed fix for zlib's default CMake output name.
# OUTPUT_NAME is changed before generation, so the import library embeds zlib1.dll.
$null = New-Item -ItemType Directory -Force -Path $Wrapper
$WrapperText = @"
cmake_minimum_required(VERSION 3.20)
project(zlib-wrapper LANGUAGES C)
set(ZLIB_BUILD_TESTING ON CACHE BOOL "" FORCE)
set(ZLIB_BUILD_SHARED ON CACHE BOOL "" FORCE)
set(ZLIB_BUILD_STATIC OFF CACHE BOOL "" FORCE)
set(ZLIB_INSTALL ON CACHE BOOL "" FORCE)
enable_testing()
add_subdirectory("$($Source.Replace('\','/'))" zlib-source-build)
set_target_properties(zlib PROPERTIES OUTPUT_NAME zlib1)
"@
Set-Content -LiteralPath (Join-Path $Wrapper 'CMakeLists.txt') -Value $WrapperText -Encoding UTF8

$BuildCmd = Join-Path $BuildRoot 'build-zlib.cmd'
$BuildLog = Join-Path $Evidence 'windows-zlib-native-output.log'
$null = New-Item -ItemType File -Force -Path $BuildLog
Set-Content -LiteralPath $BuildLog -Value '' -Encoding UTF8
$CommandLines = @(
    '@echo off',
    'setlocal DisableDelayedExpansion',
    'echo === stage: initialize x86 MSVC environment ===',
    "call `"$VcVars`" x86",
    'if not "%errorlevel%"=="0" exit /b %errorlevel%',
    'echo === stage: locate compiler ===',
    'where cl',
    'if not "%errorlevel%"=="0" exit /b %errorlevel%',
    'echo === stage: locate CMake ===',
    'where cmake',
    'if not "%errorlevel%"=="0" exit /b %errorlevel%',
    'echo === stage: configure zlib Win32 shared ===',
    "cmake -S `"$Wrapper`" -B `"$CMakeBuild`" -G `"Visual Studio 17 2022`" -A Win32 -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL",
    'if not "%errorlevel%"=="0" exit /b %errorlevel%',
    'echo === stage: compile zlib ===',
    "cmake --build `"$CMakeBuild`" --config Release -- /m",
    'if not "%errorlevel%"=="0" exit /b %errorlevel%',
    'echo === stage: test zlib ===',
    "ctest --test-dir `"$CMakeBuild`" -C Release --output-on-failure --no-tests=error",
    'if not "%errorlevel%"=="0" exit /b %errorlevel%',
    'echo === stage: install zlib ===',
    "cmake --install `"$CMakeBuild`" --config Release --prefix `"$Prefix`"",
    'if not "%errorlevel%"=="0" exit /b %errorlevel%'
)
[IO.File]::WriteAllLines($BuildCmd,$CommandLines,(New-Object Text.UTF8Encoding($false)))
# Keep one PowerShell-owned log handle for the whole child process. Reopening
# the same file for each batch command is unnecessary. Stage markers identify
# failures without guessing which command or file caused the previous lock.
& cmd.exe @('/d','/s','/c',"`"$BuildCmd`"") 2>&1 | Tee-Object -FilePath $BuildLog
$BuildExitCode = $LASTEXITCODE
if ($BuildExitCode -ne 0) { throw "cmd.exe failed with exit code $BuildExitCode; see $BuildLog" }

$Expected = [ordered]@{
    'include/zlib.h' = Join-Path $Prefix 'include/zlib.h'
    'include/zconf.h' = Join-Path $Prefix 'include/zconf.h'
    'lib/zlib1.lib' = Join-Path $Prefix 'lib/zlib1.lib'
    'bin/zlib1.dll' = Join-Path $Prefix 'bin/zlib1.dll'
}
$Outputs = [ordered]@{}
foreach ($Name in $Expected.Keys) { $Outputs[$Name] = FileRecord $Expected[$Name] }
Assert-Win32Dll $Expected['bin/zlib1.dll']

$VersionHeader = Get-Content -LiteralPath $Expected['include/zlib.h'] -Raw
if ($VersionHeader -notmatch '#\s*define\s+ZLIB_VERSION\s+"1\.3\.2"') {
    throw 'Installed zlib header does not identify version 1.3.2'
}
Set-Content -LiteralPath (Join-Path $Evidence 'version-output.txt') -Value @(
    'ZLIB_VERSION=1.3.2',
    ($VersionHeader | Select-String -Pattern '#\s*define\s+ZLIB_VERSION\s+"[^"]+"' -AllMatches).Line
) -Encoding UTF8

$ZlibProject = Get-ChildItem -LiteralPath $CMakeBuild -Filter 'zlib.vcxproj' -Recurse -File | Select-Object -First 1
if (-not $ZlibProject) { throw 'Generated zlib MSBuild project missing' }
$ZlibProjectText = Get-Content -LiteralPath $ZlibProject.FullName -Raw
$ReleaseRuntime = @([regex]::Matches($ZlibProjectText,
    '(?s)<ItemDefinitionGroup Condition="[^"]*Release\|Win32[^"]*">.*?<RuntimeLibrary>([^<]+)</RuntimeLibrary>.*?</ItemDefinitionGroup>') |
    ForEach-Object { $_.Groups[1].Value })
if (@($ReleaseRuntime).Count -ne 1 -or $ReleaseRuntime[0] -cne 'MultiThreadedDLL') {
    throw 'Release zlib MSBuild project does not prove exact MultiThreadedDLL (/MD)'
}
Set-Content -LiteralPath (Join-Path $Evidence 'msbuild-runtime-library.txt') -Value @(
    "project=$($ZlibProject.FullName)",
    'configuration=Release|Win32',
    "RuntimeLibrary=$($ReleaseRuntime[0])"
) -Encoding UTF8

$Dumpbin = Get-ChildItem -Path (Join-Path $VisualStudio 'VC/Tools/MSVC') -Filter dumpbin.exe -Recurse -File |
    Where-Object { $_.FullName -match '\\Host(?:x64|x86)\\x86\\dumpbin\.exe$' } | Select-Object -First 1
if (-not $Dumpbin) { throw 'x86 dumpbin.exe missing' }
$Imports = & $Dumpbin.FullName /DEPENDENTS $Expected['bin/zlib1.dll'] 2>&1 | Out-String
if ($LASTEXITCODE -ne 0) { throw 'dumpbin import inspection failed' }
if ($Imports -notmatch '(?im)^\s*VCRUNTIME140[^\r\n]*\.dll\s*$' -or
    ($Imports -notmatch '(?im)^\s*api-ms-win-crt-[^\r\n]*\.dll\s*$' -and $Imports -notmatch '(?im)^\s*ucrtbase\.dll\s*$')) {
    throw 'zlib1.dll imports do not prove the expected dynamic MSVC CRT'
}
Set-Content -LiteralPath (Join-Path $Evidence 'dumpbin-dependents.txt') -Value $Imports -Encoding UTF8
$Exports = & $Dumpbin.FullName /EXPORTS $Expected['bin/zlib1.dll'] 2>&1 | Out-String
if ($LASTEXITCODE -ne 0) { throw 'dumpbin export inspection failed' }
foreach ($Symbol in @('deflate','inflate','zlibVersion')) {
    if ($Exports -notmatch "(?im)\s$Symbol\s*$") { throw "zlib1.dll is missing the undecorated $Symbol export" }
}
if ($Exports -match '(?im)^\s*[^\r\n]*@\d+\s*$') { throw 'zlib1.dll contains decorated exports; CDECL compatibility is not preserved' }
Set-Content -LiteralPath (Join-Path $Evidence 'dumpbin-exports.txt') -Value $Exports -Encoding UTF8

$Manifest = [ordered]@{
    schemaVersion = 1
    library = 'zlib'
    version = '1.3.2'
    configuration = 'Win32 shared'
    architecture = 'Win32'
    abi = 'x86'
    runtime = 'MultiThreadedDLL (/MD)'
    source = [ordered]@{ url=$Lock.url; archive=$Lock.archive; sha256=$Lock.sha256; bytes=$Lock.bytes; signingPrimaryFingerprint=$Lock.signingPrimaryFingerprint }
    buildSteps = [ordered]@{ configure='passed'; compile='passed'; test='passed'; install='passed' }
    outputs = $Outputs
}
$Json = $Manifest | ConvertTo-Json -Depth 8
# The manifest is published only after all source, version, PE, CRT, and output checks pass.
$Json | Set-Content -LiteralPath $ManifestPath -Encoding UTF8
$Json | Set-Content -LiteralPath (Join-Path $Evidence 'zlib-build.json') -Encoding UTF8
Write-Output "Built and verified zlib 1.3.2 Win32 shared x86 (/MD)"
