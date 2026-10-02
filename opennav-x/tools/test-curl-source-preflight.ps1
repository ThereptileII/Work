# Diagnostic only: configure pinned source and run unchanged upstream source tests.
# No curl/OpenSSL/application targets are built. CMake compiler checks still run.
param(
    [Parameter(Mandatory=$true)][string]$TestPerl,
    [Parameter(Mandatory=$true)][string]$Evidence,
    [string]$Archive
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'Curl source preflight requires native Windows'
}
$Root = Split-Path $PSScriptRoot -Parent
$TestPerl = (Resolve-Path -LiteralPath $TestPerl).Path
$Evidence = [IO.Path]::GetFullPath($Evidence)
if (Test-Path -LiteralPath $Evidence) { throw 'Use a new evidence directory for each diagnostic' }
$null = New-Item -ItemType Directory -Path $Evidence
$LockPath = Join-Path $PSScriptRoot 'windows-curl.lock.json'
$Lock = Get-Content -LiteralPath $LockPath -Raw | ConvertFrom-Json
function Digest([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function Run([string]$Name, [string]$Exe, [string[]]$Arguments, [string]$Cwd, [int]$Seconds = 120) {
    $Info = [Diagnostics.ProcessStartInfo]::new()
    $Info.FileName = $Exe
    $Info.WorkingDirectory = $Cwd
    $Info.UseShellExecute = $false
    $Info.RedirectStandardOutput = $true
    $Info.RedirectStandardError = $true
    foreach ($Argument in $Arguments) { $Info.ArgumentList.Add($Argument) }
    $Process = [Diagnostics.Process]::new()
    $Process.StartInfo = $Info
    $Timer = [Diagnostics.Stopwatch]::StartNew()
    $null = $Process.Start()
    $StdoutTask = $Process.StandardOutput.ReadToEndAsync()
    $StderrTask = $Process.StandardError.ReadToEndAsync()
    $TimedOut = -not $Process.WaitForExit($Seconds * 1000)
    if ($TimedOut) { $Process.Kill($true); $Process.WaitForExit() }
    $Stdout = $StdoutTask.GetAwaiter().GetResult()
    $Stderr = $StderrTask.GetAwaiter().GetResult()
    [IO.File]::WriteAllText((Join-Path $Evidence "$Name.stdout.txt"), $Stdout)
    [IO.File]::WriteAllText((Join-Path $Evidence "$Name.stderr.txt"), $Stderr)
    $Result = [ordered]@{command=@($Exe)+$Arguments; workingDirectory=$Cwd;
        exitCode=$Process.ExitCode; timedOut=$TimedOut; elapsedMs=$Timer.ElapsedMilliseconds;
        stdoutFile="$Name.stdout.txt"; stderrFile="$Name.stderr.txt"}
    $Result | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $Evidence "$Name.command.json")
    $Process.Dispose()
    return $Result
}
function Checked($Result) {
    if ($Result.timedOut -or $Result.exitCode -ne 0) { throw "Command failed: $($Result.stdoutFile); inspect retained stderr" }
}
function EnvironmentFacts {
    $Commands = [ordered]@{}
    foreach ($Name in @('perl.exe','cl.exe','cpp.exe','cmake.exe','sh.exe')) {
        $Commands[$Name] = @(Get-Command $Name -CommandType Application -All -ErrorAction SilentlyContinue | ForEach-Object { $_.Source })
    }
    return [ordered]@{path=$env:PATH; include=$env:INCLUDE; lib=$env:LIB; libpath=$env:LIBPATH;
        perl5lib=$env:PERL5LIB; perl5opt=$env:PERL5OPT; targetArchitecture=$env:VSCMD_ARG_TGT_ARCH;
        windowsSdkVersion=$env:WindowsSDKVersion; vcToolsVersion=$env:VCToolsVersion;
        msys2ArgConvExcl=$env:MSYS2_ARG_CONV_EXCL; commands=$Commands}
}
$Report = [ordered]@{schemaVersion=1; purpose='Native source-analysis diagnostic, not dependency or release qualification';
    commit=(& git -C $Root rev-parse HEAD); archiveSha256=$Lock.sha256; lockSha256=(Digest $LockPath);
    limitations=@('Configure uses Schannel and no external TLS/compression dependencies; producer uses OpenSSL/zlib.',
        'The inherited case reproduces the producer MSYS Perl PATH prepend, but does not stage curl/OpenSSL/zlib binary directories.',
        'Direct script invocation does not reproduce the entire MSBuild/runtests process environment.',
        'No curl/OpenSSL/application binary or full upstream suite is qualified.'); cases=@(); passed=$false}
$SavedEnvironment = @{}
Get-ChildItem Env: | ForEach-Object { $SavedEnvironment[$_.Name] = $_.Value }
try {
    # Match build-pristine-windows.ps1's scoped curl producer invocation. Keep
    # the absolute selected Perl for execution; PATH also controls its children.
    $env:PATH = "$(Split-Path $TestPerl -Parent);$env:PATH"
    $ResolvedPerl = (Get-Command perl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    if ($ResolvedPerl -ine $TestPerl) { throw 'Producer-style PATH did not select the fixed MSYS2 test Perl' }
    $Report.testHostPathPrepend = Split-Path $TestPerl -Parent
    $CMake = (Get-Command cmake.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    if (-not $Archive) {
        $Archive = Join-Path $Evidence $Lock.archive
        Invoke-WebRequest -Uri $Lock.url -OutFile $Archive -TimeoutSec 120
    }
    $Archive = (Resolve-Path -LiteralPath $Archive).Path
    if ((Digest $Archive) -cne $Lock.sha256 -or (Get-Item -LiteralPath $Archive).Length -ne $Lock.bytes) {
        throw 'Locked curl source digest or size mismatch'
    }
    Checked (Run 'extract' $CMake @('-E','tar','xf',$Archive) $Evidence)
    $Source = Join-Path $Evidence "curl-$($Lock.version)"
    $Build = Join-Path $Evidence 'build'
    $Tests = ($Source.Replace('\','/') + '/tests')
    $Report.sourceFiles = [ordered]@{}
    foreach ($Relative in @('tests/test1119.pl','tests/test1167.pl','tests/data/test1119','tests/data/test1167','tests/CMakeLists.txt','tests/configurehelp.pm.in')) {
        $Report.sourceFiles[$Relative] = Digest (Join-Path $Source $Relative)
    }
    $MsysRuntime = Join-Path (Split-Path $TestPerl -Parent) 'msys-2.0.dll'
    $Report.testPerl = @{path=$TestPerl; sha256=(Digest $TestPerl); runtimeSha256=(Digest $MsysRuntime)}
    Checked (Run 'perl-version' $TestPerl @('-V') $Evidence)
    Checked (Run 'perl-host' $TestPerl @('-e','print $^O') $Evidence)
    if ([IO.File]::ReadAllText((Join-Path $Evidence 'perl-host.stdout.txt')) -notin @('msys','cygwin')) {
        throw 'Selected Perl must be the MSYS2 test host'
    }
    Checked (Run 'cmake-version' $CMake @('--version') $Evidence)
    $Configure = @('-S',$Source,'-B',$Build,'-G','Visual Studio 17 2022','-A','Win32',
        "-DPERL_EXECUTABLE:FILEPATH=$TestPerl", '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL',
        '-DBUILD_SHARED_LIBS=ON','-DBUILD_STATIC_LIBS=OFF','-DBUILD_CURL_EXE=ON','-DBUILD_TESTING=ON',
        '-DBUILD_EXAMPLES=OFF','-DBUILD_LIBCURL_DOCS=OFF','-DBUILD_MISC_DOCS=OFF',
        '-DCURL_USE_OPENSSL=OFF','-DCURL_USE_SCHANNEL=ON','-DCURL_USE_CMAKECONFIG=OFF',
        '-DCURL_ZLIB=OFF','-DCURL_USE_LIBPSL=OFF','-DCURL_USE_LIBSSH2=OFF','-DCURL_USE_LIBSSH=OFF',
        '-DUSE_NGHTTP2=OFF','-DCURL_BROTLI=OFF','-DCURL_ZSTD=OFF','-DENABLE_ARES=OFF',
        '-DCURL_USE_GSSAPI=OFF','-DUSE_WIN32_IDN=ON','-DCURL_DISABLE_FORM_API=OFF','-DHTTP_ONLY=OFF')
    Checked (Run 'configure' $CMake $Configure $Evidence 300)
    $TestBuild = Join-Path $Build 'tests'
    $Config = Join-Path $TestBuild 'configurehelp.pm'
    Copy-Item -LiteralPath $Config -Destination (Join-Path $Evidence 'configurehelp.pm')
    Copy-Item -LiteralPath (Join-Path $Build 'CMakeCache.txt') -Destination (Join-Path $Evidence 'CMakeCache.txt')
    $Report.configurehelpSha256 = Digest $Config
    # The upstream scripts import from -I. and use this generated preprocessor.
    # Probe selection separately, without editing the module or either script.
    $Probe = 'use configurehelp qw($Cpreprocessor); print "module=$INC{q(configurehelp.pm)}\npreprocessor=$Cpreprocessor\n";'
    foreach ($Mode in @('inherited','msvc-x86','msvc-x86-args')) {
        if ($Mode -eq 'msvc-x86') {
            $Vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
            $Vs = & $Vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
            if ($LASTEXITCODE -ne 0 -or -not $Vs) { throw 'Visual Studio x86 tools unavailable' }
            $Vcvars = Join-Path $Vs 'VC/Auxiliary/Build/vcvarsall.bat'
            # Import only the environment changes; never retain the complete environment,
            # which may contain CI credentials. The selected tool variables are recorded below.
            $EnvironmentCmd = Join-Path ([IO.Path]::GetTempPath()) ("xnav-curl-source-env-$([guid]::NewGuid().ToString('N')).cmd")
            foreach ($CmdPath in @($EnvironmentCmd, $Vcvars)) {
                if ($CmdPath.Contains('%') -or $CmdPath.Contains('"') -or
                    $CmdPath.Contains([char]10) -or $CmdPath.Contains([char]13)) {
                    throw 'Unsupported MSVC environment command path'
                }
            }
            $EnvironmentLines = @(
                '@echo off',
                'setlocal DisableDelayedExpansion',
                "call `"$Vcvars`" x86 >nul || exit /b 1",
                'set'
            )
            try {
                [IO.File]::WriteAllLines($EnvironmentCmd, $EnvironmentLines, [Text.UTF8Encoding]::new($false))
                # Match the existing OpenSSL/zlib producer .cmd invocation pattern.
                $Lines = & $env:ComSpec /d /s /c "`"$EnvironmentCmd`""
                if ($LASTEXITCODE -ne 0) { throw 'MSVC x86 environment initialization failed' }
            } finally {
                if (Test-Path -LiteralPath $EnvironmentCmd) { Remove-Item -LiteralPath $EnvironmentCmd -Force }
            }
            $Report.vcvarsall = @{path=$Vcvars; sha256=(Digest $Vcvars); arguments=@('x86')}
            foreach ($Line in $Lines) {
                if ($Line -match '^((?:PATH|INCLUDE|LIB|LIBPATH|VSCMD_ARG_TGT_ARCH|VSCMD_ARG_HOST_ARCH|WindowsSDKVersion|WindowsSdkDir|WindowsSdkBinPath|WindowsSdkVerBinPath|VCToolsVersion|VCToolsInstallDir|VCINSTALLDIR|VSINSTALLDIR|UniversalCRTSdkDir|UCRTVersion))=(.*)$') {
                    [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process')
                }
            }
        }
        if ($Mode -eq 'msvc-x86-args') {
            # Preserve compiler /D defines as arguments when MSYS Perl launches
            # native cl.exe; keep any exclusions already supplied by the caller.
            if (-not $env:MSYS2_ARG_CONV_EXCL) { $env:MSYS2_ARG_CONV_EXCL = '/D' }
            elseif (@($env:MSYS2_ARG_CONV_EXCL -split ';') -notcontains '/D') {
                $env:MSYS2_ARG_CONV_EXCL += ';/D'
            }
        }
        $Facts = EnvironmentFacts
        $Facts | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Evidence "$Mode.environment.json")
        $Import = Run "$Mode-configurehelp" $TestPerl @('-I.',"-I$Tests",'-e',$Probe) $TestBuild
        Checked $Import
        $Selected = [IO.File]::ReadAllText((Join-Path $Evidence $Import.stdoutFile))
        if ($Selected -notmatch '(?m)^preprocessor="([^"\r\n]+)" -E') { throw 'Generated module did not select quoted MSVC preprocessor' }
        $Compiler = $Matches[1]
        if (-not (Test-Path -LiteralPath $Compiler) -or (Split-Path $Compiler -Leaf) -ine 'cl.exe') { throw 'Generated preprocessor is not native cl.exe' }
        $Report["$Mode-compiler"] = @{path=$Compiler; sha256=(Digest $Compiler); version=(Get-Item -LiteralPath $Compiler).VersionInfo.FileVersion}
        $null = Run "$Mode-cl-version" $Compiler @('/Bv') $TestBuild
        foreach ($Id in @(1119,1167)) {
            # Match tests/data/test1119 and test1167, including their distinct include args.
            $Arguments = @('-I.',"-I$Tests","$Tests/test$Id.pl","$Tests/..")
            if ($Id -eq 1119) { $Arguments += '../include/curl' }
            $Result = Run "$Mode-test$Id" $TestPerl $Arguments $TestBuild
            $Output = [IO.File]::ReadAllText((Join-Path $Evidence $Result.stdoutFile))
            $Passed = -not $Result.timedOut -and $Result.exitCode -eq 0
            if ($Id -eq 1119) { $Passed = $Passed -and $Output -ceq "OK`n" }
            $SymbolCount = $null
            if ($Id -eq 1167 -and $Output -match '^(\d+) fine symbols found\r?\n$') { $SymbolCount = [int]$Matches[1] }
            $Report.cases += [ordered]@{environment=$Mode; test=$Id; msys2ArgConvExcl=$env:MSYS2_ARG_CONV_EXCL; upstreamPassed=$Passed; symbolCount=$SymbolCount; analysisNonempty=($Id -eq 1119 -or $SymbolCount -gt 0); result=$Result}
        }
    }
    foreach ($Relative in $Report.sourceFiles.Keys) {
        if ((Digest (Join-Path $Source $Relative)) -cne $Report.sourceFiles[$Relative]) { throw "Upstream source changed: $Relative" }
    }
    $Valid = @($Report.cases | Where-Object { $_.environment -eq 'msvc-x86-args' })
    $Report.inheritedFailureReproduced = @($Report.cases | Where-Object { $_.environment -eq 'inherited' -and -not $_.upstreamPassed }).Count -gt 0
    $Report.passed = $Valid.Count -eq 2 -and @($Valid | Where-Object { -not $_.upstreamPassed -or -not $_.analysisNonempty }).Count -eq 0
    if (-not $Report.passed) { throw 'MSVC environment with /D argument preservation did not pass both unchanged source tests; diagnosis remains unresolved' }
} catch {
    $Report.error = $_.Exception.Message
    throw
} finally {
    $Report | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $Evidence 'source-preflight.json')
    Get-ChildItem Env: | Where-Object { -not $SavedEnvironment.ContainsKey($_.Name) } | ForEach-Object { [Environment]::SetEnvironmentVariable($_.Name, $null, 'Process') }
    foreach ($Name in $SavedEnvironment.Keys) { [Environment]::SetEnvironmentVariable($Name, $SavedEnvironment[$Name], 'Process') }
    # Keep a bounded, useful diagnosis in the job log even if artifact retrieval
    # is unavailable. Never print the environment files or vcvars 'set' output.
    $LogFiles = @('source-preflight.json')
    foreach ($Mode in @('inherited','msvc-x86','msvc-x86-args')) {
        foreach ($Id in @(1119,1167)) {
            foreach ($Stream in @('stdout','stderr')) { $LogFiles += "$Mode-test$Id.$Stream.txt" }
        }
    }
    foreach ($Name in $LogFiles) {
        $LogPath = Join-Path $Evidence $Name
        if (Test-Path -LiteralPath $LogPath -PathType Leaf) {
            $Text = [IO.File]::ReadAllText($LogPath)
            $Limit = if ($Name -eq 'source-preflight.json') { 24000 } else { 4096 }
            Write-Output "--- retained diagnostic: $Name ---"
            Write-Output $Text.Substring(0, [Math]::Min($Text.Length, $Limit))
            if ($Text.Length -gt $Limit) { Write-Output '[truncated in job log; complete file retained in artifact]' }
        }
    }
}
Write-Output "Unchanged curl source tests pass with MSVC x86 environment and /D argument preservation; inspect all case results in $Evidence"
