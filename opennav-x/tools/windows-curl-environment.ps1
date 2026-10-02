# Scoped native x86 environment for the MSYS Perl -> MSVC curl test boundary.
function Invoke-WindowsCurlEnvironment {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$VisualStudio,
        [Parameter(Mandatory=$true)][string]$TestPerl,
        [Parameter(Mandatory=$true)][scriptblock]$Action
    )
    if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Curl build environment requires native Windows' }
    $TestPerl = (Resolve-Path -LiteralPath $TestPerl -ErrorAction Stop).Path
    $VcVars = Join-Path $VisualStudio 'VC/Auxiliary/Build/vcvarsall.bat'
    if (-not (Test-Path -LiteralPath $VcVars -PathType Leaf)) { throw 'MSVC vcvarsall.bat missing' }
    $Names = @('PATH','INCLUDE','LIB','LIBPATH','VSCMD_ARG_TGT_ARCH','VSCMD_ARG_HOST_ARCH',
        'WindowsSDKVersion','WindowsSdkDir','WindowsSdkBinPath','WindowsSdkVerBinPath',
        'VCToolsVersion','VCToolsInstallDir','VCINSTALLDIR','VSINSTALLDIR',
        'UniversalCRTSdkDir','UCRTVersion','VisualStudioVersion','MSYS2_ARG_CONV_EXCL')
    $Saved = @{}
    foreach ($Name in $Names) { $Saved[$Name] = [Environment]::GetEnvironmentVariable($Name, 'Process') }
    $EnvironmentCmd = Join-Path ([IO.Path]::GetTempPath()) ("xnav-curl-env-$([guid]::NewGuid().ToString('N')).cmd")
    foreach ($Path in @($EnvironmentCmd,$VcVars)) {
        if ($Path.Contains('%') -or $Path.Contains('"') -or $Path.Contains([char]10) -or $Path.Contains([char]13)) {
            throw 'Unsupported MSVC environment command path'
        }
    }
    try {
        [IO.File]::WriteAllLines($EnvironmentCmd, @(
            '@echo off', 'setlocal DisableDelayedExpansion',
            "call `"$VcVars`" x86 >nul || exit /b 1", 'set'
        ), (New-Object Text.UTF8Encoding($false)))
        # Capture in memory only: unrelated CI environment values may contain secrets.
        $Lines = & $env:ComSpec /d /s /c "`"$EnvironmentCmd`""
        if ($LASTEXITCODE -ne 0) { throw 'MSVC x86 environment initialization failed' }
        foreach ($Line in $Lines) {
            if ($Line -match '^([^=]+)=(.*)$' -and $Names -contains $Matches[1]) {
                [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process')
            }
        }
        $Lines = $null
        if ($env:VSCMD_ARG_TGT_ARCH -cne 'x86' -or -not $env:INCLUDE) { throw 'MSVC x86 header environment missing' }
        # Exact tested policy: keep path conversion for all non-/D arguments.
        $env:MSYS2_ARG_CONV_EXCL = '/D'
        $env:PATH = "$(Split-Path $TestPerl -Parent);$env:PATH"
        $Selected = (Get-Command perl.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
        if ($Selected -ine $TestPerl) { throw 'Curl environment selected a different Perl' }
        & $Action
    } finally {
        foreach ($Name in $Names) { [Environment]::SetEnvironmentVariable($Name, $Saved[$Name], 'Process') }
        if (Test-Path -LiteralPath $EnvironmentCmd) { Remove-Item -LiteralPath $EnvironmentCmd -Force }
    }
}

function Invoke-WindowsCurlSourceChecks {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)][string]$TestPerl,
        [Parameter(Mandatory=$true)][string]$Source,
        [Parameter(Mandatory=$true)][string]$Build,
        [Parameter(Mandatory=$true)][string]$Evidence
    )
    $Tests = (Join-Path $Source 'tests').Replace('\','/')
    $TestBuild = Join-Path $Build 'tests'
    $Config = Join-Path $TestBuild 'configurehelp.pm'
    if (-not (Test-Path -LiteralPath $Config -PathType Leaf)) { throw 'Generated curl configurehelp.pm missing' }
    $null = New-Item -ItemType Directory -Force -Path $Evidence
    $Report = [ordered]@{schemaVersion=1; configurehelpSha256=(Get-FileHash -LiteralPath $Config -Algorithm SHA256).Hash.ToLowerInvariant(); cases=@(); passed=$false}
    try {
        foreach ($Id in @(1119,1167)) {
            $Script = "$Tests/test$Id.pl"
            $Before = (Get-FileHash -LiteralPath $Script -Algorithm SHA256).Hash.ToLowerInvariant()
            $Arguments = @('-I.',"-I$Tests",$Script,"$Tests/..")
            if ($Id -eq 1119) { $Arguments += '../include/curl' }
            # Start-Process joins ArgumentList on Windows. These are only validated
            # source paths; reject command-line metacharacters instead of guessing.
            foreach ($Argument in $Arguments) {
                if ($Argument.Contains('"') -or $Argument.Contains([char]10) -or $Argument.Contains([char]13)) { throw 'Unsupported curl source-check argument' }
            }
            $Quoted = @($Arguments | ForEach-Object { '"' + $_ + '"' })
            $Stdout = Join-Path $Evidence "test$Id.stdout.txt"
            $Stderr = Join-Path $Evidence "test$Id.stderr.txt"
            $Process = New-Object Diagnostics.Process
            $Process.StartInfo = New-Object Diagnostics.ProcessStartInfo
            $Process.StartInfo.FileName = $TestPerl
            $Process.StartInfo.Arguments = $Quoted -join ' '
            $Process.StartInfo.WorkingDirectory = $TestBuild
            $Process.StartInfo.UseShellExecute = $false
            $Process.StartInfo.RedirectStandardOutput = $true
            $Process.StartInfo.RedirectStandardError = $true
            $OutStream = $null; $ErrStream = $null; $Started = $false
            $OutTask = $null; $ErrTask = $null
            try {
                $OutStream = [IO.File]::Open($Stdout, [IO.FileMode]::Create, [IO.FileAccess]::Write, [IO.FileShare]::Read)
                $ErrStream = [IO.File]::Open($Stderr, [IO.FileMode]::Create, [IO.FileAccess]::Write, [IO.FileShare]::Read)
                $null = $Process.Start(); $Started = $true
                $OutTask = $Process.StandardOutput.BaseStream.CopyToAsync($OutStream)
                $ErrTask = $Process.StandardError.BaseStream.CopyToAsync($ErrStream)
                $Timer = [Diagnostics.Stopwatch]::StartNew()
                while (-not $Process.HasExited -or -not $OutTask.IsCompleted -or -not $ErrTask.IsCompleted) {
                    if ($Timer.ElapsedMilliseconds -ge 60000) { throw "Curl source test $Id exceeded 60 seconds" }
                    if ((Get-Item -LiteralPath $Stdout).Length + (Get-Item -LiteralPath $Stderr).Length -gt 1048576) {
                        throw "Curl source test $Id exceeded 1 MiB output"
                    }
                    Start-Sleep -Milliseconds 25
                }
                $null = $OutTask.GetAwaiter().GetResult(); $null = $ErrTask.GetAwaiter().GetResult()
                $OutStream.Dispose(); $OutStream = $null
                $ErrStream.Dispose(); $ErrStream = $null
                if ((Get-Item -LiteralPath $Stdout).Length + (Get-Item -LiteralPath $Stderr).Length -gt 1048576) { throw "Curl source test $Id exceeded 1 MiB output" }
                $ExitCode = $Process.ExitCode
                $Output = [IO.File]::ReadAllText($Stdout)
                $Passed = $ExitCode -eq 0
                if ($Id -eq 1119) { $Passed = $Passed -and $Output -ceq "OK`n" }
                if ($Id -eq 1167) { $Passed = $Passed -and $Output -match '^(\d+) fine symbols found\r?\n$' -and [int]$Matches[1] -gt 0 }
                $After = (Get-FileHash -LiteralPath $Script -Algorithm SHA256).Hash.ToLowerInvariant()
                $Report.cases += [ordered]@{test=$Id; sourceSha256=$Before; sourceUnchanged=($After -ceq $Before); exitCode=$ExitCode; passed=$Passed;
                    stdoutSha256=(Get-FileHash -LiteralPath $Stdout -Algorithm SHA256).Hash.ToLowerInvariant(); stderrSha256=(Get-FileHash -LiteralPath $Stderr -Algorithm SHA256).Hash.ToLowerInvariant()}
                if (-not $Passed -or $After -cne $Before) { throw "Unchanged curl source test $Id failed; see $Evidence" }
            } finally {
                if ($Started -and -not $Process.HasExited) {
                    & (Join-Path $env:SystemRoot 'System32/taskkill.exe') /PID $Process.Id /T /F | Out-Null
                    $null = $Process.WaitForExit(5000)
                }
                if ($null -ne $OutStream) { $OutStream.Dispose() }
                if ($null -ne $ErrStream) { $ErrStream.Dispose() }
                $Process.Dispose()
            }
        }
        $Report.passed = $true
    } finally {
        $Report | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Evidence 'source-analysis.json') -Encoding UTF8
    }
    Write-Output 'Unchanged curl source tests 1119 and 1167 passed before compilation'
}
