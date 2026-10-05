# Execute the actual orchestration function with native work mocked, never a
# chart/plugin/dependency build. Hash and same-job guards remain real.
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$Script = Join-Path $PSScriptRoot 'build-pristine-windows.ps1'
$Errors = $null
$Ast = [Management.Automation.Language.Parser]::ParseFile($Script,[ref]$null,[ref]$Errors)
if ($Errors) { throw ($Errors | Out-String) }
foreach ($Name in @('Digest','Assert-ManifestRecord','Build-PrivateOCharts')) {
    $Nodes = @($Ast.FindAll({param($Node)
        $Node -is [Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -eq $Name
    },$true))
    if ($Nodes.Count -ne 1) { throw "Expected one production function $Name" }
    # Extracted ScriptBlocks have no script path; bind only that automatic
    # location to this real tools directory, preserving the function logic.
    $Body = $Nodes[0].Extent.Text.Replace('$PSScriptRoot', "'" + $PSScriptRoot.Replace("'","''") + "'")
    . ([scriptblock]::Create($Body))
}
$Text = $Ast.Extent.Text
$Call = $Text.IndexOf('if ($PrivateOCharts) { Build-PrivateOCharts')
$DependencyCheck = $Text.IndexOf('Maintained curl manifest does not prove')
$Configure = $Text.IndexOf("Run cmake (@('-S',")
if ($Call -lt 0 -or $DependencyCheck -lt 0 -or $Configure -lt 0 -or
    $Call -lt $DependencyCheck -or $Call -gt $Configure) {
    throw 'Private build must follow maintained dependency checks and precede host configure'
}
# PrivateOCharts remains an explicit default-off switch regardless of which
# approved delivery/dependency parameters follow it in the param block.
$PrivateParameters = @($Ast.ParamBlock.Parameters | Where-Object {
    $_.Name.VariablePath.UserPath -ceq 'PrivateOCharts'
})
if ($PrivateParameters.Count -ne 1 -or
    $PrivateParameters[0].StaticType -ne [Management.Automation.SwitchParameter] -or
    $null -ne $PrivateParameters[0].DefaultValue -or
    $Text -notmatch '"-DSKAGER_OCHARTS_PACKAGE=\$OChartsPackage"') {
    throw 'Explicit default-off private opt-in/package wiring changed'
}
$PrivateProbeNodes = @($Ast.FindAll({param($Node)
    $Node -is [Management.Automation.Language.IfStatementAst] -and
    $Node.Clauses.Count -eq 1 -and
    $Node.Clauses[0].Item1.Extent.Text -ceq '$PrivateOCharts -and $Production'
},$true))
$Tests = $Text.IndexOf('Run ctest @(')
$DeferredReturn = $Text.IndexOf('if ($DeferRuntimeQualification) {')
if ($PrivateProbeNodes.Count -ne 1 -or $Tests -lt 0 -or $DeferredReturn -lt 0 -or
    $PrivateProbeNodes[0].Extent.StartOffset -lt $Tests -or
    $PrivateProbeNodes[0].Extent.EndOffset -gt $DeferredReturn) {
    throw 'Installed private TLS gate must follow CTest and precede deferred runtime return'
}
$PrivateProbeText = $PrivateProbeNodes[0].Extent.Text.Replace('$PSScriptRoot', "'" + $PSScriptRoot.Replace("'","''") + "'")
$PrivateProbe = [scriptblock]::Create($PrivateProbeText)
if (-not $Text.Contains("'-A', " + '$Architecture, $PythonCMakeArgument,') -or
    -not $Text.Contains("if (" + '$Program' + " -ceq 'python') { " + '$Program = $BuildPython }') -or
    $Text.IndexOf('$PythonIdentityJson = & python') -gt $Text.IndexOf('function Run(')) {
    throw 'Host configure and Python execution must share the entry interpreter'
}
$Fixture = Join-Path ([IO.Path]::GetTempPath()) ('ocharts-build-wiring-'+[guid]::NewGuid().ToString('N'))
$Root = $Fixture
$Evidence = Join-Path $Root 'evidence/local'
$Source = Join-Path $Root 'build/integration-source'
$ZlibPrefix = Join-Path $Root 'build/windows-zlib-1.3.2/install'
$OpenSslPrefix = Join-Path $Root 'build/windows-openssl-3.5.9/install'
$Calls = [Collections.Generic.List[object]]::new()
$BuildPython = (Get-Command python -CommandType Application | Select-Object -First 1).Source
$PythonCMakeArgument = "-DPython3_EXECUTABLE:FILEPATH=$BuildPython"
$Fail = ''
$OldEnv = @{}
foreach ($Key in @('GITHUB_RUN_ID','GITHUB_RUN_ATTEMPT','GITHUB_JOB','GITHUB_SHA')) {
    $OldEnv[$Key] = [Environment]::GetEnvironmentVariable($Key)
}
function Put([string]$Path,[string]$Value) {
    $null=New-Item -ItemType Directory -Force (Split-Path $Path -Parent)
    [IO.File]::WriteAllText($Path,$Value)
}
function Run([string]$Program,[string[]]$Arguments) {
    $Calls.Add(@{program=$Program;arguments=$Arguments})
    if ($Program -eq 'cmake' -and $Arguments -contains '-S' -and
        $Arguments -contains (Join-Path $Root 'cmake/ocharts-adapter')) {
        if ($Arguments -notcontains $PythonCMakeArgument) {
            throw 'Private native configure must use the captured entry Python'
        }
        if ($Arguments -notcontains "-DSKAGER_PREPARED:PATH=$(Join-Path $Root 'build/ocharts-prepared')") {
            throw 'Private native configure must pass an explicit CMake PATH argument'
        }
    }
    if ($Fail -and $Arguments -contains $Fail) { throw "Simulated native/validator failure: $Fail" }
    if ($Arguments -contains '--curl-prefix') {
        $OpenSslIndex = [array]::IndexOf($Arguments, '--openssl-prefix')
        if ($OpenSslIndex -lt 0 -or $Arguments[$OpenSslIndex+1] -cne $OpenSslPrefix) {
            throw 'Private producer verification requires the exact same-job OpenSSL prefix'
        }
        $Prepared=Join-Path $Root 'build/ocharts-prepared'
        Put (Join-Path $Prepared 'preparation.json') '{"fixture":"prepared"}'
        foreach ($Library in @('curl','zlib')) {
            $Prefix=Join-Path $Root $(if($Library -eq 'curl'){'build/windows-curl-8.22.0/install'}else{'build/windows-zlib-1.3.2/install'})
            $null=New-Item -ItemType Directory -Force (Join-Path $Prepared 'sdk')
            Copy-Item -LiteralPath (Join-Path $Prefix "$Library-build.json") -Destination (Join-Path $Prepared 'sdk')
            $null=New-Item -ItemType Directory -Force (Join-Path $Prepared 'sdk/lib')
            Copy-Item -LiteralPath (Join-Path $Prefix "lib/$Library.lib") -Destination (Join-Path $Prepared 'sdk/lib')
        }
    }
    if ($Arguments -contains '--package-dll') {
        foreach ($Name in @('manifest.json','skager-ocharts-adapter.dll','corresponding-source.zip')) {
            Put (Join-Path $Root "build/ocharts-package/$Name") ('fixture-'+$Name)
        }
    }
}
function Reject([scriptblock]$Action,[string]$Case) {
    $Rejected=$false
    try { & $Action } catch { $Rejected=$true }
    if(-not $Rejected){throw "Accepted invalid $Case"}
}
try {
    $env:GITHUB_RUN_ID='123';$env:GITHUB_RUN_ATTEMPT='1';$env:GITHUB_JOB='windows-integration';$env:GITHUB_SHA='fixture-commit'
    # Execute the real installed-probe guard with only native work mocked.
    # Deferring GUI/runtime qualification must never defer installed TLS checks.
    foreach ($PrivateOCharts in @($false,$true)) {
        foreach ($Production in @($false,$true)) {
            foreach ($DeferRuntimeQualification in @($false,$true)) {
                $Calls.Clear()
                & $PrivateProbe
                $Expected = if ($PrivateOCharts -and $Production) { 1 } else { 0 }
                if ($Calls.Count -ne $Expected) { throw 'Installed private TLS gate selected the wrong build context' }
                if ($Expected -and ($Calls[0].program -cne 'pwsh' -or
                    $Calls[0].arguments -notcontains (Join-Path $PSScriptRoot 'test-downloader-trust-windows.ps1') -or
                    $Calls[0].arguments -notcontains 'production-install' -or
                    $Calls[0].arguments -notcontains '-OChartsPrepared' -or
                    $Calls[0].arguments -notcontains (Join-Path $Root 'build/ocharts-prepared'))) {
                    throw 'Installed private TLS gate lost its actual validator or prepared-package input'
                }
            }
        }
    }
    $Calls.Clear()
    $Guards=@($Ast.FindAll({param($Node)
        $Node -is [Management.Automation.Language.IfStatementAst] -and
        $Node.Extent.Text.Contains("throw 'Private adapter build requires the explicit disposable Windows integration job'")
    },$true))
    if($Guards.Count -ne 1){throw 'Expected one private integration context guard'}
    $ContextGuard=[scriptblock]::Create($Guards[0].Extent.Text)
    $SavedActions=$env:GITHUB_ACTIONS
    try {
        $PrivateOCharts=$true;$Integration=$false;$env:GITHUB_ACTIONS='true'
        Reject { & $ContextGuard } 'private pristine build'
        $Integration=$true;$env:GITHUB_ACTIONS='false'
        Reject { & $ContextGuard } 'non-disposable private build'
        $env:GITHUB_ACTIONS='true';$env:GITHUB_JOB='other-job'
        Reject { & $ContextGuard } 'unrelated CI job';$env:GITHUB_JOB='windows-integration'
        & $ContextGuard
        $PrivateOCharts=$false;$Integration=$false;$env:GITHUB_ACTIONS='false'
        & $ContextGuard
    } finally { $env:GITHUB_ACTIONS=$SavedActions }
    $null=New-Item -ItemType Directory -Force $Evidence
    foreach($Library in @('curl','zlib')) {
        $Prefix=Join-Path $Root $(if($Library -eq 'curl'){'build/windows-curl-8.22.0/install'}else{'build/windows-zlib-1.3.2/install'})
        $Payload=Join-Path $Prefix "lib/$Library.lib";Put $Payload "actual-$Library-bytes"
        $Outputs=@{};$Outputs["lib/$Library.lib"]=@{sha256=(Digest $Payload);bytes=(Get-Item $Payload).Length}
        Put (Join-Path $Prefix "$Library-build.json") (@{outputs=$Outputs}|ConvertTo-Json -Depth 5)
    }
    Build-PrivateOCharts $false
    if(@($Calls|Where-Object {$_.program -eq 'cmake'}).Count -ne 2 -or
       -not(Test-Path (Join-Path $Evidence 'windows-ocharts-first-build.json'))) {throw 'First pass did not build and record'}
    $OriginalPython=$BuildPython
    $BuildPython=(Join-Path $Root 'other-python.exe');Put $BuildPython 'different interpreter'
    Reject {Build-PrivateOCharts $true} 'interpreter drift between development and production'
    $BuildPython=$OriginalPython
    $Calls.Clear();Build-PrivateOCharts $true
    if(@($Calls|Where-Object {$_.program -eq 'cmake'}).Count -ne 0 -or $Calls.Count -ne 3) {throw 'Reuse rebuilt or skipped validators'}
    Reject {Build-PrivateOCharts $false} 'stale first-build outputs'
    $env:GITHUB_RUN_ID='another-job';Reject {Build-PrivateOCharts $true} 'cross-job reuse';$env:GITHUB_RUN_ID='123'
    $env:GITHUB_SHA='another-commit';Reject {Build-PrivateOCharts $true} 'cross-source reuse';$env:GITHUB_SHA='fixture-commit'
    foreach($Name in @('manifest.json','skager-ocharts-adapter.dll','corresponding-source.zip')) {
        $Path=Join-Path $Root "build/ocharts-package/$Name";$Bytes=[IO.File]::ReadAllBytes($Path)
        Put $Path 'tampered';Reject {Build-PrivateOCharts $true} "changed package $Name";[IO.File]::WriteAllBytes($Path,$Bytes)
    }
    foreach($Path in @((Join-Path $Root 'build/ocharts-prepared/preparation.json'),
        (Join-Path $Root 'build/ocharts-prepared/sdk/curl-build.json'),
        (Join-Path $Root 'build/ocharts-prepared/sdk/lib/curl.lib'),
        (Join-Path $Root 'build/windows-zlib-1.3.2/install/lib/zlib.lib'))) {
        $Bytes=[IO.File]::ReadAllBytes($Path);Put $Path 'tampered'
        Reject {Build-PrivateOCharts $true} "changed live bytes $Path";[IO.File]::WriteAllBytes($Path,$Bytes)
    }
    $Fail='--verify-prepared';Reject {Build-PrivateOCharts $true} 'current source/patch verification failure'
    $Fail='--package';Reject {Build-PrivateOCharts $true} 'current resource/package verification failure';$Fail=''
    $Missing=Join-Path $Root 'build/ocharts-prepared/sdk/lib/curl.lib';Remove-Item $Missing
    Reject {Build-PrivateOCharts $true} 'receipt-only SDK reuse'
    Write-Output 'Private build wiring passed: PowerShell parse/order, eight installed TLS/defer contexts, first build, exact reuse, and 17 rejection cases; no native compile claimed.'
} finally {
    foreach($Key in $OldEnv.Keys){[Environment]::SetEnvironmentVariable($Key,$OldEnv[$Key])}
    if (Test-Path -LiteralPath $Fixture) { Remove-Item -LiteralPath $Fixture -Recurse -Force }
}
