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
if ($Call -lt $Text.IndexOf('Maintained curl manifest does not prove') -or
    $Call -gt $Text.IndexOf("Run cmake (@('-S',")) {
    throw 'Private build must follow maintained dependency checks and precede host configure'
}
if ($Text -notmatch '\[switch\]\$PrivateOCharts\)' -or
    $Text -notmatch '"-DSKAGER_OCHARTS_PACKAGE=\$OChartsPackage"' -or
    $Text.IndexOf('if ($PrivateOCharts -and $Production)') -lt $Text.IndexOf("Run ctest @(")) {
    throw 'Opt-in/package/installed TLS gate wiring changed'
}
$Fixture = Join-Path ([IO.Path]::GetTempPath()) ('ocharts-build-wiring-'+[guid]::NewGuid().ToString('N'))
$Root = $Fixture
$Evidence = Join-Path $Root 'evidence/local'
$Source = Join-Path $Root 'build/integration-source'
$ZlibPrefix = Join-Path $Root 'build/windows-zlib-1.3.2/install'
$Calls = [Collections.Generic.List[object]]::new()
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
    if ($Fail -and $Arguments -contains $Fail) { throw "Simulated native/validator failure: $Fail" }
    if ($Arguments -contains '--curl-prefix') {
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
    Write-Output 'Private build wiring passed: PowerShell parse/order, first build, exact reuse, and 16 rejection cases; no native compile claimed.'
} finally {
    foreach($Key in $OldEnv.Keys){[Environment]::SetEnvironmentVariable($Key,$OldEnv[$Key])}
    Remove-Item -LiteralPath $Fixture -Recurse -Force
}
