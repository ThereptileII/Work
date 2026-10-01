param()
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# Exercise the real implementation on any host without running its Windows-only
# producer entry point. The version probe itself is platform-independent.
$Helper = Join-Path $PSScriptRoot 'windows-native-tool-facts.ps1'
$Tokens = $null
$ParseErrors = $null
$Ast = [Management.Automation.Language.Parser]::ParseFile($Helper,[ref]$Tokens,[ref]$ParseErrors)
if ($ParseErrors.Count) { throw 'Native tool-facts helper has a PowerShell parse error' }
foreach ($Name in @('ByteSha','ProbeVersion')) {
    $Definitions = @($Ast.FindAll({param($Node)
        $Node -is [Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -ceq $Name
    },$true))
    if ($Definitions.Count -ne 1) { throw "Expected one $Name function" }
    . ([scriptblock]::Create($Definitions[0].Extent.Text))
}

function Assert([bool]$Condition,[string]$Message) {
    if (-not $Condition) { throw $Message }
}
function Refuses([scriptblock]$Action,[string]$Message) {
    $Rejected = $false
    try { & $Action } catch {
        if ($_.Exception.Message -notlike "*$Message*") { throw }
        $Rejected = $true
    }
    Assert $Rejected "Expected probe refusal: $Message"
}

$Interpreter = [Diagnostics.Process]::GetCurrentProcess().MainModule.FileName
$OriginalDirectory = [Environment]::CurrentDirectory
$Work = Join-Path ([IO.Path]::GetTempPath()) ("xnav-version-probe-$([guid]::NewGuid().ToString('N'))")
$null = New-Item -ItemType Directory -Path $Work
$Encoding = New-Object Text.UTF8Encoding($false)
function RunFixture([string]$Name,[string]$Body) {
    [IO.File]::WriteAllText((Join-Path $Work "$Name.ps1"),$Body,$Encoding)
    ProbeVersion $Interpreter @('-NoProfile','-ExecutionPolicy','Bypass','-File',"$Name.ps1")
}
try {
    [Environment]::CurrentDirectory = $Work
    $A = RunFixture 'order-a' '[Console]::Out.Write("banner`n"); [Console]::Error.Write("usage`n"); exit 7'
    $B = RunFixture 'order-b' '[Console]::Error.Write("usage`n"); [Console]::Out.Write("banner`n"); exit 7'
    Assert ($A.exitCode -eq 7 -and $B.exitCode -eq 7) 'Exact nonzero exit code changed'
    Assert ($A.versionLine -ceq 'banner' -and $B.versionLine -ceq 'banner') 'Version line is not stdout-first'
    Assert ($A.stdoutBytes -eq 7 -and $A.stderrBytes -eq 6) 'Unexpected stream byte counts'
    Assert ($A.stdoutSha256 -ceq $B.stdoutSha256 -and $A.stderrSha256 -ceq $B.stderrSha256) `
        'Reversed stream delivery changed an individual stream digest'

    $DifferentOut = RunFixture 'changed-out' '[Console]::Out.Write("change`n"); [Console]::Error.Write("usage`n"); exit 7'
    $DifferentErr = RunFixture 'changed-err' '[Console]::Out.Write("banner`n"); [Console]::Error.Write("error`n"); exit 7'
    Assert ($DifferentOut.stdoutSha256 -cne $A.stdoutSha256 -and
        $DifferentOut.stderrSha256 -ceq $A.stderrSha256) 'Changed stdout was not isolated'
    Assert ($DifferentErr.stderrSha256 -cne $A.stderrSha256 -and
        $DifferentErr.stdoutSha256 -ceq $A.stdoutSha256) 'Changed stderr was not isolated'

    Refuses { $null = RunFixture 'too-large' '[Console]::Out.Write("x" * 1048577)' } 'exceeds 1 MiB'
    Refuses { $null = ProbeVersion (Join-Path $Work 'absent.exe') @() } 'Native version probe did not start'
    Refuses { $null = RunFixture 'too-slow' 'Start-Sleep -Seconds 31' } 'timed out'
    Write-Output 'Native version probe stream, refusal, and deadline tests passed'
} finally {
    [Environment]::CurrentDirectory = $OriginalDirectory
    Remove-Item -LiteralPath $Work -Recurse -Force
}
