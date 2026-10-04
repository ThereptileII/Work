# Windows PowerShell 5.1; invoked only by the disposable installer lifecycle test.
param(
  [Parameter(Mandatory=$true)][string]$Stage,
  [Parameter(Mandatory=$true)][ValidatePattern('^[a-f0-9]{64}$')][string]$ExpectedExecutableSha256,
  [Parameter(Mandatory=$true)][ValidatePattern('^wxbase32u_vc14x\.dll$')][string]$MissingDependency,
  [Parameter(Mandatory=$true)][string]$Commit,
  [Parameter(Mandatory=$true)][string]$Version,
  [Parameter(Mandatory=$true)][string]$Report
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or
    $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or
    $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work') { throw 'Disposable repository Windows runner required.' }
$source=Join-Path (Split-Path $PSScriptRoot -Parent) 'installer/windows/Lifecycle.ps1'
$tokens=$null; $errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($source,[ref]$tokens,[ref]$errors)
if ($errors.Count) { throw 'Production Lifecycle.ps1 does not parse.' }
# Load only definitions, never the production transaction. Successful execution
# is forbidden here: the missing import must stop SelfTest before ReadJson and
# its later product/policy/profile checks. Those functions are not substituted.
$names=@('PlainPath','PeArchitecture','SelfTest')
$definitions=@($ast.FindAll({param($node)
  $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -in $names
},$true))
foreach ($name in $names) {
  if (@($definitions | Where-Object Name -ceq $name).Count -ne 1) { throw 'Missing or duplicate production function.' }
}
foreach ($definition in $definitions) { . ([scriptblock]::Create($definition.Extent.Text)) }
$Stage=PlainPath $Stage; $Report=PlainPath $Report
$generations=Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'OpenNavXAlpha1\generations'
if ([IO.Path]::GetDirectoryName($Stage) -ine $generations -or
    [IO.Path]::GetFileName($Stage) -cnotmatch '^[a-f0-9]{32}$' -or
    -not [IO.Directory]::Exists($Stage) -or (Test-Path -LiteralPath (Join-Path $Stage 'ownership.json'))) {
  throw 'Only a private unpublished installer fixture stage is allowed.'
}
if ((Test-Path -LiteralPath $Report) -or $Report.StartsWith($Stage+'\',[StringComparison]::OrdinalIgnoreCase)) {
  throw 'Require a new receipt outside the failed stage.'
}
$exe=PlainPath (Join-Path $Stage 'app\opencpn.exe')
if ((Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash.ToLowerInvariant() -cne $ExpectedExecutableSha256 -or
    (Test-Path -LiteralPath (Join-Path $Stage ('app\'+$MissingDependency)))) { throw 'Failed-stage identity or missing import differs.' }
if (@(Get-ChildItem -LiteralPath $Stage -Filter 'loader-*.json').Count) { throw 'Failed stage already contains loader output.' }
Add-Type -TypeDefinition @'
using System.Runtime.InteropServices;
public static class MissingDllProofErrorMode {
  [DllImport("kernel32.dll")] public static extern uint GetErrorMode();
  [DllImport("kernel32.dll")] public static extern uint SetErrorMode(uint mode);
}
'@
# Remove inherited suppression only in this proof process; production SelfTest
# must establish and restore its own error mode around its actual child launch.
$inheritedMode=[MissingDllProofErrorMode]::SetErrorMode(0)
$receipt=[ordered]@{status='failed';sourceSha256=(Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash.ToLowerInvariant();
  executableSha256=$ExpectedExecutableSha256;stage=$Stage;missingDependency=$MissingDependency;
  initialErrorMode=[MissingDllProofErrorMode]::GetErrorMode();restoredErrorMode=$null;error=$null;
  scope='Actual production SelfTest missing-DLL loader refusal only; no installer guard bypass or product qualification'}
try {
  try { $null=SelfTest $Stage $Commit $Version; throw 'Missing-DLL SelfTest unexpectedly succeeded.' }
  catch {
    $receipt.error=$_.Exception.Message
    if ($receipt.error -cne 'Staged executable self-test failed: -1073741515') { throw }
  }
  $receipt.restoredErrorMode=[MissingDllProofErrorMode]::GetErrorMode()
  if ($receipt.initialErrorMode -ne 0 -or $receipt.restoredErrorMode -ne 0) { throw 'SelfTest did not restore error mode.' }
  if (@(Get-ChildItem -LiteralPath $Stage -Filter 'loader-*.json').Count -or
      (Test-Path -LiteralPath (Join-Path $Stage 'ownership.json')) -or
      (Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash.ToLowerInvariant() -cne $ExpectedExecutableSha256) {
    throw 'Failed loader produced output or changed the executable/stage ownership.'
  }
  $receipt.status='passed'
} finally {
  $null=[MissingDllProofErrorMode]::SetErrorMode($inheritedMode)
  $receipt | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $Report -Encoding UTF8
}
