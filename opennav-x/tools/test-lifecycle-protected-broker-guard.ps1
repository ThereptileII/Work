# Focused disposable test of the old Lifecycle's protected-enrollment refusal.
# Only the fixed observation function is replaced inside this test process;
# production has no caller-supplied path, switch or injectable absence value.
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
  throw 'The actual Windows Lifecycle filesystem contract requires native Windows.'
}
$engine = Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'
$parseErrors = @()
$ast = [Management.Automation.Language.Parser]::ParseFile($engine, [ref]$null, [ref]$parseErrors)
if ($parseErrors.Count -ne 0) { throw ($parseErrors | Out-String) }
$definitions = @($ast.EndBlock.Statements | Where-Object { $_ -is [Management.Automation.Language.FunctionDefinitionAst] })
$required = @('Get-ProtectedBrokerObservation','Assert-NoProtectedBrokerEnrollment','Assert-LifecycleMutation','AtomicJson')
foreach ($name in $required) {
  if (@($definitions | Where-Object { $_.Name -ceq $name }).Count -ne 1) { throw "Missing unique Lifecycle guard function $name" }
}
$transaction = @($ast.EndBlock.Statements | Where-Object { $_ -is [Management.Automation.Language.TryStatementAst] })
if ($transaction.Count -ne 1) { throw 'Unexpected Lifecycle transaction boundary.' }
$entry = $transaction[0].Extent.Text
$firstGuard = $entry.IndexOf("if (`$Action -ne 'Diagnostics') { Assert-NoProtectedBrokerEnrollment }", [StringComparison]::Ordinal)
$firstWrite = $entry.IndexOf('New-Item -ItemType Directory -Path $Root', [StringComparison]::Ordinal)
if ($firstGuard -lt 0 -or $firstWrite -lt 0 -or $firstGuard -ge $firstWrite) {
  throw 'Protected broker guard must precede the first installation-root mutation.'
}
$central = @($definitions | Where-Object { $_.Name -ceq 'Assert-LifecycleMutation' })[0].Extent.Text
$durable = @($definitions | Where-Object { $_.Name -ceq 'AtomicJson' })[0].Extent.Text
if (-not $central.Contains('Assert-NoProtectedBrokerEnrollment') -or
    ([regex]::Matches($durable, 'Assert-LifecycleMutation')).Count -ne 2) {
  throw 'Repeated mutation and durable-record checks must retain the protected guard.'
}
foreach ($definition in $definitions) { . ([scriptblock]::Create($definition.Extent.Text)) }

# These two overrides exist only in this disposable test. No machine object is
# observed or changed; the production function is never called here.
$script:Observations = New-Object 'System.Collections.Generic.Queue[object]'
$script:SignedChecks = 0
function Get-ProtectedBrokerObservation {
  if ($script:Observations.Count -eq 0) { throw 'Missing isolated observation.' }
  $next = $script:Observations.Dequeue()
  if ($next -is [Exception]) { throw $next }
  return $next
}
function Assert-NoSignedUpdateArmed([string]$InstallationRoot,[bool]$AllowMissingRoot=$false) {
  $script:SignedChecks++
}
function ExpectRefusal([scriptblock]$Operation,[string]$Reason) {
  try { & $Operation; throw "Accepted $Reason" }
  catch { if ($_.Exception.Message -ceq "Accepted $Reason") { throw } }
}

$fixture = Join-Path ([IO.Path]::GetTempPath()) ('SkagerProtectedGuard-' + [guid]::NewGuid().ToString('N'))
$null = [IO.Directory]::CreateDirectory($fixture)
try {
  $Root = $fixture
  $Action = 'Install'
  $Utf8 = New-Object System.Text.UTF8Encoding($false)
  $record = Join-Path $fixture 'owned.json'
  $script:Observations.Enqueue($false)
  Assert-LifecycleMutation
  if ($script:SignedChecks -ne 1) { throw 'Absent protected state did not preserve old armed check.' }
  foreach ($unknown in @($true, $null, 'false', [InvalidOperationException]::new('SCM unavailable'))) {
    $script:Observations.Enqueue($unknown)
    ExpectRefusal { Assert-LifecycleMutation } 'present or unknown protected state'
  }
  if ($script:SignedChecks -ne 1) { throw 'Protected refusal reached old mutation checks.' }
  $script:Observations.Enqueue($false)
  $script:Observations.Enqueue($false)
  AtomicJson $record @{owner='old-layout'}
  $before = [IO.File]::ReadAllText($record)
  $script:Observations.Enqueue($true)
  ExpectRefusal { AtomicJson $record @{owner='unexpected'} } 'durable write with protected service'
  if ([IO.File]::ReadAllText($record) -cne $before) { throw 'Protected presence changed durable old-layout bytes.' }
  $script:Observations.Enqueue($false)
  $script:Observations.Enqueue($true)
  ExpectRefusal { AtomicJson $record @{owner='late'} } 'late protected namespace before commit'
  if ([IO.File]::ReadAllText($record) -cne $before -or @(Get-ChildItem -LiteralPath $fixture -Filter 'owned.json.*.tmp').Count -ne 0) {
    throw 'Late protected state replaced the durable record or retained temporary bytes.'
  }
  $Action = 'Diagnostics'
  $script:Observations.Enqueue($true)
  AtomicJson (Join-Path $fixture 'diagnostics.json') @{status='read-only'}
  if ($script:Observations.Count -ne 1) { throw 'Diagnostics consulted the protected mutation guard.' }
  Write-Host 'Lifecycle protected-broker refusal and read-only diagnostics passed.'
} finally {
  Remove-Item -LiteralPath $fixture -Recurse -Force
}
