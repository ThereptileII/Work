# Check the missing-DLL helper's actual import block without running its loader
# proof or the installer's entry point. Native checks use real guard APIs only.
[CmdletBinding()]
param(
  [string]$Helper=(Join-Path $PSScriptRoot 'test-installer-missing-dll-selftest.ps1'),
  [string]$Report='',
  [switch]$NativeWindows
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if($NativeWindows -and ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or
   $PSVersionTable.PSVersion.Major -ne 5 -or $PSVersionTable.PSVersion.Minor -ne 1 -or
   $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or
   $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work')) {throw 'Disposable repository Windows PowerShell 5.1 required.'}
$checks=0
function Check([bool]$Condition,[string]$Message) {
  if(-not $Condition){throw $Message}
  $script:checks++;Write-Host ('PASS '+$Message)
}
$parseErrors=$null
$helperAst=[Management.Automation.Language.Parser]::ParseFile($Helper,[ref]$null,[ref]$parseErrors)
if($parseErrors){throw ($parseErrors | Out-String)}
# Execute the same closed setup statements as the actual proof, stopping before
# its Stage/Report validation and any Windows error-mode or process operation.
$statements=@($helperAst.EndBlock.Statements)
$first=@($statements | Where-Object {$_ -is [Management.Automation.Language.AssignmentStatementAst] -and $_.Left.Extent.Text -ceq '$source'})
$last=@($statements | Where-Object {$_ -is [Management.Automation.Language.AssignmentStatementAst] -and $_.Left.Extent.Text -ceq '$Stage'})
if($first.Count -ne 1 -or $last.Count -ne 1){throw 'Unique helper import boundaries required.'}
$imports=@($statements | Where-Object {$_.Extent.StartOffset -ge $first[0].Extent.StartOffset -and $_.Extent.StartOffset -lt $last[0].Extent.StartOffset})
$helperTools=Split-Path ([IO.Path]::GetFullPath($Helper)) -Parent
# An in-memory AST block has no script-directory metadata. Supply only that
# original file context; the imported setup statements themselves are unchanged.
. ([scriptblock]::Create('$PSScriptRoot=$helperTools' + "`n" + (($imports | ForEach-Object {$_.Extent.Text}) -join "`n")))
Check ($null -ne (Get-Command SelfTest -CommandType Function -ErrorAction SilentlyContinue)) 'Actual helper imports production SelfTest'
foreach($name in @('Get-ProtectedBrokerObservation','Assert-NoProtectedBrokerEnrollment','Assert-LifecycleMutation',
                    'Assert-NoSignedUpdateArmed','Initialize-UpdatePipeType')) {
  $command=Get-Command $name -CommandType Function -ErrorAction SilentlyContinue
  Check ($null -ne $command) ('Actual helper imports guard dependency '+$name)
  # Compare the loaded function AST with the corresponding production source;
  # a no-op shim or test replacement cannot satisfy this boundary.
  $origin=if($name -cin @('Assert-NoSignedUpdateArmed','Initialize-UpdatePipeType')){$transactionSource}else{$source}
  $production=[Management.Automation.Language.Parser]::ParseFile($origin,[ref]$null,[ref]$parseErrors)
  if($parseErrors){throw ($parseErrors | Out-String)}
  $definition=@($production.FindAll({param($node)
    $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq $name
  },$true))
  Check ($definition.Count -eq 1 -and $command.ScriptBlock.Ast.Extent.Text -ceq $definition[0].Extent.Text) ('Guard is exact production AST: '+$name)
}
Check ($Root -ceq (Join-Path ([Environment]::GetFolderPath('LocalApplicationData')) 'OpenNavXAlpha1')) 'Guard root is the actual Lifecycle installation root'
Check ($transactionSource -ceq (Join-Path (Split-Path $source -Parent) 'UpdateTransaction.ps1')) 'Armed barrier is sourced beside the same Lifecycle'
if($NativeWindows) {
  $temporary=Join-Path ([IO.Path]::GetTempPath()) ('skager-missing-dll-imports-'+[guid]::NewGuid().ToString('N'))
  $null=[IO.Directory]::CreateDirectory($temporary)
  $Root=Join-Path $temporary 'installation'
  $null=[IO.Directory]::CreateDirectory($Root)
  try {
    Assert-LifecycleMutation
    Check $true 'Real native guards accept an unarmed root with no protected enrollment'
    $marker=Join-Path $Root 'signed-update-armed.jsonl'
    [IO.File]::WriteAllText($marker,'inert presence-only refusal fixture')
    $refused=$false
    try {SelfTest (Join-Path $Root 'never-created-stage') ('a'*40) '0.4.0-beta2.4'}
    catch {$refused=$_.Exception.Data['SkagerSignedUpdateRestriction'] -eq $true}
    Check ($refused -and [IO.File]::ReadAllText($marker) -ceq 'inert presence-only refusal fixture' -and
      -not [IO.Directory]::Exists((Join-Path $Root 'never-created-stage'))) 'Armed root stops actual SelfTest before loader work and remains unchanged'
    [IO.File]::Delete($marker)
    $Root=Join-Path $temporary 'missing-root'
    $refused=$false
    try {SelfTest (Join-Path $Root 'never-created-stage') ('a'*40) '0.4.0-beta2.4'}
    catch {$refused=$_.Exception.Data['SkagerSignedUpdateRestriction'] -eq $true}
    Check ($refused -and -not [IO.Directory]::Exists($Root)) 'Missing root stops actual SelfTest before loader work without creating it'
  } finally {Remove-Item -LiteralPath $temporary -Recurse -Force}
}
$result=[pscustomobject]@{status='passed';checks=$checks;nativeGuards=[bool]$NativeWindows;
  helperSha256=(Hash $Helper);lifecycleSha256=(Hash $source);transactionSha256=(Hash $transactionSource);
  powershellVersion=$PSVersionTable.PSVersion.ToString();processBits=([IntPtr]::Size*8);
  missingDllLoaderExecuted=$false;productAcceptance=$false}
$json=$result | ConvertTo-Json
if($Report) {
  $bytes=(New-Object Text.UTF8Encoding($false)).GetBytes($json+"`n")
  $stream=New-Object IO.FileStream($Report,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
  try {$stream.Write($bytes,0,$bytes.Length);$stream.Flush($true)} finally {$stream.Dispose()}
}
$json
