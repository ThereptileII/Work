# First-use compilation of the actual installer observer; no installation entry
# point, application launch or machine mutation. Called in fresh x86/x64 hosts.
[CmdletBinding()]
param([ValidateSet('available','unavailable')][string]$CompilerTemp = 'available')
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
if ($env:GITHUB_ACTIONS -ne 'true' -or $env:OS -ne 'Windows_NT' -or
    $PSVersionTable.PSVersion.Major -ne 5 -or $PSVersionTable.PSVersion.Minor -ne 1) {
  throw 'Disposable native Windows PowerShell 5.1 only.'
}
if ('SkagerLegacyBrokerObservation' -as [type]) { throw 'Observer type was already cached; this is not a cold test.' }
$source = Join-Path $PSScriptRoot '../installer/windows/Lifecycle.ps1'
$parseErrors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile($source, [ref]$null, [ref]$parseErrors)
if ($parseErrors) { throw ($parseErrors | Out-String) }
$definitions = @($ast.EndBlock.Statements | Where-Object {
  $_ -is [Management.Automation.Language.FunctionDefinitionAst] -and $_.Name -ceq 'Get-ProtectedBrokerObservation'
})
if ($definitions.Count -ne 1) { throw 'Expected the unique actual protected-observation function.' }
. ([scriptblock]::Create($definitions[0].Extent.Text))
$fixture = Join-Path ([IO.Path]::GetTempPath()) ('Skager cold observer ' + [guid]::NewGuid().ToString('N'))
$providerDirectory = Join-Path $fixture 'provider'
$processDirectory = Join-Path $fixture 'process'
$null = [IO.Directory]::CreateDirectory($providerDirectory)
$null = [IO.Directory]::CreateDirectory($processDirectory)
$originalDirectory = [Environment]::CurrentDirectory
$originalTemp = $env:TEMP
$originalTmp = $env:TMP
$pushed = $false
try {
  # A real native Windows DLL renamed System.dll models the metadata collision,
  # independently of NSIS version. It is never loaded or changed by this test.
  $nativeDll = Join-Path ([Environment]::GetFolderPath('SystemX86')) 'kernel32.dll'
  foreach ($directory in @($providerDirectory,$processDirectory)) {
    Copy-Item -LiteralPath $nativeDll -Destination (Join-Path $directory 'System.dll')
  }
  $nativeMetadataRefused = $false
  try { $null = [Reflection.AssemblyName]::GetAssemblyName((Join-Path $processDirectory 'System.dll')) }
  catch {
    $errorCause = $_.Exception
    while ($errorCause.InnerException) { $errorCause = $errorCause.InnerException }
    if ($errorCause -isnot [BadImageFormatException]) { throw }
    $nativeMetadataRefused = $true
  }
  if (-not $nativeMetadataRefused) { throw 'Shadow fixture unexpectedly contains managed assembly metadata.' }
  Push-Location -LiteralPath $providerDirectory
  $pushed = $true
  [Environment]::CurrentDirectory = $processDirectory
  if ($CompilerTemp -ceq 'unavailable') {
    # A file where CodeDOM needs its temporary directory forces a real compiler
    # I/O failure, without changing the C# source or inventing a parse error.
    $blockedTemp = Join-Path $fixture 'compiler-temp-is-a-file'
    [IO.File]::WriteAllText($blockedTemp, 'not a directory')
    $env:TEMP = $blockedTemp
    $env:TMP = $blockedTemp
    if ([IO.Path]::GetTempPath().TrimEnd('\') -cne $blockedTemp) { throw 'Compiler temp refusal fixture was not applied.' }
  }
  if ('SkagerLegacyBrokerObservation' -as [type]) { throw 'Observer type appeared before the first shadowed compilation.' }
  $compileError = $null
  try { $observed = Get-ProtectedBrokerObservation }
  catch { $compileError = $_ }
  if ((Get-Location).Path -cne $providerDirectory -or [Environment]::CurrentDirectory -cne $processDirectory) {
    throw 'Observer compilation did not restore both distinct caller directories.'
  }
  if ($CompilerTemp -ceq 'available') {
    if ($compileError) { throw $compileError }
    if (-not ('SkagerLegacyBrokerObservation' -as [type]) -or $observed -isnot [bool]) {
      throw 'Cold actual observer compilation and native read-only observation did not complete.'
    }
  } else {
    if (-not $compileError -or ('SkagerLegacyBrokerObservation' -as [type])) {
      throw 'Unavailable compiler temp did not refuse the first real compilation.'
    }
    $compilerIoFailure = $false
    $compilerFailureEvidence = New-Object 'System.Collections.Generic.List[string]'
    $errorCause = $compileError.Exception
    while ($errorCause) {
      $nativeCode = 'none'
      if ($errorCause -is [ComponentModel.Win32Exception]) {
        $nativeCode = [string]$errorCause.NativeErrorCode
        # PS5.1 CodeDOM reports the blocked TEMP directory through CreateProcess.
        # Accept ERROR_PATH_NOT_FOUND only, never an unrelated native failure.
        if ($errorCause.NativeErrorCode -eq 3) { $compilerIoFailure = $true }
      }
      if ($compilerFailureEvidence.Count -lt 8) {
        $compilerFailureEvidence.Add(('type={0}; nativeCode={1}' -f $errorCause.GetType().FullName, $nativeCode))
      }
      if ($errorCause -is [IO.IOException] -or $errorCause -is [UnauthorizedAccessException]) {
        $compilerIoFailure = $true
      }
      $errorCause = $errorCause.InnerException
    }
    Write-Host ('Compiler temp refusal evidence: ' + ($compilerFailureEvidence -join ' | '))
    if (-not $compilerIoFailure) { throw $compileError }
  }
  Write-Host "Cold observer passed: PowerShell 5.1 $([IntPtr]::Size * 8)-bit, compiler temp $CompilerTemp; native System.dll shadow; exact caller directories restored."
} finally {
  $env:TEMP = $originalTemp
  $env:TMP = $originalTmp
  [Environment]::CurrentDirectory = $originalDirectory
  if ($pushed) { Pop-Location }
  Remove-Item -LiteralPath $fixture -Recurse -Force
}
