param(
  [Parameter(Mandatory)][ValidateSet('Import','Remove')][string]$Action,
  [Parameter(Mandatory)][string]$Certificate,
  [Parameter(Mandatory)][string]$Ownership
)
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
if(-not $IsWindows -or $env:GITHUB_ACTIONS -cne 'true' -or
   $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or $env:RUNNER_OS -cne 'Windows' -or
   $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work') {
  throw 'Owned trust requires disposable GitHub-hosted Windows in ThereptileII/Work'
}
$cert=[Security.Cryptography.X509Certificates.X509Certificate2]::new($Certificate)
$thumb=$cert.Thumbprint
$digest=(Get-FileHash -LiteralPath $Certificate -Algorithm SHA256).Hash.ToLowerInvariant()
$path="Cert:\LocalMachine\Root\$thumb"
if($Action -ceq 'Import') {
  if((Test-Path -LiteralPath $path) -or (Test-Path -LiteralPath $Ownership)) {
    throw 'Certificate identity or ownership record already exists; cannot take ownership'
  }
  # Persist ownership of this previously absent exact identity before import:
  # caller can clean up a timed-out or partially successful certutil operation.
  $record=[ordered]@{thumbprint=$thumb;certificateSha256=$digest;runId=$env:GITHUB_RUN_ID;attempt=$env:GITHUB_RUN_ATTEMPT}
  $stream=[IO.File]::Open($Ownership,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
  try {
    $bytes=[Text.Encoding]::UTF8.GetBytes(($record|ConvertTo-Json -Compress))
    $stream.Write($bytes,0,$bytes.Length)
    $stream.Flush($true)
  } finally {$stream.Dispose()}
  & (Join-Path $env:SystemRoot 'System32/certutil.exe') -f -addstore Root $Certificate | Out-Host
  if($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $path)){throw 'Owned certificate import failed'}
  $actual=Get-Item -LiteralPath $path
  if([Convert]::ToBase64String($actual.RawData) -cne [Convert]::ToBase64String($cert.RawData)){throw 'Imported certificate bytes changed'}
} else {
  if(-not (Test-Path -LiteralPath $Ownership)){throw 'No owned import record; refusing removal'}
  $record=Get-Content -LiteralPath $Ownership -Raw|ConvertFrom-Json
  if($record.thumbprint -cne $thumb -or $record.certificateSha256 -cne $digest -or
     $record.runId -cne $env:GITHUB_RUN_ID -or $record.attempt -cne $env:GITHUB_RUN_ATTEMPT) {
    throw 'Owned certificate identity/run changed'
  }
  if(Test-Path -LiteralPath $path) {
    $actual=Get-Item -LiteralPath $path
    if([Convert]::ToBase64String($actual.RawData) -cne [Convert]::ToBase64String($cert.RawData)){throw 'Refusing changed certificate'}
    Remove-Item -LiteralPath $path -Force
  }
  if(Test-Path -LiteralPath $path){throw 'Owned root remains after removal'}
}
[ordered]@{action=$Action;thumbprint=$thumb;certificateSha256=$digest;verified=$true}|ConvertTo-Json -Compress
$cert.Dispose()
