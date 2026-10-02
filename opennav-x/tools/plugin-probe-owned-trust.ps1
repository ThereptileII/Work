param(
  [Parameter(Mandatory)][ValidateSet('Import','Remove')][string]$Operation,
  [Parameter(Mandatory)][string]$Certificate,
  [Parameter(Mandatory)][string]$Receipt
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if($env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted') { throw 'Disposable hosted Windows only' }
$Candidate=[Security.Cryptography.X509Certificates.X509Certificate2]::new($Certificate)
$Thumbprint=$Candidate.Thumbprint
$Path="Cert:\CurrentUser\Root\$Thumbprint"
if($Operation -ceq 'Import') {
  if((Test-Path -LiteralPath $Receipt) -or (Test-Path $Path)) { throw 'CA identity or ownership receipt already exists' }
  # Persist ownership before import, including partial import failure.
  [IO.File]::WriteAllText($Receipt,$Thumbprint)
  $Imported=Import-Certificate -FilePath $Certificate -CertStoreLocation 'Cert:\CurrentUser\Root'
  if($Imported.Thumbprint -cne $Thumbprint -or -not(Test-Path $Path)) { throw 'Owned CA import could not be verified' }
} else {
  if(-not(Test-Path -LiteralPath $Receipt) -or [IO.File]::ReadAllText($Receipt) -cne $Thumbprint) { throw 'Missing exact owned CA receipt' }
  if(Test-Path $Path) { Remove-Item -LiteralPath $Path -Force }
  if(Test-Path $Path) { throw 'Owned CA remains after cleanup' }
}
Write-Output "$Operation verified: $Thumbprint"
