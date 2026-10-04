# Focused native proof only: actual trust helpers, no application build or TLS claim.
param()
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
if (-not $IsWindows -or $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work') {
  throw 'Restricted to a disposable GitHub-hosted Windows runner in ThereptileII/Work'
}
$Root = Split-Path $PSScriptRoot -Parent
$Source = Join-Path $Root 'tools/test-downloader-trust-windows.ps1'
$Evidence = Join-Path $Root 'evidence/local/native-trust-import-fixed'
if (Test-Path -LiteralPath $Evidence) { throw 'Proof evidence destination must be new' }
$null = New-Item -ItemType Directory -Path $Evidence
$Names = @('Digest','Write-TrustProgress','Invoke-BoundedProbe','Import-OwnedTrust','Remove-OwnedTrust')
$Tokens = $null; $Errors = $null
$Ast = [Management.Automation.Language.Parser]::ParseFile($Source,[ref]$Tokens,[ref]$Errors)
if ($Errors.Count) { throw 'Actual trust source does not parse' }
$Definitions = foreach ($Name in $Names) {
  $Matches = @($Ast.FindAll({param($Node) $Node -is [Management.Automation.Language.FunctionDefinitionAst] -and $Node.Name -ceq $Name},$true))
  if ($Matches.Count -ne 1) { throw "Expected exactly one actual $Name function" }
  $Matches[0].Extent.Text
}
$Extracted = $Definitions -join "`n`n"
$Extracted | Set-Content -LiteralPath (Join-Path $Evidence 'actual-functions.ps1') -Encoding utf8
. ([scriptblock]::Create($Extracted))
$script:TrustedThumbprint = $null
$Cases = [Collections.Generic.List[object]]::new()
$Before = @(Get-ChildItem Cert:\LocalMachine\Root | ForEach-Object Thumbprint | Sort-Object)
$CaKey = $null; $Ca = $null; $Failure = $null; $CleanupPassed = $false
$Thumbprint = $null
try {
  $CaKey = [Security.Cryptography.RSA]::Create(2048)
  $Request = [Security.Cryptography.X509Certificates.CertificateRequest]::new(
    "CN=SKAGER-owned-trust-proof-$([guid]::NewGuid().ToString('N'))", $CaKey,
    [Security.Cryptography.HashAlgorithmName]::SHA256, [Security.Cryptography.RSASignaturePadding]::Pkcs1)
  $Request.CertificateExtensions.Add([Security.Cryptography.X509Certificates.X509BasicConstraintsExtension]::new($true,$false,0,$true))
  $Usage = [Security.Cryptography.X509Certificates.X509KeyUsageFlags]::KeyCertSign -bor [Security.Cryptography.X509Certificates.X509KeyUsageFlags]::CrlSign
  $Request.CertificateExtensions.Add([Security.Cryptography.X509Certificates.X509KeyUsageExtension]::new($Usage,$true))
  $Ca = $Request.CreateSelfSigned([DateTimeOffset]::UtcNow.AddMinutes(-5),[DateTimeOffset]::UtcNow.AddDays(1))
  $Thumbprint = $Ca.Thumbprint
  $Certificate = Join-Path $Evidence 'owned-public-ca.cer'
  [IO.File]::WriteAllBytes($Certificate,$Ca.Export([Security.Cryptography.X509Certificates.X509ContentType]::Cert))
  if (Test-Path -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint") { throw 'Fresh proof CA already trusted; refusing ownership' }
  $Chain = [Security.Cryptography.X509Certificates.X509Chain]::new($true)
  try {
    $Chain.ChainPolicy.RevocationMode = [Security.Cryptography.X509Certificates.X509RevocationMode]::NoCheck
    $Chain.ChainPolicy.DisableCertificateDownloads = $true
    if ($Chain.Build($Ca) -or -not ($Chain.ChainStatus.Status -contains [Security.Cryptography.X509Certificates.X509ChainStatusFlags]::UntrustedRoot)) { throw 'Fresh CA was not rejected as an untrusted root' }
  } finally { $Chain.Dispose() }
  Import-OwnedTrust $Certificate
  if ($script:TrustedThumbprint -cne $Thumbprint) { throw 'Actual import owns a different certificate' }
  $Chain = [Security.Cryptography.X509Certificates.X509Chain]::new($true)
  try {
    $Chain.ChainPolicy.RevocationMode = [Security.Cryptography.X509Certificates.X509RevocationMode]::NoCheck
    $Chain.ChainPolicy.DisableCertificateDownloads = $true
    if (-not $Chain.Build($Ca) -or $Chain.ChainElements.Count -ne 1 -or $Chain.ChainElements[0].Certificate.Thumbprint -cne $Thumbprint) { throw 'Windows machine-context chain did not trust the exact owned root' }
  } finally { $Chain.Dispose() }
  $Cases.Add(@{name='actual-owned-import';status='passed';untrustedBefore=$true;windowsMachineChainTrustedAfter=$true;thumbprint=$Thumbprint})
  $Pwsh = Join-Path $PSHOME 'pwsh.exe'
  $Normal = Invoke-BoundedProbe 'proof-normal' $Pwsh @('-NoProfile','-NonInteractive','-Command','[Console]::Out.Write("normal-marker"); exit 0')
  if ($Normal.exitCode -ne 0 -or $Normal.output -notmatch 'normal-marker') { throw 'Normal bounded helper result differs' }
  $Cases.Add(@{name='bounded-normal-exit';status='passed';exitCode=$Normal.exitCode})
  $Rejected = Invoke-BoundedProbe 'proof-rejected' $Pwsh @('-NoProfile','-NonInteractive','-Command','[Console]::Error.Write("rejected-marker"); exit 7')
  if ($Rejected.exitCode -ne 7 -or $Rejected.output -notmatch 'rejected-marker') { throw 'Nonzero bounded helper result was lost' }
  $Cases.Add(@{name='bounded-rejected-exit';status='passed';exitCode=$Rejected.exitCode})
  $TimeoutThrown = $false
  try { $null = Invoke-BoundedProbe 'proof-timeout' $Pwsh @('-NoProfile','-NonInteractive','-Command','Start-Sleep -Seconds 120') -TimeoutSeconds 1 }
  catch { $TimeoutThrown = $true }
  $Timeout = Get-Content -LiteralPath (Join-Path $Evidence 'proof-timeout.process.json') -Raw | ConvertFrom-Json
  if (-not $TimeoutThrown -or $Timeout.timedOut -ne $true -or $Timeout.timeoutSeconds -ne 1 -or (Get-Process -Id $Timeout.processId -ErrorAction SilentlyContinue)) { throw 'Timeout did not retain its receipt and remove the owned child' }
  $Cases.Add(@{name='bounded-timeout';status='passed';timeoutSeconds=1;ownedChildAbsent=$true})
} catch {
  $Failure = $_.ToString()
} finally {
  try {
    Remove-OwnedTrust
    if ($Thumbprint -and (Test-Path -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint")) { throw 'Exact owned proof root remains after cleanup' }
    $After = @(Get-ChildItem Cert:\LocalMachine\Root | ForEach-Object Thumbprint | Sort-Object)
    if (($Before -join "`n") -cne ($After -join "`n")) { throw 'Machine root inventory differs after exact cleanup' }
    $CleanupPassed = $true
  } catch { $Failure = $_.ToString() }
  if ($Ca) { $Ca.Dispose() }; if ($CaKey) { $CaKey.Dispose() }
  [ordered]@{status=$(if($Failure){'failed'}else{'passed'});scope='Actual helper import, Windows root chain, bounded process behavior and owned cleanup only; no TLS, application, package or boat acceptance';repository=$env:GITHUB_REPOSITORY;commit=$env:GITHUB_SHA;runId=$env:GITHUB_RUN_ID;runAttempt=$env:GITHUB_RUN_ATTEMPT;sourceSha256=(Digest $Source);proofSha256=(Digest $PSCommandPath);extractedSha256=(Digest (Join-Path $Evidence 'actual-functions.ps1'));cases=@($Cases.ToArray());exactOwnedCleanup=$CleanupPassed;error=$Failure} | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Evidence 'summary.json') -Encoding utf8
}
if ($Failure) { throw $Failure }
