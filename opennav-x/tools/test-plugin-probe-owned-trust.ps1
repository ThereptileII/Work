# Actual helper proof only; no application, SDK, network TLS or package execution.
param()
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if(-not $IsWindows -or $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_OS -cne 'Windows' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work') { throw 'Disposable GitHub-hosted Windows in ThereptileII/Work only' }
$Root=Split-Path $PSScriptRoot -Parent
$Helper=Join-Path $PSScriptRoot 'plugin-probe-owned-trust.ps1'
$Evidence=Join-Path $Root 'evidence/local/plugin-probe-owned-trust'
if(Test-Path -LiteralPath $Evidence) { throw 'Proof evidence destination must be new' }
$null=New-Item -ItemType Directory -Path $Evidence
$Receipt=Join-Path $Evidence 'owned-ca.txt'
$ExistingReceipt=Join-Path $Evidence 'existing-receipt.txt'
$Certificate=Join-Path $Evidence 'owned-public-ca.cer'
$Before=@(Get-ChildItem Cert:\LocalMachine\Root | ForEach-Object Thumbprint | Sort-Object)
$Cases=[Collections.Generic.List[object]]::new()
$Key=$null;$Ca=$null;$Thumbprint=$null;$Failure=$null;$CleanupPassed=$false
function Digest([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function Chain-Trusted($Certificate) {
  $Chain=[Security.Cryptography.X509Certificates.X509Chain]::new($true)
  try {
    $Chain.ChainPolicy.RevocationMode=[Security.Cryptography.X509Certificates.X509RevocationMode]::NoCheck
    $Chain.ChainPolicy.DisableCertificateDownloads=$true
    return $Chain.Build($Certificate)
  } finally { $Chain.Dispose() }
}
try {
  $Tokens=$null;$Errors=$null
  $null=[Management.Automation.Language.Parser]::ParseFile($Helper,[ref]$Tokens,[ref]$Errors)
  if(@($Errors).Count) { throw 'Actual trust helper did not parse' }
  $Key=[Security.Cryptography.RSA]::Create(2048)
  $Request=[Security.Cryptography.X509Certificates.CertificateRequest]::new("CN=SKAGER-plugin-trust-proof-$([guid]::NewGuid().ToString('N'))",$Key,[Security.Cryptography.HashAlgorithmName]::SHA256,[Security.Cryptography.RSASignaturePadding]::Pkcs1)
  $Request.CertificateExtensions.Add([Security.Cryptography.X509Certificates.X509BasicConstraintsExtension]::new($true,$false,0,$true))
  $Usage=[Security.Cryptography.X509Certificates.X509KeyUsageFlags]::KeyCertSign -bor [Security.Cryptography.X509Certificates.X509KeyUsageFlags]::CrlSign
  $Request.CertificateExtensions.Add([Security.Cryptography.X509Certificates.X509KeyUsageExtension]::new($Usage,$true))
  $Ca=$Request.CreateSelfSigned([DateTimeOffset]::UtcNow.AddMinutes(-5),[DateTimeOffset]::UtcNow.AddDays(1))
  $Thumbprint=$Ca.Thumbprint
  [IO.File]::WriteAllBytes($Certificate,$Ca.Export([Security.Cryptography.X509Certificates.X509ContentType]::Cert))
  if((Test-Path -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint") -or (Chain-Trusted $Ca)) { throw 'Fresh proof root was already trusted' }
  [IO.File]::WriteAllText($ExistingReceipt,'foreign receipt: must remain byte-identical')
  $ReceiptHash=Digest $ExistingReceipt
  $Refused=$false
  try { & $Helper -Operation Import -Certificate $Certificate -Receipt $ExistingReceipt } catch { $Refused=$true;$_.ToString()|Set-Content (Join-Path $Evidence 'existing-receipt-refusal.txt') }
  if(-not $Refused -or (Digest $ExistingReceipt) -cne $ReceiptHash -or (Test-Path -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint")) { throw 'Existing receipt was not refused without mutation' }
  $Cases.Add(@{name='existing-receipt-refused';status='passed';receiptUnchanged=$true;certificateAbsent=$true})
  & $Helper -Operation Import -Certificate $Certificate -Receipt $Receipt | Tee-Object -FilePath (Join-Path $Evidence 'import.log')
  $Imported=Get-Item -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint"
  if([IO.File]::ReadAllText($Receipt) -cne $Thumbprint -or [Convert]::ToBase64String($Imported.RawData) -cne [Convert]::ToBase64String($Ca.RawData) -or -not(Chain-Trusted $Ca)) { throw 'Exact certificate, receipt or Windows chain import verification failed' }
  $Cases.Add(@{name='actual-owned-import';status='passed';untrustedBefore=$true;windowsMachineChainTrustedAfter=$true;exactBytes=$true;thumbprint=$Thumbprint})
  $Refused=$false
  try { & $Helper -Operation Remove -Certificate $Certificate -Receipt $ExistingReceipt } catch { $Refused=$true;$_.ToString()|Set-Content (Join-Path $Evidence 'foreign-removal-refusal.txt') }
  if(-not $Refused -or (Digest $ExistingReceipt) -cne $ReceiptHash -or -not(Test-Path -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint")) { throw 'Non-owner removal was not refused without mutation' }
  $Cases.Add(@{name='non-owner-removal-refused';status='passed';receiptUnchanged=$true;certificateRetained=$true})
} catch { $Failure=$_.ToString() }
finally {
  try {
    if(Test-Path -LiteralPath $Receipt) { & $Helper -Operation Remove -Certificate $Certificate -Receipt $Receipt | Tee-Object -FilePath (Join-Path $Evidence 'cleanup.log') }
    if($Thumbprint -and (Test-Path -LiteralPath "Cert:\LocalMachine\Root\$Thumbprint")) { throw 'Owned proof certificate remains after actual cleanup' }
    $After=@(Get-ChildItem Cert:\LocalMachine\Root | ForEach-Object Thumbprint | Sort-Object)
    if(($Before -join "`n") -cne ($After -join "`n")) { throw 'Machine root inventory changed after exact cleanup' }
    $CleanupPassed=$true
  } catch { $Failure=$_.ToString() }
  if($Ca) { $Ca.Dispose() };if($Key) { $Key.Dispose() }
  [ordered]@{status=$(if($Failure){'failed'}else{'passed'});scope='Actual plugin probe trust helper import, receipt protections and exact cleanup only; no TLS, application, package or boat acceptance';repository=$env:GITHUB_REPOSITORY;commit=$env:GITHUB_SHA;runId=$env:GITHUB_RUN_ID;runAttempt=$env:GITHUB_RUN_ATTEMPT;helperSha256=(Digest $Helper);proofSha256=(Digest $PSCommandPath);cases=@($Cases.ToArray());exactOwnedCleanup=$CleanupPassed;error=$Failure} | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Evidence 'summary.json') -Encoding utf8
}
if($Failure) { throw $Failure }
