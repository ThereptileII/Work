# Inert disposable policy tests. No certificate provisioning, key use or signing.
[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT -and $PSVersionTable.PSVersion.Major -gt 5) {
  & (Join-Path ([Environment]::GetFolderPath('System')) 'WindowsPowerShell/v1.0/powershell.exe') -NoProfile -NonInteractive -ExecutionPolicy Bypass -File $PSCommandPath
  exit $LASTEXITCODE
}
$Mode='caller-sentinel'
. (Join-Path $PSScriptRoot 'sign-release-artifact.ps1')
function Check([bool]$Condition,[string]$Message) { if (-not $Condition) { throw $Message } }
function Reject([scriptblock]$Action) {
  $rejected=$false
  try { & $Action } catch { $rejected=$true }
  Check $rejected 'Expected fail-closed signing policy rejection.'
}
Check ($Mode -ceq 'caller-sentinel') 'Import changed caller action.'
$hash='a'*64; $thumb='B'*40; $publisher='CN=Fixture Publisher, O=Fixture'; $timestamp='https://timestamp.example.test/rfc3161'
Assert-SigningPolicy $hash $hash $thumb $publisher $timestamp
foreach ($endpoint in @('http://timestamp.example.test','https://name:secret@timestamp.example.test','https://timestamp.example.test/?x=1','https://timestamp.example.test/#fragment','https://timestamp.example.test:8443','https://127.0.0.1','https://localhost','https://timestamp.example.test\bad',"https://timestamp.example.test/`n")) {
  Reject { Assert-SigningPolicy $hash $hash $thumb $publisher $endpoint }
}
Reject { Assert-SigningPolicy ('a'*63) $hash $thumb $publisher $timestamp }
Reject { Assert-SigningPolicy $hash ('g'*64) $thumb $publisher $timestamp }
Reject { Assert-SigningPolicy $hash $hash ('B '*20) $publisher $timestamp }
Reject { Assert-SigningPolicy $hash $hash $thumb '' $timestamp }
$arguments=Get-SigningArguments $thumb $timestamp 'C:\private\Setup.exe'
Check (($arguments -join '|') -ceq ('sign|/s|My|/sha1|'+$thumb+'|/fd|SHA256|/tr|'+$timestamp+'|/td|SHA256|C:\private\Setup.exe')) 'Signing command is not exact pinned CurrentUser/SHA256/RFC3161 policy.'
foreach ($forbidden in @('/a','/sm','/f','/p','/n','/as','/t')) { Check ($arguments -cnotcontains $forbidden) 'Unsafe certificate/timestamp option present.' }

$cert=[pscustomobject]@{Thumbprint=$thumb;Subject=$publisher;HasPrivateKey=$true;NotBefore=[DateTime]::UtcNow.AddDays(-1);NotAfter=[DateTime]::UtcNow.AddDays(1);Extensions=@([pscustomobject]@{Oid=[pscustomobject]@{Value='2.5.29.37'};EnhancedKeyUsages=@([pscustomobject]@{Value='1.3.6.1.5.5.7.3.3'})})}
Assert-SigningCertificate $cert $thumb $publisher
Reject { Assert-SigningCertificate $cert ('C'*40) $publisher }
Reject { Assert-SigningCertificate $cert $thumb 'CN=Substring' }
$cert.HasPrivateKey=$false; Reject { Assert-SigningCertificate $cert $thumb $publisher }; $cert.HasPrivateKey=$true
$cert.NotAfter=[DateTime]::UtcNow.AddSeconds(-1); Reject { Assert-SigningCertificate $cert $thumb $publisher }; $cert.NotAfter=[DateTime]::UtcNow.AddDays(1)
$cert.Extensions[0].EnhancedKeyUsages[0].Value='1.3.6.1.5.5.7.3.1'; Reject { Assert-SigningCertificate $cert $thumb $publisher }
$signature=[pscustomobject]@{Status='Valid';SignatureType='Authenticode';SignerCertificate=$cert;TimeStamperCertificate=[pscustomobject]@{Thumbprint='C'*40}}
Assert-SigningResult $signature $thumb $publisher
$signature.TimeStamperCertificate=$null; Reject { Assert-SigningResult $signature $thumb $publisher }
$signature.TimeStamperCertificate=[pscustomobject]@{Thumbprint='C'*40}; $signature.Status='HashMismatch'; Reject { Assert-SigningResult $signature $thumb $publisher }
$signature.Status='Valid'; $signature.SignatureType='Catalog'; Reject { Assert-SigningResult $signature $thumb $publisher }
Write-Host 'PASS: exact certificate identity, validity/EKU/private-key policy, HTTPS endpoint policy, SHA256 command, embedded signature and timestamp requirements.'

$fixture=Join-Path ([IO.Path]::GetTempPath()) ('skager-sign-policy-'+[guid]::NewGuid().ToString('N'))
try {
  $null=New-Item -ItemType Directory -Path $fixture
  $artifact=Join-Path $fixture 'Setup.exe'; $tool=Join-Path $fixture 'signtool.exe'
  $bytes=New-Object byte[] 512; $bytes[0]=0x4d; $bytes[1]=0x5a; $bytes[0x3c]=128; $bytes[128]=0x50; $bytes[129]=0x45; $bytes[152]=0x0b; $bytes[153]=0x01
  [IO.File]::WriteAllBytes($artifact,$bytes); [IO.File]::WriteAllText($tool,'INERT; NEVER EXECUTED')
  $stream=[IO.File]::OpenRead($artifact)
  try { Assert-SigningPE $stream; $before=Get-SigningStreamHash $stream } finally { $stream.Dispose() }
  $output=Join-Path $fixture 'new-output'
  $null=Assert-SigningPath $output -NewDirectory
  Reject { Assert-SigningPath $fixture -NewDirectory }
  Reject { Assert-SigningPath 'relative.exe' }
  Reject { Assert-SigningPath (Join-Path $fixture 'absent/output') -NewDirectory }
  # Wrong hash must reject before copy, certificate lookup, tool execution or key use.
  Reject { Invoke-ReleaseArtifactSigning 'Sign' $artifact ('0'*64) $output $thumb $publisher $tool $hash $timestamp }
  Check (-not (Test-Path -LiteralPath $output)) 'Rejected signing created an output.'
  $stream=[IO.File]::OpenRead($artifact)
  try { Check ((Get-SigningStreamHash $stream) -ceq $before) 'Rejected signing modified unsigned input.' } finally { $stream.Dispose() }
  $memory=New-Object IO.MemoryStream(,$bytes)
  try { $memory.Position=128; $memory.WriteByte(0); Reject { Assert-SigningPE $memory } } finally { $memory.Dispose() }
  Write-SigningReceipt $fixture @{outcome='prepared'}
  Reject { Write-SigningReceipt $fixture @{outcome='signed'} }
  Check ((Get-Content -Raw -LiteralPath (Join-Path $fixture 'signing-receipt.json') | ConvertFrom-Json).outcome -ceq 'prepared') 'Existing receipt overwritten.'
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    $link=Join-Path $fixture 'link'; $null=New-Item -ItemType SymbolicLink -Path $link -Target $fixture
    Reject { Assert-SigningPath (Join-Path $link 'Setup.exe') }
    Remove-Item -LiteralPath $link
  }
  Initialize-SigningWinTrust
  if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT) {
    $private=Join-Path $fixture 'private-native'
    New-SigningPrivateDirectory $private
    Reject { New-SigningPrivateDirectory $private }
    $acl=Get-Acl -LiteralPath $private
    $rules=@($acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier]))
    $sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value
    Check ($acl.AreAccessRulesProtected -and $rules.Count -eq 2) 'Private signing DACL is not protected and exact.'
    foreach ($rule in $rules) {
      Check ($rule.IdentityReference.Value -in @($sid,'S-1-5-18') -and -not $rule.IsInherited -and $rule.AccessControlType.ToString() -ceq 'Allow' -and $rule.FileSystemRights.ToString() -ceq 'FullControl') 'Unexpected private signing directory access.'
    }
    Reject { [Skager.SigningTrust]::Verify($artifact) }
    Write-Host 'PASS: native atomic private DACL, existing-directory refusal and unsigned PE WinTrust rejection; no key use.'
  }
  Write-Host 'PASS: PE bounds, output no-overwrite, failure preserves unsigned bytes, receipt no-overwrite, exact WinTrust interop compiles.'
  Write-Host 'PENDING: native real-signing/certificate/TSA/WinTrust/DACL qualification. These inert checks do not qualify signed release artifacts.'
} finally { if (Test-Path -LiteralPath $fixture) { Remove-Item -LiteralPath $fixture -Recurse -Force } }
