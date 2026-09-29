# Explicit user-authorized import only. No key parameter or local plaintext file.
# Invoke over the existing SSH alias; supply a bounded binary frame over stdin.
[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
if([Environment]::OSVersion.Platform -ne 'Win32NT'){throw 'Windows Credential Manager is required.'}
Add-Type -Path (Join-Path $PSScriptRoot 'AisCredentialImport.cs')
$result=[XNavAisCredentialImport]::StoreProduction([Console]::OpenStandardInput())
[pscustomobject]@{status=$result;store='Windows Credential Manager';scope='current user, this PC';keyPrinted=$false;applicationLaunched=$false} | ConvertTo-Json -Compress
if($result -cnotin @('stored-and-verified','already-stored-and-verified')){exit 1}
