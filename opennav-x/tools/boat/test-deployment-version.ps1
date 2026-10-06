# Pure identity and invalid-request preflight checks. No installer or boat I/O.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'Common.ps1')
$checks=0
function Check([scriptblock]$Body) { & $Body; $script:checks++ }
function Refuse([scriptblock]$Body,[string]$ExpectedMessage) {
  $message=$null
  try { & $Body } catch { $message=$_.Exception.Message }
  if (-not $message -or ($ExpectedMessage -and $message -notlike $ExpectedMessage)) {
    throw ('Expected deployment refusal, got: '+$message)
  }
  $script:checks++
}
$commit='a'*40
foreach ($version in @('0.4.0-beta2','0.4.0-beta2.1','0.4.0-beta2.2','0.4.0-beta2.65535')) {
  Check { Assert-BoatDeploymentVersion $version }
  Check { Assert-BoatDeploymentIdentity ([pscustomobject]@{commit=$commit;version=$version}) $commit $version }
}
$invalid=@('', '0.4.0-beta2.0', '0.4.0-beta2.01', '0.4.0-beta2.65536',
  '0.4.0-beta2.999999', '0.4.0-beta2.1+build', '0.4.0-beta2.1.2', '0.4.0',
  '0.4.1-beta2', '0.4.0-beta3', '0.4.0-BETA2', ' 0.4.0-beta2',
  '0.4.0-beta2 ', "0.4.0-beta2`n", "0.4.0-beta2.1`r`n", "0.4.0-beta2`0")
foreach ($version in $invalid) {
  Refuse { Assert-BoatDeploymentVersion $version } 'Expected canonical*'
}
foreach ($actual in @('0.4.0-beta2','0.4.0-beta2.2','0.4.0-beta2.01','0.4.0-BETA2.1')) {
  Refuse { Assert-BoatDeploymentIdentity ([pscustomobject]@{commit=$commit;version=$actual}) $commit '0.4.0-beta2.1' } 'Installed identity differs*'
}
foreach ($actual in @(('b'*40),('A'*40),($commit+"`n"))) {
  Refuse { Assert-BoatDeploymentIdentity ([pscustomobject]@{commit=$actual;version='0.4.0-beta2.1'}) $commit '0.4.0-beta2.1' } 'Installed identity differs*'
}
Refuse { Assert-BoatDeploymentIdentity ([pscustomobject]@{commit=$commit;version=1}) $commit '0.4.0-beta2.1' } 'Installed identity differs*'
Refuse { Assert-BoatDeploymentIdentity ([pscustomobject]@{commit=$commit}) $commit '0.4.0-beta2.1' }
# Exercise the actual wrappers: invalid versions must be refused before even
# examining the intentionally invalid workspace or non-existent Setup path.
# The update wrapper must forward the exact input, not substitute a default.
foreach ($wrapper in @('install.ps1','update.ps1')) {
  $path=Join-Path $PSScriptRoot $wrapper
  $command=Get-Command $path
  $mandatory=@($command.Parameters['ExpectedVersion'].Attributes | Where-Object {$_ -is [Management.Automation.ParameterAttribute] -and $_.Mandatory})
  Check { if ($mandatory.Count -ne 1) {throw 'ExpectedVersion must be explicit and mandatory.'} }
  foreach ($version in @('0.4.0-beta2.0','0.4.0-beta2.65536',"0.4.0-beta2.1`n")) {
    Refuse { & $path -Workspace 'invalid-no-io' -Setup 'never-execute.exe' -Sha256 ('0'*64) -ExpectedCommit $commit -ExpectedVersion $version } 'Expected canonical*'
  }
}
Write-Output ('Deployment version: '+$checks+' checks passed; no installer or boat access.')
