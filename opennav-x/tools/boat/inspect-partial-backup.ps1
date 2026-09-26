# Read-only investigation of a failed cold backup. Summary avoids personal paths;
# mismatching relative names remain only in the explicitly chosen local report.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$Partial,[Parameter(Mandatory=$true)][string]$OpenCpnDirectory,[Parameter(Mandatory=$true)][string]$ProfileDirectory,[Parameter(Mandatory=$true)][string]$Report)
. (Join-Path $PSScriptRoot 'Common.ps1')
$Partial=Assert-LocalPath $Partial
if ($Partial -notlike '*.partial') {throw 'Select an incomplete .partial backup directory.'}
$differences=@();$checked=0
foreach ($pair in @(@('opencpn',$OpenCpnDirectory),@('profile',$ProfileDirectory))) {
  $root=Assert-LocalPath (Join-Path $Partial $pair[0])
  if (-not [IO.Directory]::Exists($root)) {continue}
  foreach ($file in Get-ChildItem -LiteralPath $root -File -Force -Recurse) {
    $copy=Assert-LocalPath $file.FullName
    $relative=$copy.Substring($root.Length+1)
    $source=Assert-LocalPath (Join-Path $pair[1] $relative)
    $copied=Get-Digest $copy
    $original=if ([IO.File]::Exists($source)) {Get-Digest $source} else {'missing'}
    $checked++
    if ($copied -cne $original -or $file.Length -ne (Get-Item -LiteralPath $source -Force).Length) {
      $differences+=@{tree=$pair[0];path=$relative;sourceHash=$original;copyHash=$copied;sourceBytes=(Get-Item -LiteralPath $source -Force).Length;copyBytes=$file.Length}
    }
  }
}
Write-Record $Report @{status='inspected';checkedFiles=$checked;differences=$differences;note='Comparison is against current source, not the original in-memory inventory.'}
[pscustomobject]@{status='inspected';checkedFiles=$checked;mismatchCount=$differences.Count;privateReport=$Report} | ConvertTo-Json
