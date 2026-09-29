# Evaluate only the real validator AST; never execute installer operations.
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path $PSScriptRoot -Parent
$tokens=$null; $errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Join-Path $root 'installer/windows/Lifecycle.ps1'),[ref]$tokens,[ref]$errors)
if($errors.Count){throw 'Installer parser errors'}
foreach($name in @('Assert-StatusOnlyOutput','Assert-InstalledProduct','Resolve-OutputPolicy','Test-ExactRepairPackage')) {
  $functions=@($ast.FindAll({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq $name},$true))
  if($functions.Count -ne 1){throw 'Missing unique actual output validator'}
  . ([scriptblock]::Create($functions[0].Extent.Text))
}
Assert-StatusOnlyOutput ([pscustomobject]@{xnav_hardware_output_policy='status-only'})
$count=1
foreach($value in @($null,$true,$false,0,1,'STATUS-ONLY','status-only ','test-loopback-only','physical-output',@(),@{})) {
  $rejected=$false
  try{Assert-StatusOnlyOutput ([pscustomobject]@{xnav_hardware_output_policy=$value})}catch{$rejected=$true}
  if(-not $rejected){throw 'Installer accepted unqualified output capability'}
  $count++
}
foreach($report in @($null,[pscustomobject]@{})) {
  $rejected=$false
  try{Assert-StatusOnlyOutput $report}catch{$rejected=$true}
  if(-not $rejected){throw 'Installer accepted missing output capability'}
  $count++
}
$historical=[pscustomobject]@{version='0.4.0-beta2'}
if((Resolve-OutputPolicy $historical $true) -cne 'historical-unqualified'){throw 'Historical rollback falsely qualified'}
$rejected=$false
try{Resolve-OutputPolicy $historical $false}catch{$rejected=$true}
if(-not $rejected){throw 'New payload was allowed a historical exception'}
$count+=2
foreach($value in @('test-loopback-only','physical-output',$null,$true)) {
  $rejected=$false
  try{Resolve-OutputPolicy ([pscustomobject]@{xnav_hardware_output_policy=$value}) $true}catch{$rejected=$true}
  if(-not $rejected){throw 'Recovery ignored an explicit disallowed output policy'}
  $count++
}
$previous=[pscustomobject]@{packageSha256=('a'*64);commit=('b'*40);version='0.4.0-beta2'}
$package=[pscustomobject]@{commit=('b'*40);version='0.4.0-beta2'}
if(-not (Test-ExactRepairPackage $previous $package ('a'*64))){throw 'Exact recorded package repair refused'}
$count++
foreach($field in @('packageSha256','commit','version')) {
  $changed=$previous | ConvertTo-Json | ConvertFrom-Json
  $changed.$field='different'
  if(Test-ExactRepairPackage $changed $package ('a'*64)){throw 'Unrecorded repair payload accepted'}
  $count++
}
foreach($hash in @('',('A'*64),('a'*63))) {
  if(Test-ExactRepairPackage $previous $package $hash){throw 'Invalid recovery package identity'}
  $count++
}
Assert-InstalledProduct ([pscustomobject]@{test_fixtures=$false;build_purpose='INSTALLED PRODUCT'})
$count++
foreach($value in @($null,$true,0,1,'false','False',@(),@{})) {
  $rejected=$false
  try{Assert-InstalledProduct ([pscustomobject]@{test_fixtures=$value;build_purpose='INSTALLED PRODUCT';xnav_hardware_output_policy='status-only'})}catch{$rejected=$true}
  if(-not $rejected){throw 'Installer accepted fixture identity or coerced a non-boolean fixture declaration'}
  $count++
}
foreach($value in @($null,$true,0,'installed product','DEVELOPER TEST BUILD','INSTALLED PRODUCT ',@())) {
  $rejected=$false
  try{Assert-InstalledProduct ([pscustomobject]@{test_fixtures=$false;build_purpose=$value;xnav_hardware_output_policy='status-only'})}catch{$rejected=$true}
  if(-not $rejected){throw 'Installer accepted a non-product build purpose'}
  $count++
}
foreach($report in @($null,[pscustomobject]@{},[pscustomobject]@{test_fixtures=$false},[pscustomobject]@{build_purpose='INSTALLED PRODUCT'})) {
  $rejected=$false
  try{Assert-InstalledProduct $report}catch{$rejected=$true}
  if(-not $rejected){throw 'Installer accepted incomplete product declaration'}
  $count++
}
# A truthful product identity cannot bypass the separate hardware-output gate.
$rejected=$false
$unqualified=[pscustomobject]@{test_fixtures=$false;build_purpose='INSTALLED PRODUCT';xnav_hardware_output_policy='test-loopback-only'}
try{Assert-InstalledProduct $unqualified;Assert-StatusOnlyOutput $unqualified}catch{$rejected=$true}
if(-not $rejected){throw 'Valid product identity bypassed output restriction'}
$count++
Write-Output "$count actual installer output-policy checks passed; no installer operation invoked"
