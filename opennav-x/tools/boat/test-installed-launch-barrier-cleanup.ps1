# Portable tests of the actual fixture cleanup block. No native API or child.
[CmdletBinding()]
param()
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
if([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT){throw 'Portable cleanup harness only; run the guarded inert fixture for native qualification.'}
$fixture=Join-Path $PSScriptRoot 'test-installed-launch-barrier.ps1'
$receipt=& $fixture -PortableContracts | ConvertFrom-Json
if($receipt.nativeBehavior -cne 'not-executed-portable-contracts'){throw 'Cleanup contracts require non-Windows execution; use the real isolated native fixture on Windows.'}
$parseErrors=$null;$ast=[Management.Automation.Language.Parser]::ParseFile($fixture,[ref]$null,[ref]$parseErrors)
if($parseErrors){throw 'Fixture parse failed.'}
$blocks=@($ast.FindAll({param($node)$node -is [Management.Automation.Language.TryStatementAst] -and $node.Finally -and $node.Finally.Extent.Text.Contains('Additional fixture cleanup failure; original failure preserved:')},$true))
if($blocks.Count -ne 1 -or $blocks[0].CatchClauses.Count -ne 1){throw 'Cannot identify actual fixture cleanup boundary.'}
$contextBlocks=@($ast.FindAll({param($node)$node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -ceq 'Write-FixtureFailureContext'},$true))
if($contextBlocks.Count -ne 1){throw 'Cannot identify actual failed-context writer.'}
. ([scriptblock]::Create($contextBlocks[0].Extent.Text))
$sourceFiles=$receipt.sourceFiles;$native=$false;$alias='Z:'
$cleanupHarness=[scriptblock]::Create('try {if($injectPrimary){throw "ORIGINAL FIXTURE FAILURE"}} catch '+$blocks[0].CatchClauses[0].Body.Extent.Text+' finally '+$blocks[0].Finally.Extent.Text)
Add-Type -TypeDefinition @'
using System;
public sealed class InstalledBarrierCleanupOwner {
 public bool Fail;public int Calls;
 public void Dispose(){Calls++;if(Fail)throw new InvalidOperationException("injected guard release failure");}
}
'@
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Value,[string]$Label){if(-not $Value){throw $Label};$checks.Add($Label)}
foreach($injectPrimary in @($true,$false)){
 $primaryFailure=$null;$guards=New-Object 'Collections.Generic.List[object]'
 $first=New-Object InstalledBarrierCleanupOwner;$first.Fail=$true
 $second=New-Object InstalledBarrierCleanupOwner
 $guards.Add([pscustomobject]@{owner=$first});$guards.Add([pscustomobject]@{owner=$second})
 $adapter=New-Object InstalledBarrierAliasAdapter;$aliasOwner=$adapter.NewOwner()
 $aliasOwner.Create('Z:','C:\fixture-B');$adapter.FailRemove=$true
 $root=Join-Path ([IO.Path]::GetTempPath()) ('skager-barrier-cleanup-'+[guid]::NewGuid().ToString('N'))
 $null=New-Item -ItemType Directory -Path $root
 try {
  $failure=$null;try{. $cleanupHarness}catch{$failure=$_}
  Check ($null -ne $failure) 'Cleanup uncertainty cannot produce success'
  if($injectPrimary){Check ($failure.Exception.Message -ceq 'ORIGINAL FIXTURE FAILURE') 'Actual cleanup preserves the original failure'}
  else{Check ($failure.Exception.GetBaseException() -is [AggregateException]) 'Cleanup-only failure is surfaced as a failure'}
  Check ($first.Calls -eq 1 -and $second.Calls -eq 1) 'All guard releases are attempted independently'
  Check ($adapter.LastFlags -eq 7 -and $aliasOwner.Created) 'Alias cleanup attempted exact owned removal after guard release failed'
  Check ([IO.Directory]::Exists($root)) 'Uncertain owned mapping retains its inert recovery tree'
 } finally {Remove-Item -LiteralPath $root -Recurse -Force}
}
$primaryFailure=$null;$injectPrimary=$false;$guards=@()
$adapter=New-Object InstalledBarrierAliasAdapter;$aliasOwner=$adapter.NewOwner();$aliasOwner.Create('Z:','C:\fixture-B')
$root=Join-Path ([IO.Path]::GetTempPath()) ('skager-barrier-cleanup-'+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $root
try{. $cleanupHarness;Check (-not $aliasOwner.Created -and -not [IO.Directory]::Exists($root)) 'Successful exact alias cleanup removes only the inert fixture tree'}finally{if([IO.Directory]::Exists($root)){Remove-Item -LiteralPath $root -Recurse -Force}}
Check ($receipt.status -ceq 'passed' -and $receipt.checks.Count -eq 50) 'Failure-context support preserves all portable fixture checks'
[pscustomobject]@{schema=1;status='passed';checks=@($checks);nativeBehavior='not-executed-portable-contracts';applicationLaunched=$false}|ConvertTo-Json -Depth 3 -Compress
