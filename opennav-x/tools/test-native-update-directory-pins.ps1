# Inert directory-sharing fixture. Safe while OpenCPN is open: no processes,
# application discovery, installation/profile changes, services or hardware.
# RootDirectory is the source repository root containing opennav-x.
[CmdletBinding()]
param(
 [string]$RootDirectory=(Split-Path (Split-Path $PSScriptRoot -Parent) -Parent),
 [switch]$PolicyOnly
)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
function Check([bool]$Value,[string]$Message) {
 if(-not $Value){throw $Message}
 Write-Host ('PASS: '+$Message)
}
$module=Join-Path $RootDirectory 'opennav-x/installer/windows/UpdateTransaction.ps1'
$tokens=$null;$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile($module,[ref]$tokens,[ref]$errors)
Check ($errors.Count -eq 0) 'Production transaction source parses'
$source=@($ast.FindAll({param($node) $node -is [Management.Automation.Language.StringConstantExpressionAst] -and $node.Value.StartsWith('using System;')},$true))
Check ($source.Count -eq 1) 'One exact embedded production native source selected'
if(('Skager.UpdateArmedBarrier' -as [type]) -or ('Skager.UpdateImageOwner' -as [type])) {
 throw 'Run this fixture in a fresh PowerShell process; preloaded production types cannot prove the selected source.'
}
Add-Type -TypeDefinition $source[0].Value
Check ($null -ne ('Skager.UpdateArmedBarrier' -as [type]) -and $null -ne ('Skager.UpdateImageOwner' -as [type])) 'Exact production types compile without native invocation'
$directoryOpen=[Skager.UpdateArmedBarrier].GetMethod('OpenDirectory',[Reflection.BindingFlags]'NonPublic,Static')
$nativeName=[Skager.UpdateImageOwner].GetMethod('Native',[Reflection.BindingFlags]'NonPublic,Static')
$ownerOpen=[Skager.UpdateImageOwner].GetMethod('Open',[Reflection.BindingFlags]'NonPublic,Instance')
Check ($null -ne $directoryOpen -and $null -ne $nativeName -and $null -ne $ownerOpen) 'Exact private production directory methods resolved'
if($PolicyOnly -or [Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
 Write-Host 'SKIP: native directory sharing requires Windows; compilation is not native qualification.'
 return
}
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('SkagerDirectoryPins-'+[guid]::NewGuid().ToString('N'))
$paths=New-Object 'Collections.Generic.List[string]'
$failure=$null
$cleanupErrors=New-Object 'Collections.Generic.List[string]'
try {
 $null=[IO.Directory]::CreateDirectory($fixture)
 foreach($kind in @('armed-barrier','image-owner')) {
  $empty=Join-Path $fixture $kind;$renamed=$empty+'-renamed'
  $paths.Add($empty);$paths.Add($renamed)
  $null=[IO.Directory]::CreateDirectory($empty)
  [IO.Directory]::Move($empty,$renamed);[IO.Directory]::Move($renamed,$empty)
  Check $true ($kind+' empty directory baseline rename succeeds')
  # Resolve the physical name using the exact production Native helper, then
  # close the probe so only the method under test can provide rename exclusion.
  $probe=$directoryOpen.Invoke($null,@($empty))
  try { $native=$nativeName.Invoke($null,@($probe)) } finally { $probe.Dispose() }
  $held=$null;$owner=$null
  try {
   if($kind -ceq 'armed-barrier') { $held=$directoryOpen.Invoke($null,@($empty)) }
   else {
    $owner=[Activator]::CreateInstance([Skager.UpdateImageOwner],$true)
    $null=$ownerOpen.Invoke($owner,@($native,$true))
   }
   $sharingDenied=$false
   try { [IO.Directory]::Move($empty,$renamed) }
   catch { $sharingDenied=($_.Exception.GetBaseException().HResult -band 0xffff) -eq 32 }
   Check $sharingDenied ($kind+' empty directory rename reports exact sharing error 32 while pinned')
  } finally {
   if($null -ne $held){$held.Dispose()}
   if($null -ne $owner){$owner.Dispose()}
  }
  [IO.Directory]::Move($empty,$renamed)
  Check ([IO.Directory]::Exists($renamed) -and -not [IO.Directory]::Exists($empty)) ($kind+' empty directory rename succeeds after release')
 }
} catch { $failure=$_ }
finally {
 # Never recursively remove unexpected contents, even on a failed assertion.
 foreach($path in $paths) {
  try { [IO.Directory]::Delete($path) }
  catch [IO.DirectoryNotFoundException] { }
  catch { $cleanupErrors.Add($_.Exception.Message) }
 }
 try { [IO.Directory]::Delete($fixture) }
 catch [IO.DirectoryNotFoundException] { }
 catch { $cleanupErrors.Add($_.Exception.Message) }
}
if($null -ne $failure) {
 if($cleanupErrors.Count -gt 0){Write-Warning ('Secondary fixture cleanup failure: '+($cleanupErrors -join '; '))}
 throw $failure
}
Check ($cleanupErrors.Count -eq 0) ('Only empty disposable fixture directories removed; cleanup errors: '+($cleanupErrors -join '; '))
Write-Host 'PASS: native production directory pins; no child process or application/profile interaction.'
