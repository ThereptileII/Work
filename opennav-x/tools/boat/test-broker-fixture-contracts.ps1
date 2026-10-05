# Pure synthetic fixture data/dependency check. No process, task, app or boat.
[CmdletBinding()]
param()
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
. (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'tests/commissioning-restart/New-BrokerFixture.ps1')
. (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'tests/commissioning-restart/BrokerMarkerCleanup.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
. (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'tests/commissioning-restart/TestFixtureMarkerReader.ps1')
foreach($check in (Invoke-FixtureMarkerReaderChecks)){$checks.Add($check)}
$markerLines=@('42','134349496789080144','1',('a'*64),('b'*64),'fixed path','local','roaming','','')
Assert-BrokerMarkerChildRecord $markerLines ('a'*64) ('b'*64)
$checks.Add('Cleanup accepts only complete armed marker identity and exact session binding')
foreach($change in @(@(0,'0'),@(0,'+42'),@(0,'4294967296'),@(1,'18446744073709551616'),@(1,'01'),@(2,'0'),@(3,('c'*64)),@(4,('c'*64)))) {
 $copy=[string[]]$markerLines.Clone();$copy[$change[0]]=$change[1];$failed=$false
 try{Assert-BrokerMarkerChildRecord $copy ('a'*64) ('b'*64)}catch{$failed=$true}
 if(-not $failed){throw 'Malformed cleanup marker identity accepted'}
 $checks.Add('Cleanup rejects malformed or replaced identity field '+$change[0])
}
$failed=$false;try{Assert-BrokerMarkerChildRecord $markerLines[0..8] ('a'*64) ('b'*64)}catch{$failed=$true}
if(-not $failed){throw 'Truncated child record accepted'}
$checks.Add('Cleanup rejects an incomplete marker write')
$bytes=New-BrokerFixtureProfileBytes
if($bytes.Length -ne 21380){throw 'Fixture root no longer exercises production byte-size contract'}
$checks.Add('Synthetic root has fixed 21380 bytes without substituting production lineage validation')
$inputBytes=Get-CommissioningInputBytes $bytes;$changed=@(0..($bytes.Length-1)|Where-Object {$bytes[$_] -ne $inputBytes[$_]})
if($changed.Count -ne 1 -or $bytes[$changed[0]] -ne 49 -or $inputBytes[$changed[0]] -ne 48){throw 'Fixture does not exercise exact one-byte COM8 transform'}
$checks.Add('Full padded root changes exactly one output-direction byte')
$temporary=Join-Path ([IO.Path]::GetTempPath()) ('OpenNav broker fixture contract '+[guid]::NewGuid().ToString('N'))
$null=New-Item -ItemType Directory -Path $temporary
if([Environment]::OSVersion.Platform -ne 'Win32NT'){
 # The only portable substitution is syntax for this freshly created test path.
 # Native Windows uses the production local-path guard without substitution.
 function Assert-LocalPath([string]$Path){
  $full=[IO.Path]::GetFullPath($Path)
  if(-not $full.StartsWith($temporary+[IO.Path]::DirectorySeparatorChar,[StringComparison]::Ordinal) -or
     (Get-Item -LiteralPath $full -Force).Attributes -band [IO.FileAttributes]::ReparsePoint){throw 'Portable fixture path escaped owned temporary tree'}
  return $full
 }
}
try {
 $path=Join-Path $temporary 'input-only.ini';[IO.File]::WriteAllBytes($path,$inputBytes)
 $values=Read-ProfileForAudit $path;Assert-InputOnlyProfile $values
 if($values.Keys.Count -ne 4 -or $values['OpenNav/InterfaceMode'] -cne 'xnav' -or $values['Directories/ChartDir'] -cne 'original'){throw 'Padding changed parsed fixture settings'}
 $checks.Add('Neutral comment padding leaves the same four reviewed INI values')
 $parsed=Read-RestartIni $path
 if($parsed.Count -ne 4){throw 'Restart parser sees extra fixture settings'}
 $checks.Add('Actual restart parser accepts the full-size synthetic profile')
 foreach($palette in @('XNav','Standard')) {
  $paletteBytes=New-BrokerFixtureProfileBytes $palette
  $inputPalette=Get-CommissioningInputBytes $paletteBytes
  $palettePath=Join-Path $temporary ('palette-'+$palette+'.ini');[IO.File]::WriteAllBytes($palettePath,$inputPalette)
  $parsedPalette=Read-RestartIni $palettePath;Assert-InputOnlyProfile $parsedPalette
  if($paletteBytes.Length -ne 21380 -or $parsedPalette.Count -ne 5 -or $parsedPalette['OpenNav/ChartPresentationV1'] -cne $palette){throw 'Palette fixture changed recovered-root size or parsed identity.'}
  $delta=@(0..($paletteBytes.Length-1)|Where-Object {$paletteBytes[$_] -ne $inputPalette[$_]})
  if($delta.Count -ne 1 -or $paletteBytes[$delta[0]] -ne 49 -or $inputPalette[$delta[0]] -ne 48){throw 'Palette fixture changed the exact input-only transformation.'}
  $checks.Add('Actual '+$palette+' fixture retains fixed root size, exact palette and one-byte input-only transform')
 }
 foreach($name in $script:RestartDependencies){
  if($name -cnotmatch '^[A-Za-z0-9.-]+$' -or -not[IO.File]::Exists((Join-Path $PSScriptRoot $name))){throw ('Missing source dependency: '+$name)}
  Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $temporary $name)
  if((Get-FileHash -LiteralPath (Join-Path $temporary $name) -Algorithm SHA256).Hash -cne (Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $name) -Algorithm SHA256).Hash){throw 'Copied dependency bytes differ'}
 }
 if(-not[IO.File]::Exists((Join-Path $temporary 'CommissioningBaseline.ps1'))){throw 'New baseline module omitted from fixture closure'}
 $checks.Add('Every production-pinned dependency, including baseline lineage, copies with exact bytes')
 # Import the copied production audit exactly as the broker's child does, not
 # just the already-loaded source functions. A missing transitive module must
 # fail before any launch or pipe operation.
 . (Join-Path $temporary 'Commissioning.ps1')
 $checks.Add('Copied broker dependency closure imports the actual commissioning audit and resource policy')
 if(@($script:RestartDependencies|Where-Object {$_ -ceq 'ColdBaseline.ps1'}).Count -ne 1){throw 'Cold baseline reader must be pinned exactly once in every fresh restart session'}
 $cold=Join-Path $temporary 'ColdBaseline.ps1';$coldSaved=$cold+'.saved'
 if((Get-FileHash -LiteralPath $cold -Algorithm SHA256).Hash -cne (Get-FileHash -LiteralPath (Join-Path $PSScriptRoot 'ColdBaseline.ps1') -Algorithm SHA256).Hash){throw 'Cold baseline copy differs from session dependency source'}
 $checks.Add('Fresh restart dependency closure pins exactly one byte-identical cold-baseline reader')
 Move-Item -LiteralPath $cold -Destination $coldSaved
 try {
   $refused=$false;try{. (Join-Path $temporary 'Commissioning.ps1')}catch{$refused=$true}
   if(-not $refused){throw 'Missing cold-baseline dependency did not refuse the composed commissioning reader'}
 } finally {Move-Item -LiteralPath $coldSaved -Destination $cold}
 $checks.Add('Missing cold-baseline module refuses composed audit import before any launch or pipe')
 $resource=Join-Path $temporary 'InstalledResourceReview.ps1';$saved=$resource+'.saved'
 Move-Item -LiteralPath $resource -Destination $saved
 try {
   $refused=$false;try{. (Join-Path $temporary 'Commissioning.ps1')}catch{$refused=$true}
   if(-not $refused){throw 'Missing resource policy did not refuse the copied audit'}
 } finally {Move-Item -LiteralPath $saved -Destination $resource}
 $checks.Add('A missing transitive resource policy refuses copied audit import without any process or output')

} finally {Remove-Item -LiteralPath $temporary -Recurse -Force}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();applicationLaunched=$false;boatAccess=$false} | ConvertTo-Json -Depth 5
