# Pure synthetic fixture data/dependency check. No process, task, app or boat.
[CmdletBinding()]
param()
. (Join-Path $PSScriptRoot 'RestartCommissioning.ps1')
. (Join-Path $PSScriptRoot 'Commissioning.ps1')
. (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) 'tests/commissioning-restart/New-BrokerFixture.ps1')
$checks=New-Object 'Collections.Generic.List[string]'
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
 foreach($name in $script:RestartDependencies){
  if($name -cnotmatch '^[A-Za-z0-9.-]+$' -or -not[IO.File]::Exists((Join-Path $PSScriptRoot $name))){throw ('Missing source dependency: '+$name)}
  Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination (Join-Path $temporary $name)
  if((Get-FileHash -LiteralPath (Join-Path $temporary $name) -Algorithm SHA256).Hash -cne (Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $name) -Algorithm SHA256).Hash){throw 'Copied dependency bytes differ'}
 }
 if(-not[IO.File]::Exists((Join-Path $temporary 'CommissioningBaseline.ps1'))){throw 'New baseline module omitted from fixture closure'}
 $checks.Add('Every production-pinned dependency, including baseline lineage, copies with exact bytes')
} finally {Remove-Item -LiteralPath $temporary -Recurse -Force}
[pscustomobject]@{status='passed';count=$checks.Count;checks=$checks.ToArray();applicationLaunched=$false;boatAccess=$false} | ConvertTo-Json -Depth 5
