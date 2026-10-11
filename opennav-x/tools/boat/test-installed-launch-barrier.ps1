# Disposable files only. No child, installation, profile, network or boat action.
[CmdletBinding()]
param([switch]$PortableContracts,[switch]$IsolatedLocal)
Set-StrictMode -Version Latest;$ErrorActionPreference='Stop'
$native=[Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT
if(-not $native -and -not $PortableContracts){throw 'Use explicit portable contracts off Windows.'}
if($native -and -not $IsolatedLocal -and $env:GITHUB_ACTIONS -cne 'true'){throw 'Disposable CI or explicit isolated local tests required.'}
. (Join-Path $PSScriptRoot 'InstalledLaunchBarrier.ps1')
Initialize-InstalledLaunchBarrierType # Compile real native declarations; no P/Invoke off Windows.
$checks=New-Object 'Collections.Generic.List[string]'
function Check([bool]$Value,[string]$Label){if(-not $Value){throw $Label};$checks.Add($Label)}
function Reject([scriptblock]$Call,[string]$Label){$refused=$false;try{$null=& $Call}catch{$refused=$true};Check $refused $Label}
$id='a'*32;$physical='\Device\HarddiskVolume3\fixture-A';$image=$physical+'\generations\'+$id+'\app\opencpn.exe'
$marker='\\?\GLOBALROOT'+$physical+'\signed-update-armed.jsonl'
$method=[OpenNavX.InstalledLaunchBarrierNative].GetMethod('CheckAbsent',[Reflection.BindingFlags]'Static,NonPublic')
function CheckAbsent([string]$Image,[string]$Generation,[string]$Filename,[Func[string,uint32]]$Read){$null=$method.Invoke($null,[object[]]@($Image,$Generation,$Filename,$Read))}
Check ([OpenNavX.InstalledLaunchBarrierNative]::ManagedRoot($image,$id,'opencpn.exe') -ceq ('\\?\GLOBALROOT'+$physical)) 'Root comes only from the held image exact generation suffix'
foreach($bad in @(('b'*32),'invalid')){Reject {[OpenNavX.InstalledLaunchBarrierNative]::ManagedRoot($image,$bad,'opencpn.exe')} 'Wrong/invalid generation refuses'}
Reject {[OpenNavX.InstalledLaunchBarrierNative]::ManagedRoot(('\Device\HarddiskVolume3\generations\'+$id+'\app\opencpn.exe'),$id,'opencpn.exe')} 'Volume root cannot stand in for an installation root'
foreach($bad in @('skager-start.exe','other.exe','../opencpn.exe')){Reject {[OpenNavX.InstalledLaunchBarrierNative]::ManagedRoot($image,$id,$bad)} 'Mismatched/unapproved filename refuses'}
foreach($bad in @(('C:\fixture\generations\'+$id+'\app\opencpn.exe'),('\Device\LanmanRedirector\fixture\generations\'+$id+'\app\opencpn.exe'))){Reject {[OpenNavX.InstalledLaunchBarrierNative]::ManagedRoot($bad,$id,'opencpn.exe')} 'Nonphysical image refuses'}
Add-Type -TypeDefinition @'
using System;using System.Collections.Generic;using System.ComponentModel;
public sealed class InstalledBarrierFixtureReader {
 public string Marker;public int MarkerError,RootError;public uint MarkerAttributes=16,RootAttributes=16;
 public readonly List<string> Paths=new List<string>();
 public uint Read(string path){Paths.Add(path);bool leaf=path==Marker;int error=leaf?MarkerError:RootError;if(error!=0)throw new Win32Exception(error);return leaf?MarkerAttributes:RootAttributes;}
}
'@
$reader=New-Object InstalledBarrierFixtureReader;$reader.Marker=$marker;$reader.MarkerError=2
$read=[Func[string,uint32]][Delegate]::CreateDelegate([Func[string,uint32]],$reader,'Read')
CheckAbsent $image $id 'opencpn.exe' $read
Check ($reader.Paths.Count -eq 2 -and $reader.Paths[1] -ceq $marker) 'Only checked physical root plus exact missing leaf accepts'
foreach($code in @(3,5,32,123,995)){
 $reader.MarkerError=$code
 Reject {CheckAbsent $image $id 'opencpn.exe' $read} ('Uncertain marker error '+$code+' refuses')
}
$reader.MarkerError=2
foreach($code in @(2,3,5)){
 $reader.RootError=$code
 Reject {CheckAbsent $image $id 'opencpn.exe' $read} ('Missing/inaccessible physical root '+$code+' refuses')
}
$reader.RootError=0;$reader.MarkerError=0
foreach($attributes in @(0,16,1024,1040)){
 $reader.MarkerAttributes=$attributes
 Reject {CheckAbsent $image $id 'opencpn.exe' $read} ('Present marker attributes '+$attributes+' refuse without reading bytes')
}
foreach($attributes in @(0,1024,1040)){
 $reader.RootAttributes=$attributes
 Reject {CheckAbsent $image $id 'opencpn.exe' $read} 'Wrong/reparse physical root refuses'
}
# The alias has already changed to marker-free B. The guard still reads A.
$aliasTarget='\Device\HarddiskVolume3\fixture-B';$reader.Paths.Clear();$reader.RootAttributes=16
Reject {CheckAbsent $image $id 'opencpn.exe' $read} 'Held A marker denies even when original request alias selects marker-free B'
Check ($reader.Paths.Count -eq 2 -and $reader.Paths[1] -ceq $marker -and -not $reader.Paths[1].Contains($aliasTarget)) 'No later alias resolution chooses the marker root'
# Check exact production call order, rather than an unused standalone helper.
foreach($name in @('InteractiveJob.ps1','StartupLauncher.ps1','RestartCommissioningBroker.ps1','InstalledLaunchBarrier.ps1')){
 $path=Join-Path $PSScriptRoot $name;$errors=$null;$null=[Management.Automation.Language.Parser]::ParseFile($path,[ref]$null,[ref]$errors)
 Check (-not $errors) ('Parse '+$name)
 $source=[IO.File]::ReadAllText($path)
 if($name -cin @('InteractiveJob.ps1','StartupLauncher.ps1')){
  $create=$source.IndexOf('[Diagnostics.Process]::Start($start)',[StringComparison]::Ordinal)
  $guard=$source.LastIndexOf('Assert-InstalledLaunchBarrier $installedLaunchBarrier',$create,[StringComparison]::Ordinal)
  $acquire=$source.IndexOf('$installedLaunchBarrier=Open-InstalledLaunchBarrier',[StringComparison]::Ordinal)
  Check ($acquire -ge 0 -and $guard -gt $acquire -and $guard -lt $create) ($name+': retained guard precedes actual creation')
  Check ($source.Contains('$installedLaunchBarrier.owner.Dispose()')) ($name+': retained guard is disposed')
 } elseif($name -ceq 'RestartCommissioningBroker.ps1') {
  foreach($needle in @('$permitPath=Publish-RestartPermit','[OpenNavX.RestartCommissioningNative]::WriteFrame($pipe,[OpenNavX.RestartCommissioningNative]::Reply($fields)')){
   $call=$source.IndexOf($needle,[StringComparison]::Ordinal)
   $guard=$source.LastIndexOf('Assert-InstalledLaunchBarrier $installedLaunchBarrier',$call,[StringComparison]::Ordinal)
   Check ($call -gt 0 -and $guard -gt $source.IndexOf('$installedLaunchBarrier=Open-InstalledLaunchBarrier') -and $guard -lt $call) 'Broker retained guard precedes actual permit publication/ALLOW'
  }
  Check ($source.Contains('$installedLaunchBarrier.owner.Dispose()')) 'Broker retained guard is disposed'
 }
}
$dependencySource=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'RestartCommissioning.ps1'))
Check ($dependencySource.Contains("'InstalledLaunchBarrier.ps1'")) 'Restart session hash closure includes the guard'
$sourceFiles=@('InteractiveJob.ps1','StartupLauncher.ps1','RestartCommissioningBroker.ps1','InstalledLaunchBarrier.ps1','test-installed-launch-barrier.ps1')|ForEach-Object {[pscustomobject]@{path=('tools/boat/'+$_);sha256=(Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $_) -Algorithm SHA256).Hash.ToLowerInvariant()}}
$nativeStatus='not-executed-portable-contracts';$root=$null;$alias=$null;$guards=New-Object 'Collections.Generic.List[object]'
try {
 if($native){
  $root=Join-Path ([IO.Path]::GetTempPath()) ('skager-launch-barrier-'+[guid]::NewGuid().ToString('N'))
  $module=Join-Path $PSScriptRoot '../../installer/windows/UpdateTransaction.ps1'
  $installed=@()
  foreach($name in @('A','B')){
   $install=Join-Path $root $name;$generation=Join-Path $install ('generations/'+$id);$app=Join-Path $generation 'app'
   $null=New-Item -ItemType Directory -Path $app -Force
   Copy-Item -LiteralPath $module -Destination (Join-Path $generation 'UpdateTransaction.ps1')
   foreach($filename in @('opencpn.exe','skager-start.exe')){[IO.File]::WriteAllText((Join-Path $app $filename),'INERT; NEVER EXECUTE')}
   $files=@('UpdateTransaction.ps1','app/opencpn.exe','app/skager-start.exe')|ForEach-Object {@{path=$_;sha256=(Get-FileHash (Join-Path $generation $_) -Algorithm SHA256).Hash.ToLowerInvariant()}}
   $installed+=@([pscustomobject]@{root=$install;generation=$generation;state=@{current=$id};ownership=@{managedFiles=@($files)}})
  }
  function Get-Digest([string]$Path){(Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()}
  foreach($filename in @('opencpn.exe','skager-start.exe')){
   $guard=Open-InstalledLaunchBarrier $installed[0] $filename;$guards.Add($guard)
   Check $true ('Native held '+$filename+' accepts proven marker absence')
   $leaf=Join-Path $installed[0].root 'signed-update-armed.jsonl'
   foreach($kind in @('empty','garbage','directory')){
    if($kind -ceq 'directory'){$null=New-Item -ItemType Directory -Path $leaf}else{[IO.File]::WriteAllText($leaf,$(if($kind -ceq 'empty'){''}else{'not JSON'}))}
    try{Reject {Assert-InstalledLaunchBarrier $guard} ('Native '+$kind+' marker denies '+$filename)}finally{Remove-Item -LiteralPath $leaf -Force}
   }
   $guard.owner.Dispose();$guards.Remove($guard)|Out-Null
  }
  Add-Type -TypeDefinition @'
using System;using System.Runtime.InteropServices;
public static class InstalledBarrierFixtureAlias {
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] public static extern bool DefineDosDevice(uint flags,string name,string target);
 [DllImport("kernel32.dll")] public static extern uint GetLogicalDrives();
}
'@
  $mask=[InstalledBarrierFixtureAlias]::GetLogicalDrives()
  if($mask -eq 0){throw 'Cannot prove unused fixture alias.'}
  foreach($letter in [char[]]'ZYXWVUTSRQPONMLKJIHGFED') {if(($mask -band (1 -shl ([int]$letter-65))) -eq 0){$alias=[string]$letter+':';break}}
  if(-not $alias){throw 'No unused private fixture alias available.'}
  if(-not [InstalledBarrierFixtureAlias]::DefineDosDevice(0,$alias,$installed[0].root)){throw 'Cannot create isolated fixture alias.'}
  $aliasInstall=[pscustomobject]@{generation=($alias+'\generations\'+$id);state=@{current=$id};ownership=$installed[0].ownership}
  $guard=Open-InstalledLaunchBarrier $aliasInstall;$guards.Add($guard)
  if(-not [InstalledBarrierFixtureAlias]::DefineDosDevice(2,$alias,$installed[0].root)){throw 'Cannot remove initial fixture alias.'}
  if(-not [InstalledBarrierFixtureAlias]::DefineDosDevice(0,$alias,$installed[1].root)){throw 'Cannot retarget isolated fixture alias.'}
  $leaf=Join-Path $installed[0].root 'signed-update-armed.jsonl';[IO.File]::WriteAllText($leaf,'A remains armed; B has no marker')
  try {
   $message='';try{Assert-InstalledLaunchBarrier $guard}catch{$message=$_.Exception.GetBaseException().Message}
   Check ($message.Contains('Signed update session is armed')) 'Native retargeted alias still denies exact held A marker'
  } finally {Remove-Item -LiteralPath $leaf -Force}
  Reject {Assert-InstalledLaunchBarrier $guard} 'Native retargeted request refuses even after A marker removed'
  $nativeStatus='passed-inert-files-no-process-created'
 }
} finally {
 foreach($guard in $guards){$guard.owner.Dispose()}
 if($alias -and -not [InstalledBarrierFixtureAlias]::DefineDosDevice(2,$alias,$null)){throw 'Fixture alias cleanup failed.'}
 if($root){Remove-Item -LiteralPath $root -Recurse -Force}
}
[pscustomobject]@{schema=1;status='passed';checks=@($checks);nativeBehavior=$nativeStatus;sourceFiles=@($sourceFiles);applicationLaunched=$false;
 scope='Fixed marker restriction at actual entrypoints; no continuously pinned absence or signed broker authority'}|ConvertTo-Json -Depth 4 -Compress
