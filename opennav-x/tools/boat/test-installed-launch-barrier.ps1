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
# Declaring P/Invoke is portable; only the native branch uses its default adapter.
Add-Type -TypeDefinition @'
using System;using System.ComponentModel;using System.Runtime.InteropServices;
public sealed class InstalledBarrierFixtureAlias {
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern bool DefineDosDevice(uint flags,string name,string target);
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern uint QueryDosDevice(string name,[Out]char[] targets,int length);
 [DllImport("kernel32.dll",EntryPoint="GetLogicalDrives",SetLastError=true)] static extern uint NativeGetLogicalDrives();
 public static uint GetLogicalDrives(){uint mask=NativeGetLogicalDrives();if(mask==0){int code=Marshal.GetLastWin32Error();throw new Win32Exception(code,"Cannot prove unused fixture alias; Win32 error "+code);}return mask;}
 readonly Func<uint,string,string,bool> define;readonly Func<string,string[]> query;readonly Func<int> error;
 public string Name {get;private set;}public string Target {get;private set;}public bool Created {get;private set;}
 public InstalledBarrierFixtureAlias():this(DefineDosDevice,ReadTargets,Marshal.GetLastWin32Error){}
 public InstalledBarrierFixtureAlias(Func<uint,string,string,bool> define,Func<string,string[]> query,Func<int> error){this.define=define;this.query=query;this.error=error;}
 static string[] ReadTargets(string name){char[] buffer=new char[32768];uint count=QueryDosDevice(name,buffer,buffer.Length);if(count==0){int code=Marshal.GetLastWin32Error();if(code==2)return new string[0];throw new Win32Exception(code,"Cannot query fixture alias "+name+"; Win32 error "+code);}if(count>=buffer.Length)throw new InvalidOperationException("Fixture alias query is truncated.");return new string(buffer,0,(int)count).Split(new char[]{'\0'},StringSplitOptions.RemoveEmptyEntries);}
 public void Create(string name,string dosTarget){
  if(Created)throw new InvalidOperationException("Fixture already owns an alias.");
  if(query(name).Length!=0)throw new InvalidOperationException("Selected fixture alias already has a mapping: "+name);
  if(dosTarget.Length<3||!((dosTarget[0]>='A'&&dosTarget[0]<='Z')||(dosTarget[0]>='a'&&dosTarget[0]<='z'))||dosTarget[1]!=':'||dosTarget[2]!='\\')throw new InvalidOperationException("Fixture alias requires a local DOS target.");
  string raw=@"\??\"+dosTarget;
  if(!define(1,name,raw)){int code=error();throw new Win32Exception(code,"Cannot create fixture alias "+name+" -> "+raw+"; Win32 error "+code);}
  // Ownership begins only after success; retain the exact target on all later errors.
  Name=name;Target=raw;Created=true;VerifyOwned();
 }
 void VerifyOwned(){string[] targets=query(Name);if(targets.Length!=1||!String.Equals(targets[0],Target,StringComparison.Ordinal))throw new InvalidOperationException("Fixture alias ownership uncertain: "+Name+" expected "+Target+"; observed "+String.Join(", ",targets));}
 public void Remove(){
  if(!Created)return;
  VerifyOwned();
  // Never pop an unspecified target or another mapping from the DOS-device stack.
  if(!define(7,Name,Target)){int code=error();throw new Win32Exception(code,"Cannot remove exact fixture alias "+Name+" -> "+Target+"; Win32 error "+code);}
  Created=false;Target=null;
 }
}
public sealed class InstalledBarrierAliasAdapter {
 public string[] Targets=new string[0];public bool FailCreate,FailRemove;public int Calls;public uint LastFlags;public string LastTarget;
 public bool Define(uint flags,string name,string target){Calls++;LastFlags=flags;LastTarget=target;if(flags==1){if(FailCreate)return false;Targets=new string[]{target};return true;}if(flags!=7||FailRemove)return false;Targets=new string[0];return true;}
 public string[] Query(string name){return Targets;}public int Error(){return 87;}
 public InstalledBarrierFixtureAlias NewOwner(){return new InstalledBarrierFixtureAlias(Define,Query,Error);}
}
'@
# Deterministic alias/recovery cases use injected adapters, never native calls.
$adapter=New-Object InstalledBarrierAliasAdapter;$aliasOwner=$adapter.NewOwner();$adapter.FailCreate=$true
$createError=$null;try{$aliasOwner.Create('Z:','C:\fixture-A')}catch{$createError=$_.Exception.GetBaseException()}
Check ($createError -is [ComponentModel.Win32Exception] -and $createError.NativeErrorCode -eq 87 -and $createError.Message.Contains('Win32 error 87')) 'Failed creation reports immediate native error without claiming alias ownership'
$aliasOwner.Remove();Check (-not $aliasOwner.Created -and $adapter.Calls -eq 1) 'Cleanup after failed creation never removes a selected alias'
$adapter.FailCreate=$false;$aliasOwner.Create('Z:','C:\fixture-A');$ownedTarget=$aliasOwner.Target
Check ($aliasOwner.Created -and $ownedTarget -ceq '\??\C:\fixture-A') 'Successful creation retains the exact actual raw target'
$adapter.Targets=@('\??\C:\foreign')
Reject {$aliasOwner.Remove()} 'Changed alias target refuses cleanup without removing another mapping'
Check ($adapter.Calls -eq 2 -and $aliasOwner.Target -ceq $ownedTarget) 'Uncertain ownership preserves target and makes no removal call'
$adapter.Targets=@($ownedTarget,$ownedTarget)
Reject {$aliasOwner.Remove()} 'Ambiguous identical stacked mapping refuses cleanup'
$adapter.Targets=@($ownedTarget);$adapter.FailRemove=$true
Reject {$aliasOwner.Remove()} 'Exact removal failure retains ownership for recovery'
Check ($aliasOwner.Created -and $aliasOwner.Target -ceq $ownedTarget -and $adapter.LastFlags -eq 7 -and $adapter.LastTarget -ceq $ownedTarget) 'Removal specifies raw target and exact-match flags'
$adapter.FailRemove=$false;$aliasOwner.Remove();$adapter.FailCreate=$true
Reject {$aliasOwner.Create('Z:','C:\fixture-B')} 'Failed retarget creation does not retain removed A ownership'
$calls=$adapter.Calls;$aliasOwner.Remove();Check (-not $aliasOwner.Created -and $adapter.Calls -eq $calls) 'Retarget failure cleanup never pops an unowned mapping'
$adapter.FailCreate=$false;$aliasOwner.Create('Z:','C:\fixture-B')
Check ($aliasOwner.Target -ceq '\??\C:\fixture-B') 'Successful retarget records B for exact cleanup'
$aliasOwner.Remove()
$sourceFiles=@('InteractiveJob.ps1','StartupLauncher.ps1','RestartCommissioningBroker.ps1','InstalledLaunchBarrier.ps1','test-installed-launch-barrier.ps1')|ForEach-Object {[pscustomobject]@{path=('tools/boat/'+$_);sha256=(Get-FileHash -LiteralPath (Join-Path $PSScriptRoot $_) -Algorithm SHA256).Hash.ToLowerInvariant()}}
$nativeStatus='not-executed-portable-contracts';$root=$null;$alias=$null;$aliasOwner=$null;$primaryFailure=$null;$guards=New-Object 'Collections.Generic.List[object]'
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
  $mask=[InstalledBarrierFixtureAlias]::GetLogicalDrives()
  if($mask -eq 0){throw 'Cannot prove unused fixture alias.'}
  foreach($letter in [char[]]'ZYXWVUTSRQPONMLKJIHGFED') {if(($mask -band (1 -shl ([int]$letter-65))) -eq 0){$alias=[string]$letter+':';break}}
  if(-not $alias){throw 'No unused private fixture alias available.'}
  $aliasOwner=New-Object InstalledBarrierFixtureAlias
  $aliasOwner.Create($alias,$installed[0].root)
  $aliasInstall=[pscustomobject]@{generation=($alias+'\generations\'+$id);state=@{current=$id};ownership=$installed[0].ownership}
  $guard=Open-InstalledLaunchBarrier $aliasInstall;$guards.Add($guard)
  $aliasOwner.Remove()
  $aliasOwner.Create($alias,$installed[1].root)
  $leaf=Join-Path $installed[0].root 'signed-update-armed.jsonl';[IO.File]::WriteAllText($leaf,'A remains armed; B has no marker')
  try {
   $message='';try{Assert-InstalledLaunchBarrier $guard}catch{$message=$_.Exception.GetBaseException().Message}
   Check ($message.Contains('Signed update session is armed')) 'Native retargeted alias still denies exact held A marker'
  } finally {Remove-Item -LiteralPath $leaf -Force}
  Reject {Assert-InstalledLaunchBarrier $guard} 'Native retargeted request refuses even after A marker removed'
  $nativeStatus='passed-inert-files-no-process-created'
 }
} catch {
 $primaryFailure=$_
 throw
} finally {
 $cleanupFailures=New-Object 'Collections.Generic.List[Exception]'
 foreach($guard in $guards){try{$guard.owner.Dispose()}catch{$cleanupFailures.Add($_.Exception)}}
 if($aliasOwner){try{$aliasOwner.Remove()}catch{$cleanupFailures.Add($_.Exception)}}
 if($root){
  if($aliasOwner -and $aliasOwner.Created){[Console]::Error.WriteLine('Retained inert fixture root for exact alias recovery: '+$root+'; alias='+$aliasOwner.Name+'; target='+$aliasOwner.Target)}
  else{try{Remove-Item -LiteralPath $root -Recurse -Force}catch{$cleanupFailures.Add($_.Exception)}}
 }
 if($cleanupFailures.Count -gt 0){
  if($primaryFailure){foreach($failure in $cleanupFailures){[Console]::Error.WriteLine('Additional fixture cleanup failure; original failure preserved: '+$failure.Message)}}
  else{throw [AggregateException]::new('Inert fixture cleanup failed; native result is not a pass.',$cleanupFailures.ToArray())}
 }
}
[pscustomobject]@{schema=1;status='passed';checks=@($checks);nativeBehavior=$nativeStatus;sourceFiles=@($sourceFiles);applicationLaunched=$false;
 scope='Fixed marker restriction at actual entrypoints; no continuously pinned absence or signed broker authority'}|ConvertTo-Json -Depth 4 -Compress
