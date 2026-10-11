# Diagnostic only. Never changes production source, resumes a child or launches
# OpenCPN. Only newly compiled inert marker images in disposable Windows CI.
[CmdletBinding()]
param([Parameter(Mandatory=$true)][string]$EvidenceDirectory,[switch]$CompileOnly)
$ErrorActionPreference='Stop'
Set-StrictMode -Version Latest
$evidence=[IO.Path]::GetFullPath($EvidenceDirectory)
if([IO.Directory]::Exists($evidence) -or [IO.File]::Exists($evidence)){throw 'Fresh diagnostic evidence directory required.'}
$null=New-Item -ItemType Directory -Path $evidence
$module=Join-Path $PSScriptRoot '../installer/windows/UpdateTransaction.ps1'
$moduleHash=(Get-FileHash -LiteralPath $module -Algorithm SHA256).Hash.ToLowerInvariant()
$scriptHash=(Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant()
$report=[ordered]@{status='failed';productAcceptance=$false;nativeExecuted=$false;resumeAttempted=$false;productionSourceSha256=$moduleHash;diagnosticSourceSha256=$scriptHash;pointerBytes=[IntPtr]::Size;expectedRows=17;rows=@();aliasAudits=@()}
$owner=$null;$guidLease=$null;$fixture='';$alias='';$aliases=New-Object 'Collections.Generic.List[string]'
try {
 $tokens=$null;$errors=$null
 $ast=[Management.Automation.Language.Parser]::ParseFile($module,[ref]$tokens,[ref]$errors)
 if($errors.Count -ne 0){throw 'Production transaction parse failed.'}
 $source=@($ast.FindAll({param($node) $node -is [Management.Automation.Language.StringConstantExpressionAst] -and $node.Value.StartsWith('using System;')},$true))
 if($source.Count -ne 1 -or ('Skager.UpdateImageOwner' -as [type])){throw 'One exact embedded owner and a fresh PowerShell process required.'}
 $sha=[Security.Cryptography.SHA256]::Create()
 try{$report.embeddedSourceSha256=[BitConverter]::ToString($sha.ComputeHash([Text.Encoding]::UTF8.GetBytes($source[0].Value))).Replace('-','').ToLowerInvariant()}finally{$sha.Dispose()}
 Add-Type -TypeDefinition $source[0].Value
 Add-Type -TypeDefinition @'
using System;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;
public sealed class NativeCreationObservation {
 public bool Created, CleanupConfirmed, NativeImageMatches;
 public int Win32Error; public uint Pid; public string NativeImage, Failure;
}
public sealed class NativeAliasObservation { public int Win32Error;public string[] Targets; }
public sealed class GuidFixtureLease : IDisposable {
 public string Path, RootPath, NativeImage, NativeRoot;
 public uint VolumeSerial, FileIndexHigh, FileIndexLow; public int HeldAncestors;
 internal SafeFileHandle File, Root, HeldFile;
 internal object HeldRootIdentity;
 public void Dispose() { if(File!=null) File.Dispose();if(Root!=null) Root.Dispose(); }
}
public static class NativeCreationDiagnostic {
 static readonly BindingFlags Private=BindingFlags.NonPublic|BindingFlags.Static;
 static object Metadata(Type owner,SafeFileHandle file) { return owner.GetMethod("Metadata",Private).Invoke(null,new object[]{file}); }
 static uint Field(object info,string name) { return (uint)info.GetType().GetField(name).GetValue(info); }
 static string Native(Type owner,SafeFileHandle file) { return (string)owner.GetMethod("Native",Private).Invoke(null,new object[]{file}); }
 static bool Same(Type owner,object a,object b) { return (bool)owner.GetMethod("Same",Private).Invoke(null,new[]{a,b}); }
 static SafeFileHandle Reopen(Type owner,string path,bool directory) {
  var handle=(SafeFileHandle)owner.GetMethod("CreateFile",Private).Invoke(null,
   new object[]{path,directory?0x80u:0x80000000u,directory?3u:1u,IntPtr.Zero,3u,0x00200000u|(directory?0x02000000u:0u),IntPtr.Zero});
  int error=Marshal.GetLastWin32Error();
  if(handle.IsInvalid) {handle.Dispose();throw new InvalidOperationException("Held-derived GUID reopen refused: "+error);}
  return handle;
 }
 public static string[] ValidateGuidNames(string path,string nativeImage) {
   var match=System.Text.RegularExpressions.Regex.Match(path,@"\A(?<root>\\\\\?\\Volume\{[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}\}\\)(?<suffix>[^:\x00]+)\z");
   var native=System.Text.RegularExpressions.Regex.Match(nativeImage,@"\A(?<root>\\Device\\HarddiskVolume[0-9]+)\\(?<suffix>.+)\z");
   if(!match.Success || !native.Success || !String.Equals(match.Groups["suffix"].Value,native.Groups["suffix"].Value,StringComparison.Ordinal)) throw new InvalidOperationException("Held-derived GUID grammar/suffix differs from held NT file.");
   foreach(string component in match.Groups["suffix"].Value.Split('\\'))
    if(component.Length==0 || component=="." || component==".." || component.IndexOf('/')>=0 || component.EndsWith(".",StringComparison.Ordinal) || component.EndsWith(" ",StringComparison.Ordinal)) throw new InvalidOperationException("Ambiguous GUID fixture suffix.");
   return new[]{match.Groups["root"].Value,native.Groups["root"].Value};
 }
 public static GuidFixtureLease AcquireGuid(Type owner,object instance) {
  var lease=new GuidFixtureLease();
  try {
   var stream=(System.IO.FileStream)owner.GetField("image",BindingFlags.NonPublic|BindingFlags.Instance).GetValue(instance);
   lease.HeldFile=stream.SafeFileHandle;
   lease.NativeImage=(string)owner.GetProperty("NativeImage").GetValue(instance,null);
   var path=new StringBuilder(32768);
   // FILE_NAME_NORMALIZED (0) | VOLUME_NAME_GUID (1), on the held file only.
   uint count=(uint)owner.GetMethod("GetFinalPathNameByHandle",Private).Invoke(null,new object[]{lease.HeldFile,path,32768u,1u});
   int error=Marshal.GetLastWin32Error();
   if(count==0 || count>=32768) throw new InvalidOperationException("Held-derived volume GUID unavailable: "+error);
   lease.Path=path.ToString();
   var roots=ValidateGuidNames(lease.Path,lease.NativeImage);
   lease.RootPath=roots[0];lease.NativeRoot=roots[1];
   var pins=(System.Collections.IList)owner.GetField("pins",BindingFlags.NonPublic|BindingFlags.Instance).GetValue(instance);
   lease.HeldAncestors=pins.Count-1;
   if(pins.Count<2) throw new InvalidOperationException("Missing held physical ancestry.");
   lease.HeldRootIdentity=pins[0].GetType().GetField("Identity").GetValue(pins[0]);
   lease.Root=Reopen(owner,lease.RootPath,true);lease.File=Reopen(owner,lease.Path,false);
   ValidateGuid(owner,instance,lease);
   var info=Metadata(owner,lease.HeldFile);
   lease.VolumeSerial=Field(info,"Volume");lease.FileIndexHigh=Field(info,"IndexHigh");lease.FileIndexLow=Field(info,"IndexLow");
   return lease;
  } catch {lease.Dispose();throw;}
 }
 public static void ValidateGuid(Type owner,object instance,GuidFixtureLease lease) {
  owner.GetMethod("Validate").Invoke(instance,null);
  var held=Metadata(owner,lease.HeldFile);var reopened=Metadata(owner,lease.File);var root=Metadata(owner,lease.Root);
  if(!Same(owner,held,reopened) || !Same(owner,lease.HeldRootIdentity,root) ||
     (Field(reopened,"Attributes")&0x410u)!=0 || Field(reopened,"Links")!=1 ||
     Field(held,"SizeHigh")!=Field(reopened,"SizeHigh") || Field(held,"SizeLow")!=Field(reopened,"SizeLow") ||
     (Field(root,"Attributes")&0x410u)!=0x10u ||
     !String.Equals(Native(owner,lease.File),lease.NativeImage,StringComparison.Ordinal) ||
     !String.Equals(Native(owner,lease.Root),lease.NativeRoot,StringComparison.Ordinal)) throw new InvalidOperationException("GUID reopen does not retain exact held NT volume/file identity.");
 }
 public static string[] Layout(Type owner) {
  var startup=owner.GetNestedType("Startup",BindingFlags.NonPublic);
  var child=owner.GetNestedType("Child",BindingFlags.NonPublic);
  var output=new System.Collections.Generic.List<string>();
  output.Add("Startup.size="+Marshal.SizeOf(startup));
  foreach(var field in startup.GetFields()) output.Add("Startup."+field.Name+"="+Marshal.OffsetOf(startup,field.Name));
  output.Add("Child.size="+Marshal.SizeOf(child));
  foreach(var field in child.GetFields()) output.Add("Child."+field.Name+"="+Marshal.OffsetOf(child,field.Name));
  var method=owner.GetMethod("CreateProcessW",Private);
  var dll=(DllImportAttribute)Attribute.GetCustomAttribute(method,typeof(DllImportAttribute));
  output.Add("DllImport="+dll.Value+";EntryPoint="+dll.EntryPoint+";CharSet="+dll.CharSet+";SetLastError="+dll.SetLastError);
  foreach(var parameter in method.GetParameters()) output.Add("Parameter."+parameter.Name+"="+parameter.ParameterType.FullName);
  return output.ToArray();
 }
 // There is deliberately no ResumeThread invocation anywhere in this helper.
 public static NativeCreationObservation Observe(Type owner,string application,string argv0,object directory,IntPtr environment,uint flags,string expectedNative) {
  if((flags&4u)==0) throw new ArgumentException("All diagnostic children must be created suspended.");
  var startupType=owner.GetNestedType("Startup",BindingFlags.NonPublic);
  var childType=owner.GetNestedType("Child",BindingFlags.NonPublic);
  var startup=Activator.CreateInstance(startupType);
  startupType.GetField("Size").SetValue(startup,(uint)Marshal.SizeOf(startupType));
  // PowerShell converts $null to String.Empty for a string parameter. Keep an
  // object boundary so inherited-CWD rows pass an actual native NULL instead.
  string currentDirectory=directory==null?null:(string)directory;
  var args=new object[]{application,new StringBuilder("\""+argv0+"\""),IntPtr.Zero,IntPtr.Zero,false,flags,environment,currentDirectory,startup,Activator.CreateInstance(childType)};
  var method=owner.GetMethod("CreateProcessW",Private);
  bool created=(bool)method.Invoke(null,args);
  int error=Marshal.GetLastWin32Error(); // Capture before any other native call.
  var observation=new NativeCreationObservation { Created=created,Win32Error=created?0:error };
  IntPtr process=(IntPtr)childType.GetField("Process").GetValue(args[9]);
  IntPtr thread=(IntPtr)childType.GetField("Thread").GetValue(args[9]);
  observation.Pid=(uint)childType.GetField("Pid").GetValue(args[9]);
  if(!created) {
   if(process!=IntPtr.Zero || thread!=IntPtr.Zero || observation.Pid!=0) {
    observation.Failure="Failed creation returned ambiguous child state; no process signaled.";
    return observation;
   }
   observation.CleanupConfirmed=true;return observation;
  }
  try {
   if(process==IntPtr.Zero || thread==IntPtr.Zero || observation.Pid==0) throw new InvalidOperationException("Successful creation returned incomplete ownership.");
   observation.NativeImage=(string)owner.GetMethod("NativeProcess",Private).Invoke(null,new object[]{process});
   observation.NativeImageMatches=String.Equals(observation.NativeImage,expectedNative,StringComparison.Ordinal);
   if(!observation.NativeImageMatches) throw new InvalidOperationException("Created suspended image differs from held fixture.");
   if((uint)owner.GetMethod("WaitForSingleObject",Private).Invoke(null,new object[]{process,0u})!=258) throw new InvalidOperationException("Suspended child unexpectedly exited.");
  } catch(Exception problem) { observation.Failure=problem.ToString(); }
  finally {
   // Only this successful call's raw returned handle, with CREATE_SUSPENDED
   // and no resume attempt, authorizes inert-child termination.
   try {
    if(process==IntPtr.Zero) throw new InvalidOperationException("No owned raw child handle for cleanup.");
    bool terminated=(bool)owner.GetMethod("TerminateProcess",Private).Invoke(null,new object[]{process,126u});
    int terminateError=Marshal.GetLastWin32Error();
    if(!terminated) throw new InvalidOperationException("Owned suspended termination failed: "+terminateError);
    if((uint)owner.GetMethod("WaitForSingleObject",Private).Invoke(null,new object[]{process,10000u})!=0) throw new InvalidOperationException("Owned suspended cleanup uncertain.");
    observation.CleanupConfirmed=true;
   } catch(Exception problem) { observation.CleanupConfirmed=false;observation.Failure+=" Cleanup: "+problem.ToString(); }
   finally {
    foreach(var handle in new[]{thread,process}) {
     if(handle==IntPtr.Zero) continue;
     try {
      if(!(bool)owner.GetMethod("CloseHandle",Private).Invoke(null,new object[]{handle})) {
       observation.CleanupConfirmed=false;observation.Failure+=" Raw handle closure refused.";
      }
     } catch(Exception problem) { observation.CleanupConfirmed=false;observation.Failure+=" Handle closure: "+problem.ToString(); }
    }
   }
  }
  return observation;
 }
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] public static extern bool DefineDosDevice(uint flags,string name,string target);
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)] static extern uint QueryDosDevice(string name,IntPtr value,uint length);
 public static NativeAliasObservation QueryAlias(string name) {
  IntPtr buffer=Marshal.AllocHGlobal(32768*2);
  try {
   uint count=QueryDosDevice(name,buffer,32768);
   int error=Marshal.GetLastWin32Error();
   if(count==0) return new NativeAliasObservation {Win32Error=error,Targets=new string[0]};
   if(count>32768) throw new InvalidOperationException("Alias query exceeds fixed buffer.");
   var characters=new char[count];Marshal.Copy(buffer,characters,0,(int)count);
   if(characters[count-1]!='\0') throw new InvalidOperationException("Alias query is not NUL terminated.");
   return new NativeAliasObservation {Targets=new string(characters).Split(new[]{'\0'},StringSplitOptions.RemoveEmptyEntries)};
  } finally {Marshal.FreeHGlobal(buffer);}
 }
}
'@
 $report.abi=@([NativeCreationDiagnostic]::Layout([Skager.UpdateImageOwner]))
 if($CompileOnly){$report.status='compiled-only';Write-Host 'COMPILED ONLY: exact embedded owner and diagnostic helper; no native calls.';return}
 if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or $env:GITHUB_ACTIONS -cne 'true' -or $env:RUNNER_ENVIRONMENT -cne 'github-hosted' -or $env:RUNNER_OS -cne 'Windows' -or $env:GITHUB_REPOSITORY -cne 'ThereptileII/Work' -or $PSVersionTable.PSVersion.Major -ne 5){throw 'Exact repository, GitHub-hosted native Windows CI and Windows PowerShell 5.1 required.'}
 $identity=[Security.Principal.WindowsIdentity]::GetCurrent()
 try{if($identity.User.Value -ceq 'S-1-5-18'){throw 'LocalSystem/global DOS namespace mutation is refused.'};$report.operatorSid=$identity.User.Value}finally{$identity.Dispose()}
 . (Join-Path $PSScriptRoot '../installer/windows/UpdateSupervisor.ps1')
 Assert-NoUpdateApplication
 $fixture=Join-Path ([IO.Path]::GetTempPath()) ('SkagerCreateProcessDiagnostic-'+[guid]::NewGuid().ToString('N'))
 $null=New-Item -ItemType Directory -Path $fixture
 $a=Join-Path $fixture 'A';$b=Join-Path $fixture 'B'
 foreach($path in @($a,$b)){$null=New-Item -ItemType Directory -Path (Join-Path $path 'app') -Force}
 $aExe=Join-Path $a 'app/marker.exe';$bExe=Join-Path $b 'app/marker.exe'
 $marker='using System;using System.IO;static class Marker{static void Main(){File.WriteAllText(Environment.GetEnvironmentVariable("SKAGER_CREATEPROCESS_DIAGNOSTIC_MARKER"),"IMAGE");}}'
 $compiler=Join-Path ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory()) 'csc.exe'
 foreach($pair in @(@('A',$aExe),@('B',$bExe))){
  $file=Join-Path $fixture ('marker-'+$pair[0]+'.cs')
  [IO.File]::WriteAllText($file,$marker.Replace('IMAGE',$pair[0]))
  & $compiler /nologo /target:winexe /platform:x86 ('/out:'+$pair[1]) $file
  if($LASTEXITCODE -ne 0 -or -not [IO.File]::Exists($pair[1])){throw 'Harmless marker compilation failed.'}
 }
 $aHash=(Get-FileHash -LiteralPath $aExe -Algorithm SHA256).Hash.ToLowerInvariant()
 $bHash=(Get-FileHash -LiteralPath $bExe -Algorithm SHA256).Hash.ToLowerInvariant()
 if($aHash -ceq $bHash){throw 'Inert replacement bytes must differ.'}
 foreach($letter in @('Z:','Y:','X:','W:','V:')){
  $query=[NativeCreationDiagnostic]::QueryAlias($letter)
  if($query.Targets.Count -eq 0 -and $query.Win32Error -eq 2){$alias=$letter;break}
 }
 if(-not $alias){throw 'No unused disposable DOS drive letter.'}
 foreach($target in @($a,$b)){
  $native='\??\'+$target
  if(-not [NativeCreationDiagnostic]::DefineDosDevice(9,$alias,$native)){throw 'Disposable alias mapping failed.'}
  $aliases.Add($native)
  $query=[NativeCreationDiagnostic]::QueryAlias($alias)
  $expected=@($aliases.ToArray());[Array]::Reverse($expected)
  $report.aliasAudits+= [ordered]@{operation='add';alias=$alias;ownedTarget=$native;targets=@($query.Targets);win32Error=$query.Win32Error}
  if($query.Win32Error -ne 0 -or $query.Targets.Count -ne $expected.Count -or ([string]::Join([string][char]0,$query.Targets) -cne [string]::Join([string][char]0,$expected))){throw 'Exact owned alias stack differs after add.'}
  if($target -ceq $a){$owner=[Skager.UpdateImageOwner]::Acquire(($alias+'\app\marker.exe'),($alias+'\app'),$aHash)}
 }
 $owner.Validate()
 $physical='\\?\GLOBALROOT'+$owner.NativeImage
 $physicalDirectory=$physical.Substring(0,$physical.LastIndexOf('\'))
 $dosDirectory=[IO.Path]::GetDirectoryName($aExe)
 $unexpectedMarker=Join-Path $fixture 'UNEXPECTED-RESUMED'
 $start=New-Object Diagnostics.ProcessStartInfo
 $start.UseShellExecute=$false
 $start.EnvironmentVariables['SKAGER_CREATEPROCESS_DIAGNOSTIC_MARKER']=$unexpectedMarker
 $variables=New-Object 'Collections.Generic.List[string]'
 foreach($item in $start.EnvironmentVariables.GetEnumerator()){
  $key=[string]$item.Key;$value=[string]$item.Value
  if($key.IndexOf([char]0) -ge 0 -or $key.IndexOf('=') -ge 0 -or $value.IndexOf([char]0) -ge 0){throw 'Unsupported diagnostic environment.'}
  $variables.Add($key+'='+$value)
 }
 $variables.Sort([StringComparer]::OrdinalIgnoreCase)
 $ownerEnvironment=[string]::Join([string][char]0,$variables.ToArray())+[char]0+[char]0
 $minimalEnvironment='SKAGER_CREATEPROCESS_DIAGNOSTIC_MARKER='+$unexpectedMarker+[char]0+'SystemRoot='+$env:SystemRoot+[char]0+[char]0
 $cases=New-Object 'Collections.Generic.List[object]'
 foreach($application in @(@('dos',$aExe),@('globalroot',$physical))){
  foreach($directory in @(@('inherited',$null),@('dos',$dosDirectory),@('globalroot',$physicalDirectory))){
   $cases.Add([pscustomobject]@{name=$application[0]+'-app_'+$directory[0]+'-cwd_owner-unicode';application=$application[1];directory=$directory[1];environment='owner';flags=[uint32]0x404})
  }
 }
 foreach($control in @(@('inherited-env_unicode-flag','inherited',[uint32]0x404),@('inherited-env_suspended-only','inherited',[uint32]4),@('minimal-unicode','minimal',[uint32]0x404),@('owner-unicode_no-window','owner',[uint32]0x08000404))){
  $cases.Add([pscustomobject]@{name=$control[0];application=$aExe;directory=$dosDirectory;environment=$control[1];flags=$control[2]})
 }
 # The original ten rows above keep their exact application/argv0 pairing.
 foreach($cross in @(@('globalroot-app_basename-argv0',$physical,[IO.Path]::GetFileName($aExe)),@('globalroot-app_dos-argv0',$physical,$aExe),@('dos-app_globalroot-argv0',$aExe,$physical))){
  $cases.Add([pscustomobject]@{name=$cross[0];application=$cross[1];argv0=$cross[2];directory=$physicalDirectory;environment='owner';flags=[uint32]0x404})
 }
 $report.nativeExecuted=$true;$report.fixtureImageSha256=$aHash;$report.replacementImageSha256=$bHash
 $report.heldNativeImage=$owner.NativeImage;$report.aliasRemapped=$true;$report.environmentCharacterCount=$ownerEnvironment.Length
 foreach($phase in @('original-and-crossed','held-guid')){
  $activeCases=$cases
  if($phase -eq 'held-guid'){
 $guidLease=[NativeCreationDiagnostic]::AcquireGuid([Skager.UpdateImageOwner],$owner)
 $report.guidValidation=[ordered]@{path=$guidLease.Path;root=$guidLease.RootPath;pathUtf16Length=$guidLease.Path.Length;nativeImage=$guidLease.NativeImage;nativeRoot=$guidLease.NativeRoot;volumeSerial=$guidLease.VolumeSerial;fileIndexHigh=$guidLease.FileIndexHigh;fileIndexLow=$guidLease.FileIndexLow;heldAncestors=$guidLease.HeldAncestors;normalizedFromHeldFile=$true;reopenedRootAndFileIdentityMatch=$true;dosFallback=$false;namespaceLinkPinned=$false}
  $activeCases=New-Object 'Collections.Generic.List[object]'
 foreach($directory in @(@('inherited',$null),@('dos',$dosDirectory),@('globalroot',$physicalDirectory))){
  $activeCases.Add([pscustomobject]@{name='guid-app_'+$directory[0]+'-cwd_guid-argv0';application=$guidLease.Path;argv0=$guidLease.Path;directory=$directory[1];environment='owner';flags=[uint32]0x404})
 }
 $activeCases.Add([pscustomobject]@{name='guid-app_globalroot-argv0_globalroot-cwd';application=$guidLease.Path;argv0=$physical;directory=$physicalDirectory;environment='owner';flags=[uint32]0x404})
  }
  foreach($case in $activeCases){
  $owner.Validate();if($guidLease){[NativeCreationDiagnostic]::ValidateGuid([Skager.UpdateImageOwner],$owner,$guidLease)};$environment=[IntPtr]::Zero
  try {
   if($case.environment -ne 'inherited'){
    $block=if($case.environment -eq 'owner'){$ownerEnvironment}else{$minimalEnvironment}
    $environment=[Runtime.InteropServices.Marshal]::StringToHGlobalUni($block)
   }
   $argv0=if($case.PSObject.Properties.Name -contains 'argv0'){$case.argv0}else{$case.application}
   $observed=[NativeCreationDiagnostic]::Observe([Skager.UpdateImageOwner],$case.application,$argv0,$case.directory,$environment,$case.flags,$owner.NativeImage)
   $report.rows+= [ordered]@{name=$case.name;application=$case.application;applicationUtf16Length=$case.application.Length;argv0=$argv0;argv0Utf16Length=$argv0.Length;directory=$case.directory;directoryIsNull=($null -eq $case.directory);directoryUtf16Length=$(if($null -eq $case.directory){$null}else{$case.directory.Length});commandUtf16Length=($argv0.Length+2);environment=$case.environment;flags=$case.flags;created=$observed.Created;win32Error=$observed.Win32Error;pid=$observed.Pid;nativeImage=$observed.NativeImage;nativeImageMatches=$observed.NativeImageMatches;cleanupConfirmed=$observed.CleanupConfirmed;failure=$observed.Failure}
   $report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $evidence 'diagnostic.json') -Encoding UTF8
   Write-Host ('OBSERVED: '+$case.name+' created='+$observed.Created+' Win32='+$observed.Win32Error+' cleanup='+$observed.CleanupConfirmed)
   if(-not $observed.CleanupConfirmed -or $observed.Failure){throw 'Diagnostic child ownership/cleanup failed; remaining cases refused.'}
  } finally {if($environment -ne [IntPtr]::Zero){[Runtime.InteropServices.Marshal]::FreeHGlobal($environment)}}
 }
 }
 $baseline=@($report.rows | Where-Object {$_.name -ceq 'dos-app_dos-cwd_owner-unicode'})
 $uniqueNames=@($report.rows | ForEach-Object {$_.name} | Select-Object -Unique)
 if($report.rows.Count -ne $report.expectedRows -or $uniqueNames.Count -ne $report.expectedRows -or $baseline.Count -ne 1 -or -not $baseline[0].created){throw 'Incomplete named observations or failed physical DOS baseline.'}
 if([IO.File]::Exists($unexpectedMarker)){throw 'Inert marker executed despite suspended-only diagnostic.'}
 $owner.Validate();$report.status='observed'
 Write-Host 'OBSERVED ONLY: seventeen bounded rows; no resumed child and no product acceptance.'
} catch {
 $report.error=$_.Exception.ToString();throw
} finally {
 $cleanup=$true
 if($guidLease){try{$guidLease.Dispose()}catch{$cleanup=$false}}
 if($owner){try{$owner.Dispose()}catch{$cleanup=$false}}
 for($i=$aliases.Count-1;$i -ge 0;$i--){
  try {
   if(-not [NativeCreationDiagnostic]::DefineDosDevice(15,$alias,$aliases[$i])){throw 'Exact-owned alias removal refused.'}
   $query=[NativeCreationDiagnostic]::QueryAlias($alias)
   $expected=@();for($j=$i-1;$j -ge 0;$j--){$expected+=$aliases[$j]}
   $report.aliasAudits+= [ordered]@{operation='remove-exact';alias=$alias;ownedTarget=$aliases[$i];targets=@($query.Targets);win32Error=$query.Win32Error}
   if($expected.Count -eq 0){if($query.Targets.Count -ne 0 -or $query.Win32Error -ne 2){throw 'Original alias absence not restored.'}}
   elseif($query.Win32Error -ne 0 -or $query.Targets.Count -ne $expected.Count -or ([string]::Join([string][char]0,$query.Targets) -cne [string]::Join([string][char]0,$expected))){throw 'Exact owned alias stack differs after removal.'}
  } catch {$cleanup=$false;$report.aliasCleanupError=$_.Exception.ToString()}
 }
 if($aliases.Count -gt 0){
  try {
   $query=[NativeCreationDiagnostic]::QueryAlias($alias)
   $report.aliasAudits+= [ordered]@{operation='final-original-absence';alias=$alias;targets=@($query.Targets);win32Error=$query.Win32Error}
   if($query.Targets.Count -ne 0 -or $query.Win32Error -ne 2){throw 'Final original DOS alias absence not restored.'}
  } catch {$cleanup=$false;$report.aliasCleanupError=$_.Exception.ToString()}
 }
 if($fixture -and [IO.Directory]::Exists($fixture)){
  if($cleanup -and -not $report.Contains('error')){Remove-Item -LiteralPath $fixture -Recurse -Force}else{$report.retainedPrivateFixture=$fixture}
 }
 $report.fixtureCleanupConfirmed=$cleanup
 if((Get-FileHash -LiteralPath $module -Algorithm SHA256).Hash.ToLowerInvariant() -cne $moduleHash -or (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant() -cne $scriptHash){$report.status='failed';$report.sourceDrift=$true}
 if(-not $cleanup){$report.status='failed'}
 $report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $evidence 'diagnostic.json') -Encoding UTF8
 if(-not $cleanup -or $report.Contains('sourceDrift')){throw 'Diagnostic cleanup/source identity uncertain.'}
}
