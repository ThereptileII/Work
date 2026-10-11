# SCRUM-311 persistence foundation only: NO artifact authentication, IPC,
# installation, process creation, launch permission or interrupted-update recovery.
# A trusted same-process native caller must validate real evidence separately.
# New-SignedUpdateStore accepts a pre-created private directory; it NEVER resumes
# an existing armed file. Close/failure NEVER deletes it. A future integrated gate
# MUST deny ordinary startup when the marker/custodian is missing or abandoned.
# Disk snapshots alone cannot prove freshness after owner death or whole-disk
# rollback. Receipts returned here are consumed records, NOT reusable allow tokens.
Set-StrictMode -Version Latest
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'SignedUpdateOperations.ps1')
# Do not reset custody when this file is dot-sourced twice in the same scope.
if(-not (Get-Variable SignedUpdateStores -Scope Script -ErrorAction SilentlyContinue)){$script:SignedUpdateStores=@{}}
if(-not ('SignedUpdateStoreNative' -as [type])) {
Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
public static class SignedUpdateStoreNative {
  // Validate BEFORE PowerShell's parser, which otherwise collapses duplicate keys.
  // This protocol deliberately accepts integer numbers only, no exponent/fraction.
  public static string Json(byte[] bytes) {
    if(bytes==null || bytes.Length==0 || bytes.Length>65536) throw new InvalidDataException("JSON size.");
    string s=new UTF8Encoding(false,true).GetString(bytes);
    new Parser(s).Run(); return s;
  }
  sealed class Parser {
    readonly string s; int p, nodes;
    public Parser(string text){s=text;}
    void Fail(){throw new InvalidDataException("Invalid or ambiguous bounded JSON.");}
    void Space(){while(p<s.Length && (s[p]==' '||s[p]=='\r'||s[p]=='\n'||s[p]=='\t'))p++;}
    bool Take(char c){Space();if(p<s.Length&&s[p]==c){p++;return true;}return false;}
    public void Run(){Value(0);Space();if(p!=s.Length)Fail();}
    string Str(){
      if(!Take('"'))Fail();var b=new StringBuilder();bool done=false;
      while(p<s.Length){char c=s[p++];if(c=='"'){done=true;break;}if(c<32)Fail();
        if(c=='\\'){if(p==s.Length)Fail();c=s[p++];switch(c){
          case '"':case '\\':case '/':break;case 'b':c='\b';break;case 'f':c='\f';break;
          case 'n':c='\n';break;case 'r':c='\r';break;case 't':c='\t';break;
          case 'u': if(p+4>s.Length)Fail();int n=0;for(int i=0;i<4;i++){
            char h=s[p++];int v=h>='0'&&h<='9'?h-'0':h>='a'&&h<='f'?h-'a'+10:h>='A'&&h<='F'?h-'A'+10:-1;
            if(v<0)Fail();n=n*16+v;}c=(char)n;break;default:Fail();break;}}
        b.Append(c);if(b.Length>8192)Fail();
      }
      if(!done)Fail();string value=b.ToString();
      for(int i=0;i<value.Length;i++)if(char.IsSurrogate(value[i])){
        if(!char.IsHighSurrogate(value[i])||i+1==value.Length||!char.IsLowSurrogate(value[++i]))Fail();}
      return value;
    }
    void Value(int depth){
      if(depth>12 || ++nodes>4096)Fail();Space();if(p==s.Length)Fail();char c=s[p];
      if(c=='{'){p++;var keys=new HashSet<string>(StringComparer.OrdinalIgnoreCase);if(Take('}'))return;
        do {string k=Str();if(k.Length==0||k.Length>128||!keys.Add(k))Fail();if(!Take(':'))Fail();Value(depth+1);
          if(Take('}'))return;if(!Take(','))Fail();}while(true);
      }
      if(c=='['){p++;if(Take(']'))return;do{Value(depth+1);if(Take(']'))return;if(!Take(','))Fail();}while(true);}
      if(c=='"'){Str();return;}
      foreach(string literal in new[]{"true","false","null"})if(p+literal.Length<=s.Length && String.CompareOrdinal(s,p,literal,0,literal.Length)==0){p+=literal.Length;return;}
      int start=p;if(c=='-')p++;if(p==s.Length||s[p]<'0'||s[p]>'9')Fail();
      if(s[p]=='0')p++;else while(p<s.Length&&s[p]>='0'&&s[p]<='9')p++;
      long parsed;if(!Int64.TryParse(s.Substring(start,p-start),System.Globalization.NumberStyles.AllowLeadingSign,System.Globalization.CultureInfo.InvariantCulture,out parsed))Fail();
    }
  }
  [StructLayout(LayoutKind.Sequential)] struct Info {
    public uint Attributes; public System.Runtime.InteropServices.ComTypes.FILETIME Creation,Access,Write;
    public uint Volume,SizeHigh,SizeLow,Links,IndexHigh,IndexLow;
  }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern SafeFileHandle CreateFile(string path,uint access,uint share,IntPtr security,uint mode,uint flags,IntPtr template);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetFileInformationByHandle(SafeFileHandle h,out Info info);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern uint GetFinalPathNameByHandleW(SafeFileHandle h,StringBuilder name,uint length,uint flags);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern bool GetVolumeInformationByHandleW(SafeFileHandle h,StringBuilder volume,uint volumeLength,out uint serial,out uint componentLength,out uint flags,StringBuilder filesystem,uint filesystemLength);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode)] static extern uint GetDriveTypeW(string root);
  [StructLayout(LayoutKind.Sequential)] struct SecurityAttributes {
    public int Length; public IntPtr Descriptor; [MarshalAs(UnmanagedType.Bool)] public bool Inherit;
  }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true,EntryPoint="CreateFileW")]
  static extern SafeFileHandle CreatePrivateFile(string path,uint access,uint share,ref SecurityAttributes security,uint mode,uint flags,IntPtr template);
  [DllImport("advapi32.dll",SetLastError=true)] static extern uint GetSecurityInfo(SafeFileHandle h,int kind,uint flags,out IntPtr owner,out IntPtr group,out IntPtr dacl,out IntPtr sacl,out IntPtr descriptor);
  [DllImport("advapi32.dll")] static extern uint GetSecurityDescriptorLength(IntPtr descriptor);
  [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr pointer);
  public static byte[] SecurityDescriptor(SafeFileHandle h){
    IntPtr owner,group,dacl,sacl,descriptor;
    uint error=GetSecurityInfo(h,1,5,out owner,out group,out dacl,out sacl,out descriptor);
    if(error!=0)throw new System.ComponentModel.Win32Exception((int)error);
    try{uint length=GetSecurityDescriptorLength(descriptor);if(length==0||length>65536)throw new IOException("Security descriptor bound.");
      byte[] bytes=new byte[length];Marshal.Copy(descriptor,bytes,0,bytes.Length);return bytes;
    }finally{LocalFree(descriptor);}
  }
  public static FileStream CreateJournal(string path,byte[] descriptor){
    if(descriptor==null||descriptor.Length==0||descriptor.Length>65536)throw new IOException("Private descriptor required.");
    IntPtr memory=Marshal.AllocHGlobal(descriptor.Length);SafeFileHandle h=null;
    try{Marshal.Copy(descriptor,0,memory,descriptor.Length);
      var attributes=new SecurityAttributes{Length=Marshal.SizeOf(typeof(SecurityAttributes)),Descriptor=memory,Inherit=false};
      // CREATE_NEW, exclusive sharing, write-through and open-reparse-point.
      h=CreatePrivateFile(path,0xC0000000,0,ref attributes,1,0x80200000,IntPtr.Zero);
      if(h.IsInvalid)throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      Check(h,false);var stream=new FileStream(h,FileAccess.ReadWrite,4096,false);h=null;return stream;
    }finally{if(h!=null)h.Dispose();Marshal.FreeHGlobal(memory);}
  }
  public static FileStream InstallationLock(string path){
    // OPEN_ALWAYS uses the same exclusive lock as installed maintenance/startup.
    // OPEN_REPARSE_POINT plus Check refuses substituted links and hardlinks.
    SafeFileHandle h=CreateFile(path,0xC0000000,0,IntPtr.Zero,4,0x00200000,IntPtr.Zero);
    try{
      if(h.IsInvalid)throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      Check(h,false);var stream=new FileStream(h,FileAccess.ReadWrite,4096,false);h=null;return stream;
    }finally{if(h!=null)h.Dispose();}
  }
  public static FileStream ReadRecord(string path){
    // Use a native physical spelling directly, including on .NET Framework.
    SafeFileHandle h=CreateFile(path,0x80000000,1,IntPtr.Zero,3,0x00200000,IntPtr.Zero);
    try{
      if(h.IsInvalid)throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      Check(h,false);var stream=new FileStream(h,FileAccess.Read,4096,false);h=null;return stream;
    }finally{if(h!=null)h.Dispose();}
  }
  public static void Check(SafeFileHandle h,bool directory){
    Info i;if(h.IsClosed||h.IsInvalid||!GetFileInformationByHandle(h,out i))throw new IOException("Custody handle lost.");
    if((i.Attributes&0x400)!=0 || ((i.Attributes&0x10)!=0)!=directory || (!directory&&i.Links!=1))throw new IOException("Reparse/type/hardlink refused.");
  }
  public static SafeFileHandle[] PinDirectories(string path){
    if(!System.Text.RegularExpressions.Regex.IsMatch(path,@"^[A-Za-z]:\\") || path.Length>240 || path!=Path.GetFullPath(path) || path.IndexOf('/')>=0)throw new IOException("Canonical local directory required.");
    string drive=Path.GetPathRoot(path);var parts=path.Substring(drive.Length).Split('\\');
    foreach(string part in parts)
      if(part.Length==0||part=="."||part==".."||part.EndsWith(".")||part.EndsWith(" ")||part.IndexOf(':')>=0)throw new IOException("Noncanonical directory.");
    var pins=new List<SafeFileHandle>();try{
      // Capture the DOS drive only once. Subsequent opens never resolve that
      // mutable alias again, and each returned handle must name the expected child.
      var first=CreateFile(drive,0x20080,3,IntPtr.Zero,3,0x02200000,IntPtr.Zero);pins.Add(first);Check(first,true);
      string captured=NativeName(first);
      var filesystem=new StringBuilder(64);uint serial,component,flags;
      if(!GetVolumeInformationByHandleW(first,null,0,out serial,out component,out flags,filesystem,(uint)filesystem.Capacity) || filesystem.ToString()!="NTFS")throw new IOException("Held volume must be NTFS.");
      var volume=new StringBuilder(32768);
      uint count=GetFinalPathNameByHandleW(first,volume,(uint)volume.Capacity,1); // VOLUME_NAME_GUID
      var match=System.Text.RegularExpressions.Regex.Match(volume.ToString(),@"^(\\\\\?\\Volume\{[a-fA-F0-9-]{36}\}\\)");
      if(count==0||count>=volume.Capacity||!match.Success||GetDriveTypeW(match.Groups[1].Value)!=3)throw new IOException("Held volume must be a local fixed disk.");
      // A SUBST root may itself be far below the volume root. Pin its actual
      // physical ancestors too, not just descendants of the requested alias.
      var device=System.Text.RegularExpressions.Regex.Match(captured,@"^\\Device\\HarddiskVolume[0-9]+",System.Text.RegularExpressions.RegexOptions.IgnoreCase);
      string current=device.Value;
      PinPhysical(pins,current,true);
      string suffix=captured.Substring(current.Length);
      if(suffix.Length>0)foreach(string part in suffix.Substring(1).Split('\\')){
        current+="\\"+part;PinPhysical(pins,current,false);
      }
      if(!String.Equals(NativeName(first),captured,StringComparison.OrdinalIgnoreCase))throw new IOException("Captured alias target moved during ancestry acquisition.");
      foreach(string part in parts){
        current+="\\"+part;
        PinPhysical(pins,current,false);
      }
      return pins.ToArray();
    }catch{foreach(var h in pins)h.Dispose();throw;}
  }
  static void PinPhysical(List<SafeFileHandle> pins,string name,bool volumeRoot){
    var h=CreateFile(@"\\?\GLOBALROOT"+name+(volumeRoot?"\\":""),0x20080,3,IntPtr.Zero,3,0x02200000,IntPtr.Zero);
    pins.Add(h);Check(h,true);
    if(!String.Equals(NativeName(h),name,StringComparison.OrdinalIgnoreCase))throw new IOException("Held directory physical identity differs.");
  }
  public static string NativeName(SafeFileHandle handle){
    if(handle==null||handle.IsInvalid||handle.IsClosed)throw new IOException("Native identity handle unavailable.");
    var name=new StringBuilder(32768);uint count=GetFinalPathNameByHandleW(handle,name,(uint)name.Capacity,2); // VOLUME_NAME_NT
    if(count==0||count>=name.Capacity)throw new IOException("Native identity unavailable.");
    string value=name.ToString().TrimEnd('\\');
    if(!System.Text.RegularExpressions.Regex.IsMatch(value,@"^\\Device\\HarddiskVolume[0-9]+(?:\\|$)",System.Text.RegularExpressions.RegexOptions.IgnoreCase))throw new IOException("Local physical disk identity required.");
    return value;
  }
  public static string PhysicalDirectory(SafeFileHandle[] pins){
    if(pins==null||pins.Length==0)throw new IOException("Held directory ancestry required.");
    foreach(var pin in pins)Check(pin,true);
    return @"\\?\GLOBALROOT"+NativeName(pins[pins.Length-1]);
  }
  public static void CheckChild(SafeFileHandle child,SafeFileHandle parent){
    Check(child,false);Check(parent,true);
    if(!String.Equals(NativeName(child),NativeName(parent)+"\\signed-update-armed.jsonl",StringComparison.OrdinalIgnoreCase))throw new IOException("Armed file is outside its held physical directory.");
  }
}
'@
}
function ConvertFrom-SignedStoreJson([byte[]]$Bytes) {
  $text=[SignedUpdateStoreNative]::Json($Bytes)
  if($text.TrimStart()[0] -cne '{'){throw 'Protocol root must be one object.'}
  return ConvertFrom-Json -InputObject $text
}
function Get-SignedStoreBytes($Value) {return ,([Text.UTF8Encoding]::new($false,$true).GetBytes(($Value|ConvertTo-Json -Compress -Depth 16)))}
function Get-SignedStoreDigest([byte[]]$Bytes) {
  $sha=[Security.Cryptography.SHA256]::Create()
  try{return ([BitConverter]::ToString($sha.ComputeHash($Bytes))).Replace('-','').ToLowerInvariant()}finally{$sha.Dispose()}
}
function Get-SignedStoreNow {return [long][Math]::Floor(([DateTime]::UtcNow-[DateTime]::new(1970,1,1,0,0,0,[DateTimeKind]::Utc)).TotalSeconds)}
function Get-SignedStoreOwner {
  if([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT){throw 'Native Windows custody is required.'}
  $p=[Diagnostics.Process]::GetCurrentProcess()
  try {
    $image=$p.MainModule.FileName
    $stream=[IO.File]::Open($image,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::Read)
    try {
      if($stream.Length -gt 67108864){throw 'Owner image exceeds bound.'}
      $sha=[Security.Cryptography.SHA256]::Create()
      try{$digest=([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-','').ToLowerInvariant()}finally{$sha.Dispose()}
    } finally {$stream.Dispose()}
    return [pscustomobject][ordered]@{sid=[Security.Principal.WindowsIdentity]::GetCurrent().User.Value;pid=$PID;createdFiletime=$p.StartTime.ToUniversalTime().ToFileTimeUtc().ToString();sessionId=$p.SessionId;image=$image;imageSha256=$digest}
  } finally {$p.Dispose()}
}
function Assert-SignedStoreAcl($Handle,[string]$Sid,[bool]$Directory) {
  if($Directory){$acl=[Security.AccessControl.DirectorySecurity]::new()}else{$acl=[Security.AccessControl.FileSecurity]::new()}
  $acl.SetSecurityDescriptorBinaryForm([SignedUpdateStoreNative]::SecurityDescriptor($Handle))
  if($acl.GetOwner([Security.Principal.SecurityIdentifier]).Value -cne $Sid){throw 'Custody owner differs.'}
  if(-not $acl.AreAccessRulesProtected){throw 'Custody requires a protected ACL.'}
  $seen=@{}
  foreach($rule in $acl.GetAccessRules($true,$true,[Security.Principal.SecurityIdentifier])) {
    if($rule.IdentityReference.Value -cnotin @($Sid,'S-1-5-18') -or $rule.AccessControlType -ne [Security.AccessControl.AccessControlType]::Allow -or
       $rule.FileSystemRights -ne [Security.AccessControl.FileSystemRights]::FullControl -or
       $rule.PropagationFlags -ne [Security.AccessControl.PropagationFlags]::None){throw 'Unexpected custody ACL.'}
    if($Directory -and $rule.InheritanceFlags -ne ([Security.AccessControl.InheritanceFlags]::ContainerInherit -bor [Security.AccessControl.InheritanceFlags]::ObjectInherit)){throw 'Custody directory must protect child files.'}
    $seen[$rule.IdentityReference.Value]=$true
  }
  if(-not $seen.ContainsKey($Sid) -or -not $seen.ContainsKey('S-1-5-18')){throw 'Required private ACL principals missing.'}
}
function Enter-SignedStoreDirectory([string]$Directory,$Owner) {
  $pins=[SignedUpdateStoreNative]::PinDirectories($Directory)
  try {Assert-SignedStoreAcl $pins[-1] $Owner.sid $true;return ,$pins}
  catch {foreach($pin in $pins){$pin.Dispose()};throw}
}
function New-SignedStoreFile([string]$Path,[string]$Sid) {
  # Atomically create with the exact owner/protected DACL, including when an
  # elevated token would default its file owner to Administrators. Never reopen.
  $acl=[Security.AccessControl.FileSecurity]::new()
  $acl.SetOwner([Security.Principal.SecurityIdentifier]::new($Sid))
  $acl.SetAccessRuleProtection($true,$false)
  foreach($principal in @($Sid,'S-1-5-18')) {
    $acl.AddAccessRule([Security.AccessControl.FileSystemAccessRule]::new([Security.Principal.SecurityIdentifier]::new($principal),[Security.AccessControl.FileSystemRights]::FullControl,[Security.AccessControl.AccessControlType]::Allow))
  }
  return [SignedUpdateStoreNative]::CreateJournal($Path,$acl.GetSecurityDescriptorBinaryForm())
}
function Enter-SignedInstallationRoot([string]$Root) {
  return ,([SignedUpdateStoreNative]::PinDirectories($Root))
}
function Get-SignedStorePinnedDirectory([string]$Requested,$Pins) {
  # Requested is retained for diagnostics only; never reopen it after capture.
  return [SignedUpdateStoreNative]::PhysicalDirectory($Pins)
}
function Assert-SignedStoreChild($Stream,$Pins) {
  [SignedUpdateStoreNative]::CheckChild($Stream.SafeFileHandle,$Pins[-1])
}
function Open-SignedInstallationLock([string]$Root) {
  return [SignedUpdateStoreNative]::InstallationLock([IO.Path]::Combine($Root,'transaction.lock'))
}
function Assert-SignedInstallationHandle($Stream,[string]$Sid='') {
  [SignedUpdateStoreNative]::Check($Stream.SafeFileHandle,$false)
  if($Sid){Assert-SignedStoreAcl $Stream.SafeFileHandle $Sid $false}
}
function Open-SignedInstallationRecordFile([string]$Path) {
  return [SignedUpdateStoreNative]::ReadRecord($Path)
}
function Read-SignedInstallationRecord([string]$Path) {
  $stream=Open-SignedInstallationRecordFile $Path
  try {
    Assert-SignedInstallationHandle $stream
    if($stream.Length -lt 1 -or $stream.Length -gt 32768){throw 'Installed arming record exceeds bound.'}
    $bytes=[byte[]]::new([int]$stream.Length);$offset=0
    while($offset -lt $bytes.Length){$n=$stream.Read($bytes,$offset,$bytes.Length-$offset);if($n -le 0){throw 'Incomplete installed arming record.'};$offset+=$n}
    return ConvertFrom-SignedStoreJson $bytes
  } finally {$stream.Dispose()}
}
function Enter-SignedInstallationBarrier([string]$Root,$Binding,$Owner) {
  $pins=@();$lock=$null;$stream=$null
  try {
    $pins=Enter-SignedInstallationRoot $Root
    $physicalRoot=Get-SignedStorePinnedDirectory $Root $pins
    $lock=Open-SignedInstallationLock $physicalRoot
    $rootOwner=Read-SignedInstallationRecord ([IO.Path]::Combine($physicalRoot,'owner.json'))
    $state=Read-SignedInstallationRecord ([IO.Path]::Combine($physicalRoot,'state.json'))
    if($rootOwner.owner -cne 'OpenNavX.Alpha1.SideBySide.1' -or
       $state.owner -cne $rootOwner.owner -or ($state.schema -isnot [int] -and $state.schema -isnot [long]) -or $state.schema -ne 1 -or
       $state.current -cne $Binding.previous.generation){throw 'Signed arming requires the exact selected owned installation.'}
    $path=Join-Path $Root 'signed-update-armed.jsonl'
    # This root marker is a durable DENIAL, never an approval or resumable token.
    # CreateNew atomically refuses any abandoned marker, including empty bytes.
    $physicalPath=[IO.Path]::Combine($physicalRoot,'signed-update-armed.jsonl')
    $stream=New-SignedStoreFile $physicalPath $Owner.sid
    $barrier=@{root=$Root;path=$path;physicalRoot=$physicalRoot;physicalPath=$physicalPath;stream=$stream;pins=$pins;lock=$lock;owner=$Owner;length=0;digest=('0'*64)}
    $body=[pscustomobject][ordered]@{schema=1;owner='SKAGER.SignedUpdateBarrier.1';session=$Binding.session;bindingSha256=(Get-SignedUpdateReceiptHash $Binding);previousGeneration=$Binding.previous.generation;createdUnix=(Get-SignedStoreNow)}
    $bytes=[byte[]]((Get-SignedStoreBytes $body)+10)
    Write-SignedStoreFrame $barrier $bytes
    $barrier.length=$bytes.Length;$barrier.digest=Get-SignedStoreDigest $bytes
    Assert-SignedInstallationBarrier $barrier
    return $barrier
  } catch {
    $failure=$_
    # A failed flush can also make Dispose throw. Release every independent
    # handle anyway, and report the original arming failure to the caller.
    if($stream){try{$stream.Dispose()}catch{}}
    if($lock){try{$lock.Dispose()}catch{}}
    foreach($pin in $pins){try{$pin.Dispose()}catch{}}
    # Never erase a successfully created marker after partial/failed arming.
    throw $failure
  }
}
function Assert-SignedInstallationBarrier($Barrier) {
  foreach($pin in $Barrier.pins){[SignedUpdateStoreNative]::Check($pin,$true)}
  Assert-SignedStoreChild $Barrier.stream $Barrier.pins
  Assert-SignedInstallationHandle $Barrier.stream $Barrier.owner.sid
  $bytes=Read-SignedStoreHeldBytes $Barrier
  if($bytes.Length -ne $Barrier.length -or (Get-SignedStoreDigest $bytes) -cne $Barrier.digest){throw 'Installed signed-update barrier changed.'}
}
function Close-SignedInstallationBarrier($Barrier) {
  try {$Barrier.stream.Dispose()} finally {
    try {if($Barrier.lock){$Barrier.lock.Dispose()}} finally {foreach($pin in $Barrier.pins){$pin.Dispose()}}
  }
  # Deliberately no delete, disarm or disk-resume operation.
}
function Assert-SignedStoreNative($Store) {
  foreach($pin in $Store.pins){[SignedUpdateStoreNative]::Check($pin,$true)}
  [SignedUpdateStoreNative]::Check($Store.stream.SafeFileHandle,$false)
  Assert-SignedStoreChild $Store.stream $Store.pins
  Assert-SignedStoreAcl $Store.pins[-1] $Store.owner.sid $true
  Assert-SignedStoreAcl $Store.stream.SafeFileHandle $Store.owner.sid $false
  if($Store.ContainsKey('installationBarrier') -and $Store.installationBarrier){Assert-SignedInstallationBarrier $Store.installationBarrier}
}
function Write-SignedStoreFrame($Store,[byte[]]$Bytes) {
  # One append contains BOTH complete state and its current head; there is no
  # independently replaceable head. Any partial append/flush failure poisons the
  # live owner; restart cannot resume even an apparently complete last frame.
  $Store.stream.Position=$Store.stream.Length
  $Store.stream.Write($Bytes,0,$Bytes.Length)
  $Store.stream.Flush($true)
}
function Read-SignedStoreHeldBytes($Store) {
  if($Store.stream.Length -lt 1 -or $Store.stream.Length -gt 262144){throw 'Armed journal size differs.'}
  $data=[byte[]]::new([int]$Store.stream.Length);$Store.stream.Position=0;$offset=0
  while($offset -lt $data.Length){$n=$Store.stream.Read($data,$offset,$data.Length-$offset);if($n -le 0){throw 'Incomplete armed journal.'};$offset+=$n}
  return ,$data
}
function Assert-SignedStoreCurrent($Store) {
  if($Store.poisoned -or $Store.stream.SafeFileHandle.IsClosed){throw 'Armed custody is abandoned; no resume or ordinary-launch authority.'}
  try {
    if((Get-SignedUpdateReceiptHash (Get-SignedStoreOwner)) -cne $Store.ownerSha256){throw 'Owner SID/session/process/image changed.'}
    Assert-SignedStoreNative $Store
    $bytes=Read-SignedStoreHeldBytes $Store
    if($bytes.Length -ne $Store.length -or (Get-SignedStoreDigest $bytes) -cne $Store.digest){throw 'Armed journal/head changed or replayed.'}
    $now=Get-SignedStoreNow
    $null=New-SignedUpdateSession $Store.session.binding $now
    if($now -lt $Store.session.lastUnix -or ($null -ne $Store.operations -and $now -lt $Store.operations.lastUnix)){throw 'Clock moved backward.'}
    return $now
  } catch {$Store.poisoned=$true;throw}
}
function Get-SignedStoreLive([string]$Handle) {
  if(-not $script:SignedUpdateStores.ContainsKey($Handle)){throw 'No exclusive live signed-update custody. Disk records cannot resume it.'}
  return $script:SignedUpdateStores[$Handle]
}
function New-SignedUpdateStore([string]$Directory,[byte[]]$BindingJson,[switch]$OperationEvents,[string]$InstallationRoot='') {
  $binding=ConvertFrom-SignedStoreJson $BindingJson
  $session=New-SignedUpdateSession $binding (Get-SignedStoreNow)
  $operations=$null
  if($OperationEvents){$operations=New-SignedUpdateOperationSession $binding $session.lastUnix;$session=$operations.session}
  $owner=Get-SignedStoreOwner
  $pins=@();$stream=$null;$barrier=$null
  try {
    # The private journal's strict directory ACL is unchanged. Installed arming
    # additionally publishes a protected root barrier while holding maintenance's
    # transaction lock, BEFORE any journal mutation, restoration or installation.
    if($InstallationRoot){$barrier=Enter-SignedInstallationBarrier $InstallationRoot $binding $owner}
    $pins=Enter-SignedStoreDirectory $Directory $owner
    $path=Join-Path $Directory 'signed-update-armed.jsonl'
    $physicalDirectory=Get-SignedStorePinnedDirectory $Directory $pins
    $physicalPath=[IO.Path]::Combine($physicalDirectory,'signed-update-armed.jsonl')
    # CreateNew refuses missing-custodian leftovers, including corrupt/empty files.
    $stream=New-SignedStoreFile $physicalPath $owner.sid
    $store=@{directory=$Directory;path=$path;physicalDirectory=$physicalDirectory;physicalPath=$physicalPath;stream=$stream;pins=$pins;installationBarrier=$barrier;owner=$owner;ownerSha256=(Get-SignedUpdateReceiptHash $owner);session=$session;operations=$operations;poisoned=$false;length=0;digest=('0'*64)}
    Assert-SignedStoreNative $store
    $frame=[pscustomobject][ordered]@{schema=1;owner=$owner;sequence=0;previousFrameSha256=('0'*64);state=$session}
    if($OperationEvents){$frame.schema=2;$frame.state=$operations}
    $bytes=Get-SignedStoreBytes $frame;$bytes=[byte[]]($bytes+10)
    Write-SignedStoreFrame $store $bytes
    $store.length=$bytes.Length;$store.digest=Get-SignedStoreDigest $bytes
    $null=Assert-SignedStoreCurrent $store
    $handle=[guid]::NewGuid().ToString('N')
    # Installed entry points now see the durable marker and deny. The full future
    # broker must supply its separately authenticated operation path; this API
    # supplies no exception to ordinary maintenance or application startup.
    if($barrier -and $barrier.lock){$barrier.lock.Dispose();$barrier.lock=$null}
    $script:SignedUpdateStores.Add($handle,$store)
    return $handle
  } catch {
    $failure=$_
    if($stream){try{$stream.Dispose()}catch{}}
    foreach($pin in $pins){try{$pin.Dispose()}catch{}}
    if($barrier){try{Close-SignedInstallationBarrier $barrier}catch{}}
    # Preserve any created marker, even on initial write failure.
    throw $failure
  }
}
function Add-SignedUpdateStoreReceipt([string]$Handle,[byte[]]$RequestJson) {
  $store=Get-SignedStoreLive $Handle
  $now=Assert-SignedStoreCurrent $store
  if($null -ne $store.operations){throw 'Operation-event custody requires Add-SignedUpdateStoreEvent; receipt-only bypass refused.'}
  $request=ConvertFrom-SignedStoreJson $RequestJson
  $next=Copy-SignedUpdateValue $store.session
  $receipt=Add-SignedUpdateReceipt $next $request $now
  Write-SignedStoreTransition $store $next $null
  return Copy-SignedUpdateValue $receipt
}
function Add-SignedUpdateStoreEvent([string]$Handle,[byte[]]$RequestJson) {
  $store=Get-SignedStoreLive $Handle
  $now=Assert-SignedStoreCurrent $store
  if($null -eq $store.operations){throw 'Operation events require an explicitly armed operation store.'}
  $request=ConvertFrom-SignedStoreJson $RequestJson
  $next=Copy-SignedUpdateValue $store.operations
  $event=Add-SignedUpdateOperationEvent $next $request $now
  Write-SignedStoreTransition $store $next.session $next
  return Copy-SignedUpdateValue $event
}
function Write-SignedStoreTransition($Store,$NextSession,$Operations) {
  $frame=[pscustomobject][ordered]@{schema=1;owner=$Store.owner;sequence=@($NextSession.receipts).Count;previousFrameSha256=$Store.digest;state=$NextSession}
  if($null -ne $Operations){$frame.schema=2;$frame.sequence=@($Operations.events).Count;$frame.state=$Operations}
  $bytes=Get-SignedStoreBytes $frame;$bytes=[byte[]]($bytes+10)
  try {
    # Retain the prior complete journal in memory as the anti-replay anchor.
    $old=Read-SignedStoreHeldBytes $store
    $expected=[byte[]]($old+$bytes)
    if($expected.Length -gt 262144){throw 'Journal bound exceeded.'}
    Write-SignedStoreFrame $store $bytes
    $store.length=$expected.Length;$store.digest=Get-SignedStoreDigest $expected;$store.session=$NextSession;$store.operations=$Operations
    $null=Assert-SignedStoreCurrent $store
  } catch {$store.poisoned=$true;throw}
  # Transition is consumed durably before either API can return a result.
}
function Close-SignedUpdateStore([string]$Handle) {
  $store=Get-SignedStoreLive $Handle
  $store.poisoned=$true
  try {$store.stream.Dispose()}finally{
    try {foreach($pin in $store.pins){$pin.Dispose()}} finally {
      try {if($store.installationBarrier){Close-SignedInstallationBarrier $store.installationBarrier}} finally {$script:SignedUpdateStores.Remove($Handle)}
    }
  }
  # Intentionally leave armed evidence. No disarm/delete/resume API exists.
}
