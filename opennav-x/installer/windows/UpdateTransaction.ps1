# Dot-source only. Windows PowerShell 5.1 compatible. No launch, profile, shell,
# actuator, network, or installation mutation. The caller owns Lifecycle's lock,
# verifies both generation manifests, persists the attempt BEFORE launch, and
# closes the candidate before applying a returned fallback decision.
Set-StrictMode -Version Latest

function Assert-UpdateIdentity($Identity) {
  foreach ($field in @('generation','commit','packageSha256','executableSha256')) {
    if (-not $Identity -or -not $Identity.PSObject.Properties[$field] -or $Identity.$field -isnot [string]) { throw 'Incomplete update generation identity.' }
  }
  if ($Identity.generation -cnotmatch '^[a-f0-9]{32}$' -or $Identity.commit -cnotmatch '^[a-f0-9]{40}$' -or
      $Identity.packageSha256 -cnotmatch '^[a-f0-9]{64}$' -or $Identity.executableSha256 -cnotmatch '^[a-f0-9]{64}$') { throw 'Invalid update generation identity.' }
}
function Assert-UpdatePendingRecord($Record) {
  if (-not $Record -or ($Record.schema -isnot [int] -and $Record.schema -isnot [long]) -or $Record.schema -ne 1 -or $Record.owner -cne 'SKAGER.UpdateStartup.1' -or
      $Record.transaction -isnot [string] -or $Record.transaction -cnotmatch '^[a-f0-9]{32}$' -or
      ($Record.attempts -isnot [int] -and $Record.attempts -isnot [long]) -or $Record.attempts -notin @(0,1) -or $Record.session -isnot [string] -or
      $Record.session -cnotmatch '^(|[a-f0-9]{32})$' -or
      (($Record.attempts -eq 0) -ne ($Record.session -ceq ''))) { throw 'Unknown or invalid pending update record.' }
  # Distinct native-v1 fields never reinterpret historical UTC tick strings.
  # Legacy records remain readable; an attempted legacy launch cannot grant the
  # new cleanup predicate and requires explicit recovery attention.
  if ($Record.PSObject.Properties['nativeLaunchV1']) {
    if ($Record.nativeLaunchV1 -isnot [string] -or $Record.nativeLaunchV1 -cnotmatch '^(none|armed|running)$' -or
        -not $Record.PSObject.Properties['processCreationFileTimeV1'] -or $Record.processCreationFileTimeV1 -isnot [string] -or
        $Record.processCreationFileTimeV1 -cnotmatch '^(|[1-9][0-9]{0,17})$' -or
        (($Record.nativeLaunchV1 -ceq 'running') -ne ($Record.processCreationFileTimeV1 -cne '')) -or
        ($Record.nativeLaunchV1 -ceq 'none' -and $Record.attempts -ne 0) -or
        ($Record.nativeLaunchV1 -cne 'none' -and $Record.attempts -ne 1) -or
        ($Record.nativeLaunchV1 -ceq 'running' -and (-not $Record.PSObject.Properties['processId'] -or ($Record.processId -isnot [int] -and $Record.processId -isnot [long]) -or $Record.processId -lt 1 -or $Record.processId -gt [int]::MaxValue))) { throw 'Invalid native launch recovery identity.' }
  } elseif ($Record.PSObject.Properties['processCreationFileTimeV1']) { throw 'Unversioned native launch recovery identity.' }
  Assert-UpdateIdentity $Record.candidate
  Assert-UpdateIdentity $Record.previous
  if ($Record.candidate.generation -ceq $Record.previous.generation) { throw 'Candidate and recovery generation must differ.' }
}
function New-UpdatePendingRecord($Candidate, $Previous) {
  Assert-UpdateIdentity $Candidate
  Assert-UpdateIdentity $Previous
  $record = [pscustomobject]@{schema=1; owner='SKAGER.UpdateStartup.1'; transaction=[guid]::NewGuid().ToString('N');
    candidate=$Candidate; previous=$Previous; attempts=0; session=''; processId=0; processStartTicks=''; nativeLaunchV1='none'; processCreationFileTimeV1=''}
  Assert-UpdatePendingRecord $record
  return $record
}
# Presence is a restriction only: no journal bytes or receipt grant permission.
function Assert-NoSignedUpdateArmed([string]$InstallationRoot, [bool]$AllowMissingRoot=$false) {
  try {
    Initialize-UpdatePipeType
    [Skager.UpdateArmedBarrier]::AssertAbsent($InstallationRoot,$AllowMissingRoot)
  } catch {
    $_.Exception.Data['SkagerSignedUpdateRestriction']=$true
    throw
  }
}
function Assert-UpdateRecordPath([string]$Path) {
  if (-not [IO.Path]::IsPathRooted($Path) -or $Path -match '[\x00-\x1f]' -or $Path.StartsWith('\\')) { throw 'Update record requires an absolute local path.' }
  if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT -and $Path -notmatch '^[A-Za-z]:[\\/]') { throw 'Update record requires an absolute local drive path.' }
  $full = [IO.Path]::GetFullPath($Path)
  $walk = $full
  while ($walk) {
    if (Test-Path -LiteralPath $walk) {
      if ((Get-Item -LiteralPath $walk -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw 'Redirected update record path refused.' }
    }
    $walk = [IO.Path]::GetDirectoryName($walk)
  }
  return $full
}
function Write-UpdatePendingRecord([string]$Path, $Record) {
  Assert-UpdatePendingRecord $Record
  $Path = Assert-UpdateRecordPath $Path
  $temporary = $Path + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
  $encoding = New-Object Text.UTF8Encoding($false)
  $bytes = $encoding.GetBytes(($Record | ConvertTo-Json -Depth 8))
  try {
    $file = New-Object IO.FileStream($temporary, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
    try { $file.Write($bytes,0,$bytes.Length); $file.Flush($true) } finally { $file.Dispose() }
    if ([IO.File]::Exists($Path)) { [IO.File]::Replace($temporary,$Path,[Management.Automation.Language.NullString]::Value) }
    else { [IO.File]::Move($temporary,$Path) }
  } finally { if ([IO.File]::Exists($temporary)) { [IO.File]::Delete($temporary) } }
}
function Read-UpdatePendingRecord([string]$Path) {
  $Path = Assert-UpdateRecordPath $Path
  if (-not [IO.File]::Exists($Path)) { return $null }
  if ((Get-Item -LiteralPath $Path).Length -notin 1..4096) { throw 'Invalid pending update record size.' }
  $record = [IO.File]::ReadAllText($Path) | ConvertFrom-Json
  Assert-UpdatePendingRecord $record
  return $record
}
function Resolve-UpdatePendingRecovery($Record, [string]$CurrentGeneration) {
  Assert-UpdatePendingRecord $Record
  # Absence of a live authenticated startup receipt never implies success.
  # A crash before/after publication or before/after launch has one safe result.
  if ($CurrentGeneration -ceq $Record.previous.generation) { return 'retain-previous' }
  if ($CurrentGeneration -ceq $Record.candidate.generation) { return 'restore-previous' }
  return 'manual-recovery'
}
function Initialize-UpdatePipeType {
  if ('Skager.UpdateStartupPipe' -as [type]) { return }
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Authenticated startup receipt requires native Windows.' }
  $source = @'
using System;
using System.IO;
using System.IO.Pipes;
using System.Diagnostics;
using System.Security.Cryptography;
using System.Security.Principal;
using System.Runtime.InteropServices;
using System.Text;
using Microsoft.Win32.SafeHandles;
namespace Skager {
 public static class UpdateArmedBarrier {
  const uint Invalid=0xffffffff, Directory=0x10, Reparse=0x400;
  [StructLayout(LayoutKind.Sequential,Pack=4)] struct Info {
   public uint Attributes; public long Created,Accessed,Written;
   public uint Volume,SizeHigh,SizeLow,Links,IndexHigh,IndexLow;
  }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true,ExactSpelling=true)]
  static extern uint GetFileAttributesW(string path);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern SafeFileHandle CreateFile(string path,uint access,uint share,IntPtr security,uint disposition,uint flags,IntPtr template);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetFileInformationByHandle(SafeFileHandle handle,out Info info);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern uint GetFinalPathNameByHandle(SafeFileHandle handle,StringBuilder name,uint length,uint flags);
  static uint Attributes(string path) {
   uint attributes=GetFileAttributesW(path);
   if(attributes==Invalid) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
   return attributes;
  }
  // FILE_LIST_DIRECTORY makes no-share-delete enforceable; metadata access alone does not.
  static SafeFileHandle OpenDirectory(string path) {
   var handle=CreateFile(path,0x81,1,IntPtr.Zero,3,0x02200000,IntPtr.Zero);
   if(handle.IsInvalid) { int error=Marshal.GetLastWin32Error(); handle.Dispose(); throw new System.ComponentModel.Win32Exception(error); }
   return handle;
  }
  public static void AssertAbsent(string root,bool allowMissingRoot) {
   if(Environment.OSVersion.Platform!=PlatformID.Win32NT) throw new InvalidOperationException("Armed-session lookup requires native Windows.");
   var pins=new System.Collections.Generic.List<SafeFileHandle>();
   string nativeDrive=null;
   Func<string,string> physical=delegate(string path) { return nativeDrive==null?path:@"\\?\GLOBALROOT"+nativeDrive+"\\"+path.Substring(3); };
   try {
    CheckAbsent(root,allowMissingRoot,delegate(string path) { return Attributes(physical(path)); },delegate(string path) {
     // Hold plain ancestors against write/delete replacement during lookup.
     uint attributes=Attributes(physical(path));
     if((attributes&(Directory|Reparse))!=Directory) return attributes;
     var handle=OpenDirectory(physical(path));
     pins.Add(handle); Info info;
     if(!GetFileInformationByHandle(handle,out info)) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
     if(nativeDrive==null) {
      var name=new StringBuilder(32768); uint length=GetFinalPathNameByHandle(handle,name,(uint)name.Capacity,2);
      if(length==0 || length>=(uint)name.Capacity) throw new System.ComponentModel.Win32Exception(Marshal.GetLastWin32Error());
      nativeDrive=name.ToString().TrimEnd('\\');
      if(!System.Text.RegularExpressions.Regex.IsMatch(nativeDrive,@"^\\Device\\HarddiskVolume[0-9]+(?:\\|$)")) throw new IOException("Armed-session root is not a local physical volume.");
     }
     return info.Attributes;
    });
   } finally { foreach(var pin in pins) pin.Dispose(); }
  }
  // The injected attribute reader is private and used only by inert policy tests.
  // Only FILE_NOT_FOUND is absence; PATH_NOT_FOUND, ACL and all other failures deny.
  static void CheckAbsent(string root,bool allowMissingRoot,Func<string,uint> read,Func<string,uint> directory) {
   if(String.IsNullOrEmpty(root) || !System.Text.RegularExpressions.Regex.IsMatch(root,@"^[A-Za-z]:[\\/]") || root.Length<4)
    throw new InvalidOperationException("Armed-session installation root is ambiguous.");
   root=root.Replace('/','\\');
   if(root.EndsWith("\\")) root=root.Substring(0,root.Length-1);
   string[] parts=root.Substring(3).Split('\\');
   foreach(string part in parts) {
    if(part.Length==0 || part=="." || part==".." || part.EndsWith(".") || part.EndsWith(" ") ||
       part.IndexOfAny(new char[]{':','*','?','"','<','>','|'})>=0 ||
       System.Text.RegularExpressions.Regex.IsMatch(part,@"[\x00-\x1f]|^(?i:CON|PRN|AUX|NUL|COM[0-9]|LPT[0-9])(?:\.|$)"))
     throw new InvalidOperationException("Armed-session installation root is ambiguous.");
   }
   string walk=root.Substring(0,3);
   uint attributes=directory(walk);
   if((attributes&(Directory|Reparse))!=Directory) throw new InvalidOperationException("Armed-session ancestor is not a plain directory.");
   foreach(string part in parts) {
    walk=walk.TrimEnd('\\')+"\\"+part;
    try { attributes=directory(walk); }
    catch(System.ComponentModel.Win32Exception error) {
     // Fresh installation only: a checked existing ancestor plus an exact missing
     // component proves this not-yet-created root absent. Recheck after creation.
     if(allowMissingRoot && error.NativeErrorCode==2) return;
     throw;
    }
    if((attributes&(Directory|Reparse))!=Directory) throw new InvalidOperationException("Armed-session ancestor is not a plain directory.");
   }
   try { read(root+"\\signed-update-armed.jsonl"); }
   catch(System.ComponentModel.Win32Exception error) { if(error.NativeErrorCode==2) return; throw; }
   throw new InvalidOperationException("Signed update session is armed; ordinary startup and maintenance are restricted. No automatic recovery or disarm is available.");
  }
 }
 public sealed class UpdateImageOwner : IDisposable {
  const uint Read=0x80000000, Attributes=0x80, ListDirectory=0x1, Directory=0x10, Reparse=0x400;
  const uint OpenReparse=0x00200000, Backup=0x02000000;
  sealed class Pin {
   public SafeFileHandle Handle; public string Native; public Info Identity; public bool IsDirectory;
  }
  [StructLayout(LayoutKind.Sequential,Pack=4)] struct Info {
   public uint Attributes; public long Created, Accessed, Written;
   public uint Volume, SizeHigh, SizeLow, Links, IndexHigh, IndexLow;
  }
  [StructLayout(LayoutKind.Sequential,CharSet=CharSet.Unicode)] struct Startup {
   public uint Size; public string Reserved, Desktop, Title;
   public uint X,Y,XSize,YSize,XChars,YChars,Fill,Flags;
   public ushort Show,ReservedSize; public IntPtr ReservedBytes,Input,Output,Error;
  }
  [StructLayout(LayoutKind.Sequential)] struct Child { public IntPtr Process,Thread; public uint Pid,Tid; }
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern SafeFileHandle CreateFile(string name,uint access,uint share,IntPtr security,uint disposition,uint flags,IntPtr template);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetFileInformationByHandle(SafeFileHandle handle,out Info info);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern uint GetFinalPathNameByHandle(SafeFileHandle handle,StringBuilder path,uint length,uint flags);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern bool GetVolumeInformationByHandleW(SafeFileHandle handle,StringBuilder label,uint labelSize,out uint serial,out uint maximum,out uint flags,StringBuilder fs,uint fsSize);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern bool CreateProcessW(string application,StringBuilder command,IntPtr processSecurity,IntPtr threadSecurity,bool inherit,uint flags,IntPtr environment,string directory,ref Startup startup,out Child child);
  [DllImport("kernel32.dll",CharSet=CharSet.Unicode,SetLastError=true)]
  static extern bool QueryFullProcessImageNameW(IntPtr process,uint flags,StringBuilder path,ref uint length);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool GetProcessTimes(IntPtr process,out long created,out long exited,out long kernel,out long user);
  [DllImport("kernel32.dll",SetLastError=true)] static extern uint ResumeThread(IntPtr thread);
  [DllImport("kernel32.dll",SetLastError=true)] static extern bool TerminateProcess(IntPtr process,uint code);
  [DllImport("kernel32.dll",SetLastError=true)] static extern uint WaitForSingleObject(IntPtr handle,uint timeout);
  [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr handle);
  readonly System.Collections.Generic.List<Pin> pins=new System.Collections.Generic.List<Pin>();
  FileStream image; IntPtr child=IntPtr.Zero; bool disposed, mayHaveRun;
  public string OriginalImage { get; private set; }
  public string NativeImage { get; private set; }
  public string ApplicationImage { get; private set; }
  public string Hash { get; private set; }
  public int ProcessId { get; private set; }
  public long CreationFileTime { get; private set; }
  public bool CreationAttempted { get; private set; }
  public bool StartSucceeded { get; private set; }
  string directory;
  static IOException Failure(string operation) { return new IOException(operation+" refused (Win32 "+Marshal.GetLastWin32Error()+")."); }
  static bool Equal(string a,string b) { return String.Equals(a,b,StringComparison.OrdinalIgnoreCase); }
  static bool EqualNative(string a,string b) { return String.Equals(a,b,StringComparison.Ordinal); }
  static string Native(SafeFileHandle handle) {
   var value=new StringBuilder(32768); uint count=GetFinalPathNameByHandle(handle,value,(uint)value.Capacity,2);
   if(count==0 || count>=(uint)value.Capacity) throw Failure("Native file identity");
   return value.ToString().TrimEnd('\\');
  }
  // The tested native host rejects GLOBALROOT as the application spelling.
  // Select exactly one conventional application name from the held image; the
  // DOS namespace is rechecked, not pinned. Raw child identity gates resume.
  static string ParseApplication(string finalName,string native) {
   var volume=System.Text.RegularExpressions.Regex.Match(native??"",@"^\\Device\\HarddiskVolume[0-9]+(?=\\|$)");
   if(finalName==null || !finalName.StartsWith(@"\\?\",StringComparison.Ordinal) || !volume.Success) throw new IOException("Held DOS application name unavailable.");
   string application=finalName.Substring(4);
   if(!System.Text.RegularExpressions.Regex.IsMatch(application,@"^[A-Za-z]:\\") || application.Length<4 ||
      !EqualNative(application.Substring(2),native.Substring(volume.Length))) throw new IOException("Held DOS application does not match its physical volume suffix.");
   foreach(string part in application.Substring(3).Split('\\')) {
    if(part.Length==0 || part=="." || part==".." || part.EndsWith(".",StringComparison.Ordinal) || part.EndsWith(" ",StringComparison.Ordinal) ||
       System.Text.RegularExpressions.Regex.IsMatch(part,"[\\x00-\\x1f<>:\"/\\\\|?*]") ||
       System.Text.RegularExpressions.Regex.IsMatch(part,@"^(CON|PRN|AUX|NUL|COM[1-9\u00b9\u00b2\u00b3]|LPT[1-9\u00b9\u00b2\u00b3])(?:\.|$)",System.Text.RegularExpressions.RegexOptions.IgnoreCase)) throw new IOException("Ambiguous held DOS application name.");
   }
   return application;
  }
  static string Application(SafeFileHandle handle,string native) {
   var value=new StringBuilder(32768); uint count=GetFinalPathNameByHandle(handle,value,(uint)value.Capacity,0);
   if(count==0 || count>=(uint)value.Capacity) throw Failure("Held DOS application identity");
   return ParseApplication(value.ToString(),native);
  }
  void ValidateApplication(string path,Pin pin) {
   using(var handle=CreateFile(path,pin.IsDirectory?Attributes:Read,pin.IsDirectory?3u:1u,IntPtr.Zero,3,OpenReparse|(pin.IsDirectory?Backup:0u),IntPtr.Zero)) {
    if(handle.IsInvalid) throw Failure("Selected DOS application resolution");
    var current=Metadata(handle);
    if(!Same(current,pin.Identity) || (current.Attributes&Reparse)!=0 || ((current.Attributes&Directory)!=0)!=pin.IsDirectory || !EqualNative(Native(handle),pin.Native) ||
       (!pin.IsDirectory && (current.Links!=1 || current.SizeHigh!=pin.Identity.SizeHigh || current.SizeLow!=pin.Identity.SizeLow))) throw new IOException("Selected DOS application resolution changed.");
   }
  }
  static string NativeProcess(IntPtr handle) {
   var value=new StringBuilder(32768); uint count=(uint)value.Capacity;
   if(!QueryFullProcessImageNameW(handle,1,value,ref count) || count==0) throw Failure("Native process identity");
   return value.ToString();
  }
  static long Created(IntPtr handle) {
   long created,exited,kernel,user;
   if(!GetProcessTimes(handle,out created,out exited,out kernel,out user) || created<=0) throw Failure("Process creation identity");
   return created;
  }
  static Info Metadata(SafeFileHandle handle) {
   Info value; if(!GetFileInformationByHandle(handle,out value)) throw Failure("Held file metadata"); return value;
  }
  static bool Same(Info a,Info b) { return a.Volume==b.Volume && a.IndexHigh==b.IndexHigh && a.IndexLow==b.IndexLow; }
  static string Physical(string path) { return @"\\?\GLOBALROOT"+path+(System.Text.RegularExpressions.Regex.IsMatch(path,@"^\\Device\\HarddiskVolume[0-9]+$")?"\\":""); }
  Pin Open(string path,bool isDirectory) {
   var handle=CreateFile(Physical(path),isDirectory?(Attributes|ListDirectory):Read,isDirectory?3u:1u,IntPtr.Zero,3,OpenReparse|(isDirectory?Backup:0u),IntPtr.Zero);
   if(handle.IsInvalid) { handle.Dispose(); throw Failure("Image ancestry acquisition"); }
   try {
    var info=Metadata(handle);
    if((info.Attributes&Reparse)!=0 || ((info.Attributes&Directory)!=0)!=isDirectory || (!isDirectory && info.Links!=1) || !Equal(Native(handle),path)) throw new IOException("Image ancestry is redirected or ambiguous.");
    var pin=new Pin { Handle=handle,Native=Native(handle),Identity=info,IsDirectory=isDirectory };pins.Add(pin);return pin;
   } catch { handle.Dispose();throw; }
  }
  UpdateImageOwner() { }
  public static UpdateImageOwner Acquire(string executable,string workingDirectory,string expectedHash) {
   if(Environment.OSVersion.Platform!=PlatformID.Win32NT) throw new PlatformNotSupportedException("Native image custody requires Windows.");
   if(expectedHash==null || !System.Text.RegularExpressions.Regex.IsMatch(expectedHash,"^[a-f0-9]{64}$")) throw new ArgumentException("Invalid image hash.");
   string full=Path.GetFullPath(executable), parent=Path.GetFullPath(workingDirectory).TrimEnd('\\');
   if(!System.Text.RegularExpressions.Regex.IsMatch(full,@"^[A-Za-z]:\\") || full.IndexOf(':',2)>=0 || full.IndexOf('/')>=0 || !Equal(Path.GetDirectoryName(full),parent)) throw new ArgumentException("Image requires its absolute local parent directory.");
   var owner=new UpdateImageOwner();
   try {
    // Resolve the input drive once and pin every physical ancestor, including
    // those above a SUBST drive root. Application spelling is selected separately.
    using(var root=CreateFile(full.Substring(0,3),Attributes,3,IntPtr.Zero,3,Backup|OpenReparse,IntPtr.Zero)) {
     if(root.IsInvalid) throw Failure("Input drive root");
     var rootInfo=Metadata(root);string rootNative=Native(root);
     if((rootInfo.Attributes&(Directory|Reparse))!=Directory) throw new IOException("Input drive root is redirected.");
     var match=System.Text.RegularExpressions.Regex.Match(rootNative,@"^\\Device\\HarddiskVolume[0-9]+(?=\\|$)");
     if(!match.Success) throw new IOException("Only a local physical NTFS volume is supported.");
     string native=rootNative+"\\"+full.Substring(3);
     string nativeParent=native.Substring(0,native.LastIndexOf('\\'));
     string walk=match.Value;
     var volume=owner.Open(walk,true);
     uint serial,maximum,flags;var label=new StringBuilder(261);var fs=new StringBuilder(261);
     if(!GetVolumeInformationByHandleW(volume.Handle,label,261,out serial,out maximum,out flags,fs,261) || fs.ToString()!="NTFS") throw new IOException("Image custody requires native NTFS.");
     bool foundRoot=Equal(walk,rootNative) && Same(volume.Identity,rootInfo);
     foreach(string part in nativeParent.Substring(walk.Length).Split(new[]{'\\'},StringSplitOptions.RemoveEmptyEntries)) {
      if(part=="." || part==".." || part.EndsWith(".",StringComparison.Ordinal) || part.EndsWith(" ",StringComparison.Ordinal)) throw new IOException("Ambiguous image ancestry.");
      walk+="\\"+part;var pin=owner.Open(walk,true);walk=pin.Native;
      if(pin.Identity.Volume!=volume.Identity.Volume) throw new IOException("Image ancestry crossed volumes.");
      if(Equal(walk,rootNative) && Same(pin.Identity,rootInfo)) foundRoot=true;
     }
     if(!foundRoot) throw new IOException("Input root changed while acquiring physical ancestry.");
     var file=owner.Open(walk+"\\"+Path.GetFileName(full),false);
     if(file.Identity.Volume!=volume.Identity.Volume || file.Identity.SizeHigh!=0 || file.Identity.SizeLow==0 || file.Identity.SizeLow>536870912) throw new IOException("Unsupported image size or volume.");
     owner.OriginalImage=full;owner.NativeImage=file.Native;owner.directory=walk;
     owner.ApplicationImage=Application(file.Handle,file.Native);
     owner.image=new FileStream(file.Handle,FileAccess.Read,8192,false);
     using(var sha=SHA256.Create()) owner.Hash=BitConverter.ToString(sha.ComputeHash(owner.image)).Replace("-","").ToLowerInvariant();
     if(owner.Hash!=expectedHash) throw new IOException("Held executable hash changed before launch.");
     owner.Validate();return owner;
    }
   } catch { owner.Dispose();throw; }
  }
  public void Validate() {
   if(disposed) throw new ObjectDisposedException("UpdateImageOwner");
   foreach(var pin in pins) {
    var current=Metadata(pin.Handle);
    if(!Same(current,pin.Identity) || (current.Attributes&Reparse)!=0 || ((current.Attributes&Directory)!=0)!=pin.IsDirectory || !EqualNative(Native(pin.Handle),pin.Native) ||
       (!pin.IsDirectory && (current.Links!=1 || current.SizeHigh!=pin.Identity.SizeHigh || current.SizeLow!=pin.Identity.SizeLow))) throw new IOException("Held executable ancestry changed.");
   }
   // The selected DOS root must still be the same held physical NTFS root.
   ValidateApplication(ApplicationImage.Substring(0,3),pins[0]);
   ValidateApplication(ApplicationImage,pins[pins.Count-1]);
  }
  public Process Start(ProcessStartInfo start) {
   if(CreationAttempted || disposed) throw new InvalidOperationException("Image owner is one-use.");
   if(start.UseShellExecute || start.RedirectStandardInput || start.RedirectStandardOutput || start.RedirectStandardError || !String.IsNullOrEmpty(start.UserName) ||
      !Equal(Path.GetFullPath(start.FileName),OriginalImage) || !Equal(Path.GetFullPath(start.WorkingDirectory).TrimEnd('\\'),Path.GetDirectoryName(OriginalImage))) throw new ArgumentException("Unsupported native creation request.");
   Validate();
   var variables=new System.Collections.Generic.List<string>();
   foreach(System.Collections.DictionaryEntry item in start.EnvironmentVariables) {
    string key=(string)item.Key,value=(string)item.Value;
    if(key.IndexOf('\0')>=0 || key.IndexOf('=')>=0 || value==null || value.IndexOf('\0')>=0) throw new ArgumentException("Unsupported process environment.");
    variables.Add(key+"="+value);
   }
   variables.Sort(StringComparer.OrdinalIgnoreCase);
   IntPtr environment=Marshal.StringToHGlobalUni(String.Join("\0",variables.ToArray())+"\0\0");
   IntPtr thread=IntPtr.Zero;Process process=null;
   try {
    var info=new Startup { Size=(uint)Marshal.SizeOf(typeof(Startup)) };Child result;
    var command=new StringBuilder("\""+Physical(NativeImage)+"\""+(String.IsNullOrEmpty(start.Arguments)?"":" "+start.Arguments));
    // Refusal state is armed BEFORE the native call. An exception is never
    // treated as evidence that no child existed or as permission to rollback.
    CreationAttempted=true;
    if(!CreateProcessW(ApplicationImage,command,IntPtr.Zero,IntPtr.Zero,false,0x404u|(start.CreateNoWindow?0x08000000u:0u),environment,Physical(directory),ref info,out result)) throw Failure("Suspended image creation");
    child=result.Process;thread=result.Thread;ProcessId=checked((int)result.Pid);CreationFileTime=Created(child);
    Validate();
    if(!EqualNative(NativeProcess(child),NativeImage) || WaitForSingleObject(child,0)!=258) throw new IOException("Suspended child does not match held image.");
    process=Process.GetProcessById(ProcessId);
    if(Created(process.Handle)!=CreationFileTime || !EqualNative(NativeProcess(process.Handle),NativeImage)) throw new IOException("Suspended process handle identity changed.");
    // After this call starts, even a failing/ambiguous result may have run.
    // Never terminate that child as if it were still an inert suspended one.
    mayHaveRun=true;
    if(ResumeThread(thread)!=1) throw Failure("Primary-thread resume");
    StartSucceeded=true;return process;
   } catch {
    try {
     if(child!=IntPtr.Zero && !mayHaveRun) {
      // This owner created this still-suspended child. Only this inert case
      // permits termination; an uncertain result still denies all fallback.
      if(!TerminateProcess(child,126) || WaitForSingleObject(child,10000)!=0) throw new IOException("Suspended child cleanup uncertain; recovery requires attention.");
     }
    } finally { if(process!=null) process.Dispose(); }
    throw;
   } finally { if(thread!=IntPtr.Zero) CloseHandle(thread);Marshal.FreeHGlobal(environment); }
  }
  public bool Matches(Process process,string original,string expectedHash) {
   try {
    Validate();return StartSucceeded && child!=IntPtr.Zero && process.Id==ProcessId && Equal(original,OriginalImage) && expectedHash==Hash &&
      WaitForSingleObject(child,0)==258 && Created(child)==CreationFileTime && Created(process.Handle)==CreationFileTime && EqualNative(NativeProcess(child),NativeImage) && EqualNative(NativeProcess(process.Handle),NativeImage);
   } catch { return false; }
  }
  public void AssertOwnedImage(string original,string expectedHash) {
   Validate();
   if(!StartSucceeded || child==IntPtr.Zero || !Equal(Path.GetFullPath(original),OriginalImage) || expectedHash!=Hash ||
      WaitForSingleObject(child,0)!=258 || Created(child)!=CreationFileTime || !EqualNative(NativeProcess(child),NativeImage)) throw new IOException("No live owned image identity.");
  }
  public void Close(Process process,int timeout) {
   Validate();
   if(!StartSucceeded || child==IntPtr.Zero || process.Id!=ProcessId || Created(child)!=CreationFileTime) throw new IOException("No owned child to close.");
   if(WaitForSingleObject(child,0)==0) return; // Exit observed on our original kernel handle.
   if(!Matches(process,OriginalImage,Hash)) throw new IOException("Owned child identity changed; no process was signaled.");
   process.CloseMainWindow();
   if(WaitForSingleObject(child,(uint)timeout)!=0) throw new IOException("Owned child remains live; no forced termination or rollback.");
  }
  public static bool CloseInterrupted(Process process,string executable,string expectedHash,int recordedPid,long recordedCreation,int timeout) {
   if(process.Id!=recordedPid || recordedCreation<=0) return false;
   using(var owner=Acquire(executable,Path.GetDirectoryName(executable),expectedHash)) {
    // A fresh invocation cannot recreate historical ownership. These native
    // checks only restrict which recorded process may receive normal close.
    IntPtr handle=process.Handle;
    if(Created(handle)!=recordedCreation || WaitForSingleObject(handle,0)!=258 || !EqualNative(NativeProcess(handle),owner.NativeImage)) return false;
    owner.Validate();process.CloseMainWindow();
    if(WaitForSingleObject(handle,(uint)timeout)!=0) throw new IOException("Interrupted candidate remains live; recovery blocked.");
    return true;
   }
  }
  public void Dispose() {
   if(disposed) return;disposed=true;
   if(child!=IntPtr.Zero) { CloseHandle(child);child=IntPtr.Zero; }
   if(image!=null) image.Dispose();
   for(int i=pins.Count-1;i>=0;i--) pins[i].Handle.Dispose();pins.Clear();
  }
 }
 public sealed class UpdateStartupPipe : IDisposable {
  [DllImport("advapi32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern bool ConvertStringSecurityDescriptorToSecurityDescriptor(string s, uint r, out IntPtr p, out uint n);
  [DllImport("kernel32.dll")] static extern IntPtr LocalFree(IntPtr p);
  [StructLayout(LayoutKind.Sequential)] struct SA { public int length; public IntPtr descriptor; public int inherit; }
  [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
  static extern SafePipeHandle CreateNamedPipe(string name,uint open,uint mode,uint instances,uint output,uint input,uint timeout,ref SA security);
  [DllImport("kernel32.dll", SetLastError=true)]
  static extern bool GetNamedPipeClientProcessId(SafePipeHandle pipe,out uint pid);
  readonly NamedPipeServerStream pipe;
  readonly System.Threading.Tasks.Task connection;
  readonly DateTime created = DateTime.UtcNow;
  bool consumed;
  UpdateImageOwner imageOwner;
  public long OwnedCreationFileTime { get { if(imageOwner==null || !imageOwner.StartSucceeded) throw new InvalidOperationException("No owned launch.");return imageOwner.CreationFileTime; } }
  public bool NativeCreationAttempted { get { return imageOwner!=null && imageOwner.CreationAttempted; } }
  public bool UncertainCreation { get { return imageOwner!=null && imageOwner.CreationAttempted && !imageOwner.StartSucceeded; } }
  public Process StartOwned(ProcessStartInfo start,string expectedHash) {
   if(imageOwner!=null) throw new InvalidOperationException("Startup image already acquired.");
   imageOwner=UpdateImageOwner.Acquire(start.FileName,start.WorkingDirectory,expectedHash);
   return imageOwner.Start(start);
  }
  public void CloseOwned(Process process,int timeout) {
   if(imageOwner==null) throw new InvalidOperationException("No native image owner; no process was signaled.");
   imageOwner.Close(process,timeout);
  }
  public string VerifiedFrame { get; private set; }
  public string VerifiedImage { get; private set; }
  public string VerifiedHash { get; private set; }
  public string VerifiedNativeImage { get; private set; }
  public void AssertOwnedImage(string original,string expectedHash) {
   if(imageOwner==null) throw new IOException("No native creator ownership for receipt.");
   imageOwner.AssertOwnedImage(original,expectedHash);
  }
  public string FailureReason { get; private set; }
  public string Phase { get; private set; }
  bool Reject(string reason) { FailureReason=reason; return false; }
  public UpdateStartupPipe(string name) {
   Phase="startup";
   IntPtr descriptor = IntPtr.Zero; uint size;
   var sid = WindowsIdentity.GetCurrent().User.Value;
   if (!ConvertStringSecurityDescriptorToSecurityDescriptor("D:P(A;;GA;;;"+sid+")",1,out descriptor,out size)) throw new IOException("Cannot restrict startup pipe ACL.");
   try {
    var security = new SA { length=Marshal.SizeOf(typeof(SA)), descriptor=descriptor, inherit=0 };
    // Inbound, overlapped, first instance; byte mode, remote clients refused.
    // Buffer the entire bounded three-record burst while image/hash checks
    // run; protocol parsing still limits each record to 256 bytes, three total.
    var handle=CreateNamedPipe(@"\\.\pipe\"+name,0x40080001,8,1,256,1024,0,ref security);
    if (handle.IsInvalid) { handle.Dispose(); throw new IOException("Cannot create exclusive local startup pipe."); }
    try {
     pipe=new NamedPipeServerStream(PipeDirection.In,true,false,handle);
     // Arm ConnectNamedPipe before the caller can launch its child. A fast
     // client may write and close while the caller durably records its PID;
     // starting the connect only in Receive then loses that valid connection.
     connection=pipe.WaitForConnectionAsync();
    }
    catch { handle.Dispose(); throw; }
   } finally { LocalFree(descriptor); }
  }
  static string Hash(string path) {
   using(var file=File.OpenRead(path)) using(var sha=SHA256.Create()) return BitConverter.ToString(sha.ComputeHash(file)).Replace("-","").ToLowerInvariant();
  }
  bool SameLiveProcess(Process process,long ticks) {
   if(imageOwner!=null) return imageOwner.Matches(process,imageOwner.OriginalImage,imageOwner.Hash);
   using(var fresh=Process.GetProcessById(process.Id)) {
    return !process.HasExited && !fresh.HasExited && fresh.StartTime.ToUniversalTime().Ticks==ticks;
   }
  }
  bool ExactProcess(Process process,long ticks,string image,string hash) {
   if(imageOwner!=null) return imageOwner.Matches(process,Path.GetFullPath(image),hash);
   try {
    using(var fresh=Process.GetProcessById(process.Id)) {
     return !process.HasExited && !fresh.HasExited && fresh.StartTime.ToUniversalTime().Ticks==ticks &&
      String.Equals(Path.GetFullPath(fresh.MainModule.FileName),Path.GetFullPath(image),StringComparison.OrdinalIgnoreCase) && Hash(image)==hash;
    }
   } catch { return false; }
  }
  public bool Receive(Process process,string image,string hash,string expected,int timeoutMs) {
   return Receive(process,image,hash,expected,timeoutMs,300000);
  }
  public bool Receive(Process process,string image,string hash,string expected,int timeoutMs,int humanWaitMs) {
   if(consumed) throw new InvalidOperationException("Startup receipt already consumed.");
   consumed=true;
   if(timeoutMs<1 || timeoutMs>180000 || humanWaitMs<1 || humanWaitMs>300000 || expected.Length>256 ||
      !expected.StartsWith("SKAGER-UPDATE-READY/1 ",StringComparison.Ordinal) || !expected.EndsWith("\n",StringComparison.Ordinal))
    throw new ArgumentException("Invalid startup receipt bound.");
   var elapsed=Stopwatch.StartNew();
   long deadline=timeoutMs;
   string stage="process-before-connect";
   try {
    long ticks=imageOwner==null ? process.StartTime.ToUniversalTime().Ticks : DateTime.FromFileTimeUtc(imageOwner.CreationFileTime).Ticks;
    // The loader need not have initialized MainModule immediately after
    // CreateProcess. Bind live PID/start time now; require full image/hash
    // identity after its pipe connection, before reading any receipt bytes.
    if(ticks<created.Ticks || !SameLiveProcess(process,ticks)) return Reject(stage);
    stage="connect";
    if(!connection.Wait(timeoutMs)) return Reject("connect-timeout");
    uint client;
    stage="client-identity";
    if(!GetNamedPipeClientProcessId(pipe.SafePipeHandle,out client)) return Reject("client-pid-win32-"+Marshal.GetLastWin32Error());
    if(client!=(uint)process.Id || !ExactProcess(process,ticks,image,hash)) return Reject(stage);
    var waitFrame=expected.Replace("SKAGER-UPDATE-READY/1 ","SKAGER-UPDATE-WAIT/1 ");
    var continueFrame=expected.Replace("SKAGER-UPDATE-READY/1 ","SKAGER-UPDATE-CONTINUE/1 ");
    var cancelFrame=expected.Replace("SKAGER-UPDATE-READY/1 ","SKAGER-UPDATE-CANCEL/1 ");
    var record=new byte[256]; int used=0, records=0;
    var buffer=new byte[257];
    string terminal=null;
    stage="frame-read";
    while(true) {
     var read=pipe.ReadAsync(buffer,0,buffer.Length);
     while(true) {
      long remaining=deadline-elapsed.ElapsedMilliseconds;
      if(remaining<=0) return Reject(Phase=="awaiting-human" ? "human-wait-timeout" : Phase=="resuming" ? "health-timeout" : "read-timeout");
      if(read.Wait((int)Math.Min(remaining,250))) break;
      if(!SameLiveProcess(process,ticks)) return Reject("process-during-read");
     }
     int count=read.Result;
     if(elapsed.ElapsedMilliseconds>=deadline) return Reject(Phase=="awaiting-human" ? "human-wait-timeout" : Phase=="resuming" ? "health-timeout" : "read-timeout");
     if(count==0) {
      if(used!=0 || terminal==null) return Reject("frame-length");
      if(!ExactProcess(process,ticks,image,hash)) return Reject("process-after-read");
      if(elapsed.ElapsedMilliseconds>=deadline) return Reject("verification-timeout");
      if(terminal=="cancel") { Phase="cancelled";return Reject("human-cancelled"); }
      Phase="healthy";
      VerifiedFrame=expected; VerifiedImage=Path.GetFullPath(image); VerifiedHash=hash;
      if(imageOwner!=null) VerifiedNativeImage=imageOwner.NativeImage;
      return true;
     }
     for(int i=0;i<count;i++) {
      if(terminal!=null) return Reject("frame-trailing");
      if(used==record.Length) return Reject("frame-oversized");
      byte value=buffer[i];
      if(value>127) return Reject("frame-encoding");
      record[used++]=value;
      if(value!=10) continue;
      if(++records>3) return Reject("frame-count");
      var frame=Encoding.ASCII.GetString(record,0,used);used=0;
      // Every phase has the same complete identity/challenge, and each
      // transition rechecks the original live process, exact path and hash.
      if(!ExactProcess(process,ticks,image,hash)) return Reject("process-before-phase");
      if(elapsed.ElapsedMilliseconds>=deadline) return Reject(Phase=="awaiting-human" ? "human-wait-timeout" : Phase=="resuming" ? "health-timeout" : "read-timeout");
      if(frame==waitFrame && Phase=="startup" && records==1) {
       Phase="awaiting-human";deadline=elapsed.ElapsedMilliseconds+humanWaitMs;
      } else if(frame==continueFrame && Phase=="awaiting-human" && records==2) {
       Phase="resuming";deadline=elapsed.ElapsedMilliseconds+timeoutMs;
      } else if(frame==cancelFrame && Phase=="awaiting-human" && records==2) {
       // Cancellation is not health. Require bounded EOF, without granting a
       // new deadline; trailing records can never be reinterpreted as success.
       Phase="cancelled";terminal="cancel";
      } else if(frame==expected && (Phase=="startup" || Phase=="resuming")) {
       terminal="ready";
      } else return Reject("frame-mismatch-or-order");
     }
    }
   } catch(Exception error) {
    // Diagnostic category/HRESULT only: never expose frame, challenge or paths.
    error=error.GetBaseException();
    return Reject(stage+":"+error.GetType().Name+":0x"+error.HResult.ToString("X8"));
   } finally { pipe.Dispose(); }
  }
  public void Dispose() { try { pipe.Dispose(); } finally { if(imageOwner!=null) imageOwner.Dispose(); } }
 }
}
'@
  # NSIS may have a native System.dll in its cwd; compile from the framework.
  $previous = [Environment]::CurrentDirectory
  Push-Location -LiteralPath ([Runtime.InteropServices.RuntimeEnvironment]::GetRuntimeDirectory())
  try {
    [Environment]::CurrentDirectory = (Get-Location).Path
    Add-Type -TypeDefinition $source
  } finally { [Environment]::CurrentDirectory = $previous; Pop-Location }
}
function New-UpdateHealthSession($Identity, [string]$Transaction = '') {
  Assert-UpdateIdentity $Identity
  if (-not $Transaction) { $Transaction = [guid]::NewGuid().ToString('N') }
  if ($Transaction -cnotmatch '^[a-f0-9]{32}$') { throw 'Invalid startup transaction.' }
  Initialize-UpdatePipeType
  $session = [guid]::NewGuid().ToString('N')
  $bytes = New-Object byte[] 32
  $rng = [Security.Cryptography.RandomNumberGenerator]::Create()
  try { $rng.GetBytes($bytes) } finally { $rng.Dispose() }
  $challenge = ([BitConverter]::ToString($bytes)).Replace('-','').ToLowerInvariant()
  $name = 'Skager.Update.' + $session
  $server = New-Object Skager.UpdateStartupPipe($name)
  return [pscustomobject]@{server=$server; pipe=$name; challenge=$challenge; session=$session; transaction=$Transaction}
}
function New-UpdateStartupSession($Record) {
  Assert-UpdatePendingRecord $Record
  if ($Record.attempts -ne 0) { throw 'An update candidate gets one startup attempt; recovery never retries it automatically.' }
  $session = New-UpdateHealthSession $Record.candidate $Record.transaction
  $Record.attempts = 1
  # New callers arm a durable refusal state before the native creation call.
  if (-not $Record.PSObject.Properties['nativeLaunchV1']) {
    $Record | Add-Member -NotePropertyName nativeLaunchV1 -NotePropertyValue 'none'
    $Record | Add-Member -NotePropertyName processCreationFileTimeV1 -NotePropertyValue ''
  }
  $Record.nativeLaunchV1 = 'armed'
  $Record.session = $session.session
  # The caller must durably write this modified record before process creation.
  return $session
}
function Get-UpdateReadyFrame($Identity, $Session) {
  Assert-UpdateIdentity $Identity
  if ($Session.challenge -cnotmatch '^[a-f0-9]{64}$' -or $Session.session -cnotmatch '^[a-f0-9]{32}$') { throw 'Invalid startup session.' }
  return 'SKAGER-UPDATE-READY/1 ' + $Identity.generation + ' ' + $Identity.commit + ' ' + $Session.challenge + "`n"
}
function Wait-UpdateGenerationStartupSuccess($Identity, $Session, [Diagnostics.Process]$Process, [string]$ExecutablePath, [int]$TimeoutMilliseconds=90000) {
  $ExecutablePath = Assert-UpdateRecordPath $ExecutablePath
  $expected = Get-UpdateReadyFrame $Identity $Session
  return $Session.server.Receive($Process,$ExecutablePath,$Identity.executableSha256,$expected,$TimeoutMilliseconds)
}
function Wait-UpdateStartupSuccess($Record, $Session, [Diagnostics.Process]$Process, [string]$ExecutablePath, [int]$TimeoutMilliseconds=90000) {
  Assert-UpdatePendingRecord $Record
  if ($Record.attempts -ne 1 -or $Record.session -cne $Session.session -or $Record.transaction -cne $Session.transaction) { throw 'Startup session does not match pending update.' }
  return Wait-UpdateGenerationStartupSuccess $Record.candidate $Session $Process $ExecutablePath $TimeoutMilliseconds
}
function Test-UpdateIdentityEqual($Left, $Right) {
  Assert-UpdateIdentity $Left
  Assert-UpdateIdentity $Right
  return $Left.generation -ceq $Right.generation -and $Left.commit -ceq $Right.commit -and
    $Left.packageSha256 -ceq $Right.packageSha256 -and $Left.executableSha256 -ceq $Right.executableSha256
}
function Write-UpdateReceiptBytes([string]$Path, [byte[]]$Bytes) {
  $Path = Assert-UpdateRecordPath $Path
  $temporary = $Path + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
  try {
    $file = New-Object IO.FileStream($temporary, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
    try { $file.Write($Bytes,0,$Bytes.Length); $file.Flush($true) } finally { $file.Dispose() }
    if ([IO.File]::Exists($Path)) { [IO.File]::Replace($temporary,$Path,[Management.Automation.Language.NullString]::Value) }
    else { [IO.File]::Move($temporary,$Path) }
  } finally { if ([IO.File]::Exists($temporary)) { [IO.File]::Delete($temporary) } }
}
function Write-UpdateKnownGoodReceipt([string]$Path, $Identity, $Session, [string]$ExecutablePath) {
  Assert-UpdateIdentity $Identity
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Known-good startup proof requires native Windows.' }
  $ExecutablePath = Assert-UpdateRecordPath $ExecutablePath
  if ($Session.server -isnot [Skager.UpdateStartupPipe] -or
      $Session.server.VerifiedFrame -cne (Get-UpdateReadyFrame $Identity $Session) -or
      $Session.server.VerifiedHash -cne $Identity.executableSha256 -or
      -not [string]::Equals($Session.server.VerifiedImage,$ExecutablePath,[StringComparison]::OrdinalIgnoreCase)) { throw 'No authenticated live startup proof for this generation.' }
  $Session.server.AssertOwnedImage($ExecutablePath,$Identity.executableSha256)
  $record = [pscustomobject]@{schema=1; owner='SKAGER.KnownGoodStartup.1'; identity=$Identity; session=$Session.session;
    confirmedUtc=[DateTime]::UtcNow.ToString('o'); protocol='SKAGER-UPDATE-READY/1'}
  $bytes = [Text.Encoding]::UTF8.GetBytes(($record | ConvertTo-Json -Depth 6 -Compress))
  Add-Type -AssemblyName System.Security
  $protected = [Security.Cryptography.ProtectedData]::Protect($bytes,[Text.Encoding]::UTF8.GetBytes('SKAGER.KnownGoodStartup.1'),[Security.Cryptography.DataProtectionScope]::CurrentUser)
  Write-UpdateReceiptBytes $Path $protected
}
function Assert-UpdateKnownGoodReceipt([string]$Path, $Identity) {
  Assert-UpdateIdentity $Identity
  $Path = Assert-UpdateRecordPath $Path
  if (-not [IO.File]::Exists($Path) -or (Get-Item -LiteralPath $Path).Length -notin 1..8192) { throw 'No authenticated known-good startup receipt; qualify the current generation first.' }
  if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) { throw 'Known-good startup proof requires native Windows.' }
  Add-Type -AssemblyName System.Security
  try {
    $plain = [Security.Cryptography.ProtectedData]::Unprotect([IO.File]::ReadAllBytes($Path),[Text.Encoding]::UTF8.GetBytes('SKAGER.KnownGoodStartup.1'),[Security.Cryptography.DataProtectionScope]::CurrentUser)
    if ($plain.Length -gt 4096) { throw 'Oversized known-good receipt.' }
    $record = [Text.Encoding]::UTF8.GetString($plain) | ConvertFrom-Json
    if ($record.schema -ne 1 -or $record.owner -cne 'SKAGER.KnownGoodStartup.1' -or $record.protocol -cne 'SKAGER-UPDATE-READY/1' -or
        $record.session -cnotmatch '^[a-f0-9]{32}$' -or -not (Test-UpdateIdentityEqual $record.identity $Identity)) { throw 'Known-good receipt identity mismatch.' }
  } catch { throw ('Known-good startup receipt could not be authenticated: ' + $_.Exception.Message) }
}
